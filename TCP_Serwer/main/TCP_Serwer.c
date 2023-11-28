

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <lwip/netdb.h>
#include <errno.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "FreeRTOSConfig.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/dns.h"
#include "lwip/apps/sntp.h"

//-----------------------------------------
//Własne moduły do obsługi sieci WiFi
#include "mk_wifi.h"
#include "mk_tools.h"

//Definicja pinu procesora do którego podłączyc diodę testową LED
#define LED1_GPIO 		2

/* Definiowanie zdalnych adresów IP i portów dla serwera TCP */
//#ifdef PRACA

//Konfiguracja dla komunikacji TCP
#define TCP_REMOTE_PORT		9999
#define TCP_AP_SERVER_PORT		5555

//#endif

#define MAX_SRV_CLIENTS		1	// maksymalna liczba klientów serwera TCP

/***** Struktura informacji n/t klienta serwera TCP *****/
typedef struct {
	int				active;		//czy dany klient jest aktywny czy nie
	int				idx;		//index - numer klienta
	struct in_addr  sin_addr;	//adres IP klienta w postaci cyfrowej nie stringowej
	char 			ip[16];		//numer IP klienta w postaci stringa
	uint16_t		port;		//numer portu klienta, z którego przyszło zapytanie
	TaskHandle_t	handle;		//uchwyt do taska, który zostanie powołany do obsługi danego klienta
	int				sock;		//gniazdo aktualnego klienta
} TSRVCLN;

/***** Lista klientów serwera TCP ******/
static TSRVCLN cln_handles[ MAX_SRV_CLIENTS ];	//utworzenie tablicy, która przechowywać będzie dane na podstawie struktury o klientach TCP
static int cln_count;	//Zmienna do przechowywani ilości aktywnych klientów z listy
//int cln_sock;

/****** Socket (gniazdo) nasłuchowe serwera TCP *******/
static int listen_sock;

/***** Uchwyty do najważniejszych tasków *****/
TaskHandle_t tServerHandle, srvclnsendhandle;
//moje uchwyty:
TaskHandle_t tSerwerSendHandle;	//Uchwyt na taska do wysyłania kontrolnego danych

/***** KOLEJKA DO wysyłania danych do klientów serwera tcp *****/
QueueHandle_t xQueueClientTCP;

//struktura do przesyłu danych pomiędzy serwerem i klientem TCP
typedef struct {
	int	clsock;
	char data[100];
} TQUEUE_TCP_DATA;

int close_serwer = 0;

TQUEUE_TCP_DATA dane;

static const char *TAG = "TCP SRV: ";
static const char *TAG_M = "Main SYSTEM: ";

// nagłówek taska do wysyłania danych do klientów serwera
void tcp_srv_cln_send_task( void * arg );

/*  SMP (Symmetric Multiprocessing) */
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#define PORT_ENTER_CRITICAL 	portENTER_CRITICAL(&mux)
#define PORT_EXIT_CRITICAL 		portEXIT_CRITICAL(&mux)


/* ------------------------------------------------------------------------------- */
/* -------- Funkcje pomocnicze do obsługi SERWERA TCP ---------------------------- */
/* ------------------------------------------------------------------------------- */

//TASK do wysyłania danych testowych do klienta (na kolejkę wysyłającą), który się podłączyli do serwera TCP w trybie AP
void ap_datasend_srv_task( void * arg ) {

	uint8_t licznik=0;

	while(1){
		//zerowanie bufora na nadchodzące dane
		memset( dane.data, 0, sizeof(dane.data) );
		sprintf( dane.data, "Dane testowe z serwera TCP, licznik: %d \n", licznik++ );
		xQueueSend( xQueueClientTCP, &dane, 0 );
		vTaskDelay( 100 );
	}
}


/*
 * Funkcja sprawdzający czy jest wolne miejsce na liście dla nowego klienta TCP i jeśli jest zainicowanie element listy dla niego,
 * Jeśli nie ma miejsca na liście to zwróć NULL
 * Gdy pojawi się pierwszy klient uruchom dynamicznie task umożliwiający nadawanie danych
 * do podłączonych klientów serwera TCP
 *
 */
TSRVCLN * get_free_cln_slot( void ) {

	int i;
	TSRVCLN * cl = NULL;	//powołanie wskaźnika na podstawie struktury i jego wyzerowanie

	PORT_ENTER_CRITICAL;	//Zablokowanie innych tasków w systemie
	//W pętli sprawdzamy, które pole w tablicy klientów TCP jest aktualnie wolne
	for( i=0; i<MAX_SRV_CLIENTS; i++ ) {
		if( !cln_handles[i].active ) {	//czy pole aktive w tablicy jest równe 0
			cl = &cln_handles[i];		//wskazujemy, który element tablicy jest równy 0
			cl->active = 1;
			cl->idx = i;
			memset( cl->ip, 0, sizeof(cl->ip) );	//zerowanie starych danych z tabeli z adresem IP klienta TCP w postaci stringa przechowywanych
			cln_count++;	//zwiększ liczbę aktualnie podłaczonych klientów TCP
			break;
		}
	}
	PORT_EXIT_CRITICAL;

	if( cln_count == 1 ) {	//gdy podłączy sie pierwszy i tylko pierwszy klient TCP, wówczas wyświetl dane na terminal i uruchom taska od obsługi klientów TCP
		ESP_LOGW(TAG, "... Create send task for Client - TASK Create ..." );
		xTaskCreate( tcp_srv_cln_send_task, "", 2048, NULL, 2, &srvclnsendhandle  );
	}
	return cl;	//zwróć wskaźnik na wybrany element tablicy
}

/*
 *  Podaj liczbę podłączonych klientów do serwera TCP
 */
int get_cln_count( void  ) {

	int res;

	PORT_ENTER_CRITICAL;
	res = cln_count;
	PORT_EXIT_CRITICAL;

	return res;
}


/* USUŃ informacje o kliencie podłączonym do serwera TCP jeśli sesja została zakończona */
void clear_cln_slot( TSRVCLN * cl ) {

    if( cl->sock != -1) {
        ESP_LOGE(TAG, "Shutting down socket idx: %d ...", cl->idx );
        shutdown(cl->sock, 0);
        close(cl->sock);
    }

    PORT_ENTER_CRITICAL;
	cl->active = 0;
	cl->ip[0] = 0;
	cl->port = 0;
	cl->idx = -1;
	cl->handle = NULL;
	cln_count--;
	PORT_EXIT_CRITICAL;

	if( !get_cln_count() && srvclnsendhandle != NULL ) {
		vTaskDelete( srvclnsendhandle );	//Kasowanie uchwytu do taksa tworzącego połączenie TCP serwera z klientem
		srvclnsendhandle = NULL;
		vTaskDelete( tSerwerSendHandle );	//Kasowanie uchwytu do taksa wysyłającego dane testowe do klienta TCP
		tSerwerSendHandle = NULL;
		ESP_LOGW(TAG, "Conection TCP to Client - TASK DELETE..." );
		vTaskDelete( tSerwerSendHandle );
	}
    vTaskDelete( NULL );
}



/*
 * Zakończ wszystkie połączenia klientów serwera TCP, zwolnij pamięć,
 * zakończ task nadawczy do klientów jeśli wszyscy zostali odłączeni
 */
void close_all_tcp_server_clients( void ) {

    TSRVCLN * cl;
    PORT_ENTER_CRITICAL;
    for( int i=0; i<MAX_SRV_CLIENTS; i++ ) {

    	cl = &cln_handles[i];
    	if( cl->active ) {
    	    if( cl->sock != -1) {
    	        ESP_LOGE(TAG, "Shutting down srv_cln_sock..." );
    	        shutdown(cl->sock, 0);
    	        close(cl->sock);
    	        cl->sock = -1;
    	    }
    	}
    }
    PORT_EXIT_CRITICAL;
}




/* ````````````````````````````````````````````````````````````````````` */
/* ````` TASK SERWERA do wysyłania informacji do dowolnego klienta ````` */
/* ````` poprzez obsługę odbieranie danych z kolejki               ````` */
/* ````````````````````````````````````````````````````````````````````` */
void tcp_srv_cln_send_task( void * arg ) {

	BaseType_t xStatus;
//	TQUEUE_TCP_DATA dane;

	while(1) {

		int sock = 0;
		//zerowanie bufora na nadchodzące dane
		memset( dane.data, 0, sizeof(dane.data) );
		//Oczekiwanie w nieskończoność na dane do klientów TCP (sprawdzanie kolejki)
		xStatus = xQueueReceive( xQueueClientTCP, &dane, portMAX_DELAY );

		sock = dane.clsock;

		if( xStatus == pdPASS && sock > 0  ) {

			int err = send(sock, dane.data, strlen( dane.data ), 0 );

			if (err < 0) {
				ESP_LOGE(TAG, "Wystapil blad podczas wysylania: errno %d", errno);
			}
		}
	}
}



/* ``````````````````````````````````````````````````````````````` */
/* ````` TASKI Klientów SERWERA TCP - do obsługi komunikacji ````` */
/* ``````````````````````````````````````````````````````````````` */
void ap_tcp_srv_cln_task( void * arg ) {

	TSRVCLN * me = (TSRVCLN*)arg;
	int licznik=0;

	int idx = me->idx;

	//Wyświetl dane o podłączonym kliencie TCP na terminalu
	ESP_LOGW(TAG, "***** AP MODE TCP SERVER-CLIENT START *****" );
	ESP_LOGW(TAG, "Client idx: %d", me->idx );					//numer id w tabeli jaką została mu nadana
	ESP_LOGW(TAG, "Client IP/Port: %s:%d", me->ip, me->port );	//adres IP i numer portu z jakiego podłączył się klient
	//Te wyświetlane dane są mało zrozumiałe w systemie dla użytkownika
//	ESP_LOGW(TAG, "Client sock: %d", me->sock );				//numer gniazda nadany przez serwer TCP
//	ESP_LOGW(TAG, "Client Task Handle: %d", (int)me->handle );	//numer uchwytu dany przez serwer tCP

	uint16_t dat = uxTaskGetStackHighWaterMark(NULL);	//odczytaj ilość wolnej pamięci w tasku
	ESP_LOGW(TAG, "Client free stack mem: %d", dat );	//wyświetl ilość wolnej pamięci na terminalu

	//Utworzenie taska do testowego (czasowego) wysyłania danych po TCP do klienta
	xTaskCreate( ap_datasend_srv_task, "", 2048, NULL, 1, &tSerwerSendHandle );
	dane.clsock = me->sock;

	int len = 0;
	char rx_buffer[128];

	while(1) {

		//Oczekiwanie na dane od klienta TCP - funkcja blokująca
		len = recv(me->sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

        //sprawdzanie warunków jak długie są odebrane od klienta dane
		if( len < 0 ) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
            break;
        } else
        	if( len == 0 || errno == 128 ) {
				ESP_LOGW(TAG, "Connection closed");
				break;
        	} else {
        		printf("Dane odebrane");
				printf( ">>> " );
				printf( rx_buffer );
				printf( "\r\n" );
				rx_buffer[len] = 0;

				// wyślij odpowiedź o odebranych danych do klienta, który przysłał wiadomość poprzez kolejkę
				dane.clsock = me->sock;
				sprintf( dane.data, "Liczba odebranych danych: %d [Client ID: %d]\n", licznik++, me->idx );
				xQueueSend( xQueueClientTCP, &dane, 0 );
        	}
	}

	ESP_LOGW(TAG, "***** TCP SERVER-CLIENT in AP MODE idx: %d CLOSE *****", idx );
	vTaskDelay( 1 );

	clear_cln_slot( me );
	vTaskDelay( 1 );

	vTaskDelete( NULL );
}




/* ``````````````````````````````````````````````` */
/* ````` GŁÓWNY TASK SERWERA TCP W TRYBIE AP ````` */
/* ``````````````````````````````````````````````` */
void tcp_apmode_server_task( void * arg ) {

    char addr_str[128];

    vTaskDelay( 100 );

    while(1) {

		struct sockaddr_in localAddr;	//Powołanie strukturki do utworzenia gniazda
		localAddr.sin_addr.s_addr = htonl(INADDR_ANY);	//Wiązanie adresu IP z numerem portu na serwerze
		localAddr.sin_family = AF_INET;
		localAddr.sin_port = htons( TCP_AP_SERVER_PORT );	//Wpisanie do utwrzonej strukturki numeru portu
		inet_ntoa_r(localAddr.sin_addr, addr_str, sizeof(addr_str) - 1);	//zapisanie do bufora adresu IP
		printf( addr_str );

		listen_sock = socket( AF_INET, SOCK_STREAM, IPPROTO_IP );	//Tworzenie soceta i zapisanie go pod podaną nazwą
		if (listen_sock < 0) {	//Gdy nie udało się utworzyc gniazda
			ESP_LOGE( TAG, "Unable to create socket: errno %d", errno );
			break;	//Powrót z pętli nieskończonej
		}
		ESP_LOGI(TAG, "TCP SERVER Socket created in AP MODE");		//Gdy socket został stworzony to wyślij te informację na terminal

		int on= 1 ;
		if( (setsockopt(listen_sock,SOL_SOCKET,SO_REUSEADDR,&on, sizeof (on))) < 0 ) {
			ESP_LOGE(TAG, " błąd ponownego użycia adresu tcp gniazda errno = %d" ,errno);
		}	//Ustawiamy opcje za pomoca komend API dla naszego soceta

		//wiązanie soceta z portem nasłuchowym dla serwera
		int err = bind(listen_sock, (struct sockaddr *)&localAddr, sizeof(localAddr));
		if (err < 0) {	//sprawdzenie czy wystąił bląd
			ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
			break;
		}
		ESP_LOGI(TAG, "Socket binded");	//Jeżeli udało się wiązanie to wyślij komunikat na terminal

		err = listen( listen_sock, 1 );	//nasz socet ma móc wystartował z nasłuchem połączeń TCP na wskazanym gnieźdźie
		if (err != 0) {		//gdy nie można nasłuchiwać to za pomoca rozkazu continue wróc do początku pętli i jeszcze raz spróbuj wszystko ustawić
			ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
			shutdown(listen_sock, 0);
			close(listen_sock);

			vTaskDelay( 100 );
			continue;
		}
		//Wyświetl info, że właczone zostało nasłuchiwanie na gnieździe
		ESP_LOGI(TAG, "Socket listening");

		//Początek pętli do akceptowania przychodzących połaczeń od klientów TCP
		while(1) {

			ESP_LOGI(TAG, "Listen Socket in AP MODE - ACCEPT START...");

			struct sockaddr_in sourceAddr;		//Przygotowanie strukturki na adres IP i port klienta, który może się podłączy do serwera
			socklen_t socklen = sizeof(sourceAddr);

			//Polecenie API accept -blokuje pracę w tym tasku, aż zostanie wykryty jakiś klient
			int cln_sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &socklen );	//Jak się podłaczy klient to jego dane zostają zapisane do zmiennej cln_sock

			//Sprawdzenie poprawności połączenia z klientem
			if( cln_sock < 0 ) {	//Jeżeli nie udało się poprawnie uzyskać gniazda klienta wówczas przerwij te pętlę nieskończoną
				ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
				if ( errno == EINTR ){

					close( listen_sock );
				}
				break;
			}

			ESP_LOGI(TAG, "Socket accepted");	//info gdy udało sie poprawnie odczytać gniazdo klienta

			//DO tego miejsca było odczytanie danych klienta, który chce sie podłączyć do serwera TCP

			//Powołanie wskaźnika na funkcję oraz od razu za pomoca własnej funkcji sprawdzenie czy jest wolne miejsce na nowego klienta TCP,
			//gdy takie miejsce jest dostępne w nasze tablicy klientów, zwraca wskaźnik na pierwsze wolne miejsce na liście i wstępnie je inicjalizuje
			//pole active i idx są już wypełnione
			TSRVCLN * cl = get_free_cln_slot();

			if( cl != NULL ) {	//Gdy zwrocony wskaźnik na tablicę jest rózny od zera (czyli jest miejsce w niej na nowego klienta)

				//uzupełniamy wszystie pola w tabeli nowym klientem TCP
				cl->sock = cln_sock;
				cl->sin_addr = sourceAddr.sin_addr;
				cl->port = sourceAddr.sin_port;
				//Zamiana adreu IP nowego klienta na postać stringową i zapisanie tych danych w polu ip o długości równej temu polu (w razie czego na końcu dopisz 0)
				inet_ntoa_r(sourceAddr.sin_addr, cl->ip, sizeof(cl->ip) - 1);

				//powołanie taska do obsługi komunikacji z klientami podłączonego w trybie AP do serwera TCP
				ESP_LOGI(TAG, "Task w Serwerze TCP dla klienta utworzony");
				xTaskCreate( ap_tcp_srv_cln_task, "", 4096, (TSRVCLN*)cl, 1, &cl->handle );

			} else {	//W przypadku, gdy zwracane jest 0 w zmiennej cl wówczas nie można dodać nowego klienta i należy wysłać na termina odpowiednie info
				char reject_info[128];
				sprintf( reject_info,"Brak miejsca na serwerze\n" );	//Do bufora wrzucamy napis dla podłączonego klienta
				send( cln_sock, reject_info, strlen(reject_info), 0 );	//wysłanie informacji o braku miejsca do klienta
				vTaskDelay( 10 );
			    if( cln_sock != -1) {		//sprawdzamy, czy nadal socet od klienta jest działający

			        shutdown(cln_sock, 0);	//Wyłaczamy gniazdo od klienta, którego nie chcemy obsłużyć
			        close(cln_sock);		//Zamykamy istniejące gniazdo od klienta
			        ESP_LOGE(TAG, "Socket closed gracefully..." );	//komunikat na terminal
			    }
				ESP_LOGE(TAG, "BRAK MIEJSCA DLA KLIENTÓW do SERVERA TCP");	//info na terminal
			}
//			//Utworzenie taska do testowego wysyłania danych po TCP do klienta
//			xTaskCreate( ap_datasend_srv_task, "", 2048, (TSRVCLN*)cl, 1, &cl->handle );
		}	//powrót do pętli nieskończonej (oczekiwanie na nowego klienta)

	    //Gdy nastąpił break w pętli od akceptacji nowych klientów TCP wówczas zamknij nasłuchiwanie i powrót do początku taska do uruchomienia nasłuchiwania na serwerze TCP
		if( listen_sock != -1) {
	        ESP_LOGE(TAG, "Shutting down listen_sock..." );
	        shutdown(listen_sock, 0);
	        close(listen_sock);
	    }
	    vTaskDelay( 100 );
    }	//Powrót do początku taska do ustanowienia nasłuchiwania na serwerze

	vTaskDelete(NULL);	//wyłączenie taska, bo nie udało się stworzyć gniazda (wykonywany break został)
}


//Kod callbacka realizowanego w chwili otrzymania adresu IP przez układ ESP32
//Wyświetlane są adresu IP z trybu STA i AP
void mk_got_ip_cb( char * ip ) {

	esp_netif_ip_info_t ip_info;
	esp_netif_get_ip_info(mk_netif_sta, &ip_info);
	printf( "[STA] IP: " IPSTR "\n", IP2STR(&ip_info.ip) );
	printf( "[STA] MASK: " IPSTR "\n", IP2STR(&ip_info.netmask) );
	printf( "[STA] GW: " IPSTR "\n", IP2STR(&ip_info.gw) );

	esp_netif_get_ip_info(mk_netif_ap, &ip_info);
	printf( "[AP] IP: " IPSTR "\n", IP2STR(&ip_info.ip) );
	printf( "[AP] MASK: " IPSTR "\n", IP2STR(&ip_info.netmask) );
	printf( "[AP] GW: " IPSTR "\n", IP2STR(&ip_info.gw) );
}

//Kod funkcji realizowanej z chwil� roz��czenia z punktem dost�powym modu�u ESP32
void mk_sta_disconnected_cb( void ) {

	printf("\n****** STA Disconnected - callback \n");

//	//Kasowanie serwera TCP i rzeczy z nim powiązanych
//	if( tServerHandle != NULL ) {	//sprawdzamy, czy jest nie zerowy uchwyt do taska związany z serwerem TCP
//
//		    if( listen_sock != -1) {	//sprawdzmy, czy układ pracuje w trybie nasłuchiwania na połączenia przychodzące i jeżeli tak
//		        ESP_LOGE(TAG, "Shutting down listen_sock..." );
//		        shutdown(listen_sock, 0);	//wyłaczmy nasłuch
//		        close(listen_sock);			//Zamykamy nasłuchiwanie
//		    }
//
//		    close_all_tcp_server_clients();	// close all client sockets and tasks
//
//			ESP_LOGW(TAG, "SERVER TCP TASK - DELETE...");
//			vTaskDelete( tServerHandle );	//skasuj task serwera
//			tServerHandle = NULL;			//i przypisujemy mu wartość null, aby go wyczyścić z posiadanych przez niego informacji
//		}
//	 vTaskDelete( NULL );

	//Na razie do zakomentowania
//		if( !get_cln_count() && srvclnsendhandle != NULL ) {
//			vTaskDelete( srvclnsendhandle );
//			srvclnsendhandle = NULL;
//		}
}

//Kod funkcji realizowanej z chwilą połączenia się nowego urządzenia do modułu ESP32 działającego w trybie AP
void ap_mode_join_cb( char * mac ) {

	printf("\n***** Join Client to AP ***** \n");
	tcpip_adapter_ip_info_t ip_info;	//powołanie struktury do przechowywania adresu IP modułu działająego w trybie AP

	// pobieranie informacji o konfiguracji adresów IP
	tcpip_adapter_get_ip_info( TCPIP_ADAPTER_IF_AP, &ip_info );

//	esp_netif_ip_info_t ip_info;
//	esp_netif_get_ip_info(mk_netif_ap, &ip_info);
	//Wyświetlanie informacji o posiadanych adresów IP na terminalu
//	printf( "[AP] IP: " IPSTR "\n", IP2STR(&ip_info.ip) );
//	printf( "[AP] MASK: " IPSTR "\n", IP2STR(&ip_info.netmask) );
//	printf( "[AP] GW: " IPSTR "\n", IP2STR(&ip_info.gw) );


	/*************/
	//Może to wykorzytsać do sprawdzania czy klient jakis jest podłączony do serwera TCP
	//int get_cln_count( void  ) {

	//Po podłączeniu się urządzenia do wewnętrznej sieci WiFi modułu ESP tworzony jest task
	//do obsługi połaczenia z klientami i tworzony jest serwera TCP do komunikacji z nimi
	xTaskCreate( tcp_apmode_server_task, "", 4096, NULL, 1, &tServerHandle );
}

//Kod funkcji realizowanej z chwilą rozłączenia się podłączonego z ESP urządzenia, gdy moduł ESP32 działa w trybie AP
void ap_mode_leave_cb( char * mac ) {

	//Kasowanie serwera TCP i rzeczy z nim powiązanych
	if( tServerHandle != NULL ) {	//sprawdzamy, czy jest nie zerowy uchwyt do taska związany z serwerem TCP

		if( listen_sock != -1 ) {

			shutdown(listen_sock, 0);
			close(listen_sock);
			ESP_LOGE(TAG, "Shotting down listen_sock...");
		}

		close_all_tcp_server_clients();

		close_serwer = 1;
//		ESP_LOGW(TAG, "SERVER TCP TASK - DELETE...");
//		vTaskDelete( tServerHandle );	//skasuj task serwera
//		ESP_LOGW(TAG, "SERVER TCP TASK - DELETE...");
//		tServerHandle = NULL;			//i wyzerowanie uchwytu do taska
	}

	printf("\n ***** AP MODE Client LEAVE ***** \n");

}

//Funkcja wyświetlająca ilośc wolnej pamięci w module ESP32
void display_info( void ) {

	uint32_t mem=0;

	mem = esp_get_free_heap_size();
	ESP_LOGI(TAG_M, "Free memory stack: %d", mem);
	ESP_LOGI(TAG_M, "Number of Client Conected: %d", get_cln_count());

	printf("\n");
}


//Funkcja do wyświetlania daty i czasu
void display_date_time( void ) {

    char buf[128], fmt[128];
    char abuf[128];
      	buf[0] = 0;

    time_t now;
    struct tm timeinfo;


    time(&now);		//pobierz czas systemowy
	localtime_r(&now, &timeinfo);	//Pobrany czas zapisz do zmiennej timeinfo

	if( !is_time_changed() ) return;	// if time not changed - return without display

	fmt[0] = 0;
	strcat( fmt, "%F" );
	strftime( buf, sizeof(fmt), fmt, localtime(&now) );

	if( !buf[0] ) sprintf( buf, "WAITING FOR SNTP ..." );

	//Dane na terminal
	strcpy( abuf, "Date:" );
	strcat( abuf, buf );
	strcat( abuf, " Time:" );

	fmt[0] = 0;
	strcat( fmt, "%H:%M:%S %Z %z" );
	strftime( buf, sizeof(fmt), fmt, localtime(&now) );
	strcat( abuf, buf );

	// show on terminal
	printf( "%s (%d)\n", abuf, (int32_t)now );
}


/*   ******************  *
 *   POCZąTEK PROGRAMU   *
 *   ******************  */
void app_main(void) {

	vTaskDelay( 100 );	// tylko żeby łatwiej przełączać się na terminal przy starcie
	printf("\nREADY\n");

	/*... konfiguracja pinów dla testowej diody LED, która miga w czasie działania systemu ...*/
	gpio_set_direction( LED1_GPIO, GPIO_MODE_OUTPUT );
	gpio_set_level( LED1_GPIO, 1 );

//	/* ------  Inicjalizacja kolejki na potrzeby przesyłu danych po UDP  ------ */
//	xQueueClientUDP = xQueueCreate( 5, sizeof( TQUEUE_UDP_CLI_DATA ) );

	/* --- Inicjalizacja kolejki na potrzeby przesyłu danych po TCP --- */
	xQueueClientTCP = xQueueCreate( 5, sizeof( TQUEUE_TCP_DATA ) );

	/*  ------   Inicjalizacja NVS   ----- */
	nvs_flash_init();

	/* --- Wybór trybu działania modułu ESP32 w sieci WiFi --- */
//	mk_wifi_init( WIFI_MODE_APSTA, mk_got_ip_cb, mk_sta_disconnected_cb, mk_ap_join_cb, mk_ap_leave_cb  );
	mk_wifi_init( WIFI_MODE_AP, NULL, NULL, ap_mode_join_cb, ap_mode_leave_cb );
//	mk_wifi_init( WIFI_MODE_STA, mk_got_ip_cb, mk_sta_disconnected_cb, NULL, NULL );

    display_info();		//Funkcja wyświetlająca ilośc wolnej pamięci w module ESP32

//    mk_sntp_init(NULL); // inicjalizacja SNTP ze sterefą czasową dla Polski

//    int msec = 0;
//    while(1) {
//    	display_date_time();	//Wy�wietl aktualny czas na terminalu zapisany w module ESP
//    	vTaskDelay( 10 );
//    	if( ++msec > 50 ) break; //p�tla opu�niaj�ca progam g��wny
//    }

	/* ```````` Skanowanie dostępnych sieci  ``````````````````````````````````` */
//	mk_wifi_scan( NULL );	//Wyświetl dane na ekranie terminala

	/* ---- zmienne lokalne głównego tasku ------------------------------------- */
//	uint8_t licznik = 0;
	uint8_t sw = 0;

  // ****** POCZąTEK WąTKU GłóWNEGO  *****
	while (1) {

//		//sprawdzamy, czy układ ma przypisany adres IP
//		if( get_sta_ip_state() ) {
//
//			//Funkcja zapisuje tekst sformatowany (wed�ug wzorca) do wskazanego bufora znakowego
//			sprintf( dane.ip, "%s", UDP_REMOTE_ADDR );
//			dane.port 	= UDP_REMOTE_PORT;
//			sprintf( dane.data, "cnt UP %d\n", licznik );
//			//Wy�lij utworzone w linii powy�ej dane do kolejki
//			xQueueSend( xQueueClientUDP, &dane, 0 );
//
//			sprintf( dane.ip, "%s", UDP_REMOTE_ADDR2 );
//			dane.port 	= UDP_REMOTE_PORT2;
//			sprintf( dane.data, "cnt DOWN %d\n", 255 - licznik );
//			xQueueSend( xQueueClientUDP, &dane, 0 );
//
//			printf( "data sent: %d\n", licznik );
//
//			licznik++;
//		}
//		display_info();
    	gpio_set_level( LED1_GPIO, sw );
        sw ^= 1;
        vTaskDelay( 200 );

        if( close_serwer != 0 ){

    		ESP_LOGW(TAG, "SERVER TCP TASK - DELETE...");
    		vTaskDelete( tServerHandle );	//skasuj task serwera
    		ESP_LOGW(TAG, "SERVER TCP TASK - DELETE...");
    		tServerHandle = NULL;			//i przypisujemy mu wartość null, aby go wyczyścić z posiadanych przez niego informacji
    		ESP_LOGW(TAG, "SERVER TCP TASK - DELETE...");
    		close_serwer = 0;
        }
    }
}


























//
////-----------------------------------------
////Własne moduły do obsługi sieci WiFi
//#include "mk_wifi.h"
//#include "mk_tools.h"
//
////Definicja pinu procesora do którego podłączyc diodę testową LED
//#define LED1_GPIO 		2
//
///* Definiowanie zdalnych adresów IP i portów dla serwera TCP */
//
////Konfiguracja dla komunikacji TCP
//#define TCP_REMOTE_PORT		9999
//#define TCP_SERVER_PORT		5555
//
//
//#define AP_SSID      		"ESP32_TCP_Test"
//#define AP_PASS      		"12345678"
//#define AP_AUTH				WIFI_AUTH_WPA_WPA2_PSK
//#define MAX_SRV_CLIENTS		2	// maksymalna liczba klientów serwera TCP
//#define AP_MAX_CONN		2	//Tutaj okre�lamy maksymaln� po��cz�� innych urz�dze� z nasz� sieci� WiFi
//								//Gdy modu� ESP dzia�a w trybie AP
//
//static const char* TAG = "wifi softAP";
//
//
///* FreeRTOS event group to signal when we are connected*/
////static EventGroupHandle_t wifi_event_group;
//const int WIFI_CONNECTED_BIT = BIT0;
//
///* IP address settings */
//#define MY_IP_ADDR 		"10.0.4.1"
//#define MY_NETMASK 		"255.255.255.0"
//#define MY_GATEWAY 		"10.0.4.1"
//
//
///***** Struktura informacji n/t klienta serwera TCP *****/
//typedef struct {
//	int				active;		//czy dany klient jest aktywny czy nie
//	int				idx;		//index - numer klienta
//	struct in_addr  sin_addr;	//adres IP klienta w postaci cyfrowej nie stringowej
//	char 			ip[16];		//numer IP klienta w postaci stringa
//	uint16_t		port;		//numer portu klienta, z którego przyszło zapytanie
//	TaskHandle_t	handle;		//uchwyt do taska, który zostanie powołany do obsługi danego klienta
//	int				sock;		//gniazdo aktualnego klienta
//} TSRVCLN;
//
///***** Lista klientów serwera TCP ******/
//static TSRVCLN cln_handles[ AP_MAX_CONN ];	//utworzenie tablicy, która przechowywać będzie dane na podstawie struktury o klientach TCP
//static int cln_count;	//Zmienna do przechowywani ilości aktywnych klientów z listy
//
///****** Socket (gniazdo) nasłuchowe serwera TCP *******/
//static int listen_sock;
//
///***** Uchwyty do najważniejszych tasków *****/
//TaskHandle_t tServerHandle, srvclnsendhandle;
//
///***** KOLEJKA DO wysyłania danych do klientów serwera tcp *****/
//QueueHandle_t xQueueClientTCP;
//
////struktura do przesyłu danych pomiędzy serwerem i klientem TCP
//typedef struct {
//	int	clsock;
//	char data[100];
//} TQUEUE_TCP_DATA;
//
//// nagłówek taska do wysyłania danych do klientów serwera
//void tcp_srv_cln_send_task( void * arg );
//
///*  SMP (Symmetric Multiprocessing) */
//static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
//#define PORT_ENTER_CRITICAL 	portENTER_CRITICAL(&mux)
//#define PORT_EXIT_CRITICAL 		portEXIT_CRITICAL(&mux)
//
///* ------------------------------------------------------------------------------- */
///* -------- Funkcje pomocnicze do obsługi SERWERA TCP ---------------------------- */
///* ------------------------------------------------------------------------------- */
//
///*
// * Funkcja sprawdzający czy jest wolne miejsce na liście dla nowego klienta TCP i jeśli jest zainicowanie element listy dla niego,
// * Jeśli nie ma miejsca na liście to zwróć NULL
// * Gdy pojawi się pierwszy klient uruchom dynamicznie task umożliwiający nadawanie danych
// * do podłączonych klientów serwera TCP
// *
// */
//TSRVCLN * get_free_cln_slot( void ) {
//
//	int i;
//	TSRVCLN * cl = NULL;	//powołanie wskaźnika na podstawie struktury i jego wyzerowanie
//
//	PORT_ENTER_CRITICAL;	//Zablokowanie innych tasków w systemie
//	//W pętli sprawdzamy, które pole w tablicy klientów TCP jest aktualnie wolne
//	for( i=0; i<MAX_SRV_CLIENTS; i++ ) {
//		if( !cln_handles[i].active ) {	//czy pole aktive w tablicy jest równe 0
//			cl = &cln_handles[i];		//wskazujemy, który element tablicy jest równy 0
//			cl->active = 1;
//			cl->idx = i;
//			memset( cl->ip, 0, sizeof(cl->ip) );	//zerowanie starych danych z tabeli z adresem IP klienta TCP w postaci stringa przechowywanych
//			cln_count++;	//zwiększ liczbę aktualnie podłaczonych klientów TCP
//			break;
//		}
//	}
//	PORT_EXIT_CRITICAL;
//
//	if( cln_count == 1 ) {	//gdy podłączy sie pierwszy i tylko pierwszy klient TCP, wówczas wyświetl dane na terminal i uruchom taska od obsługi klientów TCP
//		ESP_LOGW(TAG, "SRV Client SEND TASK Create..." );
//		xTaskCreate( tcp_srv_cln_send_task, "", 2048, NULL, 2, &srvclnsendhandle  );
//	}
//
//	return cl;	//zwróć wskaźnik na wybrany element tablicy
//}
//
///*
// *  Podaj liczbę podłączonych klientów do serwera TCP
// */
//int get_cln_count( void  ) {
//
//	int res;
//
//	PORT_ENTER_CRITICAL;
//	res = cln_count;
//	PORT_EXIT_CRITICAL;
//
//	return res;
//}
//
///* USUŃ element informacjyjny o kliencie serwera TCP jeśli połączenie zostało zakończone */
//void clear_cln_slot( TSRVCLN * cl ) {
//
//    if( cl->sock != -1) {
//        ESP_LOGE(TAG, "Shutting down socket idx: %d ...", cl->idx );
//        shutdown(cl->sock, 0);
//        close(cl->sock);
//    }
//
//    PORT_ENTER_CRITICAL;
//	cl->active = 0;
//	cl->ip[0] = 0;
//	cl->port = 0;
//	cl->idx = -1;
//	cl->handle = NULL;
//	cln_count--;
//	PORT_EXIT_CRITICAL;
//
//	if( !get_cln_count() && srvclnsendhandle != NULL ) {
//		vTaskDelete( srvclnsendhandle );
//		srvclnsendhandle = NULL;
//		ESP_LOGW(TAG, "SRV Client SEND TASK DELETE..." );
//	}
//    vTaskDelete( NULL );
//}
//
///*
// * Zakończ wszystkie połączenia klientów serwera TCP, zwolnij pamięć,
// * zakończ task nadawczy do klientów jeśli wszyscy zostali odłączeni
// */
//void close_all_tcp_server_clients( void ) {
//
//    TSRVCLN * cl;
//    PORT_ENTER_CRITICAL;
//    for( int i=0; i<MAX_SRV_CLIENTS; i++ ) {
//
//    	cl = &cln_handles[i];
//    	if( cl->active ) {
//    	    if( cl->sock != -1) {
//    	        ESP_LOGE(TAG, "Shutting down srv_cln_sock..." );
//    	        shutdown(cl->sock, 0);
//    	        close(cl->sock);
//    	        cl->sock = -1;
//    	    }
//    	}
//    }
//    PORT_EXIT_CRITICAL;
//}
//
///* ``````````````````````````````````````````````````````````````````````````````` */
///* ```````` TASK SERWERA do wysyłania informacj do dowolnego klienta ````````````` */
///* ``````````````````````````````````````````````````````````````````````````````` */
//void tcp_srv_cln_send_task( void * arg ) {
//
//	BaseType_t xStatus;
//	TQUEUE_TCP_DATA dane;
//
//	while(1) {
//
//		int sock = 0;
//		memset( dane.data, 0, sizeof(dane.data) );
//		xStatus = xQueueReceive( xQueueClientTCP, &dane, portMAX_DELAY );
//
//		sock = dane.clsock;
//
//		if( xStatus == pdPASS && sock > 0  ) {
//
//			int err = send(sock, dane.data, strlen( dane.data ), 0 );
//
//			if (err < 0) {
//				ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
//			}
//		}
//	}
//}
//
///* ``````````````````````````````````````````````````````````````````````````````` */
///* ```````` TASKI Klientów SERWERA TCP - tylko odbierają dane ```````````````````` */
///* ``````````````````````````````````````````````````````````````````````````````` */
//void tcp_srv_cln_task( void * arg ) {
//
//	TSRVCLN * me = (TSRVCLN*)arg;
//
//	int idx = me->idx;
//
//	//Wyświetl dane o podłączonym kliencie TCP na terminalu
//	ESP_LOGW(TAG, "******** TCP SERVER-CLIENT START **************" );
//	ESP_LOGW(TAG, "Client idx: %d", me->idx );					//numer id w tabeli jaką została mu nadana
//	ESP_LOGW(TAG, "Client IP/Port: %s:%d", me->ip, me->port );	//adres IP i numer portu z jakiego podłączył się klient
//	ESP_LOGW(TAG, "Client sock: %d", me->sock );				//numer gniazda nadany przez serwer TCP
//	ESP_LOGW(TAG, "Client Task Handle: %d", (int)me->handle );	//numer uchwytu dany przez serwer tCP
//
//	uint16_t dat = uxTaskGetStackHighWaterMark(NULL);	//odczytal ilość wolnej pamięci w tasku
//	ESP_LOGW(TAG, "Client free stack mem: %d", dat );	//wyświetl ilość wolnej pamięci na terminalu
//
//	int len = 0;
//	char rx_buffer[128];
//
//	while(1) {
//
//		len = recv(me->sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
//
//        if( len < 0 ) {
//            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
//            break;
//        } else if( len == 0 || errno == 128 ) {
//            ESP_LOGW(TAG, "Connection closed");
//            break;
//        } else {
//			rx_buffer[len] = 0;
//			printf( ">>> " );
//			printf( rx_buffer );
//			printf( "\r\n" );
//
//			// parsuj dane od klienta
//
////			dane.clsock = cl->sock;
////			sprintf( dane.data, "Licznik %d [Client ID: %d]\n", licznik++, cl->idx );
////			xQueueSend( xQueueClientTCP, &dane, 0 );
//        }
//	}
//
//	ESP_LOGW(TAG, "******** TCP SERVER-CLIENT idx: %d CLOSE **************", idx );
//	vTaskDelay( 1 );
//
//	clear_cln_slot( me );
//
////	vTaskDelete( NULL );
//}
//
//
//
//
///* ``````````````````````````````````````````````````````````````````````````````` */
///* ```````` GŁÓWNY TASK SERWERA TCP `````````````````````````````````````````````` */
///* ``````````````````````````````````````````````````````````````````````````````` */
//void tcp_server_task( void * arg ) {
//
//    char addr_str[128];
//
//    vTaskDelay( 100 );
//
//    while(1) {
//
//		struct sockaddr_in localAddr;	//Powołanie strukturki do utworzenia gniazda
//		localAddr.sin_addr.s_addr = htonl(INADDR_ANY);	//Wiązanie adresu IP z numerem portu na serwerze
//		localAddr.sin_family = AF_INET;
//		localAddr.sin_port = htons( TCP_SERVER_PORT );
//		inet_ntoa_r(localAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
//
//		listen_sock = socket( AF_INET, SOCK_STREAM, IPPROTO_IP );	//Tworzenie soceta
//		if (listen_sock < 0) {	//Gdy nie udało się utworzyc gniazda
//			ESP_LOGE( TAG, "Unable to create socket: errno %d", errno );
//			break;	//Powrót z pętli nieskończonej
//		}
//		ESP_LOGI(TAG, "TCP SERVER Socket created");		//Gdy socket został stworzony to wyślij te informację na terminal
//
//		int on= 1 ;
//		if( (setsockopt(listen_sock,SOL_SOCKET,SO_REUSEADDR,&on, sizeof (on))) < 0 ) {
//			ESP_LOGE(TAG, " błąd ponownego użycia adresu tcp gniazda errno = %d" ,errno);
//		}	//Ustawiamy opcje za pomoca komend API dla naszego soceta
//
//		int err = bind(listen_sock, (struct sockaddr *)&localAddr, sizeof(localAddr));	//wiązanie soceta z portem nasłuchowym dla serwera
//		if (err != 0) {	//sprawdzenie czy wystąił bląd
//			ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
//			break;
//		}
//		ESP_LOGI(TAG, "Socket binded");	//Jeżeli udało się wiązanie to wyślij komunikat na terminal
//
//		err = listen( listen_sock, 1 );	//nasz socet ma móc wystartował z nasłuchem połączeń TCP na wskazanym gnieźdźie
//		if (err != 0) {		//gdy nie można nasłuchiwać to za pomoca rozkazu continue wróc do początku pętli i jeszcze raz spróbuj wszystko ustawić
//			ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
//			vTaskDelay( 100 );
//			continue;
//		}
//		ESP_LOGI(TAG, "Socket listening");	//Wyświetl info, że właczone zostało nasłuchiwanie na gnieździe
//
//		//Początek pętli do akceptowania przychodzących połaczeń od klientów TCP
//		while(1) {
//
//			ESP_LOGI(TAG, "Listen Socket - ACCEPT START...");
//
//			struct sockaddr_in sourceAddr;		//Przygotowanie strukturki na adres IP i port klienta, który się podłączy do serwera
//			socklen_t socklen = sizeof(sourceAddr);
//
//			//Polecenie API accept -blokuje pracę w tym tasku, aż zostanie wykryty jakiś klient
//			int cln_sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &socklen );	//Jak się podłaczy klient do jego dane zostają zapisane do zmiennej cln_sock
//
//			ESP_LOGI(TAG, "Otrzymałem dane");
//
//			if( cln_sock < 0 ) {	//Jeżlei nie udało się poprawnie uzyskać gniazda klienta wówczas przerwij te pętlę nieskończoną
//				ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
//				break;
//			}
//			ESP_LOGI(TAG, "Socket accepted");	//info gdy udało sie poprawnie odczytać gniazdo klienta
//
//			//DO tego miejsca było odczytanie danych klienta, który chce sie podłączyć do serwera TCP
//
//			//Powołanie wskaźnika na funkcję oraz od razu za pomoca własnej funkcji sprawdzenie czy jest wolne miejsce na nowego klienta TCP,
//			//gdy takie miejsce jest dostępne w nasze tablicy klientów, zwraca wskaźnik na pierwsze wolne miejsce na liście i wstępnie je inicjalizuje
//			//pole active i idx są już wypełnione
//			TSRVCLN * cl = get_free_cln_slot();
//
//			if( cl != NULL ) {	//Gdy zwrocony wskaźnik na tablicę jest rózny od zera (czyli jest miejsce w niej na nowego klienta)
//
//				//uzupełniamy wszystie pola w tabeli nowym klientem TCP
//				cl->sock = cln_sock;
//				cl->sin_addr = sourceAddr.sin_addr;
//				cl->port = sourceAddr.sin_port;
//				//Zamiana adreu IP nowego klienta na postać stringową i zapisanie tych danych w polu ip o długości równej temu polu (w razie czego na końcu dopisz 0)
//				inet_ntoa_r(sourceAddr.sin_addr, cl->ip, sizeof(cl->ip) - 1);
//
//				//powołanie taska dla danego klienta
//				xTaskCreate( tcp_srv_cln_task, "", 4096, (TSRVCLN*)cl, 1, &cl->handle );
//
//			} else {	//W przypadku, gdy zwracane jest 0 w zmiennej cl wówczas nie można dodać nowego klienta i należy wysłać na termina odpowiednie info
//				char reject_info[128];
//				sprintf( reject_info,"Brak miejsca na serwerze\n" );	//Do bufora wrzucamy napis
//				send( cln_sock, reject_info, strlen(reject_info), 0 );	//wysłanie informacji o braku miejsca do klienta
//				vTaskDelay( 10 );
//			    if( cln_sock != -1) {		//sprawdzamy, czy nadal socet od klienta jest działający
//			        ESP_LOGE(TAG, "Socket closed gracefully..." );	//komunikat na terminal
//			        shutdown(cln_sock, 0);	//Wyłaczamy gniazdo od klienta, którego nie chcemy obsłużyć
//			        close(cln_sock);		//Zamykamy istniejące gniazdo od klienta
//			    }
//				ESP_LOGE(TAG, "BRAK MIEJSCA DLA KLIENTÓW SERVERA TCP");	//info na terminal
//			}
//		}	//powrót do pętli nieskończonej (oczekiwanie na nowego klienta)
//
//	    //Gdy na stąpił break w pętli od akceptacji nowych klientów TCP wówczas zamknij nasłuchiwanie i powrót do początku taska do uruchomienia nasłuchiwania na serwerze TCP
//		if( listen_sock != -1) {
//	        ESP_LOGE(TAG, "Shutting down listen_sock..." );
//	        shutdown(listen_sock, 0);
//	        close(listen_sock);
//	    }
//	    vTaskDelay( 100 );
//    }	//Powrót do początku taska do ustanowienia nasłuchiwania na serwerze
//
//	vTaskDelete(NULL);	//wyłączenie taska, bo nie udało się stworzyć gniazda (wykonywany break został)
//}
//
//
//
//esp_netif_t * netif_ap;
//
//
//#define MAX_APs 10
//
////static EventGroupHandle_t s_wifi_event_group;
//
//
//void wifi_scan_task(void *pvParameters) {
//    uint16_t ap_count = 0;
//    wifi_ap_record_t ap_records[MAX_APs];
//
//    while (1) {
//        esp_wifi_scan_start(NULL, true); // Rozpocznij skanowanie sieci WiFi
//
//        // Poczekaj, aż skanowanie zostanie zakończone
//        uint32_t scan_time = 0;
//        while (scan_time < 5000) {
//            vTaskDelay(100 / portTICK_PERIOD_MS);
//            scan_time += 100;
//        }
//
//        // Pobierz informacje o dostępnych sieciach WiFi
//        esp_wifi_scan_get_ap_num(&ap_count);
//        esp_wifi_scan_get_ap_records(&ap_count, ap_records);
//
//        printf("Znaleziono %d sieci WiFi:\n", ap_count);
//        for (int i = 0; i < ap_count; i++) {
//            printf("SSID: %s, RSSI: %d dBm\n", ap_records[i].ssid, ap_records[i].rssi);
//        }
//
//    	esp_netif_ip_info_t ip_info;
////    	esp_netif_get_ip_info(mk_netif_sta, &ip_info);
////    	printf( "[STA] IP: " IPSTR "\n", IP2STR(&ip_info.ip) );
////    	printf( "[STA] MASK: " IPSTR "\n", IP2STR(&ip_info.netmask) );
////    	printf( "[STA] GW: " IPSTR "\n", IP2STR(&ip_info.gw) );
//
//    	esp_netif_get_ip_info(mk_netif_ap, &ip_info);
//    	printf( "[AP] IP: " IPSTR "\n", IP2STR(&ip_info.ip) );
//    	printf( "[AP] MASK: " IPSTR "\n", IP2STR(&ip_info.netmask) );
//    	printf( "[AP] GW: " IPSTR "\n", IP2STR(&ip_info.gw) );
//
//        vTaskDelay(500 / portTICK_PERIOD_MS); // Czekaj 10 sekund przed kolejnym skanowaniem
//    }
//}
//
//// Event handler for WIFI events
//static esp_err_t event_handler(void *ctx, system_event_t *event)
//{
//    switch(event->event_id) {
//
//        case SYSTEM_EVENT_AP_STACONNECTED:
//            ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
//                     MAC2STR(event->event_info.sta_connected.mac),
//                     event->event_info.sta_connected.aid);
//            break;
//
//        case SYSTEM_EVENT_AP_STADISCONNECTED:
//            ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
//                     MAC2STR(event->event_info.sta_disconnected.mac),
//                     event->event_info.sta_disconnected.aid);
//            break;
//
//        default:
//            break;
//    }
//    return ESP_OK;
//}
//
//void wifi_scan() {
//    wifi_scan_config_t scan_config = {
//        .ssid = 0,
//        .bssid = 0,
//        .channel = 0,
//        .show_hidden = true
//    };
//    esp_wifi_scan_start(&scan_config, true);
//    uint16_t ap_num = 0;
//    wifi_ap_record_t ap_records[MAX_APs];
//    esp_wifi_scan_get_ap_records(&ap_num, ap_records);
//    ESP_LOGI(TAG, "Found %d access points:", ap_num);
//    for (int i = 0; i < ap_num; i++) {
//        ESP_LOGI(TAG, "SSID: %s, RSSI: %d", ap_records[i].ssid, ap_records[i].rssi);
//    }
//}
//
//void wifi_init_ap() {
//    // Initialize the TCP/IP stack
//    tcpip_adapter_init();
//
//    // Initialize the WIFI driver
//    ESP_ERROR_CHECK(esp_event_loop_create_default());
//    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//
//    esp_wifi_set_mode( WIFI_MODE_AP );
//
//    	// Tworzenie interfejsu sieciowego dla trybu AP
////    	esp_netif_t * netif_ap;
//    	netif_ap = esp_netif_create_default_wifi_ap();
//
////    	 Konfiguracja punktu dostępowego (AP) z adresem IP
//
//
//
////    	esp_netif_ip_info_t ap_ip_info;
////    	IP4_ADDR(&ap_ip_info.ip, 192, 168, 1, 1);
////    	IP4_ADDR(&ap_ip_info.gw, 192, 168, 1, 1);
////    	IP4_ADDR(&ap_ip_info.netmask, 255, 255, 255, 0);
////    	esp_netif_set_ip_info(ESP_IF_WIFI_AP, &ap_ip_info);
//    //
//    //	esp_netif_dhcps_stop(netif_ap);
//    //
//    //	esp_netif_ip_info_t ap_ip_info;
//    //	ap_ip_info.ip.addr = ipaddr_addr( AP_IP );
//    //	ap_ip_info.netmask.addr = ipaddr_addr( AP_MASK );
//    //	ap_ip_info.gw.addr = ipaddr_addr( AP_GW );
//    //	esp_netif_set_ip_info(netif_ap, &ap_ip_info);
//    //
//    //	esp_netif_dhcps_start(netif_ap);
//
//    // Configure the WIFI driver to operate in AP mode
//    wifi_config_t wifi_config = {
//    		.ap = {
//    			.ssid = AP_SSID,
//    			.ssid_len = strlen(AP_SSID),
//    			.channel = 1,
//    			.password = AP_PASS,
//    			.max_connection = 4,
//    			.authmode = AP_AUTH,
////    			.beacon_interval = 100
//    		}
//    	};
//
////    esp_wifi_set_mode(WIFI_MODE_AP);
//	esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
//
//	esp_netif_dhcps_stop(netif_ap);
//	esp_netif_ip_info_t ap_ip_info;
//		ap_ip_info.ip.addr = ipaddr_addr( AP_IP );
//		ap_ip_info.netmask.addr = ipaddr_addr( AP_MASK );
//		ap_ip_info.gw.addr = ipaddr_addr( AP_GW );
////		esp_netif_set_ip4_addr(&ap_ip_info, 10, 0, 0, 1);
//        	esp_netif_set_ip_info(netif_ap, &ap_ip_info);
////		esp_netif_set_ip_info(ESP_IF_WIFI_AP, &ap_ip_info);
//        esp_netif_dhcps_start(netif_ap);
//
//	esp_wifi_start();
//	esp_wifi_connect();
//}
//
//
////Kod funkcji realizowanej z chwilą połączenia się nowego urządzenia do modułu ESP32 działającego w trybie AP
//void mk_ap_join_cb( char * mac ) {
//
//	printf("\n************ AP JOIN \n");
//}
//
////Kod funkcji realizowanej z chwilą rozłączenia się podłączonego z ESP urządzenia, gdy moduł ESP32 działa w trybie AP
//void mk_ap_leave_cb( char * mac ) {
//
//	printf("\n************ AP LEAVE \n");
//}
//
//
//
///*   ******************  *
// *   POCZąTEK PROGRAMU   *
// *   ******************  */
//void app_main(void) {
//
//	vTaskDelay( 100 );	// tylko żeby łatwiej przełączać się na terminal przy starcie
//	printf("\nREADY\n");
//
//	/*........ konfiguracja pinów dla testowej diody LED ...........*/
//	gpio_set_direction( LED1_GPIO, GPIO_MODE_OUTPUT );
//	gpio_set_level( LED1_GPIO, 1 );
//
////	/* ------  Inicjalizacja kolejki na potrzeby przesyłu danych po UDP  ------ */
////	xQueueClientUDP = xQueueCreate( 5, sizeof( TQUEUE_UDP_CLI_DATA ) );
//
//	/* ------  Inicjalizacja kolejki na potrzeby przesyłu danych po TCP  ------ */
////	xQueueClientTCP = xQueueCreate( 5, sizeof( TQUEUE_TCP_DATA ) );
//
//	/*  ------   Inicjalizacja NVS   ----- */
//	nvs_flash_init();
//
//	wifi_init_ap();
//
//	/* ------ Wybór oraz inicjalizacja trybu działania modułu ESP32 w sieci WiFi ------ */
////	mk_wifi_init( WIFI_MODE_APSTA, mk_got_ip_cb, mk_sta_disconnected_cb, mk_ap_join_cb, mk_ap_leave_cb  );
//	mk_wifi_init( WIFI_MODE_AP, NULL, NULL, mk_ap_join_cb, mk_ap_leave_cb );
////	mk_wifi_init( WIFI_MODE_STA, mk_got_ip_cb, mk_sta_disconnected_cb, NULL, NULL );
//
////    display_info();		//Funkcja wyświetlająca ilość wolnej pamięci w module ESP32
////    mk_sntp_init(NULL); // inicjalizacja SNTP ze sterefą czasową dla Polski
//
////    esp_netif_ip_info_t ip_info;
////
////    esp_netif_get_ip_info(mk_netif_ap, &ip_info);
////	printf( "[AP] IP: " IPSTR "\n", IP2STR(&ip_info.ip) );
////	printf( "[AP] MASK: " IPSTR "\n", IP2STR(&ip_info.netmask) );
////	printf( "[AP] GW: " IPSTR "\n", IP2STR(&ip_info.gw) );
//
//
//// Inicjalizacja WiFi
////	tcpip_adapter_init();
//	esp_netif_t * netif_ap;
//
//
//
//
////#include <stdio.h>
////#include "freertos/FreeRTOS.h"
////#include "freertos/task.h"
////#include "esp_wifi.h"
////#include "esp_event_loop.h"
////#include "esp_log.h"
////
////#define SSID "MY_AP_SSID"
////#define PASSWORD "MY_AP_PASSWORD"
////#define MAX_AP_NUM 20
////
////static const char *TAG = "wifi_scan";
//
////// Event handler for WIFI events
////static esp_err_t event_handler(void *ctx, system_event_t *event)
////{
////    switch(event->event_id) {
////
////        case SYSTEM_EVENT_AP_STACONNECTED:
////            ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
////                     MAC2STR(event->event_info.sta_connected.mac),
////                     event->event_info.sta_connected.aid);
////            break;
////
////        case SYSTEM_EVENT_AP_STADISCONNECTED:
////            ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
////                     MAC2STR(event->event_info.sta_disconnected.mac),
////                     event->event_info.sta_disconnected.aid);
////            break;
////
////        default:
////            break;
////    }
////    return ESP_OK;
////}
////
////void wifi_scan() {
////    wifi_scan_config_t scan_config = {
////        .ssid = 0,
////        .bssid = 0,
////        .channel = 0,
////        .show_hidden = true
////    };
////    esp_wifi_scan_start(&scan_config, true);
////    uint16_t ap_num = 0;
////    wifi_ap_record_t ap_records[MAX_AP_NUM];
////    esp_wifi_scan_get_ap_records(&ap_num, ap_records);
////    ESP_LOGI(TAG, "Found %d access points:", ap_num);
////    for (int i = 0; i < ap_num; i++) {
////        ESP_LOGI(TAG, "SSID: %s, RSSI: %d", ap_records[i].ssid, ap_records[i].rssi);
////    }
////}
////
////void wifi_init_ap() {
////    // Initialize the TCP/IP stack
////    tcpip_adapter_init();
////
////    // Initialize the WIFI driver
////    ESP_ERROR_CHECK(esp_event_loop_create_default());
////    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
////    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
////
////    // Configure the WIFI driver to operate in AP mode
////    wifi_config_t wifi_config = {
////    		.ap = {
////    			.ssid = AP_SSID,
////    			.ssid_len = strlen(AP_SSID),
////    			.channel = 1,
////    			.password = AP_PASS,
////    			.max_connection = 4,
////    			.authmode = AP_AUTH,
////    			.beacon_interval = 100
////    		}
////    	};
////
////    esp_wifi_set_mode(WIFI_MODE_AP);
////	esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
////
////	esp_wifi_start();
////	esp_wifi_connect();
////}
//


//
//
//
//
//
//
//
//
//
////	//	//Funkcja ustawienia pami�ci do obs�ugi WiFi w module
////	esp_wifi_set_storage( WIFI_STORAGE_RAM );		// (2)
////
////	esp_netif_init();
////
////	// Tworzenie interfejsu sieciowego dla trybu AP
////	netif_ap = esp_netif_create_default_wifi_ap();
////
////	// Konfiguracja punktu dostępowego (AP) z adresem IP
//////	esp_netif_ip_info_t ap_ip_info;
//////	IP4_ADDR(&ap_ip_info.ip, 192, 168, 1, 1);
//////	IP4_ADDR(&ap_ip_info.gw, 192, 168, 1, 1);
//////	IP4_ADDR(&ap_ip_info.netmask, 255, 255, 255, 0);
//////	esp_netif_set_ip_info(ESP_IF_WIFI_AP, &ap_ip_info);
////
////	esp_netif_dhcps_stop(netif_ap);
////
////	esp_netif_ip_info_t ap_ip_info;
////	ap_ip_info.ip.addr = ipaddr_addr( AP_IP );
////	ap_ip_info.netmask.addr = ipaddr_addr( AP_MASK );
////	ap_ip_info.gw.addr = ipaddr_addr( AP_GW );
////	esp_netif_set_ip_info(netif_ap, &ap_ip_info);
////
////	esp_netif_dhcps_start(netif_ap);
////
//////	 Konfiguracja SSID (nazwa sieci) i hasła punktu dostępowego (AP)
////	wifi_config_t wifi_config = {
////		.ap = {
////			.ssid = AP_SSID,
////			.ssid_len = strlen(AP_SSID),
////			.channel = 1,
////			.password = AP_PASS,
////			.max_connection = 4,
////			.authmode = AP_AUTH,
////			.beacon_interval = 100
////		}
////	};
////	esp_wifi_set_mode(WIFI_MODE_AP);
////	esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
////
////	esp_wifi_start();
////	esp_wifi_connect();
//
//
//
//
//
//
//// Inicjalizacja biblioteki TCP/IP
////	    esp_netif_init();
////
////	    // Tworzenie interfejsu sieciowego dla trybu AP
////	    esp_netif_create_default_wifi_ap();
////
////	    // Konfiguracja punktu dostępowego (AP) z adresem IP
////	    esp_netif_ip_info_t ap_ip_info;
////	    IP4_ADDR(&ap_ip_info.ip, 192, 168, 1, 1);
////	    IP4_ADDR(&ap_ip_info.gw, 192, 168, 1, 1);
////	    IP4_ADDR(&ap_ip_info.netmask, 255, 255, 255, 0);
////	    esp_netif_set_ip_info(ESP_IF_WIFI_AP, &ap_ip_info);
////
////	    // Konfiguracja SSID (nazwa sieci) i hasła punktu dostępowego (AP)
////	    wifi_config_t wifi_config = {
////	        .ap = {
////	            .ssid = "Nazwa_SSID",
////	            .ssid_len = 0,
////	            .password = "Haslo_SSID",
////	            .max_connection = 4,
////	            .authmode = WIFI_AUTH_WPA_WPA2_PSK
////	        }
////	    };
////	    esp_wifi_set_mode(WIFI_MODE_AP);
////	    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
////
////	    esp_wifi_start();
//
//
//
//
////	esp_netif_t * netif_ap;
//////	esp_netif_t * netif_sta;
////
////	s_wifi_event_group = xEventGroupCreate();		// (1)
////
////
////	//Funkcja ustawienia pami�ci do obs�ugi WiFi w module
////	esp_wifi_set_storage( WIFI_STORAGE_RAM );		// (2)
////
////	// Inicjalizacja biblioteki TCP/IP
////	esp_netif_init();								//(3)
////
////   //Tworzenie tzw. wewn�trzego taska obs�uguj�cego WiFi
////	esp_event_loop_create_default();				// (4)
////
////	 wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
////	//Zatwierdzamy t� struktur�
////	esp_wifi_init(&cfg);							// (5)
////
////	 //Ustawiamy tryb pracy modu�u WiFi
////	esp_wifi_set_mode( WIFI_MODE_AP );					// (6)
////
////	esp_event_handler_register( WIFI_EVENT,			// (7)
////	                                ESP_EVENT_ANY_ID,
////									&wifi_event_handler,
////	                                NULL
////	                               );
////
////
////	// Tworzenie interfejsu sieciowego dla trybu AP
////	netif_ap = esp_netif_create_default_wifi_ap();
////
////	printf("\nTEST\n");
////
////	 wifi_config_t wifi_config = {
////		.ap = {
////			.ssid = AP_SSID,
////			.ssid_len = strlen(AP_SSID),
////			.password = AP_PASS,
////			.max_connection = AP_MAX_STA_CONN,
////			.authmode = AP_AUTH
////		},
////	};
////
////	 esp_wifi_set_config( ESP_IF_WIFI_AP, &wifi_config );	// (9 AP)
////
////	 esp_netif_dhcps_stop(netif_ap);
////
////	esp_netif_ip_info_t ip_info;
////	ip_info.ip.addr = ipaddr_addr( AP_IP );
////	ip_info.netmask.addr = ipaddr_addr( AP_MASK );
////	ip_info.gw.addr = ipaddr_addr( AP_GW );
////	esp_netif_set_ip_info(netif_ap, &ip_info);
////
////	esp_netif_dhcps_start(netif_ap);
////
////	esp_wifi_start();
////
////	esp_wifi_connect();
//
////	// Konfiguracja punktu dostępowego (AP) z adresem IP
////	esp_netif_ip_info_t ap_ip_info;
////	IP4_ADDR(&ap_ip_info.ip, 192, 168, 1, 1);
////	IP4_ADDR(&ap_ip_info.gw, 192, 168, 1, 1);
////	IP4_ADDR(&ap_ip_info.netmask, 255, 255, 255, 0);
////	esp_wifi_set_config(ESP_IF_WIFI_AP,  &ap_ip_info);
//////	esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_ip_info);
////
////	esp_netif_set_ip_info(ESP_IF_WIFI_AP, &ap_ip_info);
//
////	netif_ap = esp_netif_create_default_wifi_ap();	// (8 AP)
//
//
//
////	esp_netif_get_ip_info(netif_ap, &ap_ip_info);
////	printf( "[AP] IP: " IPSTR "\n", IP2STR(&ap_ip_info.ip) );
////	printf( "[AP] MASK: " IPSTR "\n", IP2STR(&ap_ip_info.netmask) );
////	printf( "[AP] GW: " IPSTR "\n", IP2STR(&ap_ip_info.gw) );
////
////
////
////	 // Konfiguracja SSID (nazwa sieci) i hasła punktu dostępowego (AP)
////	wifi_config_t wifi_config = {
////		.ap = {
////			.ssid = "SSID",
////			.ssid_len = 0,
////			.password = "Haslo_SSID",
////			.max_connection = 4,
////			.authmode = WIFI_AUTH_WPA_WPA2_PSK
////		}
////	};
////
////	esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
//////	esp_wifi_set_mode(WIFI_MODE_AP);
////
////	esp_wifi_start();
//
//
//
//
//
//
//
//
//
//
//
//
//
////	//Funkcja ustawienia pamięci do obsługi WiFi w module
////	esp_wifi_set_storage( WIFI_STORAGE_RAM );		// (2)
////
////	//umożliwia tworzenie i obsługę wątków zdarzeń w aplikacji, co ułatwia reakcję
////	//na sytuacje wywołujące zdarzenia w czasie rzeczywistym.
////	esp_event_loop_create_default();
////
////	//Funkcja ta jest wywoływana w celu skonfigurowania interfejsów sieciowych przed ich użyciem.
////	//ustawia domyślny zdarzeniowy model programowania sieciowego oraz przydzielanie adresów
////	//IPv4 i IPv6, i rejestruje interfejsy sieciowe, które można później użyć w operacjach sieciowych.
////	esp_netif_init();								// (3)
//
//
//
//
//
//
//
//
////	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
////	esp_wifi_init(&cfg);
////	esp_wifi_set_mode(WIFI_MODE_AP); // Ustaw tryb APSTA
////
////	netif_ap = esp_netif_create_default_wifi_ap();	// (8 AP)
////
////
////	// Konfiguracja punktu dostępowego (AP)
////	wifi_config_t ap_config = {
////		.ap = {
////			.ssid = AP_SSID,
////			.ssid_len = strlen(AP_SSID),
////			.password = AP_PASS,
//////			.ssid_len = 0,
//////			.channel = 0,
////			.authmode = WIFI_AUTH_WPA_WPA2_PSK,
////			.ssid_hidden = 0,
////			.max_connection = AP_MAX_STA_CONN,
////			.beacon_interval = 100
////		}
////	};
////	esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config);
////
////
//////	netif_sta = esp_netif_create_default_wifi_sta();	// (8 STA)
////
//////	//Konfiguracja DHCP w trybie AP
////	esp_netif_dhcps_stop(netif_ap);
//////	/* skonfiguruj statyczny IP */
////	esp_netif_ip_info_t ip_info;
////	ip_info.ip.addr = ipaddr_addr( AP_IP );
////	ip_info.netmask.addr = ipaddr_addr( AP_MASK );
////	ip_info.gw.addr = ipaddr_addr( AP_GW );
////	esp_netif_set_ip_info(netif_ap, &ip_info);
//////
////	/* włącz ponownie DHCP */
////	esp_netif_dhcps_start(mk_netif_ap);
////
////	// Uruchomienie WiFi
////	esp_wifi_start();
////
////	esp_wifi_connect();
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//	// Uruchomienie zadania skanowania sieci WiFi
//	xTaskCreate(&wifi_scan_task, "wifi_scan_task", 4096, NULL, 5, NULL);
//
////	netif_ap = esp_netif_create_default_wifi_ap();
//
//
////	/* ```````` Skanowanie dostępnych sieci  ``````````````````````````````````` */
////	mk_wifi_scan( NULL );	//Wyświetl dane na ekranie terminala
//
//	/* ---- zmienne lokalne głównego tasku ------------------------------------- */
////	uint8_t licznik = 0;
//	uint8_t sw = 0;
//
//  // ****** POCZąTEK WąTKU GłóWNEGO  *****
//	while (1) {
//
//		wifi_scan();
//
////		esp_netif_ip_info_t ip_info;
////		esp_netif_get_ip_info(netif_ap, &ip_info);
////		printf( "[AP] IP: " IPSTR "\n", IP2STR(&ip_info.ip) );
////		printf( "[AP] MASK: " IPSTR "\n", IP2STR(&ip_info.netmask) );
////		printf( "[AP] GW: " IPSTR "\n", IP2STR(&ip_info.gw) );
//
//
//    	gpio_set_level( LED1_GPIO, sw );
//        sw ^= 1;
//        vTaskDelay( 1000 );
//    }
//}
