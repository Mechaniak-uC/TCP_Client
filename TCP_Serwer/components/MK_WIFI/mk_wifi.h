/*
 * Plik nag��wkoy biblioteki do obs�ugi sieci WiFi
 *
 *  Created on: 5 kwi 2022
 *      Author: Miros�aw Karda�
 *
 *      ver: 1.01
 *
 *      - 25.04.2022 dodano obs�ug� synchronizacji czasu przez SNTP
 */

#ifndef COMPONENTS_MK_WIFI_MK_WIFI_H_
#define COMPONENTS_MK_WIFI_MK_WIFI_H_

#include "esp_wifi_types.h"

/*----------------- USTAWIENIA SNTP ----------------------------*/
#define USE_SNTP					1		// 0-do not use SNTP, 1-use SNTP

/*----------------- USTAWIENIA WiFi STA ----------------------------*/
#define DOM
//#define PRACA
//#define TELEFON


#define MAXIMUM_RETRY  			0		// 0-infinite, maksymalna ilo�� pr�b ��czenia si� STA do AP
#define USE_STA_STATIC_IP		0		// 0-IP form DHCP, 1-Static IP


/*:::::::: praca ::::::::*/
#ifdef PRACA
//#define STA_SSID      	"WiFi"
//#define STA_PASS      	"Zasilanie#13"
//#define STA_PASS      	"Zasilanie#13"
#endif

/*:::::::: dom ::::::::*/
#ifdef DOM
//#define STA_SSID      	NULL
//#define STA_PASS      	NULL
#define STA_SSID      	"admin"
#define STA_PASS      	"admin"

#endif

/*:::::::: telefon ::::::::*/
#ifdef TELEFON
#define STA_SSID      	"AndroidAP8039"
#define STA_PASS      	"55AA55AA55AA"
#endif



#if USE_STA_STATIC_IP == 1

/*:::::::: dom ::::::::*/
#ifdef DOM
#define STA_IP		"192.168.2.170"
#define STA_GW		"192.168.2.2"
#define STA_MASK	"255.255.255.0"
#endif

/*:::::::: praca ::::::::*/
#ifdef PRACA
#define STA_IP		"192.168.1.222"
#define STA_GW		"192.168.1.1"
#define STA_MASK	"255.255.255.0"
#define STA_DNS		"8.8.8.8"
#endif

/*:::::::: telefon ::::::::*/
#ifdef TELEFON
#define STA_IP		"10.0.0.2"
#define STA_GW		"10.0.0.1"
#define STA_MASK	"255.255.255.0"
#endif

#endif
/*----------------- USTAWIENIA STA KONIEC ---------------------*/


/*.................... USTAWIENIA AP ..............................*/
#define AP_SSID      		"ESP32_AP_TCP"
#define AP_PASS      		"12345678"
#define AP_AUTH				WIFI_AUTH_WPA_WPA2_PSK
#define AP_MAX_STA_CONN		1	//Tutaj określamy maksymalną liczbę połączeń z innymi urządzeniami z naszej
								//sieci WiFi gdy moduł ESP działa w trybie AP

#define USE_AP_USER_IP		1	// 0-Default IP for AP, 1-User IP for AP

#if USE_AP_USER_IP == 1

/*::::: Set USER IP for AP :::::*/
#define AP_IP		"10.0.1.1"					// domy�lne ip: 192.168.4.1
#define AP_GW		"10.0.1.1"					// domy�lna brama: 192.168.4.1
#define AP_MASK		"255.255.255.0"				// domy�lna maska: 255.255.255.0

#endif
/*.................... USTAWIENIA AP KONIEC.......................*/


/*  ------  sta�e na potrzeby obs�ugi czasu w RTOS  ------  */
#if USE_SNTP == 1
	#define DEFAULT_NTP_SERVER			"ntp.certum.pl"
#endif

#define CENTRAL_EUROPEAN_TIME_ZONE	"CET-1CEST,M3.5.0/2,M10.5.0/3"  // for Poland
/*  ------ sta�e na potrzeby obs�ugi czasu w RTOS - koniec  ------  */


extern esp_netif_t * mk_netif_sta;
extern esp_netif_t * mk_netif_ap;


//'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
typedef void (*TSTA_GOT_IP_CB)( char * ip );
typedef void (*TSTA_DISCONNECTED)( void );
typedef void (*TAP_JOIN_CB)( char * mac );
typedef void (*TAP_LEAVE_CB)( char * mac );


extern void wifi_init_sta(void);

/*'''''''''''''''' dost�pne funkcje ''''''''''''''''''''''''''''''*/
extern void mk_wifi_init( wifi_mode_t wifi_mode,
		TSTA_GOT_IP_CB got_ip_cb,
		TSTA_DISCONNECTED sta_discon_cb,
		TAP_JOIN_CB ap_join_cb,
		TAP_LEAVE_CB ap_leave_cb );


extern uint8_t get_sta_ip_state( void );
extern uint8_t bind_set_get_info( int8_t bnd );


/*
 *	Sposoby u�ycia:
 *
 *	1.) Wyszukanie wszystkich dost�pnych AP w otoczeniu WIFI - mk_wifi_scan( NULL );
 *	2.) Wyszukanie/sprawdzenie konkretnego AP czy dost�pny:
 *
 *	uint8_t myssid[] = "MirMUR";
 *	mk_wifi_scan( myssid );
 *
 *	albo
 *
 *	mk_wifi_scan( (uint8_t*)"MirMUR" );
 *
 */
extern uint8_t mk_wifi_scan( uint8_t * assid );



#if USE_SNTP == 1
	/* inicjalizacja SNTP ze steref� czasow� dla Polski, i synchronizacja czasu co godzin� */
	extern void mk_sntp_init( char * sntp_srv );
#endif




#endif /* COMPONENTS_MK_WIFI_MK_WIFI_H_ */
