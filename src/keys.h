/**
  ******************************************************************************
  * @file    keys.h
  * @author  chstke1
  * @version V1.0
  * @date    10.12.12
  * @brief   Routinen für die Entprellung und universelle 
			 Tastenabfrage mit bis zu 16 Tasten an einem Port
  ******************************************************************************/

#ifndef _KEYS_H_
#define _KEYS_H_


#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_gpio.h"
#include "eeprom.h"
#include "misc.h"


#define TASTER	12	// Port Pin Taster


#define KEY_INPUT	(GPIO_ReadInputData(GPIOA)) //Eingänge parallel einlesen
#define ALL_KEYS	(1<<TASTER) 	//Eingänge parallel einlesen
#define REPEAT_MASK (1<<TASTER)	// Maske der aktiven Taster
#define REPEAT_START 50		// * 10ms			// Zeit nachdem die Repeat Funktion anspringt
#define REPEAT_NEXT	2		// * 10ms			// Zeit zweichen einzelnen counts der Repeat Funktion

enum KEY { NONE, R_SHORT, R_LONG, R_DOUBLE, R_TRIPLE, L_SHORT, L_LONG};

/*
 * Name         : process_keys
 * Synopsis     : void process_keys(void) *
 * Description  : Entprellroutine muss alle 10ms durch Interrupt Routine aufgerugen werden
 */
void process_keys(void);

/*
 * Name         : get_key_press
 * Synopsis     : uint16_t get_key_press( uint16_t key_mask ) *
 * Arguments    : uint16_t  key_mask : Taster welche angefragt werden soll
 * Description  : Prüft ob eine Taste gedrückt wurde
 * Returns      : uint16_t Der Wert der übergegenen Taster wird zurückgegeben
 */
uint16_t get_key_press( uint16_t key_mask );
/*
 * Name         : get_key_rpt
 * Synopsis     : uint16_t get_key_rpt( uint16_t key_mask ) *
 * Arguments    : uint16_t  key_mask : Taster welche angefragt werden soll
 * Description  : Prüfen ob eine anhaltend gedrückt wird
 * Returns      : uint16_t Werte der anhaletnd gedrückten Tasten
 */
uint16_t get_key_rpt( uint16_t key_mask );
/*
 * Name         : get_key_short
 * Synopsis     : uint16_t get_key_short( uint16_t key_mask ) *
 * Arguments    : uint16_t  key_mask : Taster welche angefragt werden soll
 * Description  : Prüfen ob eine Taste kurz gedrückt wird
 * Returns      : uint16_t Werte der kurz gedrückten Tasten
 */
uint16_t get_key_short( uint16_t key_mask );

/*
 * Name         : get_key_long
 * Synopsis     : uint16_t get_key_long( uint16_t key_mask ) *
 * Arguments    : uint16_t  key_mask : Taster welche angefragt werden soll
 * Description  : Prüfen ob eine Taste lang gedrückt wird, ohne repeat Funktion
 * Returns      : uint16_t Werte der lang gedrückten Tasten
 */
uint16_t get_key_long( uint16_t key_mask );
#endif /* _KEYS_H_ */
