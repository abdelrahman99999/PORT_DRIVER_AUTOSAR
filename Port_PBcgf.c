/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_PBcfg.c
 *
 * Description: Post Build Configuration Source file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Abdelrahman Elsayed
 ******************************************************************************/

#include "Port.h"

/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between Dio_PBcfg.c and Dio.h files */
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
		||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
		||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of PBcfg.c does not match the expected version"
#endif

/* Software Version checking between Dio_PBcfg.c and Dio.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
		||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
		||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
#error "The SW version of PBcfg.c does not match the expected version"
#endif

/* PB structure used with Dio_Init API */
const Port_ConfigType Port_Configuration = {

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  		/* port_num  |	  pin_num   |   pin_mode   |   initial_value   |  direction  |    resistor       |       changeableDirection      |   changeableMode */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/*pins in PORTF*/
		PORT_PORTF_ID, PORT_PIN0_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE, 
		PORT_PORTF_ID, PORT_PIN1_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_OUT , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE, 	/*led*/
		PORT_PORTF_ID, PORT_PIN2_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTF_ID, PORT_PIN3_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTF_ID, PORT_PIN4_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_PULL_UP, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE, /*switch*/
		/*pins in PORTA*/
		PORT_PORTA_ID, PORT_PIN0_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTA_ID, PORT_PIN1_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTA_ID, PORT_PIN2_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTA_ID, PORT_PIN3_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTA_ID, PORT_PIN4_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTA_ID, PORT_PIN5_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTA_ID, PORT_PIN6_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTA_ID, PORT_PIN7_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		/*pins in PORTB*/
		PORT_PORTB_ID, PORT_PIN0_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTB_ID, PORT_PIN1_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTB_ID, PORT_PIN2_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTB_ID, PORT_PIN3_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTB_ID, PORT_PIN4_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTB_ID, PORT_PIN5_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTB_ID, PORT_PIN6_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTB_ID, PORT_PIN7_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		/*pins in PORTC*/
		
		/*comment jtag pins for safety */
		/*PORT_PORTC_ID, PORT_PIN0_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTC_ID, PORT_PIN1_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTC_ID, PORT_PIN2_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTC_ID, PORT_PIN3_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,*/
		PORT_PORTC_ID, PORT_PIN4_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTC_ID, PORT_PIN5_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTC_ID, PORT_PIN6_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTC_ID, PORT_PIN7_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		/*pins in PORTD*/
		PORT_PORTD_ID, PORT_PIN0_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTD_ID, PORT_PIN1_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTD_ID, PORT_PIN2_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTD_ID, PORT_PIN3_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTD_ID, PORT_PIN4_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTD_ID, PORT_PIN5_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTD_ID, PORT_PIN6_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTD_ID, PORT_PIN7_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		/*pins in PORTE*/
		PORT_PORTE_ID, PORT_PIN0_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTE_ID, PORT_PIN1_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTE_ID, PORT_PIN2_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTE_ID, PORT_PIN3_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTE_ID, PORT_PIN4_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,
		PORT_PORTE_ID, PORT_PIN5_ID, PORT_MODE_DIO, PORT_PIN_LEVEL_LOW , PORT_PIN_IN , PORT_RESISTOR_OFF, PORT_PIN_DIRECTION_UNCHANGEABLE , PORT_PIN_MODE_UNCHANGEABLE,

};