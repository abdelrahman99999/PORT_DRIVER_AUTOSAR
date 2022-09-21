 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Abdelrahman Elsayed
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H


/* Id for the company in the AUTOSAR */
#define PORT_VENDOR_ID    (1000U)

/* Port Module Id */
#define PORT_MODULE_ID    (124U)

/* Port Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)


/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"


/******************************************************************************
 *                      Special Values                                        *
 ******************************************************************************/
#define UNLOCKING_VALUE		     (0x4C4F434BU)

/******************************************************************************
 *                      Module definations                                    *
 ******************************************************************************/

#define	PORT_MODE_DIO	            (0U)
#define	PORT_MODE_ALTERNATIVE1    (1U)
#define PORT_MODE_ALTERNATIVE2    (2U)
#define PORT_MODE_ALTERNATIVE3	 	(3U)
#define	PORT_MODE_ALTERNATIVE4		(4U)
#define	PORT_MODE_ALTERNATIVE5		(5U)
#define	PORT_MODE_ALTERNATIVE6		(6U)
#define	PORT_MODE_ALTERNATIVE7		(7U)
#define	PORT_MODE_ALTERNATIVE8		(8U)
#define	PORT_MODE_ALTERNATIVE9		(9U)
#define	PORT_MODE_ALTERNATIVE10		(10U)
#define	PORT_MODE_ALTERNATIVE11		(11U)
#define	PORT_MODE_ALTERNATIVE12		(12U)
#define	PORT_MODE_ALTERNATIVE13		(13U)
#define	PORT_MODE_ALTERNATIVE14		(14U)
#define	PORT_MODE_ADC 		        (15U)

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for Port init */
#define PORT_INIT_SID                          (uint8)0x00

/* Service ID for Port Set Pin Direction */
#define PORT_SET_PIN_DIRECTION_SID             (uint8)0x01

/* Service ID for Refresh Port Direction */
#define PORT_REFRESH_PORT_DIRECTION_SID        (uint8)0x02

/* Service ID for Port GetVersionInfo */
#define PORT_GET_VERSION_INFO_SID              (uint8)0x03

/* Service ID for port set pin mode */
#define PORT_SET_PIN_MODE_SID                  (uint8)0x04




/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code used for Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN                          (uint8)0x0A

/* DET code used for Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE             (uint8)0x0B

/* DET code used for API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG                       (uint8)0x0C

/* DET code used for API Port_SetPinMode service called with mode is not valid.*/
#define PORT_E_PARAM_INVALID_MODE                 (uint8)0x0D

/* DET code used for API Port_SetPinMode service called with mode is unchangeable.*/
#define PORT_E_MODE_UNCHANGEABLE                  (uint8)0x0E

/* DET code used for APIs service called without module initialization */
#define PORT_E_UNINIT                             (uint8)0x0F

/*DET code used for APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER                      (uint8)0x10



/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

typedef enum
{
	PORT_PORTA_ID,         
	PORT_PORTB_ID,         
	PORT_PORTC_ID,        
	PORT_PORTD_ID,       
	PORT_PORTE_ID,      
	PORT_PORTF_ID         
}Port_PortID;

typedef enum
{
	PORT_PIN0_ID,        
	PORT_PIN1_ID,         
	PORT_PIN2_ID,          
	PORT_PIN3_ID,          
	PORT_PIN4_ID,          
	PORT_PIN5_ID,          
	PORT_PIN6_ID,          
	PORT_PIN7_ID          
}Port_PortPinID;

/* pin index from zero to no of configured pins*/
typedef uint8 Port_PinType;

/* Pin mode like DIO, ADC, SSI, UART ...*/
typedef uint8 Port_PinModeType;

/*Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN=0,
    PORT_PIN_OUT=1
}Port_PinDirectionType;

/* Enum to hold internal resistor type for PIN */
typedef enum
{
    PORT_RESISTOR_OFF=0,
    PORT_RESISTOR_PULL_UP=1,
    PORT_RESISTOR_PULL_DOWN=2
}Port_InternalResistorType;

/* Enum to indicate if pin direction is changeable or not  */
typedef enum
{
    PORT_PIN_DIRECTION_UNCHANGEABLE=0,
    PORT_PIN_DIRECTION_CHANGEABLE=1
}PORT_PinDirectionChangeabilityType;

/* Enum to indicate if pin mode is changeable or not  */
typedef enum
{
	PORT_PIN_MODE_UNCHANGEABLE=0 ,
	PORT_PIN_MODE_CHANGEABLE=1
}Port_PinModeChangeabilityType;

/* low/high for output pins */
typedef enum
{
	PORT_PIN_LEVEL_LOW =0,
	PORT_PIN_LEVEL_HIGH=1
}Port_PinLevelValueType;


typedef struct 
{
  /* port number (0-6) */
  Port_PortID port_num;		
  /* pin number (0-7) */									
  Port_PortPinID pin_num; 
  /* Pin mode like DIO, ADC, SSI, UART ...*/											
  Port_PinModeType pin_mode;
  /* Pin initial value (Low/High) for ouput pins */								
  Port_PinLevelValueType initial_value;
  /* Pin direction (input/output) */						
  Port_PinDirectionType direction;	
  /* (off, pull down, pull up) resistor for input pins*/						
  Port_InternalResistorType resistor;   					
  PORT_PinDirectionChangeabilityType changeableDirection;  
  Port_PinModeChangeabilityType changeableMode;
}Port_ConfigPinType;

/* Structure required for initializing the Port Driver */
typedef struct 
{
  Port_ConfigPinType pins[PORT_CONFIGURED_PINS];
}Port_ConfigType;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/* API to Initializes the Port Driver module */
void Port_Init(const Port_ConfigType* ConfigPtr);

/* API to Set the port pin direction*/
#if (PORT_SET_PIN_DIRECTION_API==STD_ON)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction);
#endif

/* API to Refreshe port direction */
void Port_RefreshPortDirection(void);

/* API to Return the version information of this module */
#if ( PORT_VERSION_INFO_API==STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo);
#endif

/* API to Set the port pin mode */
#if (PORT_SET_PIN_MODE_API==STD_ON)
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode);
#endif



/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_Configuration;




#endif /* PORT_H */
