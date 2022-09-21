 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Abdelrahman Elsayed
 * 
 ******************************************************************************/
#ifndef PORT_CGF_H_
#define PORT_CGF_H_

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)


/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_ON)

/* Pre-compile option for set pin direction API */
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)

/* Pre-compile option for set pin mode API */
#define PORT_SET_PIN_MODE_API                (STD_ON)

/* Number of the configured Port Pins */
#define PORT_CONFIGURED_PINS                 (39U)

/* Number of the configured Ports */
#define PORT_CONFIGURED_PORTS			     (6U)

/* Channel Index in the array of structures in Port_PBcfg.c */
#define PortConf_LED1_CHANNEL_ID_INDEX        (1U)     /* (PF1) */
#define PortConf_SW1_CHANNEL_ID_INDEX         (4U)     /* (PF4) */


#endif  /* PORT_CFG_H_ */