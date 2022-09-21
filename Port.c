 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Abdelrahman Elsayed
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

	#include "Det.h"
	/* AUTOSAR Version checking between Det and Port Modules */
	#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
	|| (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
	|| (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
	#error "The AR version of Det.h does not match the expected version"
	#endif

#endif



STATIC const Port_ConfigPinType * Port_Pins = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
 * Service Name: Port_Init
 * Service ID[hex]: 0x00
 * Sync/Async: Synchronous
 * Reentrancy: Non reentrant
 * Parameters (in): ConfigPtr - Pointer to post-build configuration data
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function to Initialize the Port Driver module.
 ************************************************************************************/
void Port_Init(const Port_ConfigType* ConfigPtr){
    boolean error = FALSE;
    uint8 Port_ClockPorts_flags[PORT_CONFIGURED_PORTS]={FALSE};

    /* intialize Port_Pins pointer to point to configured_pins[0] */
    Port_Pins=ConfigPtr->pins;
    volatile uint32 * Port_Base_Address_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    volatile uint32 delay = 0;
    Port_PinType counter;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    if(NULL_PTR == ConfigPtr){

            error = TRUE;
            /* Report to DET  */
            Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
                    PORT_INIT_SID,PORT_E_PARAM_CONFIG);

    }
    else{
        /* No Action Required */
    }

#endif


    if(FALSE == error){

        
        for(counter = 0 ; counter < PORT_CONFIGURED_PINS ; counter++){

            /* get base address */
                switch(Port_Pins[counter].port_num)
                {
                    case  0: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                    break;
                    case  1: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                    break;
                    case  2: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                    break;
                    case  3: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                    break;
                    case  4: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                    break;
                    case  5: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                    break;
                }

                ////////////////////////////////////////////////////////////////////////////
                /* Enable clock for PORT and allow time for clock to start*/
                ///////////////////////////////////////////////////////////////////////////                
                if(Port_ClockPorts_flags[ Port_Pins[counter].port_num ] == FALSE){
                    SET_BIT(SYSCTL_REGCGC2_REG,Port_Pins[counter].port_num);
                    delay = SYSCTL_REGCGC2_REG;
                    Port_ClockPorts_flags[ Port_Pins[counter].port_num ]=TRUE;
                }

                
                ////////////////////////////////////////////////////////////////////////////
                /* Unlocking If Needed */
                ///////////////////////////////////////////////////////////////////////////  
                if( ((Port_Pins[counter].port_num == PORT_PORTD_ID) && (Port_Pins[counter].pin_num == PORT_PIN7_ID)) || ((Port_Pins[counter].port_num == PORT_PORTF_ID) && (Port_Pins[counter].pin_num == PORT_PIN0_ID)) ) /* PD7 or PF0 */
                {
                    *(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_LOCK_REG_OFFSET) = UNLOCKING_VALUE;                     /* Unlock the GPIOCR register */   
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_COMMIT_REG_OFFSET) , Port_Pins[counter].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                }
                else if( (Port_Pins[counter].port_num == PORT_PORTC_ID) && (Port_Pins[counter].pin_num <= PORT_PIN3_ID) ) /* PC0 to PC3 */
                {
                    /* Do Nothing ...  this is the JTAG pins */
                    
                }
                else
                {
                    /* Do Nothing ... No need to unlock the commit register for this pin */
                }

                ////////////////////////////////////////////////////////////////////////////
                /*    analog VS digital    */
                ////////////////////////////////////////////////////////////////////////////
                if(Port_Pins[counter].pin_mode == PORT_MODE_ADC){

					SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pins[counter].pin_num);
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Pins[counter].pin_num);
				}
				else
                {
					CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pins[counter].pin_num);
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Pins[counter].pin_num);
				}

                ////////////////////////////////////////////////////////////////////////////
                /*    DIO VS alternative function    */
                ////////////////////////////////////////////////////////////////////////////
                if(Port_Pins[counter].pin_mode==PORT_MODE_DIO)
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pins[counter].pin_num);    /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                    *(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Pins[counter].pin_num * 4));   /* Clear the PMCx bits for this pin */
                }
                else
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pins[counter].pin_num);
                    *(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_CTL_REG_OFFSET) |= (Port_Pins[counter].pin_mode << (Port_Pins[counter].pin_num * 4));
                }   

                ////////////////////////////////////////////////////////////////////////////
                /* direction
                            if output pin -> (set initial value with high or low)
                            if input pin -> (set type of internal resistor) 
                */
                ////////////////////////////////////////////////////////////////////////////
                if(Port_Pins[counter].direction == PORT_PIN_OUT)/* output pin */
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[counter].pin_num);     /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                    /* init output pin with high value */
                    if(Port_Pins[counter].initial_value == PORT_PIN_LEVEL_HIGH)
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DATA_REG_OFFSET) , Port_Pins[counter].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                    }
                    else/* init output pin with low value*/
                    {
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DATA_REG_OFFSET) , Port_Pins[counter].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                    }
                }
                else if(Port_Pins[counter].direction  == PORT_PIN_IN)/* input pin */
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[counter].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                    /* pull up resistior */
                    if(Port_Pins[counter].resistor == PORT_RESISTOR_PULL_UP)
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_Pins[counter].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                    }
                    /* pull down resistor */
                    else if(Port_Pins[counter].resistor == PORT_RESISTOR_PULL_DOWN)
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_Pins[counter].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                    }
                    /* resistor off */
                    else if(Port_Pins[counter].resistor == PORT_RESISTOR_OFF)
                    {
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_Pins[counter].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_Pins[counter].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                    }
                }
                else
                {
                /* Do Nothing */
                }

                
        }
        Port_Status = PORT_INITIALIZED;

    }else{
        /* No Action Required */
    }

}



/************************************************************************************
 * Service Name: Port_SetPinDirection
 * Service ID[hex]: 0x01
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): Pin - Port Pin ID number
 * 					Direction - Port Pin Direction
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function that Sets the port pin direction.
 ************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API==STD_ON)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction){

	boolean error = FALSE;
     volatile uint32 * Port_Base_Address_Ptr = NULL_PTR; /* point to the required Port Registers base address */
#if (PORT_DEV_ERROR_DETECT == STD_ON)

	if(PORT_NOT_INITIALIZED == Port_Status){
		error = TRUE;
        /* Report to DET  */
		Det_ReportError(PORT_MODULE_ID,	PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID,	PORT_E_UNINIT);
	}
	else{
		/* No Action Required */
	}

	if( (Pin >= PORT_CONFIGURED_PINS) || (Pin < PORT_PIN0_ID) ){
		error = TRUE;
        /* Report to DET  */
		Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID,	PORT_E_PARAM_PIN);
	}
	else{
		/* No Action Required */
	}

	if(PORT_PIN_DIRECTION_UNCHANGEABLE == Port_Pins[Pin].changeableDirection){
		error = TRUE;
        /* Report to DET  */
		Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
				PORT_SET_PIN_DIRECTION_SID,PORT_E_DIRECTION_UNCHANGEABLE);
	}
	else{
		/* No Action Required */
	}

#endif
   
    if(FALSE == error){
        switch(Port_Pins[Pin].port_num)
        {
            case  0: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
            break;
            case  1: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
            break;
            case  2: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
            break;
            case  3: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
            break;
            case  4: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
            break;
            case  5: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
            break;
        }
        if(Direction == PORT_PIN_OUT)/* output pin */
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[Pin].pin_num);    
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[Pin].pin_num);  
        }
	}
	else
    {
		/* No Action Required */
	}
}
#endif



/************************************************************************************
 * Service Name: Port_RefreshPortDirection
 * Service ID[hex]: 0x02
 * Sync/Async: Synchronous
 * Reentrancy: Non-Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Function that Refreshes port direction.
 ************************************************************************************/
void Port_RefreshPortDirection(void){
	boolean error = FALSE;
    volatile uint32 * Port_Base_Address_Ptr = NULL_PTR; /* point to the required Port Registers base address */
#if (PORT_DEV_ERROR_DETECT == STD_ON)

	if(PORT_NOT_INITIALIZED == Port_Status){
		error = TRUE;
        /* Report to DET  */
		Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
				PORT_REFRESH_PORT_DIRECTION_SID,PORT_E_UNINIT);
	}
	else{
		/* No Action Required */
	}

#endif
    
    if(FALSE == error){
        Port_PinType counter;
        for(counter = 0 ; counter < PORT_CONFIGURED_PINS ; counter++){
             switch(Port_Pins[counter].port_num)
            {
                case  0: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                break;
                case  1: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                break;
                case  2: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                break;
                case  3: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                break;
                case  4: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                break;
                case  5: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                break;
            }
            /* The function Port_RefreshPortDirection shall exclude those port pins from refreshing that are
                 configured as ‘pin direction changeable during runtime */
            if(PORT_PIN_DIRECTION_UNCHANGEABLE == Port_Pins[counter].changeableDirection){
				if(Port_Pins[counter].direction == PORT_PIN_OUT)/* output pin */
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[counter].pin_num);     /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                }else{
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[counter].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                }

			}
			else{
				/* No Action Required */
			}
        }
    }else{
        /* No Action Required */
    }
}




/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non Reentran
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): VersionInfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Function to get the version information of this module.
************************************************************************************/
#if ( PORT_VERSION_INFO_API==STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo){
    boolean error=FALSE;
#if (PORT_DEV_ERROR_DETECT == STD_ON)

    if(PORT_NOT_INITIALIZED == Port_Status){
		error = TRUE;
        /* Report to DET  */
		Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID,PORT_E_UNINIT);

	}
	else{
		/* No Action Required */
	}
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{   
        error = TRUE;
		/* Report to DET  */
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
	}
	else{

    }
#endif /* (PORT_DEV_ERROR_DETECT == STD_OFF) */
    if(FALSE == error)
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	}else{
        /* No Action Required */
    }
}
#endif


/************************************************************************************
 * Service Name: Port_SetPinMode
 * Service ID[hex]: 0x04
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): Pin - Port Pin ID number
 * 					Mode - New Port Pin mode to be set on port pin.
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Sets the port pin mode.
 ************************************************************************************/
#if (PORT_SET_PIN_MODE_API==STD_ON)
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode){
	boolean error = FALSE;
    volatile uint32 * Port_Base_Address_Ptr = NULL_PTR; /* point to the required Port Registers base address */
#if (PORT_DEV_ERROR_DETECT == STD_ON)

	if(PORT_NOT_INITIALIZED == Port_Status){
		error = TRUE;
        /* Report to DET  */
		Det_ReportError(PORT_MODULE_ID,	PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
	}
	else{
		/* No Action Required */
	}

	if( (Pin >= PORT_CONFIGURED_PINS) || (Pin < PORT_PIN0_ID) ){
		error = TRUE;
        /* Report to DET  */
		Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID,	PORT_E_PARAM_PIN);
	}
	else{
		/* No Action Required */
	}

	if(PORT_PIN_MODE_UNCHANGEABLE == Port_Pins[Pin].changeableMode){
		error = TRUE;
		Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID,PORT_E_MODE_UNCHANGEABLE);
	}
	else{
		/* No Action Required */
	}

    if((Mode < PORT_MODE_DIO) || (Mode > PORT_MODE_ADC)){

		error = TRUE;
		Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,
				PORT_SET_PIN_MODE_SID,PORT_E_PARAM_INVALID_MODE);
	}
	else{
		/* No Action Required */
	}

#endif
    
    if(FALSE == error){
        switch(Port_Pins[Pin].port_num)
        {
            case  0: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
            break;
            case  1: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
            break;
            case  2: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
            break;
            case  3: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
            break;
            case  4: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
            break;
            case  5: Port_Base_Address_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
            break;
        }

        /*    analog VS digital    */
        if( PORT_MODE_ADC==Mode){

            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pins[Pin].pin_num);
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Pins[Pin].pin_num);
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pins[Pin].pin_num);
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Pins[Pin].pin_num);
        }

        /*    DIO VS alternative function    */
        if(PORT_MODE_DIO==Mode)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pins[Pin].pin_num);    /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            *(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Pins[Pin].pin_num * 4));   /* Clear the PMCx bits for this pin */
        }
        else
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pins[Pin].pin_num);
            *(volatile uint32 *)((volatile uint8 *)Port_Base_Address_Ptr + PORT_CTL_REG_OFFSET) |= (Port_Pins[Pin].pin_mode << (Port_Pins[Pin].pin_num * 4));
        }
	}
	else
    {
		/* No Action Required */
	}

}
#endif




