/**
  System Interrupts Generated Driver File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the generated manager file for the MPLAB(c) Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for MPLAB(c) Code Configurator interrupts.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - pic24-dspic-pic32mm : 1.65
        Device            :  PIC24FJ128GA204
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.35
        MPLAB             :  MPLAB X v4.20

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/

#ifndef _PIN_MANAGER_H
#define _PIN_MANAGER_H
/**
    Section: Includes
*/
#include <xc.h>
/**
    Section: Device Pin Macros
*/
/**
  @Summary
    Sets the GPIO pin, RA10, high using LATA10.

  @Description
    Sets the GPIO pin, RA10, high using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA10 high (1)
    IO_RA10_SetHigh();
    </code>

*/
#define IO_RA10_SetHigh()          _LATA10 = 1
/**
  @Summary
    Sets the GPIO pin, RA10, low using LATA10.

  @Description
    Sets the GPIO pin, RA10, low using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA10 low (0)
    IO_RA10_SetLow();
    </code>

*/
#define IO_RA10_SetLow()           _LATA10 = 0
/**
  @Summary
    Toggles the GPIO pin, RA10, using LATA10.

  @Description
    Toggles the GPIO pin, RA10, using LATA10.

  @Preconditions
    The RA10 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA10
    IO_RA10_Toggle();
    </code>

*/
#define IO_RA10_Toggle()           _LATA10 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA10.

  @Description
    Reads the value of the GPIO pin, RA10.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA10
    postValue = IO_RA10_GetValue();
    </code>

*/
#define IO_RA10_GetValue()         _RA10
/**
  @Summary
    Configures the GPIO pin, RA10, as an input.

  @Description
    Configures the GPIO pin, RA10, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA10 as an input
    IO_RA10_SetDigitalInput();
    </code>

*/
#define IO_RA10_SetDigitalInput()  _TRISA10 = 1
/**
  @Summary
    Configures the GPIO pin, RA10, as an output.

  @Description
    Configures the GPIO pin, RA10, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA10 as an output
    IO_RA10_SetDigitalOutput();
    </code>

*/
#define IO_RA10_SetDigitalOutput() _TRISA10 = 0
/**
  @Summary
    Sets the GPIO pin, RA9, high using LATA9.

  @Description
    Sets the GPIO pin, RA9, high using LATA9.

  @Preconditions
    The RA9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA9 high (1)
    IO_RA9_SetHigh();
    </code>

*/
#define IO_RA9_SetHigh()          _LATA9 = 1
/**
  @Summary
    Sets the GPIO pin, RA9, low using LATA9.

  @Description
    Sets the GPIO pin, RA9, low using LATA9.

  @Preconditions
    The RA9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RA9 low (0)
    IO_RA9_SetLow();
    </code>

*/
#define IO_RA9_SetLow()           _LATA9 = 0
/**
  @Summary
    Toggles the GPIO pin, RA9, using LATA9.

  @Description
    Toggles the GPIO pin, RA9, using LATA9.

  @Preconditions
    The RA9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RA9
    IO_RA9_Toggle();
    </code>

*/
#define IO_RA9_Toggle()           _LATA9 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RA9.

  @Description
    Reads the value of the GPIO pin, RA9.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RA9
    postValue = IO_RA9_GetValue();
    </code>

*/
#define IO_RA9_GetValue()         _RA9
/**
  @Summary
    Configures the GPIO pin, RA9, as an input.

  @Description
    Configures the GPIO pin, RA9, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA9 as an input
    IO_RA9_SetDigitalInput();
    </code>

*/
#define IO_RA9_SetDigitalInput()  _TRISA9 = 1
/**
  @Summary
    Configures the GPIO pin, RA9, as an output.

  @Description
    Configures the GPIO pin, RA9, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RA9 as an output
    IO_RA9_SetDigitalOutput();
    </code>

*/
#define IO_RA9_SetDigitalOutput() _TRISA9 = 0
/**
  @Summary
    Sets the GPIO pin, RC5, high using LATC5.

  @Description
    Sets the GPIO pin, RC5, high using LATC5.

  @Preconditions
    The RC5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC5 high (1)
    IO_RC5_SetHigh();
    </code>

*/
#define IO_RC5_SetHigh()          _LATC5 = 1
/**
  @Summary
    Sets the GPIO pin, RC5, low using LATC5.

  @Description
    Sets the GPIO pin, RC5, low using LATC5.

  @Preconditions
    The RC5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC5 low (0)
    IO_RC5_SetLow();
    </code>

*/
#define IO_RC5_SetLow()           _LATC5 = 0
/**
  @Summary
    Toggles the GPIO pin, RC5, using LATC5.

  @Description
    Toggles the GPIO pin, RC5, using LATC5.

  @Preconditions
    The RC5 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC5
    IO_RC5_Toggle();
    </code>

*/
#define IO_RC5_Toggle()           _LATC5 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC5.

  @Description
    Reads the value of the GPIO pin, RC5.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC5
    postValue = IO_RC5_GetValue();
    </code>

*/
#define IO_RC5_GetValue()         _RC5
/**
  @Summary
    Configures the GPIO pin, RC5, as an input.

  @Description
    Configures the GPIO pin, RC5, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC5 as an input
    IO_RC5_SetDigitalInput();
    </code>

*/
#define IO_RC5_SetDigitalInput()  _TRISC5 = 1
/**
  @Summary
    Configures the GPIO pin, RC5, as an output.

  @Description
    Configures the GPIO pin, RC5, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC5 as an output
    IO_RC5_SetDigitalOutput();
    </code>

*/
#define IO_RC5_SetDigitalOutput() _TRISC5 = 0
/**
  @Summary
    Sets the GPIO pin, RC6, high using LATC6.

  @Description
    Sets the GPIO pin, RC6, high using LATC6.

  @Preconditions
    The RC6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC6 high (1)
    IO_RC6_SetHigh();
    </code>

*/
#define IO_RC6_SetHigh()          _LATC6 = 1
/**
  @Summary
    Sets the GPIO pin, RC6, low using LATC6.

  @Description
    Sets the GPIO pin, RC6, low using LATC6.

  @Preconditions
    The RC6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC6 low (0)
    IO_RC6_SetLow();
    </code>

*/
#define IO_RC6_SetLow()           _LATC6 = 0
/**
  @Summary
    Toggles the GPIO pin, RC6, using LATC6.

  @Description
    Toggles the GPIO pin, RC6, using LATC6.

  @Preconditions
    The RC6 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC6
    IO_RC6_Toggle();
    </code>

*/
#define IO_RC6_Toggle()           _LATC6 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC6.

  @Description
    Reads the value of the GPIO pin, RC6.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC6
    postValue = IO_RC6_GetValue();
    </code>

*/
#define IO_RC6_GetValue()         _RC6
/**
  @Summary
    Configures the GPIO pin, RC6, as an input.

  @Description
    Configures the GPIO pin, RC6, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC6 as an input
    IO_RC6_SetDigitalInput();
    </code>

*/
#define IO_RC6_SetDigitalInput()  _TRISC6 = 1
/**
  @Summary
    Configures the GPIO pin, RC6, as an output.

  @Description
    Configures the GPIO pin, RC6, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC6 as an output
    IO_RC6_SetDigitalOutput();
    </code>

*/
#define IO_RC6_SetDigitalOutput() _TRISC6 = 0
/**
  @Summary
    Sets the GPIO pin, RC7, high using LATC7.

  @Description
    Sets the GPIO pin, RC7, high using LATC7.

  @Preconditions
    The RC7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC7 high (1)
    IO_RC7_SetHigh();
    </code>

*/
#define IO_RC7_SetHigh()          _LATC7 = 1
/**
  @Summary
    Sets the GPIO pin, RC7, low using LATC7.

  @Description
    Sets the GPIO pin, RC7, low using LATC7.

  @Preconditions
    The RC7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC7 low (0)
    IO_RC7_SetLow();
    </code>

*/
#define IO_RC7_SetLow()           _LATC7 = 0
/**
  @Summary
    Toggles the GPIO pin, RC7, using LATC7.

  @Description
    Toggles the GPIO pin, RC7, using LATC7.

  @Preconditions
    The RC7 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC7
    IO_RC7_Toggle();
    </code>

*/
#define IO_RC7_Toggle()           _LATC7 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC7.

  @Description
    Reads the value of the GPIO pin, RC7.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC7
    postValue = IO_RC7_GetValue();
    </code>

*/
#define IO_RC7_GetValue()         _RC7
/**
  @Summary
    Configures the GPIO pin, RC7, as an input.

  @Description
    Configures the GPIO pin, RC7, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC7 as an input
    IO_RC7_SetDigitalInput();
    </code>

*/
#define IO_RC7_SetDigitalInput()  _TRISC7 = 1
/**
  @Summary
    Configures the GPIO pin, RC7, as an output.

  @Description
    Configures the GPIO pin, RC7, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC7 as an output
    IO_RC7_SetDigitalOutput();
    </code>

*/
#define IO_RC7_SetDigitalOutput() _TRISC7 = 0
/**
  @Summary
    Sets the GPIO pin, RC8, high using LATC8.

  @Description
    Sets the GPIO pin, RC8, high using LATC8.

  @Preconditions
    The RC8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC8 high (1)
    IO_RC8_SetHigh();
    </code>

*/
#define IO_RC8_SetHigh()          _LATC8 = 1
/**
  @Summary
    Sets the GPIO pin, RC8, low using LATC8.

  @Description
    Sets the GPIO pin, RC8, low using LATC8.

  @Preconditions
    The RC8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC8 low (0)
    IO_RC8_SetLow();
    </code>

*/
#define IO_RC8_SetLow()           _LATC8 = 0
/**
  @Summary
    Toggles the GPIO pin, RC8, using LATC8.

  @Description
    Toggles the GPIO pin, RC8, using LATC8.

  @Preconditions
    The RC8 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC8
    IO_RC8_Toggle();
    </code>

*/
#define IO_RC8_Toggle()           _LATC8 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC8.

  @Description
    Reads the value of the GPIO pin, RC8.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC8
    postValue = IO_RC8_GetValue();
    </code>

*/
#define IO_RC8_GetValue()         _RC8
/**
  @Summary
    Configures the GPIO pin, RC8, as an input.

  @Description
    Configures the GPIO pin, RC8, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC8 as an input
    IO_RC8_SetDigitalInput();
    </code>

*/
#define IO_RC8_SetDigitalInput()  _TRISC8 = 1
/**
  @Summary
    Configures the GPIO pin, RC8, as an output.

  @Description
    Configures the GPIO pin, RC8, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC8 as an output
    IO_RC8_SetDigitalOutput();
    </code>

*/
#define IO_RC8_SetDigitalOutput() _TRISC8 = 0
/**
  @Summary
    Sets the GPIO pin, RC9, high using LATC9.

  @Description
    Sets the GPIO pin, RC9, high using LATC9.

  @Preconditions
    The RC9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC9 high (1)
    IO_RC9_SetHigh();
    </code>

*/
#define IO_RC9_SetHigh()          _LATC9 = 1
/**
  @Summary
    Sets the GPIO pin, RC9, low using LATC9.

  @Description
    Sets the GPIO pin, RC9, low using LATC9.

  @Preconditions
    The RC9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Set RC9 low (0)
    IO_RC9_SetLow();
    </code>

*/
#define IO_RC9_SetLow()           _LATC9 = 0
/**
  @Summary
    Toggles the GPIO pin, RC9, using LATC9.

  @Description
    Toggles the GPIO pin, RC9, using LATC9.

  @Preconditions
    The RC9 must be set to an output.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Toggle RC9
    IO_RC9_Toggle();
    </code>

*/
#define IO_RC9_Toggle()           _LATC9 ^= 1
/**
  @Summary
    Reads the value of the GPIO pin, RC9.

  @Description
    Reads the value of the GPIO pin, RC9.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    uint16_t portValue;

    // Read RC9
    postValue = IO_RC9_GetValue();
    </code>

*/
#define IO_RC9_GetValue()         _RC9
/**
  @Summary
    Configures the GPIO pin, RC9, as an input.

  @Description
    Configures the GPIO pin, RC9, as an input.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC9 as an input
    IO_RC9_SetDigitalInput();
    </code>

*/
#define IO_RC9_SetDigitalInput()  _TRISC9 = 1
/**
  @Summary
    Configures the GPIO pin, RC9, as an output.

  @Description
    Configures the GPIO pin, RC9, as an output.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    // Sets the RC9 as an output
    IO_RC9_SetDigitalOutput();
    </code>

*/
#define IO_RC9_SetDigitalOutput() _TRISC9 = 0

/**
    Section: Function Prototypes
*/
/**
  @Summary
    Configures the pin settings of the PIC24FJ128GA204
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description
    This is the generated manager file for the MPLAB(c) Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Example
    <code>
    void SYSTEM_Initialize(void)
    {
        // Other initializers are called from this function
        PIN_MANAGER_Initialize();
    }
    </code>

*/
void PIN_MANAGER_Initialize(void);

#endif
