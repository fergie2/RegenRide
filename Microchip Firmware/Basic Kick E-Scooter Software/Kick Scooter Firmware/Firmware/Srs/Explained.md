## Overview of Files
- Main.c: This is the main file and contains the main functions of the project.
- init_dsPIC.c: This file contains the initialization functions for the processor.
- X2CScope.c: This file contains the main X2C Scope functions.
- X2CScopeComm.c: This file contains the communication functions for the X2C Scope.
- uart1.c: This file contains the UART 1 initialization functions necessary for the X2C Scope.
- FOC.h: This file includes all the definitions for the functioning of the Field Oriented Control algorithm.
- UserParms.h: This file includes all the definitions of the parameters for the functionality of the motor driver.
- init_dsPIC.h: This file includes the definitions of the processor initialization functions.
- X2CScope.h: This file includes the definitions for the X2C Scope functions.
- X2CScopeComm.h: This file includes the definitions for the X2C Scope communication functions.
- uart1.h: This file includes the definitions for the UART 1 functionality.
- Kick_scooter.x.a: This library file includes all the functions necessary for the motor control algorithm.
- libx2cscope-33ck-elf.a: This library file includes the X2C Scope functions.

## Resources Used by Our Microcontroller (MDR Prototype -> Original Design Without Updates)
- ADC: ADC Core 0 and ADC Shared Core are used for sensing various signals.
- ADC Channel 17 Interrupt: This is the main interrupt. The FOC state machine function runs here.
- Operational Amplifiers 1, 2, and 3: These are the internal operational amplifiers used to implement the current sense amplifiers.
- PWM Module: The PWM module is used to generate the three-phase inverter's control signals.
- CCP 1: Capture/Compare 1 is used for the motor speed measurement in sensored mode.
- UART 1: UART 1 interface is used for X2C Scope communications.
- CMP1, 2, 3: Interrupts of the DAC/CMP modules 1, 2, and 3 are used for hardware overcurrent protection.
- WDT: The Watchdog Timer is used to prevent accidental firmware stall.
- DSP Core Registers: Used for all computations of the control algorithm.

## Code Speed & Size
- Program Memory	3606 words	X2C_SCOPE option is disabled
- Data Memory	712 bytes	X2C_SCOPE option is disabled
- Program Memory	7392 words	X2C_SCOPE option is enabled
- Data Memory	6198 bytes	X2C_SCOPE option is enabled
- FOC State Machine Speed	15 µs	X2C_SCOPE option is enabled
- FOC State Machine Speed	14 µs	X2C_SCOPE option is disabled
