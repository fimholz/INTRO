/* ###################################################################
**     Filename    : Events.h
**     Project     : INTRO_FRDM_Master
**     Processor   : MKL25Z128VLK4
**     Component   : Events
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2016-09-20, 21:02, # CodeGen: 0
**     Abstract    :
**         This is user's event module.
**         Put your event handler code here.
**     Contents    :
**         Cpu_OnNMIINT - void Cpu_OnNMIINT(void);
**
** ###################################################################*/
/*!
** @file Events.h
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         

#ifndef __Events_H
#define __Events_H
/* MODULE Events */

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "SW1.h"
#include "BitIoLdd1.h"
#include "SW2.h"
#include "BitIoLdd2.h"
#include "SW3.h"
#include "BitIoLdd3.h"
#include "SW4.h"
#include "BitIoLdd4.h"
#include "SW5.h"
#include "BitIoLdd5.h"
#include "SW6.h"
#include "BitIoLdd6.h"
#include "SW7.h"
#include "BitIoLdd7.h"
#include "LEDPin1.h"
#include "BitIoLdd10.h"
#include "LEDPin2.h"
#include "BitIoLdd11.h"
#include "LEDPin3.h"
#include "BitIoLdd12.h"
#include "TI1.h"
#include "TimerIntLdd1.h"
#include "TU1.h"
#include "CLS1.h"
#include "AS1.h"
#include "ASerialLdd1.h"
#include "RTT1.h"
#include "WAIT1.h"
#include "CS1.h"
#include "HF1.h"
#include "KSDK1.h"
#include "UTIL1.h"
#include "KIN1.h"

#ifdef __cplusplus
extern "C" {
#endif 

/*
** ===================================================================
**     Event       :  Cpu_OnNMIINT (module Events)
**
**     Component   :  Cpu [MKL25Z128LK4]
*/
/*!
**     @brief
**         This event is called when the Non maskable interrupt had
**         occurred. This event is automatically enabled when the [NMI
**         interrupt] property is set to 'Enabled'.
*/
/* ===================================================================*/
void Cpu_OnNMIINT(void);


/*
** ===================================================================
**     Event       :  TI1_OnInterrupt (module Events)
**
**     Component   :  TI1 [TimerInt]
**     Description :
**         When a timer interrupt occurs this event is called (only
**         when the component is enabled - <Enable> and the events are
**         enabled - <EnableEvent>). This event is enabled only if a
**         <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void TI1_OnInterrupt(void);

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif 
/* ifndef __Events_H*/
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
