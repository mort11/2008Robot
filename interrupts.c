/*******************************************************************************
*
*	TITLE:		interrupts.c
*
*	VERSION:	0.3 (Beta)
*
*	DATE:		03-Jan-2008
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This file contains template interrupt initialization & handling
*				code for the IFI FRC robot controller.
*
*				This version is compatible with Microchip C18 3.0+ only.
*
*               This file best viewed with tabs set to four.
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or elsewhere
*				without permission. Thanks.
*
*				Copyright ©2004-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	22-Dec-2003  0.1  RKW Original
*	25-Feb-2004  0.2  RKW - Added the ability to clear the interrupt flag before
*	                  enabling the interrupt.
*	03-Jan-2008  0.3  RKW - Renamed all ISRs for consistancy across all
*	                  modules of the new robot controller code.
*
*******************************************************************************/

#include "ifi_frc.h"
#include "pwm.h"
#include "timers.h"
#include "interrupts.h"
#include "encoder.h"
#include "serial_ports.h"
#include "teleop.h"
#include "autonomous.h"
#include "disabled.h"
#include "mort_defines.h"

extern unsigned char volatile Old_Port_B;

/*******************************************************************************
*
*	FUNCTION:		Initialize_Int_1()
*
*	PURPOSE:		Initializes interrupt 1
*
*	CALLED FROM:	main.c/Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_1_ISR
void Initialize_Int_1(void)
{
	// initialize external interrupt 1 (INT2 on user 18F8520/18F8722)

	TRISBbits.TRISB2 = 1;		// make sure the RB2/INT2 pin is configured as an input
								//
	INTCON3bits.INT2IP = 0;		// 0: interrupt 1 is low priority (leave at 0 for IFI controllers)
								// 1: interrupt 1 is high priority
								//
	INTCON2bits.INTEDG2 = 1;	// 0: trigger on the falling-edge
								// 1: trigger on the rising-edge
								//
	INTCON3bits.INT2IF = 0;		// 0: external interrupt 1 hasn't happened (set to 0 before enabling the interrupt)
								// 1: external interrupt 1 has happened
								//
	INTCON3bits.INT2IE = 1;		// 0: disable interrupt	1
								// 1: enable interrupt 1
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Initialize_Int_2()
*
*	PURPOSE:		Initializes interrupt 2
*
*	CALLED FROM:	main.c/Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_2_ISR
void Initialize_Int_2(void)
{
	// initialize external interrupt 2 (INT3 on user 18F8520/18F8722)
	TRISBbits.TRISB3 = 1;		// make sure the RB3/CCP2/INT3 pin is configured as an input
								//
	INTCON2bits.INT3IP = 0;		// 0: interrupt 2 is low priority (leave at 0 for IFI controllers)
								// 1: interrupt 2 is high priority
								//
	INTCON2bits.INTEDG3 = 1;	// 0: trigger on the falling-edge
								// 1: trigger on the rising-edge
								//
	INTCON3bits.INT3IF = 0;		// 0: external interrupt 2 hasn't happened (set to 0 before enabling the interrupt)
								// 1: external interrupt 2 has happened
								//
	INTCON3bits.INT3IE = 1;		// 0: disable interrupt	2
								// 1: enable interrupt 2
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Initialize_Int_3_6()
*
*	PURPOSE:		Initializes interrupts 3 through 6
*
*	CALLED FROM:	main.c/Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_3_6_ISR
void Initialize_Int_3_6(void)
{
	// initialize external interrupts 3-6 (KBI0 - KBI3 on user 18F8520/18F8722)
	TRISBbits.TRISB4 = 1;		// make sure the RB4/HBI0 pin is configured as an input
	TRISBbits.TRISB5 = 1;		// make sure the RB5/KBI1/PGM pin is configured as an input
	TRISBbits.TRISB6 = 1;		// make sure the RB6/KBI2/PGC pin is configured as an input
	TRISBbits.TRISB7 = 1;		// make sure the RB7/KBI3/PGD pin is configured as an input
								//
  	INTCON2bits.RBIP = 0;		// 0: interrupts 3-6 are low priority (leave at 0 for IFI controllers)
								// 1: interrupts 3-6 are high priority
								//
	INTCONbits.RBIF = 0;		// 0: none of the interrupt 3-6 pins has changed state (set to 0 before enabling the interrupts)
								// 1: at least one of the interrupt 3-6 pins has changed state
								//
	INTCONbits.RBIE = 1;		// 0: disable interrupts 3-6
								// 1: enable interrupts 3-6
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_1_ISR()
*
*	PURPOSE:		If enabled, the interrupt 1 handler is called when the
*					interrupt 1/digital input 1 pin changes logic level. The
*					edge that the interrupt 1 pin reacts to is programmable
*					(see comments in the Initialize_Interrupts() function,
*					above).
*
*	CALLED FROM:	ifi_frc.c/Interrupt_Handler_Low()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_1_ISR
#pragma tmpdata low_isr_tmpdata
void Int_1_ISR(void)
{
	// this function will be called when an interrupt 1 occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_2_ISR()
*
*	PURPOSE:		If enabled, the interrupt 2 handler is called when the
*					interrupt 2/digital input 2 pin changes logic level. The
*					edge that the interrupt 2 pin reacts to is programmable
*					(see comments in the Initialize_Interrupts() function,
*					above).
*
*	CALLED FROM:	ifi_frc.c/Interrupt_Handler_Low()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_2_ISR
#pragma tmpdata low_isr_tmpdata
void Int_2_ISR(void)
{

}
#pragma tmpdata
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_3_ISR()
*
*	PURPOSE:		If enabled, the interrupt 3 handler is called when the
*					interrupt 3/digital input 3 pin changes logic level.
*
*	CALLED FROM:	ifi_frc.c/Interrupt_Handler_Low()
*
*	PARAMETERS:		RB4_State is the current logic level of the
*					interrupt 3 pin.
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_3_ISR
#pragma tmpdata low_isr_tmpdata
void Int_3_ISR(unsigned char RB4_State)
{

}
#pragma tmpdata
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_4_ISR()
*
*	PURPOSE:		If enabled, the interrupt 4 handler is called when the
*					interrupt 4/digital input 4 pin changes logic level.
*
*	CALLED FROM:	ifi_frc.c/Interrupt_Handler_Low()
*
*	PARAMETERS:		RB5_State is the current logic level of the
*					interrupt 4 pin.
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_4_ISR
#pragma tmpdata low_isr_tmpdata
void Int_4_ISR(unsigned char RB5_State)
{
	// this function will be called when an interrupt 4 occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_5_ISR()
*
*	PURPOSE:		If enabled, the interrupt 5 handler is called when the
*					interrupt 5/digital input 5 pin changes logic level.
*
*	CALLED FROM:	ifi_frc.c/Interrupt_Handler_Low()
*
*	PARAMETERS:		RB6_State is the current logic level of the
*					interrupt 5 pin.
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_5_ISR
#pragma tmpdata low_isr_tmpdata
void Int_5_ISR(unsigned char RB6_State)
{
	// this function will be called when an interrupt 5 occurs
}
#pragma tmpdata
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_6_ISR()
*
*	PURPOSE:		If enabled, the interrupt 6 handler is called when the
*					interrupt 6/digital input 6 pin changes logic level.
*
*	CALLED FROM:	ifi_frc.c/Interrupt_Handler_Low()
*
*	PARAMETERS:		RB7_State is the current logic level of the
*					interrupt 6 pin.
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_INT_6_ISR
#pragma tmpdata low_isr_tmpdata
void Int_6_ISR(unsigned char RB7_State)
{
	// this function will be called when an interrupt 6 occurs
}
#pragma tmpdata
#endif


/*******************************************************************************
*
*	FUNCTION:		Interrupt_Vector_Low()
*
*	PURPOSE:		Installs the low priority interrupt code at the low
*					priority interrupt vector, which is a fixed place in
*					memory where the microcontroller will start executing
*					code when it detects an interrupt condition. Because
*					this place in memory, at address 0x818, is intended
*					to contain only a very small amount of code, general
*					practice is to place a "goto" instruction here that
*					will point to the real interrupt handler somewhere else
*					in memory. More information on interrupts can be found
*					in the PIC18F8520 and PIC18F8722 data sheets.
*
*	CALLED FROM:	Called in response to a hardware generated interrupt
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#pragma code Interrupt_Vector_Low = LOW_INT_VECTOR
void Interrupt_Vector_Low (void)
{
  _asm
    goto Interrupt_Handler_Low  // jump to interrupt routine below
  _endasm
}
#pragma code

/*******************************************************************************
*
*	FUNCTION:		Interrupt_Handler_Low()
*
*	PURPOSE:		Determines which individual interrupt handler
*					should be called, clears the interrupt flag and
*					then calls the interrupt handler.
*
*	CALLED FROM:	Interrupt_Vector_Low()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		Before altering this code, make sure you understand
*					how interrupts work. Documentation can be found in
*					the C18 user's manual, the PIC18F8520/PIC18F8722 data
*					sheet, and the included C18_ISR.pdf document.
*
*					It is assumed that you won't be reading or writing
*					to program memory in your interrupt service routines.
*					If this is not the case, you'll need to remove
*					TBLPTRU, TBLPTRH, TBLPTRL and TABLAT from the nosave
*					section below.
*
*					It is also assumed that you won't be using function
*					pointers in your interrupt service routines. If this
*					is not the case, you'll need to remove the PCLATH and
*					PCLATU entries from the nosave section below.
*
*******************************************************************************/
#pragma tmpdata low_isr_tmpdata
#pragma interruptlow Interrupt_Handler_Low nosave=section(".tmpdata"),TBLPTRU,TBLPTRH,TBLPTRL,TABLAT,PCLATH,PCLATU
void Interrupt_Handler_Low()
{
	unsigned char Port_B;
	unsigned char Port_B_Delta;

	if (PIR1bits.RC1IF && PIE1bits.RC1IE) // rx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_RX
		Rx_1_ISR(); // call the rx1 interrupt handler (in serial_ports.c)
		#endif
	}
	else if (PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_RX
		Rx_2_ISR(); // call the rx2 interrupt handler (in serial_ports.c)
		#endif
	}

	#ifdef ENABLE_INT_1
	else if (INTCON3bits.INT2IF && INTCON3bits.INT2IE) // external interrupt 1?
	{
		INTCON3bits.INT2IF = 0; // clear the interrupt flag
		Int_1_ISR(); // call the interrupt 1 handler (in interrupts.c or encoder.c)
	}
	#endif
	#ifdef ENABLE_INT_2
	else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE) // external interrupt 2?
	{
		INTCON3bits.INT3IF = 0; // clear the interrupt flag
		Int_2_ISR(); // call the interrupt 2 handler (in interrupts.c or encoder.c)
	}
	#endif
	#ifdef ENABLE_INT_3_6
	else if (INTCONbits.RBIF && INTCONbits.RBIE) // external interrupts 3 through 6?
	{
		Port_B = PORTB; // remove the "mismatch condition" by reading port b
		INTCONbits.RBIF = 0; // clear the interrupt flag
		Port_B_Delta = Port_B ^ Old_Port_B; // determine which bits have changed
		Old_Port_B = Port_B; // save a copy of port b for next time around

		#ifdef ENABLE_INT_3
		if(Port_B_Delta & 0x10) // did external interrupt 3 change state?
		{
			Int_3_ISR(Port_B & 0x10 ? 1 : 0); // call the interrupt 3 handler (in interrupts.c or encoder.c)
		}
		#endif
		#ifdef ENABLE_INT_4
		if(Port_B_Delta & 0x20) // did external interrupt 4 change state?
		{
			Int_4_ISR(Port_B & 0x20 ? 1 : 0); // call the interrupt 4 handler (in interrupts.c or encoder.c)
		}
		#endif
		#ifdef ENABLE_INT_5
		if(Port_B_Delta & 0x40) // did external interrupt 5 change state?
		{
			Int_5_ISR(Port_B & 0x40 ? 1 : 0); // call the interrupt 5 handler (in interrupts.c or encoder.c)
		}
		#endif
		#ifdef ENABLE_INT_6
		if(Port_B_Delta & 0x80) // did external interrupt 6 change state?
		{
			Int_6_ISR(Port_B & 0x80 ? 1 : 0); // call the interrupt 6 handler (in interrupts.c or encoder.c)
		}
		#endif
	}
	#endif
	#ifdef ENABLE_TIMER_0
	else if (INTCONbits.TMR0IF && INTCONbits.TMR0IE) // timer 0 interrupt?
	{
		INTCONbits.TMR0IF = 0; // clear the timer 0 interrupt flag
		Timer_0_ISR(); // call the timer 0 interrupt handler (in timers.c)
	}
	#endif
	#ifdef ENABLE_TIMER_1
	else if (PIR1bits.TMR1IF && PIE1bits.TMR1IE) // timer 1 interrupt?
	{
		PIR1bits.TMR1IF = 0; // clear the timer 1 interrupt flag
		Timer_1_ISR(); // call the timer 1 interrupt handler (in timers.c)
	}
	#endif
	#ifdef ENABLE_TIMER_2
	else if (PIR1bits.TMR2IF && PIE1bits.TMR2IE) // timer 2 interrupt?
	{
		PIR1bits.TMR2IF = 0; // clear the timer 2 interrupt flag
		Timer_2_ISR(); // call the timer 2 interrupt handler (in timers.c)
	}
	#endif
	#ifdef ENABLE_TIMER_3
	else if (PIR2bits.TMR3IF && PIE2bits.TMR3IE) // timer 3 interrupt?
	{
		PIR2bits.TMR3IF = 0; // clear the timer 3 interrupt flag
		Timer_3_ISR(); // call the timer 3 interrupt handler (in timers.c)
	}
	#endif
	#ifdef ENABLE_TIMER_4
	else if (PIR3bits.TMR4IF && PIE3bits.TMR4IE) // timer 4 interrupt?
	{
		PIR3bits.TMR4IF = 0; // clear the timer 4 interrupt flag
		Timer_4_ISR(); // call the timer 4 interrupt handler (in timers.c)
	}
	#endif
	else if (PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_TX
		Tx_1_ISR(); // call the tx1 interrupt handler (in serial_ports.c)
		#endif
	}
	else if (PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_TX
		Tx_2_ISR(); // call the tx2 interrupt handler (in serial_ports.c)
		#endif
	}
}
#pragma tmpdata
