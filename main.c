/*******************************************************************************
*
*	TITLE:		ifi_frc.c
*
*	VERSION:	0.3 (Beta)
*
*	DATE:		31-Dec-2007
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This file contains the startup and interrupt service routine
*				dispatch code for the IFI FRC robot controller.
*
*				This file best viewed with tabs set to four.
*
********************************************************************************
*
*	Change log:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	22-Dec-2007  0.1  RKW Original
*	24-Dec-2007  0.2  RKW - Added memory initialization code to _startup().
*	31-Dec-2007  0.3  RKW - Added calls to Autonomous_Init() and Teleop_Init().
*
*******************************************************************************/
#include <adc.h>
#include "adc.h"
#include <delays.h>
#include "ifi_frc.h"
#include "timers.h"
#include "interrupts.h"
#include "encoder.h"
#include "serial_ports.h"
#include "teleop.h"
#include "autonomous.h"
#include "disabled.h"
#include "gyro.h"

// So that we'll know which interrupt pin changed state, the
// state of port b is saved in this variable each time the
// interrupt handler for interrupts 3 through 6 is called. This
// variable should be initialized to the current state of port
// B just before enabling interrupts 3 through 6.
unsigned char volatile Old_Port_B = 0xFF;

/*******************************************************************************
*
*	FUNCTION:		Initialization()
*
*	PURPOSE:		This function is called once when the robot controller
*					is cold or warm booted. You should initialize your code
*					here.
*
*	CALLED FROM:	main() in ifi_frc.c
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Initialization(void)
{
	// Setup the digital I/O pins. Use "INPUT" to setup the pin
	// as an input and "OUTPUT" to setup the pin as an output.
	digital_io_01 = INPUT;
	digital_io_02 = INPUT;
	digital_io_03 = INPUT;
	digital_io_04 = INPUT;
	digital_io_05 = INPUT;
	digital_io_06 = INPUT;
	digital_io_07 = INPUT;
	digital_io_08 = INPUT;
	digital_io_09 = INPUT;
	digital_io_10 = INPUT;
	digital_io_11 = INPUT;
	digital_io_12 = INPUT;
	digital_io_13 = INPUT;
	digital_io_14 = OUTPUT;     // brakes
	digital_io_15 = OUTPUT;     // brakes
	digital_io_16 = OUTPUT;     // brakes
	digital_io_17 = OUTPUT;     // brakes
	digital_io_18 = OUTPUT;     // program button


	// Initialize the digital outputs. If the pin is configured
	// as an input above, it doesn't matter what state you
	// initialize it to here.
	rc_dig_out01 = 0;
	rc_dig_out02 = 0;
	rc_dig_out03 = 0;
	rc_dig_out04 = 0;
	rc_dig_out05 = 0;
	rc_dig_out06 = 0;
	rc_dig_out07 = 0;
	rc_dig_out08 = 0;
	rc_dig_out09 = 0;
	rc_dig_out10 = 0;
	rc_dig_out11 = 0;
	rc_dig_out12 = 0;
	rc_dig_out13 = 0;
	rc_dig_out14 = 0;
	rc_dig_out15 = 0;
	rc_dig_out16 = 0;
	rc_dig_out17 = 0;
	rc_dig_out18 = 1;

    // send printf() output to serial port one.
	stdout_serial_port = SERIAL_PORT_ONE;

	// initialize the serial ports
	// (see serial_ports_readme.txt and serial_ports.c/.h)
	Init_Serial_Port_One(BAUD_115200);
	Init_Serial_Port_Two(BAUD_115200);
    printf("Initialized Serial Ports >>\r\n");

    // Initialize ADC
    Initialize_ADC();
    printf("Initializing ADC >>\r\n");

    // Initialize Gyro
    Initialize_Gyro();

   	// Remove the // below to initialize encoder #1
    Initialize_Encoder_1();
    printf("Initialized Encoder >>\r\n");

    //Initialize Interrupts
    printf("Initialized Interrupts >>\r\n");

    //Initialize timers
    Initialize_Timer_2(); // Our timer, used in robocoach
    printf("Initialized Timers >>\r\n");

	printf("IFI User Processor Initialized ...\r\n");
}

/*******************************************************************************
*
*	FUNCTION:		main()
*
*	PURPOSE:		Entry point for your C code.
*
*	CALLED FROM:	_startup()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
void main(void)
{
	// initialization state flags
	static unsigned char disabled_init_flag = 1;
	static unsigned char autonomous_init_flag = 1;
	static unsigned char teleop_init_flag = 1;

	// take a snapshot of port b before interrupts are enabled
	Old_Port_B = PORTB;

	// call the system initialization code which initializes
	// processor registers and the SPI communication channel
	// with the master processor
	IFI_Initialization();

	// call the user initialization code in main.c
	Initialization();

	// let the master processor know we're done initializing
	Putdata(&txdata);
	User_Proc_Is_Ready();

	statusflag.NEW_SPI_DATA = 0;

	while(TRUE)
	{
		if(disabled_mode)
		{
			// set the initialization flags because at some point
			// we'll transition out of the disabled state and need
			// to reset the PWM and relay values before transitioning
			// to the autonomous or teleoperation mode
			teleop_init_flag = 1;
			autonomous_init_flag = 1;

			// have we received new data from the master processor?
			if(statusflag.NEW_SPI_DATA)
			{
				// call Disabled_Init() if this is the first time
				// we've entered disabled_mode
				if(disabled_init_flag == 1)
				{
					Disabled_Init();
					disabled_init_flag = 0;
				}
				Getdata(&rxdata);	// get updated data from the master processor
				Disabled();			// call the user's Disabled() function (in disabled.c)
				Putdata(&txdata);	// send updated data to the master processor
			}
			else
			{
				Disabled_Spin();	// located in disabled.c
			}
		}
		else if(autonomous_mode)
		{
			// have we received new data from the master processor?
			if(statusflag.NEW_SPI_DATA)
			{
				// if we just transitioned to autonomous mode from
				// a different mode, we need to reset all motors
				// and relays to the off state before executing the
				// code in Autonomous_Init() and Autonomous() for
				// the first time
				if(autonomous_init_flag == 1)
				{
					Reset_Outputs();
					Autonomous_Init();
					autonomous_init_flag = 0;
					teleop_init_flag = 1;
				}

				Getdata(&rxdata);	// get updated data from the master processor
				Autonomous();		// call the user's autonomous code (in autonomous.c)
				Putdata(&txdata);	// send updated data to the master processor
			}
			else
			{
				Autonomous_Spin();	// located in autonomous.c
			}
		}
		else	// only option left is teleoperation mode
		{
			// have we received new data from the master processor?
			if(statusflag.NEW_SPI_DATA)
			{
				// if we just transitioned to teleoperation mode
				// a different mode, we need to reset all motors
				// and relays to the off state before executing
				// the code in Teleop_Init() and Teleop() for the
				// first time
				if(teleop_init_flag == 1)
				{
					Reset_Outputs();
					Teleop_Init();
					teleop_init_flag = 0;
					autonomous_init_flag = 1;
				}

				Getdata(&rxdata);	// get updated data from the master processor
				Teleop();			// call the user's teleoperation code (in teleop.c)
				Putdata(&txdata);	// send updated data to the master processor
			}
			else
			{
				Teleop_Spin();		// located in teleop.c
			}
		}
	}
}


/*******************************************************************************
*
*	FUNCTION:		_entry()
*
*	PURPOSE:		Installs the bootstrap code at location 0x800, which
*					is a fixed location in program memory where the boot-
*					loader expects the startup code to be found. Because
*					this place in memory, at address 0x800, is intended
*					to contain only a very small amount of code, general
*					practice is to place a "goto" instruction here that
*					will point to the real boot code somewhere else in
*					memory.
*
*	CALLED FROM:	Called by the bootloader after a reset
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		This code is based upon version 1.7 of Microchip's
*					MPLAB-C18 startup code.
*
*******************************************************************************/
#pragma code _entry_scn = RESET_VECTOR
void _entry (void)
{
	_asm
	goto _startup
	_endasm
}

/*******************************************************************************
*
*	FUNCTION:		_startup()
*
*	PURPOSE:
*
*	CALLED FROM:	_entry()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		This code is based upon version 1.7 of Microchip's
*					MPLAB-C18 startup code.
*
*******************************************************************************/
#pragma code _startup_scn
void _startup (void)
{
	_asm
    // Initialize the stack pointer
    lfsr 1, _stack
    lfsr 2, _stack
    clrf TBLPTRU, 0		// 1st silicon doesn't do this on POR
    bcf __FPFLAGS,6,0	// Initialize rounding flag for floating point libs
    _endasm

	// initialize memory to all zeros
	_asm
	lfsr   0, 0
	movlw  0xF
	clear_loop:
	clrf   POSTINC0, 0
	cpfseq FSR0H, 0
	bra    clear_loop
	_endasm

	// initialize variables
    _do_cinit();

	loop:

	// Call the user's main routine
	main ();

	goto loop;
}

// MPLAB-C18 initialized data memory support
// The linker will populate the _cinit table
extern far rom struct
{
	unsigned short num_init;
	struct _init_entry
	{
		unsigned long from;
		unsigned long to;
		unsigned long size;
	}
  	entries[];
}_cinit;

/*******************************************************************************
*
*	FUNCTION:		_do_cinit()
*
*	PURPOSE:		Initializes C variables
*
*	CALLED FROM:	_startup()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		This code is based upon version 1.7 of Microchip's
*					MPLAB-C18 startup code.
*
*******************************************************************************/
#pragma code _cinit_scn
void _do_cinit (void)
{
	// we'll make the assumption in the following code that these statics
	// will be allocated into the same bank.
	static short long prom;				// flash memory source address
	static unsigned short curr_byte;
	static unsigned short curr_entry;
	static short long data_ptr;

	// get the number of entries in the _cinit initialization table
	// and save in the curr_entry variable
	TBLPTR = (short long)&_cinit;
	_asm
	movlb data_ptr
	tblrdpostinc
	movf TABLAT,0,0
	movwf curr_entry,1
	tblrdpostinc
	movf TABLAT, 0, 0
	movwf curr_entry+1,1
	_endasm

	// check to see if we have any (more) entries to process?
zero_test:
    _asm
	bnz get_entry
	tstfsz curr_entry,1
	bra get_entry
	_endasm
	goto done;

	// At this point we know that the table pointer points to the top
	// of the current entry in _cinit, so we can just start reading the
	// from, to, and size values...
get_entry:
	_asm
	tblrdpostinc		// read the low byte of the source address into the prom variable
	movf TABLAT,0,0
	movwf prom, 1
	tblrdpostinc		// read the middle byte of the source address into the prom variable
	movf TABLAT,0,0
	movwf prom+1,1
	tblrdpostinc		// read the high byte of the source address into the prom variable
	movf TABLAT,0,0
	movwf prom+2,1
	tblrdpostinc		// skip a byte since the source addresss is stored as a 32-bit value
	tblrdpostinc		// read the lower byte of the destination address directly into FSR0L
	movf TABLAT,0,0
	movwf FSR0L,0
	tblrdpostinc		// read the upper byte of the destination address directly into FSR0H
	movf TABLAT,0,0
	movwf FSR0H,0
	tblrdpostinc		// skip the next two bytes since the destination address is stored as a 32-bit value
	tblrdpostinc
	tblrdpostinc		// read the lower byte of the size and store in the lower byte of curr_byte
	movf TABLAT,0,0
	movwf curr_byte,1
	tblrdpostinc		// read the upper byte of the size and store in the upper byte of curr_byte
	movf TABLAT,0,0
	movwf curr_byte+1,1
	tblrdpostinc		// skip the next two bytes since the size is stored as a 32-bit value
	tblrdpostinc
	_endasm

	data_ptr = TBLPTR;	// save the table pointer because it now points to the next table entry and we'll need it later
	TBLPTR = prom;		// now assign the source address to the table pointer

	// determine if we have any more data to copy from this block
	_asm
	movlb curr_byte
	movf curr_byte,1,1
copy_loop:
	bnz copy_one_byte
	movf curr_byte+1,1,1
	bz done_copying

	// we've read the table entry and know that we have data to copy
	// so now do the actual transfer from flash memory to SRAM
copy_one_byte:
	tblrdpostinc
	movf TABLAT,0,0
	movwf POSTINC0,0
	decf curr_byte,1,1	// decrement and test the byte counter
	bc copy_loop
	decf curr_byte+1,1,1
	bra copy_one_byte
done_copying:
	_endasm

	// restore the table pointer for the next entry
	TBLPTR = data_ptr;
	// next entry...
	curr_entry--;
	goto zero_test;
done:
;
}

/*******************************************************************************
*
*	FUNCTION:		Reset_Outputs()
*
*	PURPOSE:		Resets the robot controller PWM outputs to neutral and
*					turns off all relays.
*
*	CALLED FROM:	main()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Reset_Outputs(void)
{
	pwm01 = 127;
	pwm02 = 127;
	pwm03 = 127;
	pwm04 = 127;
	pwm05 = 127;
	pwm06 = 127;
	pwm07 = 127;
	pwm08 = 127;
	pwm09 = 127;
	pwm10 = 127;
	pwm11 = 127;
	pwm12 = 127;
	pwm13 = 127;
	pwm14 = 127;
	pwm15 = 127;
	pwm16 = 127;

	relay1_fwd = 0;
	relay1_rev = 0;
	relay2_fwd = 0;
	relay2_rev = 0;
	relay3_fwd = 0;
	relay3_rev = 0;
	relay4_fwd = 0;
	relay4_rev = 0;
	relay5_fwd = 0;
	relay5_rev = 0;
	relay6_fwd = 0;
	relay6_rev = 0;
	relay7_fwd = 0;
	relay7_rev = 0;
	relay8_fwd = 0;
	relay8_rev = 0;
}

/*******************************************************************************
* FUNCTION NAME: Get_Analog_Value
* PURPOSE:       Reads the analog voltage on an A/D port and returns the
*                10-bit value read stored in an unsigned int.
* CALLED FROM:
* ARGUMENTS:
*      Argument         Type        IO   Description
*     -----------   -------------   --   -----------
*     ADC_channel       alias       I    alias found in ifi_aliases.h
* RETURNS:       unsigned int
*******************************************************************************/
unsigned int Get_Analog_Value (unsigned char ADC_channel)
{
  unsigned int result;

#if defined(__18F8722)
  OpenADC( ADC_FOSC_RC & ADC_RIGHT_JUST & ADC_0_TAD,
           ADC_channel & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,15);
#else
  OpenADC( ADC_FOSC_RC & ADC_RIGHT_JUST & ifi_analog_channels,
          ADC_channel & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS );
#endif
  Delay10TCYx(10);
  ConvertADC();
  while(BusyADC());
  ReadADC();
  CloseADC();
  result = (int) ADRESH << 8 | ADRESL;
  return result;
}
