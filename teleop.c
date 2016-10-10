/*******************************************************************************
*
*	TITLE:		teleop.c
*
*	VERSION:	0.1 (Beta)
*
*	DATE:		31-Dec-2007
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This file best viewed with tabs set to four.
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or elsewhere
*				without permission. Thanks.
*
*				Copyright ©2007-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	Change log:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	31-Dec-2007  0.1  RKW Original
*
*******************************************************************************/

#include <adc.h>
#include "ifi_frc.h"
#include "timers.h"
#include "interrupts.h"
#include "encoder.h"
#include "serial_ports.h"
#include "ifi_code.h"
#include "teleop.h"
#include "mort_defines.h"
#include "adc.h"
#include "autonomous.h"

unsigned char height_flag = HEIGHT_FLAG_MANUAL;
unsigned char tower_at_target = FALSE;

int drive_x = 0;
int drive_y = 0;
int drive_z = 0;
int tower_joystick = 0;
//const rom near static volatile unsigned short long double super_variable_of_dooooom = 7;

// CALLED ONCE
void Teleop_Init(void)
{
    height_flag = HEIGHT_FLAG_MANUAL;
    tower_at_target = FALSE;
}

// CALLED WHEN DRIVING ROBOT
void Teleop(void)
{
   //printf("x:%3d   y:%3d   \r\n",(int)T_Packet_Data.mx, (int)T_Packet_Data.my);

    Service_Joysticks();
    Service_Height_Flags();
    Service_Tower();
    Service_Leds();
    Service_Brakes(DRIVE_TRIG);
    Mec_Drive(drive_x, drive_y, drive_z);

    Service_Program_Button();
    Service_Dashboard_Data();
}

// CALLED BETWEEN MASTER DATA
void Teleop_Spin(void)
{
}

void Service_Program_Button(void)
{
    int byte_count = 0;
    int i = 0;

    byte_count = Serial_Port_One_Byte_Count();
    if (byte_count > 0)
    {
        for (i=0; i<byte_count; i++)
        {
            if (Read_Serial_Port_One() == 'p')
            {
                PROGRAM_BUTTON = 0;
            }
        }
    }
}

void Mec_Drive(int x, int y, int z)
{
    /**
    Objective
        Controls the motor outputs on 4 mecanum wheels using a 3 axis joystick input

    Called By
        ??

    Parameters
        int x : Strafing Direction (-127,127)
        int y : Forward and Reverse (-127,127)
        int z : Spin Direction (-127,127)

    Notes
        Based upon the ChiefDelphi white paper for initial determination of wheel speeds,
        then scales them based upon a reduction value (a double).
    **/

	int left_front = 0;
	int left_back = 0;
	int right_front = 0;
	int right_back = 0;
    double reduction;

	right_front=y-x-z;
	left_front=y+x+z;
	right_back=y+x-z;
	left_back=y-x+z;

    // Finds the absolute maximum value of the 4 wheel speeds
	reduction = absmax( absmax(right_front,right_back) , absmax(left_front, left_back) );

	// Determine the multiple to decrease the motor values by
	if (reduction > 127.0)
	{
		reduction /= 127.0;
	}
	else if (reduction < -127.0)
	{
	    //
		reduction /= -127.0;
	}
	else
	{
	    // If we are within the range, no reduction
		reduction = 1;
	}

    //Reduce the motor values so that the max is at 127 or min at -127
	left_front /= reduction;
	left_back /= reduction;
	right_back /= reduction;
	right_front /= reduction;

    // Make the value range from 0 to 255, with 128 and 127 of the converted value being equal
	left_front 	+= 127;
	left_back 	+= 127;
	right_front += 127;
	right_back 	+= 127;

    left_front = Limit(left_front, 0, 255);
    left_back = Limit(left_back, 0, 255);
    right_front = Limit(right_front, 0, 255);
    right_back = Limit(right_back, 0, 255);

	// Flop reversed motors
	left_front = 255 - left_front;
	left_back = 255 - left_back;

	// Scale to overcome victor deadzone
	if (left_front > 128)
	{
	    left_front = (((127.0 - PWM_DEADZONE_LEFT) / 127.0) * (left_front-127.0)) + PWM_DEADZONE_LEFT + 127.0;
	}

    if (left_back > 128)
	{
	    left_back = (((127.0 - PWM_DEADZONE_LEFT) / 127.0) * (left_back-127.0)) + PWM_DEADZONE_LEFT + 127.0;
	}

    if (right_front > 127)
	{
	    right_front = (((127.0 - PWM_DEADZONE_RIGHT) / 127.0) * (right_front-127.0)) + PWM_DEADZONE_RIGHT + 127.0;
	}

    if (right_back > 127)
	{
	    right_back = (((127.0 - PWM_DEADZONE_RIGHT) / 127.0) * (right_back-127.0)) + PWM_DEADZONE_RIGHT + 127.0;
	}

    left_front = Limit(left_front, 0, 255);
    left_back = Limit(left_back, 0, 255);
    right_front = Limit(right_front, 0, 255);
    right_back = Limit(right_back, 0, 255);

    BACK_LEFT 	= left_back;
    FRONT_LEFT  = left_front;
	FRONT_RIGHT = right_front;
	BACK_RIGHT 	= right_back;


}

void Service_Joysticks(void)
{
    // Makes copies
    drive_x = DRIVE_X;
    drive_y = DRIVE_Y;
    drive_z = DRIVE_Z;
    tower_joystick = TOWER_JOYSTICK;

    // Translate to zero (center)
    drive_x -= DRIVE_X_OFFSET;
    drive_y -= DRIVE_Y_OFFSET;
    drive_z -= DRIVE_Z_OFFSET;
    tower_joystick -= TOWER_JOYSTICK_OFFSET;

    // Scales inputs to -127  127
    if (drive_x > 0)
    {
        drive_x = (127.0 / DRIVE_X_MAX) *  drive_x;
    }
    else if (drive_x < 0)
    {
        drive_x = ((-1.0 * 127.0) / DRIVE_X_MIN) * drive_x;
    }

    if (drive_y > 0)
    {
        drive_y = (127.0 / DRIVE_Y_MAX) *  drive_y;
    }
    else if (drive_y < 0)
    {
        drive_y = ((-1.0 * 127.0) / DRIVE_Y_MIN) * drive_y;
    }

    if (drive_z > 0)
    {
        drive_z = (127.0 / DRIVE_Z_MAX) *  drive_z;
    }
    else if (drive_z < 0)
    {
        drive_z = ((-1.0 * 127.0) / DRIVE_Z_MIN) * drive_z;
    }

    if (tower_joystick > 0)
    {
        tower_joystick = (127.0 / TOWER_JOYSTICK_MAX) * tower_joystick;
    }
    else if (tower_joystick < 0)
    {
        tower_joystick = ((-1.0 * 127.0) / TOWER_JOYSTICK_MIN) * tower_joystick;
    }


    // Puts deadband on all joysticks
    drive_x = Deadband(drive_x, DRIVE_X_DEADBAND);
    drive_y = Deadband(drive_y, DRIVE_Y_DEADBAND);
    drive_z = Deadband(drive_z, DRIVE_Z_DEADBAND);
    tower_joystick = Deadband(tower_joystick, TOWER_JOYSTICK_DEADBAND);

    drive_x *= -1;
    drive_y *= 1;
    drive_z *= -1;
    tower_joystick *= -1;

    drive_x = Limit(drive_x, -127, 127);
    drive_y = Limit(drive_y, -127, 127);
    drive_z = Limit(drive_z, -127, 127);
    tower_joystick = Limit(tower_joystick, -127, 127);

    printf("drive_x: %d drive_y:%d drive_z:%d p1_x:%d p1_y: %d p2_x: %d\r\n", drive_x, drive_y, drive_z, p1_x, p1_y, p2_x);
}

void Service_Height_Flags (void)
{
	unsigned char button_pressed = 0;
	static unsigned char button_cancel = 0;
	static unsigned char button_released = 0;

	button_pressed = TOWER_BUTTON_LOW + TOWER_BUTTON_MIDDLE + TOWER_BUTTON_HIGH;
	button_pressed = Limit(button_pressed, 0, 1); //retard check

	if (TOWER_BUTTON_LOW && !button_cancel)
    {
        height_flag = HEIGHT_FLAG_LOW;
    }
    else if (TOWER_BUTTON_MIDDLE && !button_cancel)
    {
        height_flag = HEIGHT_FLAG_MIDDLE;
    }
    else if (TOWER_BUTTON_HIGH && !button_cancel)
    {
        height_flag = HEIGHT_FLAG_HIGH;
    }

	if (height_flag != HEIGHT_FLAG_MANUAL && !button_pressed)
	{
		button_released = 1;
	}

	if (button_pressed && button_released)
	{
		height_flag = HEIGHT_FLAG_MANUAL;
		button_released = 0;
		button_cancel = 1;
	}
	else if (!button_pressed && !button_released)
	{
        button_cancel = 0;
	}
}

void Service_Tower(void)
{
    static int set_height = 0;       // Desired height variable

    static int old_encoder_count = 0;
    int encoder_count = 0;
	int encoder_error = 0;
	static int tower_auto_speed = 0;

    static unsigned char hold_position_flag = 1;
    static int hold_position_encoder_count = 0;
    int hold_position_encoder_error = 0;

	int temp_tower = 0;              // temp value for tower speed
	int temp_rollers = 0;             // temp value for roller speed
	static int tower_integral = 0;

	static unsigned char tower_okay_to_run = 1; // make sure joystick is at zero before running tower after running rollers

    encoder_count = Get_Encoder_1_Count();
    tower_at_target = FALSE;

    if (height_flag == HEIGHT_FLAG_MANUAL)        // If we are in manual mode
    {
        tower_integral = 0; //in manual mode we dont want an integral

        /** Handle Rollers **/
        // If the rollers do random junk in autonomous, its electrical's fault
        if (TOWER_BUTTON_ROLLERS)
        {
            temp_rollers = tower_joystick;
            temp_tower = 0;
            tower_okay_to_run = 0;
            //printf("rollers running \r\n");
        }
        else
        {
            temp_rollers = 0;
            if (tower_okay_to_run)
            {
                temp_tower = tower_joystick;
            }
        }

        /* Hacking the rollers to stay on when the tower is going up */
        /** Changed the speed controler to brake, probably not necessary
        if (!TOWER_BUTTON_ROLLERS && temp_tower < 0)
        {
            temp_rollers = -20;
        }
        */

        if (tower_joystick == 0)
        {
            tower_okay_to_run = 1;
        }

        if (tower_joystick == 0 || TOWER_BUTTON_ROLLERS)
        {
            if (hold_position_flag)
            {
                hold_position_encoder_count = encoder_count;
                hold_position_flag = 0;
            }

            hold_position_encoder_error = encoder_count - hold_position_encoder_count;

            if ((hold_position_encoder_error >= TOWER_HOLD_POSITION_DEADZONE) || (hold_position_encoder_error <= (-1 * TOWER_HOLD_POSITION_DEADZONE)))
            {
                temp_tower = hold_position_encoder_error * TOWER_HOLD_POSITION_GAIN;
            }
        }
        else
        {
            hold_position_flag = 1;
        }
    }
    else if (height_flag == HEIGHT_FLAG_LOW) // If we are trying to go to a low value
    {
    /** Deccelerates the speed when tower is coming down **/

        temp_tower = encoder_count/2.0;   //TOWER_DOWN_SPEED;

    /** Prevents speed from being reduced exponentially after a low or a high value **/
        if (temp_tower < 50)//23
        {
            temp_tower = 50;//23
        }
        else if (temp_tower > 80)
        {
            temp_tower = 80;
        }
    /** arm holds current position **/
        hold_position_flag = 1;
    }
    else //Same logic for going to Middle or High height
    {
        if (height_flag == HEIGHT_COUNT_HIGH)
        {
            set_height = HEIGHT_COUNT_HIGH;
        }
        if (height_flag == HEIGHT_FLAG_MIDDLE)
        {
            set_height = HEIGHT_COUNT_MIDDLE;
        }

        // check how far arm is at target position hold the position
        encoder_error = encoder_count - set_height;
        hold_position_flag = 1;

        /** Ramping Tower Speeds **/
        if (encoder_error <= (-1 * TOWER_ENCODER_DEADZONE))
        {
            // Going up
            tower_auto_speed -= 15;
        }
        else if (encoder_error >= TOWER_ENCODER_DEADZONE)
        {
            // Going down
            tower_auto_speed = encoder_error;
        }

        tower_auto_speed = Limit(tower_auto_speed, encoder_error, 127);

        /** If tower is at target set in manual mode **/
        if (abs(encoder_error) <= TOWER_ENCODER_DEADZONE)
        {
            tower_at_target = TRUE;
            height_flag = HEIGHT_FLAG_MANUAL;
            tower_auto_speed = 0;
            tower_integral = 0;
        }
        /** Otherwise add to the integral and increase value**/
        else if (encoder_count == old_encoder_count)
        {
            tower_integral+=15;
        }

        tower_integral = Limit(tower_integral, -50, 50);

        tower_auto_speed -= tower_integral;

        temp_tower = tower_auto_speed;
    }

    /** Checks if Limit at Top has been reached and stops arm if true**/
    if (TOWER_LIMIT_ENABLED)
    {
        if ((encoder_count >= TOWER_ENCODER_LIMIT) && (temp_tower < 0)) // Less then zero means were going up
        {
            temp_tower = 0;
        }
    }

    /** Handle Bottom Limit Switch **/
    if (TOWER_LIMIT_DOWN)
    {
//        printf("Tower Down\r\n");
        Reset_Encoder_1_Count();

        if (temp_tower > 0)
        {
            temp_tower = 0;
            height_flag = HEIGHT_FLAG_MANUAL;
        }
    }

/** Final Assignments to motors**/
    old_encoder_count = encoder_count;

	temp_tower = Limit(temp_tower, -127, 127);
	temp_rollers = Limit(temp_rollers, -127, 127);

	TOWER_MOTOR_1 = temp_tower + 127;
	TOWER_MOTOR_2 = TOWER_MOTOR_1;

	ROLLER_MOTOR = temp_rollers + 127;
/***                           ***/

	//printf("integral: %d auto speed: %d\r\n", tower_integral, tower_auto_speed);
	//printf("roller speed: %d tower_joystick: %d roller_motor: %d\r\n", temp_rollers, tower_joystick, ROLLER_MOTOR);

//	printf("tower joystick: %d encoder: encoder_count: %d\r\n", tower_joystick, encoder_count);
}

void Service_Brakes(unsigned char onoff)
{
    BRAKE_1 = !onoff;
    BRAKE_2 = !onoff;
    BRAKE_3 = !onoff;
    BRAKE_4 = !onoff;
}

void Service_Leds(void)
{
    switch (height_flag)
    {
        case HEIGHT_FLAG_LOW:

            HEIGHT_LED_HIGH = 0;
            HEIGHT_LED_MIDDLE = 0;
            HEIGHT_LED_LOW = 1;

            break;

        case HEIGHT_FLAG_MIDDLE:

            HEIGHT_LED_HIGH = 0;
            HEIGHT_LED_MIDDLE = 1;
            HEIGHT_LED_LOW = 0;

            break;

        case HEIGHT_FLAG_HIGH:

            HEIGHT_LED_HIGH = 1;
            HEIGHT_LED_MIDDLE = 0;
            HEIGHT_LED_LOW = 0;

            break;

        case HEIGHT_FLAG_MANUAL:

            HEIGHT_LED_HIGH = 0;
            HEIGHT_LED_MIDDLE = 0;
            HEIGHT_LED_LOW = 0;

            break;

        default:

            HEIGHT_LED_HIGH = 0;
            HEIGHT_LED_MIDDLE = 0;
            HEIGHT_LED_LOW = 0;

            break;
    }
}

/**
Outputs :
    User_Byte{1-6}
**/
void Service_Dashboard_Data( void )
{
    static unsigned char p_num = 0;
    User_Byte2 = p_num;
    if (p_num == 0)
    {
        User_Byte1 = Limit(drive_x+127, 0, 255);      // Joystick X
        User_Byte3 = Limit(drive_y+127, 0, 255);      // Joystick Y
        User_Byte4 = Limit(drive_z+127, 0, 255);   // Joystick Z
        User_Byte5 = Limit(tower_joystick+127, 0, 255);      // Throttle
        User_Byte6 = 0;//Get_Encoder_1_Count(); // Encoder 1
    }
    else if (p_num == 1)
    {
        extern unsigned int range_left_front;
        User_Byte1 = TOWER_BUTTON_HIGH<<8 | TOWER_BUTTON_MIDDLE<<7 | TOWER_BUTTON_LOW<<6
                   | TOWER_BUTTON_ROLLERS<<5 | HEIGHT_LED_HIGH<<4 | HEIGHT_LED_MIDDLE<<3
                   | HEIGHT_LED_LOW<<2 | TOWER_BALL_SWITCH<<1 | 0;
      /*  User_Byte3 = HIBYTE(range_left_front);
        User_Byte4 = LOBYTE(range_left_front);*/
        User_Byte5 = HIBYTE(Get_Encoder_1_Count());
        User_Byte6 = LOBYTE(Get_Encoder_1_Count());
    }
    else if (p_num == 2)
    {

        User_Byte1 = Get_Auto_Mode();
        User_Byte3 = auto_state; // auto_state
        User_Byte4 = 0;
        User_Byte5 = 0;
        User_Byte6 = 0;
    }

    if(p_num < NUM_PACKETS - 1)
    {
        p_num ++;
    }
    else
    {
        p_num = 0;
    }
}

int Deadband(int value, int band)
{
	if (value <= (band/2.0) && value >= (-band/2.0))
	{
		return 0;
	}
	else if (value > 0)
	{
		return (127* (value - band/2.0) /(127 - band/2.0));
	}
	else
	{
		return (127* (value + band/2.0) /(127 - band/2.0));
	}
}

int Limit (int num, int low, int high)
{
	if (num > high)
		return high;
	else if (num < low)
		return low;

	return num;
}

int absmax(int n1,int n2)
{
    if (n1 < 0)
        n1 = -1 * n1;
    if( n2 < 0)
        n2 = -1 * n2;

	if(n1 > n2)
		return n1;
	else
		return n2;
}

int abs(int n)
{
    if (n < 0)
        return -n;
    return n;
}
