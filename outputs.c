/*****
outputs.c

Contains all code which is used to control outputs ont eh robot.
For example, motors.
******/

#include "mort_defines.h"
#include "ifi_frc.h"
#include "outputs.h"
#include "encoder.h"

void Mec_Drive_1(int x, int y, int z)
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
    Divides by a rounded number, causing major speed reductions when using non-maximum values,
    for example "Mec_Drive_1(1,127,2);"
**/
	int left_front, left_back, right_front, right_back;

    //Round up to the nearest integer
	int reduction_factor = (x && 1) + (y && 1) + (z && 1);

	if (!reduction_factor)
	{
	    left_front = 0;
	    left_back = 0;
	    right_front = 0;
	    right_back = 0;
	}
	else
	{
		int left_back_x, left_back_z;
		int right_front_x, right_front_z;
		int right_back_x, right_back_z;

	    x = x / reduction_factor;
		z = z / reduction_factor;
		y = y / reduction_factor;

		right_back_x = -x;
		right_front_x = x;
		left_back_x = right_front_x;

		right_back_z = -z;
		right_front_z = right_back_z;
		left_back_z = z;

		left_front 	= 	-x + y + z;
		left_back 	= 	left_back_x  + 	y + left_back_z;
		right_front =	right_front_x + y + right_front_z;
		right_back 	= 	right_back_x + 	y + right_back_z;
	}

	// Make the value range from 0 to 255, with 128 and 127 of the converted value being equal
	left_front 	+= 127 + (left_front 	> 0);
	left_back 	+= 127 + (left_back 	> 0);
	right_front += 127 + (right_front 	> 0);
	right_back 	+= 127 + (right_back 	> 0);

	// CHANGE TO DEFINES FOR PWMS
	FRONT_LEFT 	= left_front;
	FRONT_RIGHT = right_front;
	BACK_LEFT 	= left_back;
	BACK_RIGHT 	= right_back;
}

void Mec_Drive_2(int x, int y, int z)
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
        Based upon the CheifDelphi white paper for initial determination of wheel speeds,
        then scales them based upon a reduction value (a double).
    **/

	int left_front;
	int left_back;
	int right_front;
	int right_back;
    double reduction;

	right_front=y-x+z;
	left_front=y+x-z;
	right_back=y+x+z;
	left_back=y-x-z;

    // Using the reduction variable here to avoid movre variables.
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
	left_front 	+= 127 + (left_front 	> 0);
	left_back 	+= 127 + (left_back 	> 0);
	right_front += 127 + (right_front 	> 0);
	right_back 	+= 127 + (right_back 	> 0);

	// CHANGE TO DEFINES FOR PWMS
	FRONT_LEFT 	= left_front;
	FRONT_RIGHT = right_front;
	BACK_LEFT 	= left_back;
	BACK_RIGHT 	= right_back;
}
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;

  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}


int absmax(int n1,int n2)
{
    /**
    Objective
        Determines the input value that has a greater absolute value and returns it.

    Called By
        outputs.c/Mec_Drive_2

    Parameters
        int n1 : An integer to compare
        int n2 : An integer to compare

    Return
        The intiger which has a greater absolute value
    **/

	if(abs(n1) > abs(n2))
		return n1;
	else
		return n2;

}

int abs(int n)
{
    /**
    Objective
        Returns the absolute value of the parameter.

    Called By
        outputs.c/Mec_Drive_2

    Parameters
        int n : value to determine absolute value of.

    Retrun
        The absolute value of n.
    **/

	if (n < 0)
		return -n;
	else
		return  n;
}

int Limit (int num, int low, int high)
{
    /**
    Objective
        Prevent a value from exceeding a specified range by setting it the max or min of the range if
        it falls below the range or rests above the range.

    Parameters
        int num : The number to be forced within the range
        int low : The bottom of the range (<high)
        int high: The top of the range (>low)

    Return
        int : num if the value falls bettween low and high, otherwise low or high
    **/

	if (num > high)
		return high;
	else if (num < low)
		return low;

	return num;
}

int Deadband(int value, int band)
{
    /**
    Objective
        Scale an inputed value between a range "A" while a range "D"
        at the center of the range "A" scales to 0.

    Parameters
        int value : The value to be scaled based upon the "dead" zone.
        int band  : The side length of the "dead" range.

    Return
        int : The scaled value
    **/

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

void Service_Arm (int height)
{
	int encoder_count = 0;
	int encoder_error = 0;
	int y = 0;

	encoder_count = Get_Encoder_1_Count();

	if (height >= 0)
	{
		encoder_error = height - encoder_count;
		y = encoder_error * ARM_GAIN;
	}
	else
	{
		y = ARM_JOYSTICK;
	}

	Limit(y, -127, 127);

	ARM_MOTOR = y + 127;
}

/**
*/
void Service_Rollers(int speed)
{
    int y = 0;

	if (ARM_BUTTON)
	{
		y = ARM_JOYSTICK;
	}
	else
	{
		y = speed;
	}

	Limit(y, -127, 127);

	ROLLER_MOTOR = y + 127;
}

//Does not work currently
void Non_Mec_Drive(int x, int y)
{

	LEFT_MOTOR_1= LEFT_MOTOR_2= Limit_Mix(2000 + y + x - 127);
	RIGHT_MOTOR_1 = RIGHT_MOTOR_2 	= Limit_Mix(2000 + y  - x + 127);

//    RIGHT_MOTOR_1 = RIGHT_MOTOR_2 = r + 127 + (r > 0);
//    LEFT_MOTOR_1 = LEFT_MOTOR_2 =   l + 127 + (r > 0);

/*    int r = f - t;
    int l = f + t;

    double reduction = absmax(r,l);

    if (reduction > 127)
    {
        reduction /= 127.0;
        r /= reduction;
        l /= reduction;
    }
    else if (reduction < -127)
    {
        reduction /= -127.0;
        r /= reduction;
        l /= reduction;
    }
    RIGHT_MOTOR_1 = RIGHT_MOTOR_2 = -r + 127 + (r > 0);
    LEFT_MOTOR_1 = LEFT_MOTOR_2 = -l + 127 + (r > 0);
*/

}
