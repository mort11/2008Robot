#include <stdio.h>
#include "mort_defines.h"
#include "tracking.h"
#include "camera.h"

extern int x,y,z;

void Track_Ball(void)
{
	int pan_error = 0;     //set all these to 0 normally until we calculate
	int tilt_error = 0;

	static signed char previous_direction = LEFT; //left because if you think about how the field is, it makes more sense. we start in corner
	static unsigned int old_camera_t_packets = 0;

	x = 0;
	y = 0;
	z = 0;

	if(camera_t_packets != old_camera_t_packets) //if all is good with the camera
	{
		old_camera_t_packets = camera_t_packets; //update so we know next time if the camera is working

		if(T_Packet_Data.confidence >= CONFIDENCE_THRESHOLD_DEFAULT)
		{

			pan_error = (int)T_Packet_Data.mx - PAN_TARGET_PIXEL_DEFAULT;  //calculate errors
			tilt_error = (int)T_Packet_Data.my - TILT_TARGET_PIXEL_DEFAULT;

			if(pan_error > PAN_ALLOWABLE_ERROR_DEFAULT) // need two if's because of previous direction
			{
				previous_direction = RIGHT; //we know it's going right
				z = pan_error * Z_GAIN; //set z
			}
			else if (pan_error < -1 * PAN_ALLOWABLE_ERROR_DEFAULT)
			{
				previous_direction = LEFT; //we know it's going left
				z = pan_error * Z_GAIN; // set x
			}
			else //on target
			{
				z = 0; //x is 0 if it's ligned up
			}

			if(tilt_error > TILT_ALLOWABLE_ERROR_DEFAULT || tilt_error < -1 * TILT_ALLOWABLE_ERROR_DEFAULT) //only need one if, no previous direction
			{
				y = tilt_error * Y_GAIN; //set y
			}
			else //on target
			{
				y = 0; // y is zero if it's ligned up
			}

		}
		else //cant see ball - searching
		{
			y = 0;
			z = SEARCH_SPEED * previous_direction;
		}
	}

	x = Limit(x, -127, 127); // check to see that nothing is overflowing
	y = Limit(y, -127, 127);
	z = Limit(z, -127, 127);

	Mec_Drive(x, y, z); // send everything to our motor function

}
