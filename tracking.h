/*******************************************************************************
*
*	TITLE:		tracking.h
*
*	VERSION:	0.2 (Beta)
*
*	DATE:		21-Feb-2006
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This is the "streamlined" version of tracking.h
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or elsewhere
*				without permission. Thanks.
*
*				Copyright ©2005-2007 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	01-Jan-2006  0.1  RKW - Original code.
*	21-Feb-2006  0.2  RKW - Provided two new functions to set the pan and
*	                  tilt servo position. This was done to provide a level
*	                  of indirection so that the servos could be commanded
*	                  from the robot controller or the CMUcam2.
*	                  RKW - Fixed bug in search initialization code where
*	                  temp_pan_servo was initialized to zero instead of
*	                  Tracking_Config_Data.Pan_Min_PWM.
*	                  RKW - Altered tracking algorithm to use the t-packet
*	                  confidence value to determine whether the code should
*	                  track or search.
*	                  RKW - Added Get_Tracking_State() function, which can
*	                  be used to determine if the camera is on target.
*
*******************************************************************************/
#ifndef _TRACKING_H
#define _TRACKING_H

// By default, PWM output one is used for the pan servo.
// Change it to another value if you'd like to use PWM
// output one for another purpose.
//#define PAN_SERVO pwm01

// By default, PWM output two is used for the tilt servo.
// Change it to another value if you'd like to use PWM
// output two for another purpose.
//#define TILT_SERVO pwm02


// This defines the minimum t-packet confidence value needed
// before the tracking software will transition from search
// mode to tracking mode.
#define CONFIDENCE_THRESHOLD_DEFAULT 20


// These two values define the image pixel that we're
// going to try to keep the tracked object on. By default
// the center of the image is used.
#define PAN_TARGET_PIXEL_DEFAULT 94 //79
#define TILT_TARGET_PIXEL_DEFAULT 100

// These values define how much error, in pixels, is
// allowable when trying to keep the center of the tracked
// object on the center pixel of the camera's imager. Too
// high a value and your pointing accuracy will suffer, too
// low and your camera may oscillate because the servos
// don't have enough pointing resolution to get the center
// of the tracked object into the square/rectangle defined
// by these values
#define PAN_ALLOWABLE_ERROR_DEFAULT 1
#define TILT_ALLOWABLE_ERROR_DEFAULT 6

// parameters for CMUcam2 with OV7620 camera module
#define IMAGE_WIDTH 159
#define IMAGE_HEIGHT 239

// Tracking_State values
#define STATE_SEARCHING 0
#define STATE_TARGET_IN_VIEW 1
#define STATE_PAN_ON_TARGET 2
#define STATE_TILT_ON_TARGET 4

// Get_Tracking_State() return values
#define SEARCHING 0
#define TARGET_IN_VIEW 1
#define CAMERA_ON_TARGET 2

// Constants
#define RIGHT           1
#define LEFT            -1

#define Z_GAIN          1
#define	Y_GAIN          1

#define SEARCH_SPEED    30

// function prototypes
void Track_Ball(void);

#endif
