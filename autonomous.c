/*******************************************************************************
*
*	TITLE:		autonomous.c
*
*	VERSION:	0.1 (Beta)
*
*	DATE:		31-Dec-2007
*
*	ATHOR:		R. Kevin Watson
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
#include "ifi_frc.h"
#include "timers.h"
#include "interrupts.h"
#include "encoder.h"
#include "serial_ports.h"
#include "ifi_code.h"
#include "autonomous.h"
#include "mort_defines.h"
#include "adc.h"
#include "gyro.h"

int x = 0;
int y = 0;
int z = 0;

int mode;


extern unsigned char height_flag;

unsigned char auto_state = 0;

int Get_Auto_Mode(void)
{
    if (rc_dig_in10 == 0)        // red 1
        return 1;
    else if (rc_dig_in11 == 0)   // red 2
        return 2;
    else if (rc_dig_in12 == 0)   // blue 1
        return 3;
    else if (rc_dig_in13 == 0)   // blue 2
        return 4;
    else
        return -1;
}

void Autonomous_Init(void)
{
    height_flag = HEIGHT_FLAG_MANUAL;
    Timer_2_Start();
    mode = Get_Auto_Mode();

    //Stop_Gyro_Bias_Calc();
        //Reset_Gyro_Angle();
}

void Autonomous(void)
{
    Service_Brakes(1);

    if (mode == 1) // red 1
        IR_Drive();
    else if (mode == 2) // red 2
        Drive_Straight();
    else if (mode == 3) // blue 1   (drives straight with a delay)
        Drive_Straight_Delay();
    else if (mode == 4) // blue 2   (no way)
        Drive_Continuous();
    else
        Reset_Outputs();

    Service_Tower();
    Mec_Drive(x,y,z);
    Service_Dashboard_Data();
}

void Autonomous_Spin(void)
{
//    Process_Gyro_Data(TRUE);
}
void Drive_Straight()
{
    if ((Timer_2_Get_Time() <= 3300))
    {
        x = 0;
        y = -127;
        z = 3;
    }
    else // 5800 //3000
    {
        Timer_2_Reset();
        mode = 1;
    }
}

void Drive_Straight_Delay()
{
    if (Timer_2_Get_Time() < 6000)
    {
        x= 0;
        y= 0;
        z= 0;
    }
    else if (Timer_2_Get_Time() >= 6000 && Timer_2_Get_Time() <= (6000 + 3300))
    {
        x = 0;
        y = -127;
        z = 3;
    }
    else // 5800 //3000
    {
        Timer_2_Reset();
        mode = 1;
    }
}

/*void Drive_Straight()
{
    int delay = 0;
    static int early_cancel_fix = 0;

    if (mode == AUTOMODE_D)
    {
        delay = 3000;
    }

    if ((ROBOCOACH_1 || ROBOCOACH_2 || ROBOCOACH_3 || ROBOCOACH_4) && mode == AUTOMODE_D)
    {
        mode = AUTOMODE_NO_D;
        early_cancel_fix = Timer_2_Get_Time();

    }
    else if ((ROBOCOACH_1 || ROBOCOACH_2 || ROBOCOACH_3 || ROBOCOACH_4) && (Timer_2_Get_Time() > (250 + delay - early_cancel_fix)))
    {
        Timer_2_Reset();
        mode = AUTOMODE_IR;
    }
    else if (Timer_2_Get_Time() < delay)
    {
        x = y = z = 0;
    }
    else if ((Timer_2_Get_Time()) < (3100 + delay - early_cancel_fix)) // 5800 //3000
    {
        x = 0;
        y = -127;
        z = 3;
    }
    else
    {
        Timer_2_Reset();
        mode = AUTOMODE_IR;
    }
}*/

void IR_Drive(void)
{
    //z < 0 is left, z > 0 is right
    //y > 0 is backwards, y < 0 is forwards
    //x < 0 is

    static char flag;
    if(ROBOCOACH_1)
    {
        x = 0;
        y = 0;
        z = -70;
        Timer_2_Reset();
    }
    else if (ROBOCOACH_2)
    {
        x = 0;
        y = -127;
        z = -3;
        Timer_2_Reset();
    }
    else if (ROBOCOACH_3)
    {
        x = 0;
        y = 0;
        z = 70;
        Timer_2_Reset();
    }
    else if (ROBOCOACH_4 || flag == 1)
    {
        if(Timer_2_Get_Time() <= 2500)
        {
            x = 0;
            y = -127;
            z = -3;
            flag = 1;
        }
        else
        {
            flag = 0;
            Timer_2_Reset();
        }
    }
    else if (Timer_2_Get_Time() > 125 && flag == 0)
    {
        x = 0;
        y = 0;
        z = 0;
        Timer_2_Reset();
    }
}

void Drive_Continuous()
{
    static unsigned char auto_state = DRIVE_FORWARD;
    static unsigned char flag;

    if(auto_state == DRIVE_FORWARD)
    {
        x = 0;
        z = 0;

        if (Timer_2_Get_Time() < 2700)
        {
            y = -127;
        }
        else
        {
            Timer_2_Reset();
            flag = 0;
            auto_state = TURN;
        }
    }
    if(auto_state == TURN_FORWARD)
    {
        x = 0;
        z = 0;

        if (Timer_2_Get_Time() < 800)
        {
            y = -127;
        }
        else
        {
            Timer_2_Reset();
            flag = 1;
            auto_state = TURN;
        }
    }
    if(auto_state == TURN)
    {
        y = 0;
        x = 0;

        if (Timer_2_Get_Time() < 300)
        {
            z = -127;
        }
        else
        {
            Timer_2_Reset();
            if(flag == 0)
                auto_state = TURN_FORWARD;
            if(flag == 1)
                auto_state = DRIVE_FORWARD;
        }
    }

    if (ROBOCOACH_1 + ROBOCOACH_2 + ROBOCOACH_3 + ROBOCOACH_4 != 0)
        mode = 1;
}
