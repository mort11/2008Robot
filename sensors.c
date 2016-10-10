#include <adc.h>
#include "adc.h"
#include "ifi_frc.h"
#include "sensors.h"
#include "serial_ports.h"
#include "gyro.h"

/** TODO: Determine Which input is which (and what)
        We should have (at least) an accelerometer, a gyro, and a temperature sensor.
        All need conversion into real world units.

CHIP_1 = Gyro
    _T = Twist
    _R = Relative Temperature
CHIP_2 = Accelerometer
    _X
    _Y
**/
/*
The “T” output varies by 12.5 millivolts per degrees per second of rotation. No rotation
will give a 2.5-volt output. The current draw is 8mA. Accelerating clockwise will provide
a voltage above 2.5 volts. The 3 dB bandwidth of the Yaw Rate Gyro is set at 40 Hz.

The “R” output varies by 8.4 millivolts per degree Kelvin. A temperature of 298 K (about
25 Celsius) will give an analog output of 2.5 volts. Higher temperatures will provide
higher voltages. Cooler temperatures will provide lower voltages.
*/

/*
TODO 1-31-08
    Gyro Angle always 0 (unchanging) while actual Gyro analog value does vary
    Temperature returns a value, unsure of correctness
    ACCEL_X is continualy 0
    ACCEL_Y is continualy 772
        Both are unchanging.

    Initialize_ADC() causes a code error continualy, currently commented out in main.c/Initialization()
    ADC of IR sensors works without ADC initialized.
*/

/**
Get the current relative temperature from the given analog channel
**/
unsigned int Get_Temp( unsigned char temp_channel )
{
    //return Convert_ADC_to_mV(Get_ADC_Result(temp_channel)) / 8.4;
    return Convert_ADC_to_mV(Get_Analog_Value(temp_channel)) / 8.4;
}

/**

**/
int Get_Accel( unsigned char accel_channel )
{
    //return Convert_ADC_to_mV(Get_ADC_Result(accel_channel));
    return Convert_ADC_to_mV(Get_Analog_Value(accel_channel));
}

/**
Returns
    Degrees Turning Per Second
**/
int Get_Spin( unsigned char gyro_channel )
{
    //return Convert_ADC_to_mV(Get_ADC_Result(gyro_channel)) / 12.5;
    return Convert_ADC_to_mV(Get_Analog_Value(gyro_channel)) / 12.5;
}

/**
TODO: Currently Returns mV, needs to be converted to mm
**/
unsigned int Get_IR_Distance( unsigned char ir_channel )
{
    //return Convert_ADC_to_mV(Get_ADC_Result(ir_channel));
    return Convert_ADC_to_mV(Get_Analog_Value(ir_channel));
}

void Chip_Diag( void )
{
    int chips[4] = { Get_Spin(GYRO_CHANNEL), Get_Temp(TEMP_CHANNEL), Get_Accel(ACCEL_X_CHANNEL), Get_Accel(ACCEL_Y_CHANNEL) };
    printf("Chips :");
    printf("GYRO: %d, TEMP: %d, X_accel: %d, Y_accel: %d\r\n",chips[0],chips[1],chips[2],chips[3]);

    printf("Gyro Angle : %d ;", Get_Gyro_Angle());
}

/**
TODO : Check high and lows compared to averages over 100 samples

100 Sample Averages:
    IR1 : Left Side :
        108.5 cm    -> 62
        89 cm       -> 73
        62.5 cm     -> 100
        31 cm       -> 185
    IR2 : Front :
        278 cm      -> 45 (~)
        97 cm       -> 77
        78.5 cm     -> 90
        56 cm       -> 120
        32 cm       -> 184
*/
void IR_Diag(void)
{
    static unsigned char ir_loop_count;
    static int ir1, ir2;
    static int ir1_h, ir2_h;
    static int ir1_l = 2000;
    static int ir2_l = 2000;
    int ir_temp;

    if( ir_loop_count < 100)
    {

        ir_temp = Get_Analog_Value(IR_FRONT);
        if(ir_temp > ir1_h)
            ir1_h = ir_temp;
        else if (ir_temp < ir1_l)
            ir1_l = ir_temp;
        ir1 += ir_temp;

        ir_temp = Get_Analog_Value(IR_SIDE);
        if(ir_temp > ir2_h)
            ir2_h = ir_temp;
        else if (ir_temp < ir2_l)
            ir2_l = ir_temp;
        ir2 += ir_temp;

        ++ir_loop_count;
    }
    else
    {
        ir1 /= 100.0;
        ir2 /= 100.0;
        printf("IR Sensor: \r\n\
                Left Side: Ave: %d, High: %d, Low: %d; \r\n\
                Front: Ave: %d, High: %d, Low: %d; \r\n", \
                ir1, ir1_h, ir1_l, ir2, ir2_h, ir2_l);
        ir1 = ir2 = 0;
        ir1_h = ir2_h = 0;
        ir1_l = ir2_l = 2000;
        ir_loop_count = 0;
    }
}
