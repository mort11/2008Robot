// Functions of sensors.c
// Handle the input of data from various sensors on the RC
#ifndef _sensors_h
#define _sensors_h

/** Defines **/
#define ROBOCOACH_1     rc_dig_in10
#define ROBOCOACH_2     rc_dig_in11
#define ROBOCOACH_3     rc_dig_in12
#define ROBOCOACH_4     rc_dig_in13

#define IR_LIMIT        rc_dig_in01
#define IR_SIDE         rc_ana_in10
#define IR_FRONT        rc_ana_in11

#define GYRO_CHANNEL    rc_ana_in01
#define TEMP_CHANNEL    rc_ana_in02
#define ACCEL_X_CHANNEL rc_ana_in03
#define ACCEL_Y_CHANNEL rc_ana_in04

/**** Functions ****/
/** Diagnostics **/
void Chip_Diag( void );
void IR_Diag( void );

/** Get Meaningful Inputs **/
int Get_Accel(unsigned char);
unsigned int Get_Temp(unsigned char);
int Get_Spin(unsigned char);

/** Track Continuous Changes **/
void Track_Gyro_Angle( void );
int Get_Gyro_Angle_MORT( void );

void Track_X_Distance ( void );
int Get_X_Distance( void );

void Track_Y_Distance ( void );
int Get_Y_Distance( void );

#endif
