// -----Other Config---------

//#define SAFE    //Arming procedure

//***********RF24 CONFIG**************
//RF24 Pins
#define RF24Pin_0   7
#define RF24Pin_1   8
//*************************************

#define BLOCK_ON_FAIL
#define SCREAM_PIN 10

//-------PID Config----------

//#define ROLL_PID_KP  0.250
//#define ROLL_PID_KI  0.950
//#define ROLL_PID_KD  0.011
//#define ROLL_PID_MIN  -100.0
//#define ROLL_PID_MAX  100.0


#define PID_SAMPLE_TIME 1 // Time in milliseconds for PID calculation

#define PITCH_PID_KP  2.25
#define PITCH_PID_KI  9.50
#define PITCH_PID_KD  0.11
#define PITCH_PID_MIN  -300.0
#define PITCH_PID_MAX  300.0

#define YAW_PID_KP  4.0
#define YAW_PID_KI  4.0
#define YAW_PID_KD  0.01
#define YAW_PID_MIN  -100.0
#define YAW_PID_MAX  100.0

//#define ANGLEX_KP 5.0
//#define ANGLEX_KI 0.02
//#define ANGLEX_KD -0.015
//#define ANGLEX_MIN -100.0
//#define ANGLEX_MAX 100.0
//
//#define ANGLEY_KP 5.0
//#define ANGLEY_KI 0.02
//#define ANGLEY_KD -0.015
//#define ANGLEY_MIN -100.0
//#define ANGLEY_MAX 100.0

//-------------------------
//..........................

//----Debug Config---------

//#define DEBUG_SAMPLE_COUNT 100
//#define DEBUG
//#define DEBUG_
//#define DEBUG_ANGLES
//#define DEBUG_GYRO
//#define DEBUG_PID
//#define DEBUG_RX
//#define DEBUG_SERIAL_CHART

//-------------------------


//*********Sensor Config****************


#define ACC_X_OFFSET  2546
#define ACC_Y_OFFSET  -295
#define ACC_Z_OFFSET  964


#define GYRO_X_OFFSET  87
#define GYRO_Y_OFFSET  -21
#define GYRO_Z_OFFSET  34



//#define SPLIT 0.99 //COMP-filter nr
#define RadToDeg 180.0/PI
#define DegToRad PI/180.0
//#define  ACC_HPF_NR  98  //high pass filter nr
//#define  GYRO_HPF_NR 0   //high pass filter nr
//#define  GYRO_MAF_NR  2  //Moving average filter nr
//*************************************


//***********RX ANGLE CONFIG*****************
#define RX_MAX_ANGLE 20.0
//#define RX_PRECISION 40


//***********MOTOR CONFIG**************

//Motor PWM Levels
#define MOTOR_ZERO_LEVEL  1000
#define MOTOR_ARM_LEVEL  1300
#define MOTOR_MAX_LEVEL  2000

//Motor Pins
#define MOTOR0  3
#define MOTOR1  5
#define MOTOR2  6
#define MOTOR3  9


//*************************************

// Motors 3 5 6 9 | RF24 7 8 | RX 2 4 14 15 16 17 | 


//.......RX PINS............
#define RX_PIN_ROLL  15     //PCINT2
#define RX_PIN_PITCH  14    //PCINT3
#define RX_PIN_YAW  16      //PCINT1
#define RX_PIN_THROTTLE  4  //INT6
#define RX_PIN_AUX1  2      //PCINT4
#define RX_PIN_AUX2  17      //INT2
//........................
