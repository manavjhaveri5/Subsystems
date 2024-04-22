#ifndef INTERFACEDEFIN_H
#define INTERFACEDEFIN_H
/*General Output*/
 
//Motor and Servo
const int MotorRightPWM = 11;
const int MotorLeftPWM = 10;
const int MotorLeftInput1 = 26;
const int MotorLeftInput2 = 27;
const int MotorRightInput1 = 28;
const int MotorRightInput2 = 29;

const int PWMSortUD = 6;
const int PWMBucket3 = 5;
const int PWMBucket2 = 4;
const int PWMBucket1 = 3;
const int PWMSortLR = 2;

//LCD
const int LCD_RS = 40;
const int LCD_EN = 41;
const int LCD_D4 = 42;
const int LCD_D5 = 43;
const int LCD_D6 = 44;
const int LCD_D7 = 45;

//Sensor
const int TrigFront = 22;
const int TrigRight = 24;

/*General Input*/
//Moter and servo
const int LeftMotorEncoderCA = 30;
const int LeftMotorEncoderCB = 31;
const int RightMotorEncoderCA = 32;
const int RightMotorEncoderCB = 33;

//sensor
const int EchoFront = 23;
const int EchoRight = 25;

//button
const int EmergencyStop = 35;
const int ButtonEncoder1 = 36;
const int ButtonEncoder2 = 37;
const int ButtonEncoder3 = 38;

const int BNO08X_RESET = -1;


//parameters
const float WheelRadius = 10.16 * 0.01 / 2; // 4in  
const float WheelPulseTotal = 19.2 * 7;

#endif
