#include <Adafruit_BNO08x.h>
#include "ArduPID.h"
#include <Encoder.h>
// For SPI mode, we need a CS pin
#define BNO08X_CS 7
#define BNO08X_INT 6

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET 5

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

ArduPID myControllerL;
ArduPID myControllerR;
double setpoint_L = 0;
double setpoint_R = 0;
double p = 10;
double i = 1;
double d = 0.5;
double motor_L_I=0;
double motor_L_O=0;
double motor_R_I=0;
double motor_R_O=0;

Encoder myEnc_L(3, 4);
Encoder myEnc_R(5, 6);
/*
double siny;
double cosy;  
double yawv;
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10);
  bno08x.begin_I2C();
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);

  myControllerL.begin(&motor_L_I, &motor_L_O, &setpoint_L, p, i, d);
  myControllerL.setOutputLimits(0, 255);
  myControllerL.setBias(255.0 / 2.0);
  myControllerL.setWindUpLimits(-10, 10); 
  myControllerL.start();
  
  myControllerR.begin(&motor_R_I, &motor_R_O, &setpoint_R, p, i, d);
  myControllerR.setOutputLimits(0, 255);
  myControllerR.setBias(0);
  myControllerR.setWindUpLimits(-10, 10); 
  myControllerR.start();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
  }
  if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  double yawv = getyaw();
  
}

double getyaw(){
  double r=sensorValue.un.gameRotationVector.real;
  double i=sensorValue.un.gameRotationVector.i;
  double j=sensorValue.un.gameRotationVector.j;
  double k=sensorValue.un.gameRotationVector.k;
  double siny = +2.0 * (r * k + i * j);
  double cosy = +1.0 - 2.0 * (j * j + k * k);  
  double yawv = atan2(siny, cosy)*180.0/M_PI;
  return yawv;
}

void drivingforward(double yawv, double ulr){
  int statedf = 0;

  if(ulr>=7) statedf = 1; //ultrasonic reading >= 7inches
  else if(ulr<=6) statedf = 2;
  else if(yawv>=10) statedf = 1;
  else if (yawv<=-10) statedf = 2;

  switch(statedf) {
    case 0:
    {
      setpoint_L = 21;//encoder count (ppr=7)
      setpoint_R = 21;
    }
    case 1: //towards right
    {
      setpoint_L = 21;
      setpoint_R = 7;
    }
    case 2: //towards left
    {
      setpoint_L = 7;
      setpoint_R = 21;
    }
  }
  motor_L_I = speedMeasure(myEnc_L)*10.0;
  motor_R_I = speedMeasure(myEnc_R)*10.0;
  myControllerL.compute();
  myControllerR.compute();
  return;
}
void turningleft(){
  //for robustness could have 4 cases
  double init_yawv = getyaw();
  double yawv = init_yawv;
  while(abs(abs(yawv)-abs(init_yawv))<90){
    setpoint_L = 7;
    setpoint_R = 21;
    motor_L_I = speedMeasure(myEnc_L)*10.0;
    motor_R_I = speedMeasure(myEnc_R)*10.0;
    myControllerL.compute();
    myControllerR.compute();
    yawv = getyaw();
  }
  
}
long speedMeasure(Encoder &myEnc){//perhaps another board ?
  long oldPosition = myEnc.read();
  delay(100); //need adjustment
  long newPosition = myEnc.read();
  long speedE = newPosition - oldPosition;
  return speedE;
}