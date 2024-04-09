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

ArduPID ControllerL;
ArduPID ControllerR;
double setpoint_L = 0;
double setpoint_R = 0;
double p = 10;
double i = 1;
double d = 0.5;
double motor_L_I=0;
double motor_L_O=0;
double motor_R_I=0;
double motor_R_O=0;

Encoder Enc_L(3, 4);
Encoder Enc_R(5, 6);

//delivering global variables
#include <Servo.h>
Servo servo_1;
Servo servo_2;
Servo servo_3;
int servo_1_pin = 10;
int servo_2_pin = 11;
int servo_3_pin = 12;

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

  ControllerL.begin(&motor_L_I, &motor_L_O, &setpoint_L, p, i, d);
  ControllerL.setOutputLimits(0, 255);
  ControllerL.setBias(255.0 / 2.0);
  ControllerL.setWindUpLimits(-10, 10); 
  ControllerL.start();
  
  ControllerR.begin(&motor_R_I, &motor_R_O, &setpoint_R, p, i, d);
  ControllerR.setOutputLimits(0, 255);
  ControllerR.setBias(0);
  ControllerR.setWindUpLimits(-10, 10); 
  ControllerR.start();

  //delivering setup
  servo_1.attach(servo_1_pin);
  servo_2.attach(servo_2_pin);
  servo_3.attach(servo_3_pin);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
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
long getulr(int trigPin, int echoPin){
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}
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
  motor_L_I = speedMeasure(Enc_L)*10.0;
  motor_R_I = speedMeasure(Enc_R)*10.0;
  ControllerL.compute();
  ControllerR.compute();
  return;
}
void turningleft(){
  //for robustness could have 4 cases
  double init_yawv = getyaw();
  double yawv = init_yawv;
  while(abs(abs(yawv)-abs(init_yawv))<90){
    setpoint_L = 7;
    setpoint_R = 21;
    motor_L_I = speedMeasure(Enc_L)*10.0;
    motor_R_I = speedMeasure(Enc_R)*10.0;
    ControllerL.compute();
    ControllerR.compute();
    yawv = getyaw();
  }
  
}
long speedMeasure(Encoder &Enc){//perhaps another board ?
  long oldPosition = Enc.read();
  delay(100); //need adjustment
  long newPosition = Enc.read();
  long speedE = newPosition - oldPosition;
  return speedE;
}

//delivering
void delivering(char color[],int i){
  switch(color[i]){
    case 'R':
    {
      servo_1.write(200);
    }
    case 'G':
    {
      servo_2.write(200);
    }
    case 'B':
    {
      servo_3.write(200);
    }
  }
}
