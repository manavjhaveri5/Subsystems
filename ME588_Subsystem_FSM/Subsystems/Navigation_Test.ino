#include <Adafruit_BNO08x.h> // IMU
#include <Adafruit_TCS34725.h> // COLOR SENSOR
#include "ArduPID.h" 
#include <Encoder.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>

//IMU variables
#define BNO08X_CS 7
#define BNO08X_INT 6
#define BNO08X_RESET 5
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

//Ultrasonic Sensor mounted on the side
#define trigPin1 8
#define echoPin1 9

//Ultrasonic Sensor mounted on the front
#define trigPin2 1
#define echoPin2 2

//pid control variables
ArduPID ControllerL;
ArduPID ControllerR;
double setpoint_L = 0;
double setpoint_R = 0;
double p = 10;
double i = 1;
double d = 0.5;
double motor_L_I=0; //left motor input
double motor_L_O=0; //left motor output
double motor_R_I=0; //right motor input
double motor_R_O=0; //left motor output

//Encoder variables
Encoder Enc_L(3, 4);
Encoder Enc_R(5, 6);

//motor pins
int motor_L_pin = 11;
int motor_R_pin = 12;

//state, input & output variables
int state = 2; 
int TPR = 0; //Turning point reached
int LTM = 0; // Left turns made

int DFP = 0; //Driving Forward Pattern
int TLP = 0; //Turning Left Pattern


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  //Driving setup
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
  pinMode(motor_L_pin, OUTPUT);
  pinMode(motor_R_pin, OUTPUT);

  SPI.begin();

}

void loop() {

  // put your main code here, to run repeatedly:
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
  }
  if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch(state){
    case 2: //Driving Forward
    {
      DFP = 1;
      TLP = 0;
      //calling driving forward
      double yawv = getyaw();
      double ulr = getulr(trigPin2, echoPin2);
      drivingforward(yawv,ulr);

      if (ulr <= 7)TPR = 1;
      else if (TPR ==1)state=3;
      else state = 2;
    } break;
    case 3: //Turning Left
    {
      DFP = 0;
      TLP = 1;
      //calling turning left
      turningleft();
      LTM += 1; //left turns made incremented
      TPR = 0;
      //if(GS == 0) state = 0;
      if(TPR == 1) state = 3;
      //else if(PAC == 0) state = 2;
      //else state = 1;
    } break;
  }
  
  Serial.print("state: ");
  Serial.println(state);
  Serial.println(LTM);
  //Serial.println(yawv);
 
  delay(1000);
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
      break;
    }
    case 1: //towards right
    {
      setpoint_L = 21;
      setpoint_R = 7;
      break;
    }
    case 2: //towards left
    {
      setpoint_L = 7;
      setpoint_R = 21;
      break;
    }
  }
  motor_L_I = speedMeasure(Enc_L)*10.0;
  motor_R_I = speedMeasure(Enc_R)*10.0;
  ControllerL.compute();
  ControllerR.compute();
  courseCorrection(); 

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
  delay(50); //need adjustment
  long newPosition = Enc.read();
  long speedE = newPosition - oldPosition;
  return speedE;
}
void courseCorrection(){
  double desiredYaw = 90; 
  double yawv = getyaw(); 
  double error;  
  double yawThreshold = 5; // 5 degrees deviation
  int correctionFactor = 3 ;

  switch (LTM % 4) {
    case 0:
      desiredYaw = 90;
      break;
    case 1:
      desiredYaw = 180;
      break;
    case 2:
      desiredYaw = 270;
      break;
    case 3:
      desiredYaw = 360;
      break;
  }
  error = yawv - desiredYaw;
  if (error > yawThreshold) {
    // bot veering to left, decrease setpoint for the left motor and increase it for the right motor
    setpoint_L += correctionFactor;
    setpoint_R -= correctionFactor;
  } else if (error <= -yawThreshold) {
    // bot is veering to right, increase setpoint for the left motor and decrease it for the right motor
    setpoint_L -= correctionFactor;
    setpoint_R += correctionFactor;
  }

  *(ControllerL.setpoint) = setpoint_L;
  *(ControllerR.setpoint) = setpoint_R;
}

long getulr(int trigPin, int echoPin) {
  long duration, distance;

  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond pulse to trigger pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  duration = pulseIn(echoPin, HIGH);

  // Calculate distance in cm
  distance = (duration * 0.034 / 2) / 2.54;

  return distance;
}

