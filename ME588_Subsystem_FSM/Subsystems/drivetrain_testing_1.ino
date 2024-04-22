//#include <Servo.h>
#include <Adafruit_BNO08x.h>
#include "ArduPID.h"
#include "interface_definition.hpp"
#include <RunningMedian.h>
#include <Wire.h>


//imu init
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

//pid parameters
ArduPID ControllerL;
ArduPID ControllerR;
double setpoint_L = 0;
double setpoint_R = 0;
double p = 4;
double i = 1.2;
double d = 0.5;
double motor_L_I=0;
double motor_L_O=0;
double motor_R_I=0;
double motor_R_O=0;

//encoder
volatile int Enc_L_count;
volatile int Enc_R_count;


////delivering global variables
//Servo ServoLeftBucket;
//Servo ServoMidBucket;
//Servo ServoRightBucket;
//
////sorting
//Servo ServoSortUD;
//Servo ServoSortLR;

//state, input & output variables
int state = 2; 
int GS = 1; //Game starting
int POC = 0; //Pollen counting (how many pollen sorted)
int TPR = 0; //Turning point reached
int PD = 0; //Plant detected
int PAC = 0; //Plant counting
int LTM = 0; // Left turns made

//ultrasonic sensors
RunningMedian ulr_F = RunningMedian(3);
RunningMedian ulr_R = RunningMedian(3);
/*
double siny;
double cosy;  
double yawv;
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("looping");
  while (!Serial) delay(20);
  bno08x.begin_I2C();
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
  
  //encoder interrupt pin definition
  attachInterrupt(digitalPinToInterrupt(RightMotorEncoderCA), EncoderCount_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightMotorEncoderCB), EncoderCount_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LeftMotorEncoderCA), EncoderCount_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LeftMotorEncoderCB), EncoderCount_L, CHANGE);

  //pid controller
  ControllerL.begin(&motor_L_I, &motor_L_O, &setpoint_L, p, i, d);
  ControllerL.setOutputLimits(0, 255);
  ControllerL.setBias(40.0);
  ControllerL.setWindUpLimits(-10, 10); 
  ControllerL.start();
  ControllerR.begin(&motor_R_I, &motor_R_O, &setpoint_R, p, i, d);
  ControllerR.setOutputLimits(0, 255);
  ControllerR.setBias(40.0);
  ControllerR.setWindUpLimits(-10, 10); 
  ControllerR.start();

//  //delivering setup
//  ServoLeftBucket.attach(PWMBucket1);
//  ServoMidBucket.attach(PWMBucket2);
//  ServoRightBucket.attach(PWMBucket3);
//  //sorting setup
//  ServoSortUD.attach(PWMSortUD);
//  ServoSortLR.attach(PWMSortLR);
  
  pinMode(MotorLeftInput1, OUTPUT);
  pinMode(MotorLeftInput2, OUTPUT);
  pinMode(MotorRightInput1, OUTPUT);
  pinMode(MotorRightInput2, OUTPUT);
  //dc motors initial
  digitalWrite(MotorLeftInput1,0);
  digitalWrite(MotorRightInput1,0);
  digitalWrite(MotorLeftInput2,0);
  digitalWrite(MotorRightInput2,0);
  analogWrite(MotorLeftPWM,0);
  analogWrite(MotorRightPWM,0);

  //
  
}

void loop() {
  // imu connection:
  if (bno08x.wasReset()) {
    Serial.println("sensor was reset ");
  }
  if (! bno08x.getSensorEvent(&sensorValue)) {
    Serial.println("Error");
    return;
  }
  //double yawv = getyaw();
  //Serial.print("z axis rotation:");
  //Serial.println(yawv);
  //Serial.println("1");
  //turningleft();
  //Serial.println("2");
  //encoder test//one round ~540, pwm40 ~444
//  int sp=speedMeasure("L");
//  Serial.print("left: ");
//  Serial.println(sp);
//  sp=speedMeasure("R");
//  Serial.print("right: ");
//  Serial.println(sp);
  //analogWrite(MotorLeftInput1,60);
  //analogWrite(MotorLeftInput2,0);
  //analogWrite(MotorRightInput1,0);
  //analogWrite(MotorRightInput2,0);
  //drivingforward(yawv,6.5);
  //Serial.print("right: ");
  //Serial.println(setpoint_R);
  //Serial.print(", ");
  //Serial.println(motor_R_O);
  //Serial.print("left: ");
  //Serial.println(setpoint_L);
  //Serial.print(", ");
  //Serial.println(motor_L_O);
  //encoder checked
  //double speed=speedMeasure(Enc_L);
  //Serial.println(speed);
  //delay(100);
  switch(state){
    case 0: //initial stand-by state
    {
      if(GS == 1) state = 1;
      else state = 0;
    } break;
    
    case 1: //Loading & Sorting
    {
//      int POC = 0;  // Plant Count
//
//      while (totalcount <100){
//        uint8_t colorDetected = detectColor();
//        totalcount = countBlue+countGreen+countRed;
//        rotateChute(colorDetected);
//        if(totalcount == 100){
//          POC = 1;
//        }
//      } 
//      if(GS == 0) state = 0;
//      else if(POC == 1) state = 2;
//      else state = 1;
    } break;


    case 2: //Driving Forward
    {
      //calling driving forward
      double yawv = getyaw();
      double ulr_F_c = getulr(TrigFront,EchoFront);
      double ulr_R_c = getulr(TrigRight,EchoRight);
      ulr_F.add(ulr_F_c);
      ulr_R.add(ulr_R_c);
      
      if (ulr_F.getMedian() <= 7){
        TPR == 1;
      }
//      if (ulr_R.getMedian() >= 10){
//        PD == 1;
//      }
      
      if(GS == 0) state = 0;
      //else if(PD == 1) state = 4;//PAC=PAC+1;
      else if(TPR == 1) state = 3;
      else {
        drivingforward(yawv,ulr_R.getMedian(),LTM);
        state = 2;
      }
    } break;
    case 3: //Turning Left
    {
      //calling turning left
      turningleft();
      LTM += 1; 
      TPR = 0;
      if(GS == 0) state = 0;
      //else if(TPR == 1) state = 3;
      else if(LTM < 4) state = 2;
      else state = 1;
    } break;
//    case 4: //Delivering
//    {
//      //calling delivering 
//      delivering(color[],i) //i is plant count
//      PAC += 1; 
//      if(GS == 0) state = 0;
//      else if(PD == 1) state = 4;
//      else state = 2;
//    } break;
  }

}
long getulr(int trigPin, int echoPin){
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034/2.0/2.54; //in inches
  return distance;
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

void drivingforward(double yawv, double ulr, int LTM_DF){
  int statedf = 0;
  if (LTM_DF == 1) yawv = abs(yawv) - 90;
  else if (LTM_DF == 2) yawv = abs(yawv) - 180;
  else if (LTM_DF == 3) yawv = abs(yawv) - 90;
  if(ulr>=7) statedf = 1; //ultrasonic reading >= 7inches
  else if(ulr<=6) statedf = 2;
  else if(yawv>=5) statedf = 1;
  else if (yawv<=-5) statedf = 2;
  else statedf=0;
  Serial.print("state: ");
  Serial.println(statedf);
  switch(statedf) {
    case 0:
    {
      setpoint_L = 40;//encoder count (ppr=7)
      setpoint_R = 40;
    }break;
    case 1: //towards right
    {
      setpoint_L = 50;
      setpoint_R = 40;
    }break;
    case 2: //towards left
    {
      setpoint_L = 40;
      setpoint_R = 50;
    }break;
  }
  motor_L_I = speedMeasure("L")*40.0/444.0;//map directly to pwm
  motor_R_I = speedMeasure("R")*40.0/444.0;
  ControllerL.compute();
  ControllerR.compute();
  analogWrite(MotorLeftPWM,motor_L_O);
  analogWrite(MotorRightPWM,motor_R_O);
  digitalWrite(MotorLeftInput1,HIGH);
  digitalWrite(MotorRightInput1,HIGH);
  digitalWrite(MotorLeftInput2,LOW);
  digitalWrite(MotorRightInput2,LOW);
  return;
}
void turningleft(){
  //for robustness could have 4 cases
  double init_yawv_TL = getyaw();
  double yawv_TL = init_yawv_TL;
  Serial.print("initial: ");
  Serial.println(init_yawv_TL);
  while(abs(abs(yawv_TL)-abs(init_yawv_TL))<90){
    setpoint_L = 40;
    setpoint_R = 40;
    motor_L_I = speedMeasure("L")*40.0/444.0;
    motor_R_I = speedMeasure("R")*40.0/444.0;
    ControllerL.compute();
    ControllerR.compute();
    analogWrite(MotorLeftPWM,motor_L_O);
    analogWrite(MotorRightPWM,motor_R_O);
    digitalWrite(MotorLeftInput1,LOW);
    digitalWrite(MotorRightInput1,HIGH);
    digitalWrite(MotorLeftInput2,HIGH);
    digitalWrite(MotorRightInput2,LOW);
    
    Serial.print("z axis rotation:");
    yawv_TL = getyaw();
    Serial.println(yawv_TL);
  }
  return;
}
long speedMeasure(const char* cho){//perhaps another board ?
  long speedE;
  if (cho == "R"){
    long oldPosition = Enc_R_count;
    delay(54);
    long newPosition = Enc_R_count;
    speedE = newPosition - oldPosition;
  }
  else{
    long oldPosition = Enc_L_count;
    delay(54);
    long newPosition = Enc_L_count;
    speedE = newPosition - oldPosition;
  }
  return speedE*18.519;
}

////delivering
//void delivering(char color[],int i){
//  int lift = 18;
//  switch(color[i]){
//    case 'R':
//    {
//      while(lift<150){
//        ServoLeftBucket.write(lift);
//        delay(20);
//        lift=lift+1;
//      } 
//      delay(2500);
//      while(lift>18){
//        ServoLeftBucket.write(lift);
//        delay(20);
//        lift=lift+1;
//      } 
//    }
//    case 'B':
//    {
//      while(lift<150){
//        ServoMidBucket.write(lift);
//        delay(20);
//        lift=lift+1;
//      } 
//      delay(2500);
//      while(lift>18){
//        ServoMidBucket.write(lift);
//        delay(20);
//        lift=lift+1;
//      } 
//    }
//    case 'G':
//    {
//      while(lift<150){
//        ServoRightBucket.write(lift);
//        delay(20);
//        lift=lift+1;
//      } 
//      delay(2500);
//      while(lift>18){
//        ServoRightBucket.write(lift);
//        delay(20);
//        lift=lift+1;
//      } 
//    }
//  }
//  return;
//}
void EncoderCount_L(){
  Enc_L_count += 1;
  return;
}
void EncoderCount_R(){
  Enc_R_count += 1;
  return;
}
