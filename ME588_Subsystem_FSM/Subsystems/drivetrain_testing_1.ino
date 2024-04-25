#include <Servo.h>
#include <Adafruit_BNO08x.h>
#include "ArduPID.h"
#include "interface_definition.hpp"
#include <RunningMedian.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

char plantcolor[4] = "RGB";

//imu init
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

//pid parameters
ArduPID ControllerL;
ArduPID ControllerR;
double setpoint_L = 0;
double setpoint_R = 0;
double p = 1.5;
double i = 1;
double d = 1;
double motor_L_I=0;
double motor_L_O=0;
double motor_R_I=0;
double motor_R_O=0;
double yawv=0;

//encoder
volatile int Enc_L_count;
volatile int Enc_R_count;


//delivering global variables
Servo ServoLeftBucket;
Servo ServoMidBucket;
Servo ServoRightBucket;

//sorting
Servo ServoSortUD;
Servo ServoSortLR;

//state, input & output variables
int state = 2; 
int GS = 1; //Game starting
int POC = 0; //Pollen counting (how many pollen sorted)
int TPR = 0; //Turning point reached
int PD = 0; //Plant detected
int PAC = 0; //Plant counting
int LTM = 0; // Left turns made
int start_time = 0;
int last_turn = 0;
int last_dump = 0;
int current_time;

//ultrasonic sensors
RunningMedian ulr_F = RunningMedian(3);
RunningMedian ulr_R = RunningMedian(3);
RunningMedian ulr_R_2 = RunningMedian(3);
/*
double siny;
double cosy;  
double yawv;
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Serial.println("looping");
  bno08x.begin_I2C((0x4B),&Wire1,0);
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
  delay(500);
  while (! bno08x.getSensorEvent(&sensorValue)) {
    //Serial.println("Error");
  }
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
  lcd.init();
  lcd.backlight();
  

  //delivering setup
  ServoLeftBucket.attach(PWMBucket1);
  ServoMidBucket.attach(PWMBucket2);
  ServoRightBucket.attach(PWMBucket3);
  //sorting setup
  ServoSortUD.attach(PWMSortUD);
  ServoSortLR.attach(PWMSortLR);
  
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
  delay(1000);
  for (int k = 1; k<100; k++){
    double ulr_F_c = getulr(TrigFront,EchoFront);
    double ulr_R_c = getulr(TrigRight,EchoRight);
    double ulr_R_c_2 = getulr(TrigRight2,EchoRight2);
    ulr_R.add(ulr_R_c);
    ulr_F.add(ulr_F_c);
    ulr_R_2.add(ulr_R_c_2);
  }
  //Serial.println(ulr_F.getLowest());
  
  //
  //start_time = millis();
  
}

void loop() {
  // imu connection:
  if (bno08x.wasReset()) {
    //Serial.println("sensor was reset ");
  }
  if (! bno08x.getSensorEvent(&sensorValue)) {
    //Serial.println("Error");
    return;
  }
  
  //Ulr and IMU reading test
  //yawv = getyaw();
  
//  double ulr = getulr(TrigRight,EchoRight);
//  Serial.print("Right Ulr Sensor: ");
//  Serial.println(ulr);
  //Serial.println("1");

  //drive logic test
  //turningleft();
  //drivingforward(yawv, 6.5, 0);
  
  //encoder test//one round ~540, pwm40 ~444
  //int sp=speedMeasure("L");
  //Serial.print("left: ");
  //Serial.println(sp);
  //sp=speedMeasure("R");
  //Serial.print("right: ");
  //Serial.println(sp);

  //motor direction test
  //analogWrite(MotorLeftInput1,60);
  //analogWrite(MotorLeftInput2,0);
  //analogWrite(MotorRightInput1,0);
  //analogWrite(MotorRightInput2,0);
  
  //pid test with encoder
  //Serial.print("right: ");
  //Serial.println(setpoint_R);
  //Serial.print(", ");
  //Serial.println(motor_R_O);
  //Serial.print("left: ");
  //Serial.println(setpoint_L);
  //Serial.print(", ");
  //Serial.println(motor_L_O);
//  double speed=speedMeasure(Enc_L);
//  Serial.println(speed);
  //delay(100);
//  
  switch(state){
    case 0: //initial stand-by state
    {
      if(GS == 1) state = 1;
      else state = 0;
      analogWrite(MotorLeftPWM,0);
      analogWrite(MotorRightPWM,0);
      lcd.setCursor(0, 0);
      lcd.print("State 0");
    } break;
    
    case 1: //Loading & Sorting
    {
//      int POC = 0;  // Plant Count
        lcd.setCursor(0, 0);
        lcd.print("State 1");

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
//      else if(POC == 1) state = 2; start_time = millis();
//      else state = 1;
      analogWrite(MotorLeftPWM,0);
      analogWrite(MotorRightPWM,0);
    } break;

    case 2: //Driving Forward
    {
      lcd.setCursor(0, 0);
      lcd.print("State 2");
      yawv = getyaw();
      Serial.print("z axis rotation:");
      Serial.println(yawv);
      double ulr_F_c = getulr(TrigFront,EchoFront);
      double ulr_R_c = getulr(TrigRight,EchoRight);
      double ulr_R_c_2 = getulr(TrigRight2,EchoRight2);
      ulr_F.add(ulr_F_c);
      ulr_R.add(ulr_R_c);
      ulr_R_2.add(ulr_R_c_2);
      //Serial.println("Forward");
      //Serial.println(ulr_R_c);
      double frontDis = ulr_F.getAverage();
      //Serial.println(frontDis);
      current_time = millis();
      if ((frontDis <= 6.5) && (current_time-last_turn>1000)){
        TPR = 1;
        Serial.print("Turning point: ");
        Serial.println(frontDis);
      }
      //lcd.setCursor(0, 0);
      Serial.print("Ultrasonic middle reading diff: ");
      Serial.println(ulr_R_c_2-ulr_R_2.getLowest());
      Serial.print("Current middle reading: ");
      Serial.println(ulr_R_c_2);
      Serial.print("Front reading: ");
      Serial.println(ulr_R_c);
      if (((ulr_R_c_2-ulr_R_2.getLowest()) >= 5)&&(ulr_R_c_2<30)){//&&(ulr_R_c<90)
        //delay(50);
        //ulr_R_c = getulr(TrigRight,EchoRight);
        if (((ulr_R_c) >= 10)&&(ulr_R_c<30)){
          PD = 1;
        }
      }
      
      if(GS == 0) state = 0;
      else if((PD == 1)&& (current_time-last_dump>5000)) {
        drivingforward(yawv,ulr_R.getLowest(),LTM);
        delay(500);
        state = 4;//PAC=PAC+1;
      }
      else if(TPR == 1) state = 3;
      else {
        drivingforward(yawv,ulr_R.getLowest(),LTM);
        state = 2;
      }
    } break;
    case 3: //Turning Left
    {
      //calling turning left
      lcd.setCursor(0, 0);
      lcd.print("State 3:");
      double init_yawv_TL = getyaw();
      double diff = 0;
      yawv = getyaw();
      //Serial.println("Left");
      while(diff<90){
        turningleft();
        while (! bno08x.getSensorEvent(&sensorValue)) {
         //Serial.println("Error");
        }
        bno08x.getSensorEvent(&sensorValue);
        yawv=getyaw();
        if (LTM == 0) diff = abs(yawv-init_yawv_TL) + 3.0;
        else if (LTM == 1) diff = abs(yawv-init_yawv_TL) + 10.0;
        else if (LTM == 2) diff = abs(abs(yawv)-abs(init_yawv_TL))+3.0;
        else if (LTM == 3) diff = abs(yawv-init_yawv_TL) + 3.0;
        //Serial.println(yawv);
      }
      LTM += 1; 
      last_turn = millis();
      Serial.println("LTM: ");
      Serial.println(LTM);
      TPR = 0;
      if(GS == 0) state = 0;
      //else if(TPR == 1) state = 3;
      else if(LTM < 4) state = 2;
      else state = 1;
      analogWrite(MotorLeftPWM,0);
      analogWrite(MotorRightPWM,0);
      delay(200);
    } break;
    case 4: //Delivering
    {
//      //calling delivering
      lcd.setCursor(0, 0);
      lcd.print("State 4:");
      analogWrite(MotorLeftPWM,0);
      analogWrite(MotorRightPWM,0);
      last_dump = millis();
      delivering(plantcolor,PAC); //i is plant count
      PD = 0;
      PAC += 1;
      //delay(5000);
      for (int k = 1; k<10; k++){
        double ulr_F_c = getulr(TrigFront,EchoFront);
        double ulr_R_c = getulr(TrigRight,EchoRight);
        double ulr_R_c_2 = getulr(TrigRight2,EchoRight2);
        ulr_R.add(ulr_R_c);
        ulr_F.add(ulr_F_c);
        ulr_R_2.add(ulr_R_c_2);
      }
      //drivingforward(0,ulr_R.getLowest(),LTM);
      //delay(200);
      if(GS == 0) state = 0;
      //else if(PD == 1) state = 4;
      else state = 2;
    } break;
  }
  Serial.print("State: ");
  Serial.println(state);
}
double getulr(int trigPin, int echoPin){
  double duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 20000);
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
  //Serial.println(yawv);
  return yawv;
}

void drivingforward(double yawv_DF, double ulr, int LTM_DF){
  int statedf = 0;
  if (LTM_DF == 1) yawv_DF = yawv_DF - 90;
  else if (LTM_DF == 2) yawv_DF = (abs(yawv_DF) - 180)*(abs(yawv_DF)/yawv_DF);
  else if (LTM_DF == 3) yawv_DF = -(abs(yawv_DF) - 90);
  Serial.print("Corrected yawv: ");
  Serial.println(yawv_DF);
  if((ulr>=5.5)&&(ulr<=10.0)) statedf = 1; //ultrasonic reading >= 7inches
  //else if(ulr<=4.0) statedf = 2;
  else if(yawv_DF>=3.0) statedf = 1;
  else if (yawv_DF<=-3.0) statedf = 2;
  else statedf=0;
  switch(statedf) {
    case 0:
    {
      setpoint_L = 40;//pwm (encoder ppr=7 540 pulse change per round)
      setpoint_R = 40;
    }break;
    case 1: //towards right
    {
      setpoint_L = 60;
      setpoint_R = 40;
    }break;
    case 2: //towards left
    {
      setpoint_L = 40;
      setpoint_R = 60;
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
  //Serial.println(yawv_TL);
  //for robustness could have 4 cases
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
  //delay(850);
  return;
}
double speedMeasure(const char* cho){
  double speedE;
  if (cho == "R"){
    noInterrupts();
    long oldPosition = Enc_R_count;
    interrupts();
    delay(27);
    noInterrupts();
    long newPosition = Enc_R_count;
    interrupts();
    speedE = float(newPosition) - float(oldPosition);
  }
  else{
    noInterrupts();
    long oldPosition = Enc_L_count;
    interrupts();
    delay(27);
    noInterrupts();
    long newPosition = Enc_L_count;
    interrupts();
    speedE = float(newPosition) - float(oldPosition);
  }
  return speedE*37.04;
}

//delivering
void servoDelivering(Servo &ServoTarget){
  int lift = 18;
  while(lift<150){
        ServoTarget.write(lift);
        delay(20);
        lift=lift+1;
      } 
      delay(2500);
      while(lift>18){
        ServoTarget.write(lift);
        delay(20);
        lift=lift+1;
      } 
  return;
}
void delivering(char color[],int i){
  switch(color[i]){
    case 'R': 
    {
      analogWrite(MotorLeftPWM,40);
      analogWrite(MotorRightPWM,40);
      digitalWrite(MotorLeftInput1,HIGH);
      digitalWrite(MotorRightInput1,HIGH);
      digitalWrite(MotorLeftInput2,LOW);
      digitalWrite(MotorRightInput2,LOW);
      delay(300);
      analogWrite(MotorLeftPWM,0);
      analogWrite(MotorRightPWM,0);
      
      drivingforward(yawv, 0, LTM);
      delay(100);
      analogWrite(MotorLeftPWM,0);
      analogWrite(MotorRightPWM,0);
      int lift = 16;
      while(lift<100){//148
        ServoRightBucket.write(lift);
        delay(20);
        lift=lift+1;
      } 
      delay(2500);
      while(lift>16){
        ServoRightBucket.write(lift);
        delay(20);
        lift=lift-1;
      } 
    }break;
    case 'B': 
    {
      int lift_sort_UD = 140;
      while(lift_sort_UD>50){
        ServoSortUD.write(i);
        delay(30);
        lift_sort_UD=lift_sort_UD-1;
      }
      int lift_sort_LR = 117;
      while(lift_sort_LR>50){
        ServoSortLR.write(i);
        delay(30);
        lift_sort_LR=lift_sort_LR-1;
      }
      int lift = 29;
      while(lift<100){//161
        ServoMidBucket.write(lift);
        delay(20);
        lift=lift+1;
      } 
      delay(2500);
      while(lift>29){//29
        ServoMidBucket.write(lift);
        delay(20);
        lift=lift-1;
      }
      delay(500); 
      lift_sort_LR = 50;
      while(lift_sort_LR<117){
        ServoSortLR.write(i);
        delay(30);
        lift_sort_LR=lift_sort_LR+1;
      }
      lift_sort_UD = 50;
      while(lift_sort_UD<140){
        ServoSortUD.write(i);
        delay(30);
        lift_sort_UD=lift_sort_UD+1;
      }
      
    }break;
    case 'G':
    {
      analogWrite(MotorLeftPWM,60);
      analogWrite(MotorRightPWM,60);
      digitalWrite(MotorLeftInput1,LOW);
      digitalWrite(MotorRightInput1,LOW);
      digitalWrite(MotorLeftInput2,HIGH);
      digitalWrite(MotorRightInput2,HIGH);
      delay(1600);
      analogWrite(MotorLeftPWM,0);
      analogWrite(MotorRightPWM,0);
      digitalWrite(MotorLeftInput1,HIGH);
      digitalWrite(MotorRightInput1,HIGH);
      digitalWrite(MotorLeftInput2,LOW);
      digitalWrite(MotorRightInput2,LOW);
      int lift = 176;
      while(lift>100){//44
        ServoLeftBucket.write(lift);
        delay(20);
        lift=lift-1;
      } 
      delay(2500);
      while(lift<176){
       ServoLeftBucket.write(lift);
        delay(20);
        lift=lift+1;
      }
    }break;
  }
  return;
}
void EncoderCount_L(){
  Enc_L_count += 1;
  return;
}
void EncoderCount_R(){
  Enc_R_count += 1;
  return;
}
