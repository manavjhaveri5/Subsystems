#include <Adafruit_BNO08x.h>
#include "ArduPID.h"
//#include <Encoder.h>
// For SPI mode, we need a CS pin
#define BNO08X_CS 33                                                                                          
#define BNO08X_INT 32
// For SPI mode, we also need a RESET 
// but not for I2C or UART
#define BNO08X_RESET 31

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

volatile int Enc_L_count;
volatile int Enc_R_count;
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
int motor_left_pin = 11;
int motor_left_pin_2 = 10;
int motor_right_pin = 9;
int motor_right_pin_2 = 8;
//Encoder Enc_L(5,6);
//Encoder Enc_R(3, 4);

//delivering global variables
#include <Servo.h>
Servo servo_1;
Servo servo_2;
Servo servo_3;
int servo_1_pin = 34;
int servo_2_pin = 35;
int servo_3_pin = 36;

/*
double siny;
double cosy;  
double yawv;
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(20);
  bno08x.begin_I2C();
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);

  attachInterrupt(digitalPinToInterrupt(3), EncoderCount_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), EncoderCount_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), EncoderCount_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(6), EncoderCount_L, CHANGE);
  
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

  //delivering setup
  servo_1.attach(servo_1_pin);
  servo_2.attach(servo_2_pin);
  servo_3.attach(servo_3_pin);
  //dc motors initial
  analogWrite(motor_left_pin,0);
  analogWrite(motor_right_pin,0);
  analogWrite(motor_left_pin_2,0);
  analogWrite(motor_right_pin_2,0);
}

void loop() {
  // put your main code here, to run repeatedly:
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
  turningleft();
  //Serial.println("2");
  delay(5000);
  //encoder test//one round ~540, pwm40 ~444
//  int sp=speedMeasure("L");
//  Serial.print("left: ");
//  Serial.println(sp);
//  sp=speedMeasure("R");
//  Serial.print("right: ");
//  Serial.println(sp);
  
  //analogWrite(motor_left_pin,60);
  //analogWrite(motor_left_pin_2,0);
  //analogWrite(motor_right_pin,0);
  //analogWrite(motor_right_pin_2,0);
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
double getyaw(){
  double r=sensorValue.un.gameRotationVector.real;
  double i=sensorValue.un.gameRotationVector.i;
  double j=sensorValue.un.gameRotationVector.j;
  double k=sensorValue.un.gameRotationVector.k;
  double siny = +2.0 * (r * k + i * j);
  double cosy = +1.0 - 2.0 * (j * j + k * k);  
  double yawv = atan2(siny, cosy)*180.0/M_PI;
  yawv = getyaw();
  return yawv;
}

void drivingforward(double yawv, double ulr){
  int statedf = 0;

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
  analogWrite(motor_left_pin,motor_L_O);
  //Serial.println(motor_L_O);
  //Serial.println(motor_L_O);
  analogWrite(motor_right_pin,motor_R_O);
  analogWrite(motor_left_pin_2,0);
  analogWrite(motor_right_pin_2,0);
  return;
}
void turningleft(){
  //for robustness could have 4 cases
  double init_yawv = getyaw();
  double yawv = init_yawv;
  Serial.print("initial: ");
  Serial.println(yawv);
  while(abs(abs(yawv)-abs(init_yawv))<90){
    setpoint_L = 40;
    setpoint_R = 40;
    motor_L_I = speedMeasure("L")*40.0/444.0;
    motor_R_I = speedMeasure("R")*40.0/444.0;
    ControllerL.compute();
    ControllerR.compute();
    analogWrite(motor_left_pin_2,motor_L_O);
    analogWrite(motor_right_pin,motor_R_O);
    analogWrite(motor_left_pin,0);
    analogWrite(motor_right_pin_2,0);
    
    Serial.print("z axis rotation:");
    yawv = getyaw();
    Serial.println(yawv);
  }
  return;
}
long speedMeasure(const char* cho){//perhaps another board ?
  long speedE;
  if (cho == "R"){
    long oldPosition = Enc_R_count;
    delay(54); //need adjustment
    long newPosition = Enc_R_count;
    speedE = newPosition - oldPosition;
  }
  else{
    long oldPosition = Enc_L_count;
    delay(54); //need adjustment
    long newPosition = Enc_L_count;
    speedE = newPosition - oldPosition;
  }
  return speedE*18.519;
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
void EncoderCount_L(){
  Enc_L_count += 1;
}
void EncoderCount_R(){
  Enc_R_count += 1;
}
