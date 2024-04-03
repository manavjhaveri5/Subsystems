#include <Adafruit_BNO08x.h>
#include "ArduPID.h"
#include <Encoder.h>
#include <Servo.h>
#include <SPI.h>



//Button Declaration
const int button1Pin = 2; // Button 1 (Green)
const int button2Pin = 3; // Button 2 (Red)
const int button3Pin = 4; // Button 3 (Blue)
const int resetButtonPin = 5; // Reset Button

//Input Color
const int maxColors = 3;
int colorList[3]; // Array to store colors
int colorIndex = 0; // Index to keep track of current color
char color[3]; 



//IMU variables
#define BNO08X_CS 7
#define BNO08X_INT 6
#define BNO08X_RESET 5
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

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

//Servo variables
Servo servo_1;
Servo servo_2;
Servo servo_3;

//Preset threshold for general FSM
const int Pollen_limit = 50;
const int Plant_limit = 3;

//motor pins
int motor_L_pin = 11;
int motor_R_pin = 12;
//servo pins
int servo_1_pin = 8;
int servo_2_pin = 9;
int servo_3_pin = 10;

//pins
int GS_pin = 2; //game start button
int POC_pin = 3;
int TPR_pin = 4; 
int PD_pin = 5; 
int PAC_pin = 6;

int STP_pin = 7;
int DFP_pin = 8; 
int TLP_pin = 9;
int DP_pin = 10;

//state, input & output variables
int state = 0; 
int GS = 0; //Game starting
int POC = 0; //Pollen counting (how many pollen sorted)
int TPR = 0; //Turning point reached
int PD = 0; //Plant detected
int PAC = 0; //Plant counting

int STP = 0; //Sorting Pattern
int DFP = 0; //Driving Forward Pattern
int TLP = 0; //Turning Left Pattern
int DP = 0; //Delivering Pattern


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(GS_pin, INPUT);
  pinMode(POC_pin, INPUT);
  pinMode(TPR_pin, INPUT);
  pinMode(PD_pin, INPUT);
  pinMode(PAC_pin, INPUT);
  
  pinMode(STP_pin, OUTPUT);
  pinMode(DFP_pin, OUTPUT);
  pinMode(TLP_pin, OUTPUT);
  pinMode(DP_pin, OUTPUT);

  //Buttons
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  pinMode(button3Pin, INPUT);
  pinMode(resetButtonPin, INPUT);




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

  //Delivering setup
  servo_1.attach(servo_1_pin);
  servo_2.attach(servo_2_pin);
  servo_3.attach(servo_3_pin);
  

  SPI.begin();

}

void loop() {

  if (digitalRead(button1Pin) == LOW) {
    addColor('G', 0, 255, 0); 
    delay(200); 
  } else if (digitalRead(button2Pin) == LOW) {
    addColor('R', 255, 0, 0);
    delay(200);
  } else if (digitalRead(button3Pin) == LOW) {
    addColor('B', 0, 0, 255);
    delay(200); 
  } else if (digitalRead(resetButtonPin) == LOW) {
    resetColors(); 
    delay(200); 
  }

  updateLEDStrip();


  // put your main code here, to run repeatedly:
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
  }
  if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  //update this portion with sensor readings
  int GS = digitalRead(GS_pin);
  int POC = digitalRead(POC_pin);
  int TPR = digitalRead(TPR_pin);
  int PD = digitalRead(PD_pin);
  int PAC = digitalRead(PAC_pin);


  switch(state){
    case 0: //initial stand-by state
    {
      STP = 0;
      DFP = 0;
      TLP = 0;
      DP = 0;
      if(GS == 1) state = 1;
      else state = 0;
    } break;
    case 1: //Loading & Sorting
    {
      STP = 1;
      DFP = 0;
      TLP = 0;
      DP = 0;
      
      //PAC = 0; //update plant count
      if(GS == 0) state = 0;
      else if(POC == 1) state = 2;
      else state = 1;
    } break;
    case 2: //Driving Forward
    {
      STP = 0;
      DFP = 1;
      TLP = 0;
      DP = 0;
      //calling driving forward
      double yawv = getyaw();
      //double ulr = getulr();
      drivingforward(yawv,ulr);
      //Still need ultrasonic reading

      if(GS == 0) state = 0;
      else if(PD == 1) state = 4; //PAC=PAC+1;
      else if(TPR == 1) state = 3;
      else state = 2;
    } break;
    case 3: //Turning Left
    {
      STP = 0;
      DFP = 0;
      TLP = 1;
      DP = 0;
      //calling turning left
      turningleft();
      TPR = 0;
      if(GS == 0) state = 0;
      else if(TPR == 1) state = 3;
      else if(PAC == 0) state = 2;
      else state = 1;
    } break;
    case 4: //Delivering
    {
      STP = 0;
      DFP = 0;
      TLP = 0;
      DP = 1;
      //calling delivering 
      delivering(color[],i) //i is plant count
      if(GS == 0) state = 0;
      else if(PD == 1) state = 4;
      else state = 2;
    } break;
  }
  digitalWrite(STP_pin, STP);
  digitalWrite(DFP_pin, DFP);
  digitalWrite(TLP_pin, TLP);
  digitalWrite(DP_pin, DP);
  Serial.print("state: ");
  Serial.println(state);
  Serial.print("GS: ");
  Serial.println(GS);
  Serial.print("POC: ");
  Serial.println(POC);
  Serial.print("TPR: ");
  Serial.println(TPR);
  Serial.print("PD: ");
  Serial.println(PD);
  Serial.print("PAC: ");
  Serial.println(PAC);
  Serial.print("STP: ");
  Serial.println(STP);
  Serial.print("DFP: ");
  Serial.println(DFP);
  Serial.print("TLP: ");
  Serial.println(TLP);
  Serial.print("DP: ");
  Serial.println(DP);
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

// Input Color Setting
void addColor(char colorChar, byte R, byte G, byte B) {
  if (colorIndex < maxColors) {
    color[colorIndex] = colorChar;
    colorList[colorIndex][0] = R;
    colorList[colorIndex][1] = G;
    colorList[colorIndex][2] = B;
    colorIndex++;

  }
}

void resetColors() {
  for (int i = 0; i < maxColors; i++) {
    colorList[i][0] = 0;
    colorList[i][1] = 0;
    colorList[i][2] = 0;
  }
  colorIndex = 0;
}

void updateLEDStrip() {
  sendStartFrame();
  for (int i = 0; i < maxColors; i++) {
    sendColorFrame(0b11111111, colorList[i][2], colorList[i][1], colorList[i][0]);
  }
  sendEndFrame();
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