//Preset threshold
const int Pollen_limit = 50;
const int Plant_limit = 3;

//pins
int GS_pin = 2; 
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
  Serial.begin(9600);
  
  pinMode(GS_pin, INPUT);
  pinMode(POC_pin, INPUT);
  pinMode(TPR_pin, INPUT);
  pinMode(PD_pin, INPUT);
  pinMode(PAC_pin, INPUT);
  
  pinMode(STP_pin, OUTPUT);
  pinMode(DFP_pin, OUTPUT);
  pinMode(TLP_pin, OUTPUT);
  pinMode(DP_pin, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
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
