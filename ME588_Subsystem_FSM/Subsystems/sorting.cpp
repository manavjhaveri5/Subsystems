#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Servo.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Servo objects
Servo servoRed, servoGreen, servoBlue;

const int servoPinRed = 6;
const int servoPinGreen = 7;
const int servoPinBlue = 8;

const int ledPinRed = 9;  
const int ledPinGreen = 10;
const int ledPinBlue = 11;



const unsigned long debounce = 1000; // MODIFY
unsigned long lastdetect = 0; 
int countRed = 0;
int countGreen = 0;
int countBlue = 0;
int totalcount = 0; 

void blinkLED(int pin){
  digitalWrite(pin,HIGH);
  delay(1000);
  digitalWrite(pin,LOW);
}

void setupSortingSystem() {
  servoRed.attach(servoPinRed);
  servoGreen.attach(servoPinGreen);
  servoBlue.attach(servoPinBlue);

  servoRed.write(0);
  servoGreen.write(0);
  servoBlue.write(0);

  pinMode(ledPinRed, OUTPUT);
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinBlue, OUTPUT);

  if (tcs.begin()) {
    Serial.println("Found color sensor");
  } else {
    Serial.println("No color sensor found");
  }
}

// Read the color of the foam ball
uint8_t detectColor() {
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);
  
  uint8_t r = red; uint8_t g = green; uint8_t b = blue;
  
  
  if (r > g && r > b) {
    return 1; // Red
  } else if (g > r && g > b) {
    return 2; // Green
  } else if (b > r && b > g) {
    return 3; // Blue
  }
  return 0; // No color detected or unclear
}


void kickBallIntoBucket(uint8_t color) {
  unsigned long currentTime = millis();
  if (currentTime - lastdetect >= debounce){
    switch (color) {
    case 1: // Red
      delay(500);
      servoRed.write(90);
      delay(500); 
      servoRed.write(0); 
      countRed++;
      blinkLED(ledPinRed);
      break;
    case 2: // Green
      delay(1000);
      servoGreen.write(90);
      delay(500);
      servoGreen.write(0);
      countGreen++;
      blinkLED(ledPinGreen);
      break;
    case 3: // Blue
      delay(1500);
      servoBlue.write(90);
      delay(500);
      servoBlue.write(0);
      countBlue++;
      blinkLED(ledPinBlue);
      break;
    default: // In case of no color or error
      break;
    }
  lastdetect = currentTime;
  }
}

void loop() {
  totalcount = countRed + countBlue + countGreen;
  uint8_t colorDetected = detectColor();

  while ( totalcount < 100) {
      if(colorDetected != 0) {
        kickBallIntoBucket(colorDetected);
      }
    Serial.println(totalcount);
  }
}  

void setup() {
  Serial.begin(9600);
  setupSortingSystem();
}
