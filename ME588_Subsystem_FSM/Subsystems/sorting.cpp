#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Servo.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Servo objects
Servo servo;

const int servoPin = 6;


const int ledPinRed = 9;  
const int ledPinGreen = 10;
const int ledPinBlue = 11;


const unsigned long debounce = 10; // MODIFY

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

void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
  servo.write(0);
  
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
  
  uint16_t r = red; 
  uint16_t g = green; 
  uint16_t b = blue;
  
  
  if (r > g && r > b) {
    return 1; // Red
  } else if (g > r && g > b) {
    return 2; // Green
  } else if (b > r && b > g) {
    return 3; // Blue
  }
  return 0; // No color detected 
}


void rotateChute(uint8_t color) {
  unsigned long currentTime = millis();
  if (currentTime - lastdetect >= debounce){
    switch (color) {
    case 1: // Red
      servo.write(45);
      countRed++;
      blinkLED(ledPinRed);
      break;
    case 2: // Green
      servo.write(90);
      countGreen++;
      blinkLED(ledPinGreen);
      break;
    case 3: // Blue
      servo.write(135);
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
  uint8_t colorDetected = detectColor();
  totalcount = countBlue+countGreen+countRed;
  if ( totalcount < 100) {
      if(colorDetected != 0) {
        rotateChute(colorDetected);
      }
    Serial.println(totalcount);
  }
}  


