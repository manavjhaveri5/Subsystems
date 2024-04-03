#include <SPI.h>

const int button1Pin = 2; // Button 1 (Green)
const int button2Pin = 3; // Button 2 (Red)
const int button3Pin = 4; // Button 3 (Blue)
const int resetButtonPin = 5; // Reset Button

const int maxColors = 3;
int colorList[maxColors][3]; // Array to store colors
int colorIndex = 0; // Index to keep track of the current color
char color[3]; 

void setup() {
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);
  pinMode(resetButtonPin, INPUT_PULLUP);

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
}

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

void sendStartFrame() {
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
}

void sendEndFrame() {
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
}

void sendColorFrame(byte brightness, byte B, byte G, byte R) {
  SPI.transfer(brightness | 0xE0); 
  SPI.transfer(B);
  SPI.transfer(G);
  SPI.transfer(R);
}

