#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <stdint.h>


/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
uint16_t r, g, b, c, colorTemp, new_lux;
uint16_t old_lux=0;
int diff;


/* tcs.getRawData() does a delay(Integration_Time) after the sensor readout.
We don't need to wait for the next integration cycle because we receive an interrupt when the integration cycle is complete*/
void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}


void setup() {
  Serial.begin(115200);
  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  getRawData_noDelay(&r, &g, &b, &c);
  old_lux = tcs.calculateLux(r, g, b);
}


void loop() {
    getRawData_noDelay(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    new_lux = tcs.calculateLux(r, g, b);
    if((new_lux> 6000) || ((new_lux<1000)&&(colorTemp>45000))){
      Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
      Serial.print("Lux: "); Serial.print(new_lux); Serial.print(" - ");
      Serial.print("old_lux: "); Serial.print(old_lux); Serial.print(" ");
      Serial.print("bool: "); Serial.print(diff); Serial.print(" ");
      Serial.print("R: "); Serial.print(r); Serial.print(" ");
      Serial.print("G: "); Serial.print(g); Serial.print(" ");
      Serial.print("B: "); Serial.print(b); Serial.print(" ");
      Serial.print("C: "); Serial.print(c); Serial.print(" ");
      Serial.println(" ");
      
      if (r > g && r > b) {
        Serial.println(1); // Red
      } else if (g > r && g > b) {
        Serial.println(2); // Green
      } else if (b > r && b > g) {
        Serial.println(3); // Blue
      }
      delay(1000);
    }
}
