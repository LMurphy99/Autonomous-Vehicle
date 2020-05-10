#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Example code for the Adafruit TCS34725 breakout library */

/* Connect SCL to analog 5
  Connect SDA to analog 4
  Connect VDD to 3.3V DC
  Connect GROUND to common ground */

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

  int arrival = 0;
  int count = 0;
  int luxreadings[5];
  int startcount = 0;
  int initial[5];
  int mean = 0;
  int baseline = 0;
  int moveforward = 0;

  int black = 0;
  int white = 1;

  int baseplus = 0;
  int baseminus = 0;

  int stopper = 0;


void setup(void) {
  Serial.begin(9600);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  
}

void loop(void) 
{
  while (stopper==0)
  {
    if (arrival == 0)
    {
      while(startcount < 5)
      {
        uint16_t r, g, b, c, colorTemp, lux;
      
        tcs.getRawData(&r, &g, &b, &c);
        colorTemp = tcs.calculateColorTemperature(r, g, b);
        lux = tcs.calculateLux(r, g, b);
      
        Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
        Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
        Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
        Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
        Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
        Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
        Serial.println(" ");
  
        Serial.print("Start Count "); Serial.print(startcount, DEC); Serial.print(" ");
        Serial.println(" ");
  
        initial[startcount] = lux;
        startcount++;
      }
  
      baseline = getmean(initial);
      baseplus = baseline+20;
      baseminus = baseline-20;
      arrival = 1;
    }
    Serial.print(arrival, DEC);
    if (arrival == 1)
    {
      while(count < 5)
      {
        uint16_t r, g, b, c, colorTemp, lux;
      
        tcs.getRawData(&r, &g, &b, &c);
        colorTemp = tcs.calculateColorTemperature(r, g, b);
        lux = tcs.calculateLux(r, g, b);
      
        Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
        Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
        Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
        Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
        Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
        Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
        Serial.println(" ");
  
        luxreadings[count] = lux;
        Serial.print("Count "); Serial.print(count, DEC); Serial.print(" ");
        Serial.println(" ");
        count++;
       
      }
  
      mean = getmean(luxreadings);
  
      if (mean>baseminus)
      {
        if(mean<baseplus)
        {
          Serial.print("  Move Closer  ");
          Serial.println(" ");
          moveforward = 1;
          count = 0;
        }
        else if (mean>baseplus)
        {
         Serial.print("  White Found  ");
         Serial.println(" ");
          white = 1;
          black = 0;
          count = 0;
        }
        else
        {
         Serial.print("  Give up  ");
         Serial.println(" ");
        }
      }
    
      else if (mean<baseminus)
      {
        Serial.print(" Black Found  ");
        Serial.println(" ");
        black = 1;
        white = 0;
        count =0;
      }
    
    }

  }
}

int getmean(int n[5])
{
  int temp = 0;
  int ans = 0;
  for(int i = 0; i<5; i++)
  {
    temp = temp + n[i];
  }
  ans = temp/5;
  Serial.print (ans, DEC);
  Serial.println(" ");  
  return(ans);
}
