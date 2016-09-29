/*
Code for a slave arduino who will collect the wind data for weather station 
Based on: NMEA Wind Instrument by Tom Kuehn
Uses the peet bros ananometer
*/

/***   DEFINES, VARIABLES, INCLUDES   *******************************************************************************/
//#include "PString.h"

#define windSpeedPin 6
#define windDirPin 7
#define windSpeedINT 0 // INT0
#define windDirINT 1   // INT1

//SLAVE PART
#include <Wire.h>
int table[] = {0, 0, 0};

// Pin 13 has an LED connected on most Arduino boards.
int LED = 13;

const unsigned long DEBOUNCE = 10000ul;      // Minimum switch time in microseconds
const unsigned long DIRECTION_OFFSET = 0ul;  // Manual direction offset in degrees, if required
const unsigned long TIMEOUT = 1500000ul;       // Maximum time allowed between speed pulses in microseconds
const unsigned long UPDATE_RATE = 500ul;     // How often to send out NMEA data in milliseconds
const float filterGain = 0.25;               // Filter gain on direction output filter. Range: 0.0 to 1.0
// 1.0 means no filtering. A smaller number increases the filtering

// Knots is actually stored as (Knots * 100). Deviations below should match these units.
const int BAND_0 =  10 * 100;
const int BAND_1 =  80 * 100;

const int SPEED_DEV_LIMIT_0 =  5 * 100;     // Deviation from last measurement to be valid. Band_0: 0 to 10 knots
const int SPEED_DEV_LIMIT_1 = 10 * 100;     // Deviation from last measurement to be valid. Band_1: 10 to 80 knots
const int SPEED_DEV_LIMIT_2 = 30 * 100;     // Deviation from last measurement to be valid. Band_2: 80+ knots

// Should be larger limits as lower speed, as the direction can change more per speed update
const int DIR_DEV_LIMIT_0 = 25;     // Deviation from last measurement to be valid. Band_0: 0 to 10 knots
const int DIR_DEV_LIMIT_1 = 18;     // Deviation from last measurement to be valid. Band_1: 10 to 80 knots
const int DIR_DEV_LIMIT_2 = 10;     // Deviation from last measurement to be valid. Band_2: 80+ knots

volatile unsigned long speedPulse = 0ul;    // Time capture of speed pulse
volatile unsigned long dirPulse = 0ul;      // Time capture of direction pulse
volatile unsigned long speedTime = 0ul;     // Time between speed pulses (microseconds)
volatile unsigned long directionTime = 0ul; // Time between direction pulses (microseconds)
volatile boolean newData = false;           // New speed pulse received
volatile unsigned long lastUpdate = 0ul;    // Time of last serial output

volatile int knotsOut = 0;    // Wind speed output in knots * 100
volatile int dirOut = 0;      // Direction output in degrees
volatile boolean ignoreNextReading = false;

boolean debug = false;

int windRichting = 0;
int windSnelheid = 0;

/***   SETUP   ******************************************************************************************************/

void setup()
{

  Wire.begin(2);                // join i2c bus with address #2
  Wire.onRequest(requestEvent); // register event

  pinMode(LED, OUTPUT);
  delay(5000);
  Serial.begin(38400, SERIAL_8N1);
  //Serial.println(VERSION);
  Serial.print("Direction Filter: ");
  Serial.println(filterGain);
  delay(5000);
  pinMode(windSpeedPin, INPUT);
  attachInterrupt(windSpeedINT, readWindSpeed, FALLING);

  pinMode(windDirPin, INPUT);
  attachInterrupt(windDirINT, readWindDir, FALLING);

  interrupts();
}

/***   LOOP   *******************************************************************************************************/

void loop()
{
  int i;
  const unsigned int LOOP_DELAY = 50;
  const unsigned int LOOP_TIME = TIMEOUT / LOOP_DELAY;

  digitalWrite(LED, !digitalRead(LED));    // Toggle LED

  i = 0;
  // If there is new data, process it, otherwise wait for LOOP_TIME to pass
  while ((newData != true) && (i < LOOP_TIME))
  {
    i++;
    delayMicroseconds(LOOP_DELAY);
  }

  calcWindSpeedAndDir();    // Process new data
  newData = false;
}

/***   FUNCTIONS   **************************************************************************************************/

void readWindSpeed()
{
  // Despite the interrupt being set to FALLING edge, double check the pin is now LOW
  if (((micros() - speedPulse) > DEBOUNCE) && (digitalRead(windSpeedPin) == LOW))
  {
    // Work out time difference between last pulse and now
    speedTime = micros() - speedPulse;

    // Direction pulse should have occured after the last speed pulse
    if (dirPulse - speedPulse >= 0) directionTime = dirPulse - speedPulse;

    newData = true;
    speedPulse = micros();    // Capture time of the new speed pulse
  }
}

void readWindDir()
{
  if (((micros() - dirPulse) > DEBOUNCE) && (digitalRead(windDirPin) == LOW))
  {
    dirPulse = micros();        // Capture time of direction pulse
  }
}

boolean checkDirDev(long knots, int dev)
{
  if (knots < BAND_0)
  {
    if ((abs(dev) < DIR_DEV_LIMIT_0) || (abs(dev) > 360 - DIR_DEV_LIMIT_0)) return true;
  }
  else if (knots < BAND_1)
  {
    if ((abs(dev) < DIR_DEV_LIMIT_1) || (abs(dev) > 360 - DIR_DEV_LIMIT_1)) return true;
  }
  else
  {
    if ((abs(dev) < DIR_DEV_LIMIT_2) || (abs(dev) > 360 - DIR_DEV_LIMIT_2)) return true;
  }
  return false;
}

boolean checkSpeedDev(long knots, int dev)
{
  if (knots < BAND_0)
  {
    if (abs(dev) < SPEED_DEV_LIMIT_0) return true;
  }
  else if (knots < BAND_1)
  {
    if (abs(dev) < SPEED_DEV_LIMIT_1) return true;
  }
  else
  {
    if (abs(dev) < SPEED_DEV_LIMIT_2) return true;
  }
  return false;
}

void calcWindSpeedAndDir()
{
  unsigned long dirPulse_, speedPulse_;
  unsigned long speedTime_;
  unsigned long directionTime_;
  long windDirection = 0l, rps = 0l, knots = 0l;

  static int prevKnots = 0;
  static int prevDir = 0;
  int dev = 0;

  // Get snapshot of data into local variables. Note: an interrupt could trigger here
  noInterrupts();
  dirPulse_ = dirPulse;
  speedPulse_ = speedPulse;
  speedTime_ = speedTime;
  directionTime_ = directionTime;
  interrupts();

  // Make speed zero, if the pulse delay is too long
  if (micros() - speedPulse_ > TIMEOUT) speedTime_ = 0ul;

  // The following converts revolutions per 100 seconds (rps) to knots x 100
  // This calculation follows the Peet Bros. piecemeal calibration data
  if (speedTime_ > 0)
  {
    rps = 100000000 / speedTime_;                //revolutions per 100s

    if (rps < 323)
    {
      knots = (rps * rps * -11) / 11507 + (293 * rps) / 115 - 12;
    }
    else if (rps < 5436)
    {
      knots = (rps * rps / 2) / 11507 + (220 * rps) / 115 + 96;
    }
    else
    {
      knots = (rps * rps * 11) / 11507 - (957 * rps) / 115 + 28664;
    }
    //knots = mph * 0.86897

    if (knots < 0l) knots = 0l;  // Remove the possibility of negative speed
    // Find deviation from previous value
    dev = (int)knots - prevKnots;

    // Only update output if in deviation limit
    if (checkSpeedDev(knots, dev))
    {
      knotsOut = knots;

      // If speed data is ok, then continue with direction data
      if (directionTime_ > speedTime_)
      {
        windDirection = 999;    // For debugging only (not output to knots)
      }
      else
      {
        // Calculate direction from captured pulse times
        windDirection = (((directionTime_ * 360) / speedTime_) + DIRECTION_OFFSET) % 360;

        // Find deviation from previous value
        dev = (int)windDirection - prevDir;

        // Check deviation is in range
        if (checkDirDev(knots, dev))
        {
          int delta = ((int)windDirection - dirOut);
          if (delta < -180)
          {
            delta = delta + 360;    // Take the shortest path when filtering
          }
          else if (delta > +180)
          {
            delta = delta - 360;
          }
          // Perform filtering to smooth the direction output
          dirOut = (dirOut + (int)(round(filterGain * delta))) % 360;
          if (dirOut < 0) dirOut = dirOut + 360;
        }
        prevDir = windDirection;
      }
    }
    else
    {
      ignoreNextReading = true;
    }

    prevKnots = knots;    // Update, even if outside deviation limit, cause it might be valid!?
  }
  else
  {
    knotsOut = 0;
    prevKnots = 0;
  }

  if (debug)
  {
    Serial.print(millis());
    Serial.print(",");
    Serial.print(dirOut);
    Serial.print(",");
    Serial.print(windDirection);
    Serial.print(",");
    Serial.println(knotsOut / 100);
    //Serial.print(",");
    //Serial.print(knots/100);
    //Serial.print(",");
    //Serial.println(rps);
  }
  else
  {
    if (millis() - lastUpdate > UPDATE_RATE)
    {
      printWind();
      lastUpdate = millis();
    }
  }
}

void printWind()
{
  float spd = knotsOut / 100.0 * 1.852;
  byte cs;
  //Assemble a sentence of the various parts so that we can calculate the proper checksum

//  PString str(windSentence, sizeof(windSentence));
//  str.print("Windrichting: ");
//  str.print(dirOut);
//  str.print("* ");
//  str.print(spd);
//  str.print("Kn ");
//  str.print(spd * 10);
//  str.print("Kn ");
//  Serial.println(windSentence);
  
  windRichting = dirOut;
  windSnelheid = spd * 100;

}






void requestEvent2()
{
  
  /*
  Wire.write(data, length)

  Parameters

  value: a value to send as a single byte

  string: a string to send as a series of bytes

  data: an array of data to send as bytes

  length: the number of bytes to transmit
  */
   
  int16_t bigNum1 =  windRichting;
  int16_t bigNum2 = windSnelheid;
  byte myArray[4];
 
  myArray[0] = (bigNum1 >> 8) & 0xFF;
  myArray[1] = bigNum1 & 0xFF;
  myArray[2] = (bigNum2 >> 8) & 0xFF;
  myArray[3] = bigNum2 & 0xFF;
  Wire.write(myArray, 4);

}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  //  for(int i=0;i<3;i++)
  //  {
  uint8_t Buffer[3];
  Buffer[0] = table[0];
  Buffer[1] = table[1];
  Buffer[2] = table[2];
  Serial.print("Buffer: ");
  Serial.println(Buffer[1]);

  Wire.write(Buffer, 3);

}
