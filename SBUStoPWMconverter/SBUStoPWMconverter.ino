/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2018 Bolder Flight Systems
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
* and associated documentation files (the "Software"), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
* Description: This program will use the 
* Teensy PWM Backpack (http://bolderflight.com/products/teensy/pwm) to read 
* SBUS commands from an SBUS capable receiver and convert those commands to 
* PWM output for driving standard servos. The SBUS to PWM mapping / gains,
* and bias can be adjusted. Also includes end point settings, failsafe
* positions, and a motor safety switch.
*/

#include "SBUS.h"

/* The Teensy SBUS Backpack interfaces on Serial2 */
SBUS sbus(Serial2);
/* 
* An array to store the SBUS commands from the receiver,
* these will be normalized in a +/- 1 range.
*/
float SbusCmds[16];
/*
* Variables to store the SBUS failsafe and lost frame
* information from the receiver.
*/
bool Failsafe, LostFrame;
/* Struct to store PWM settings */
struct PwmSetup {
  bool motor;
  unsigned int FailsafePosition;
  unsigned int Limits[2];
  float Gain[15];
  float Bias;
};
/* Array of structs, one for each PWM output */
struct PwmSetup PwmSettings[7];
/*
* Redefine the pin numbers to match the PWM Backpack numbering
* Note that we have 7 PWM channels now that one is being used
* for SBUS input.
*/
const unsigned int PWM[7] = {20,21,22,23,5,6,10};
/* PWM update frequency, 50 Hz */
const unsigned int Freq = 50;
/* PWM update period */
const unsigned int Period_us = 1000000 / Freq;
/* PWM resolution */
const unsigned int PWM_Resolution = 16;
/* Helper function for setting up PWM channels */
struct PwmSetup SetupPwmChannel(bool motor, unsigned int FailsafePosition, unsigned int min, unsigned int max, float *gain, float bias) 
{
  struct PwmSetup Setup;
  Setup.motor = motor;
  Setup.FailsafePosition = FailsafePosition;
  Setup.Limits[0] = min;
  Setup.Limits[1] = max;
  for (unsigned int i = 0; i < sizeof(Setup.Gain) / sizeof(float); ++i) {
    Setup.Gain[i] = gain[i];
  }
  Setup.Bias = bias;
  return Setup;
}

void setup()
{
  /* Starting SBUS communication */
  sbus.begin();
  /* Setting the analog frequency to 50 Hz and resolution to 16 bit */
  for (unsigned int i = 0; i < sizeof(PWM) / sizeof(unsigned int); ++i) {
    analogWriteFrequency(PWM[i], Freq);
    analogWriteResolution(PWM_Resolution);
  }  
  /* Setting up each PWM, for this example, I'll have a motor on channel 1 and a elevon on channel 2 and channel 3 */
  {
    float gain[15] = {};
    gain[0] = 1.0f;
    PwmSettings[0] = SetupPwmChannel(true,1000,1000,2000,gain,0);
  }
  {
    float gain[15] = {};
    gain[1] = 1.0f;
    gain[2] = 1.0f;
    PwmSettings[1] = SetupPwmChannel(false,1000,1000,2000,gain,0);
  }
  {
    float gain[15] = {};
    gain[1] = -1.0f;
    gain[2] = 1.0f;
    PwmSettings[2] = SetupPwmChannel(false,1000,1000,2000,gain,0);
  }
}

void loop()
{
  /* Good SBUS packet received */ 
  if (sbus.readCal(&SbusCmds[0],&Failsafe,&LostFrame)) {
    /* Stepping through each PWM channel */
    for (unsigned int i = 0; i < sizeof(PWM) / sizeof(unsigned int); ++i) {
      float Cmd = 0;
      /* If we're in failsafe, command the failsafe value */
      if (Failsafe) {
        Cmd = PwmSettings[i].FailsafePosition;
      /* If this is a motor and the motor is safed, command the minimum value */
      } else if ((SbusCmds[0] < 0.5) && (PwmSettings[i].motor)) {
        Cmd = PwmSettings[i].Limits[0];
      } else {
        /* Compute the PWM command from the gain and bias applied to each SBUS channel */
        for (unsigned int j = 0; j < sizeof(PwmSettings[i].Gain) / sizeof(float); ++j) {
          Cmd += SbusCmds[j + 1] * PwmSettings[i].Gain[j];
        }
        Cmd += PwmSettings[i].Bias;
        /* Scale to a PWM value */
        Cmd = Cmd * 500.0f + 1500.0f;
        /* Saturate to limits */
        if (Cmd < PwmSettings[i].Limits[0]) {
          Cmd = PwmSettings[i].Limits[0];
        } else if (Cmd > PwmSettings[i].Limits[1]) {
          Cmd = PwmSettings[i].Limits[1];
        }
      }
      /* Write command to servo */
      analogWrite(PWM[i],Cmd / Period_us * powf(2,PWM_Resolution));
    }
  }
}
