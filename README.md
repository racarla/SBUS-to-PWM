# SBUS to PWM Converter
This tutorial will serve as a capstone to what has been covered about writing and reading PWM and SBUS using a [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) and the [Teensy SBUS Backpack](http://bolderflight.com/products/teensy/sbus/) and [Teensy PWM Backpack](http://bolderflight.com/products/teensy/pwm/). If you haven't already, you may consider following the previous tutorials:
1. [PWM Introduction](https://github.com/bolderflight/PWM-Intro)
2. [SBUS Introduction](https://github.com/bolderflight/SBUS-Intro)
3. [Reading PWM](https://github.com/bolderflight/Reading-PWM)
4. [Reading SBUS](https://github.com/bolderflight/Reading-SBUS)

In this project, we will use a [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) and the [Teensy PWM Backpack](http://bolderflight.com/products/teensy/pwm/) to create an SBUS to PWM converter. These converters are typically sold by [Futaba](https://www.towerhobbies.com/cgi-bin/wti0001p?&I=LXZKA6) or [FrSky](https://alofthobbies.com/frsky-sd1-sbus-converter.html) and convert the SBUS signal from SBUS capable RC receivers to a PWM signal for using standard servos. This is often useful because the selection of SBUS servos can be limiting or the hobbyist may already have a collection of standard servos and would like to use those with the newest SBUS transmitters and receivers. 

In our case, we'll convert our SBUS input to 7 PWM output channels. We'll also go a step beyond these commercial options by adding some pretty advanced mixing as well as safety switches for any motors. Let's get started!

# Necessary Hardware
   * [Teensy 3.2](https://www.pjrc.com/store/teensy32.html)
   * [Teensy PWM Backpack](http://bolderflight.com/products/teensy/pwm/)
   * A Futaba or FrSky SBUS capable transmitter and receiver
   * Up to 7 standard servos
   * ESC with a battery eliminator circuit (BEC) to provide servo power
   * [Teensy Solo Frame or Double Frame](http://bolderflight.com/products/teensy/frame/) if you intend to use this in an RC aircraft or drone, the power regulation will be important to power the [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) from the aircraft battery.

# Wiring
We would like to power our SBUS receiver from the regulated 5V source rather than the bussed servo power, so ensure the solderpad on the [Teensy PWM Backpack](http://bolderflight.com/products/teensy/pwm/) has 5V selected instead of VDD. Now, connect your SBUS receiver to the [Teensy PWM Backpack](http://bolderflight.com/products/teensy/pwm/) SBUS RX pin with the signal wire up, the power wire in the middle, and the ground at the bottom.

<img src="/images/sbus2pwm.JPG" alt="sbus to pwm" width="500">

Connect your servos and ESC to the remaining pins, keeping note of the channels that they are connected to.

Plug your [Teensy 3.2](https://www.pjrc.com/store/teensy32.html) into the [Teensy PWM Backpack](http://bolderflight.com/products/teensy/pwm/) with the USB connector on the end of the backpack that has a white dot; this dot marks the location of the Teensy ground pin. Finally, plug this into the [Teensy Backpack Frame](http://bolderflight.com/products/teensy/frame/) with the USB end on the side with the power regulator and screw terminal.

<img src="/images/sbus2pwm_setup.JPG" alt="sbus to pwm setup" width="500">

# Software
We'll use the [SBUS library](https://github.com/bolderflight/SBUS) from Bolder Flight Systems for this tutorial. This library contains methods for reading and writing SBUS packets using either raw counts (i.e. 172 - 1811) or normalized values (i.e. +/- 1). We'll also use the [Teensy PWM](https://www.pjrc.com/teensy/td_pulse.html) library for creating the PWM signals.

## Installation
Simply clone or download the [SBUS library](https://github.com/bolderflight/SBUS) to your Arduino libraries folder. For this tutorial, you can clone or download the code in this [repository](https://github.com/bolderflight/SBUS-to-PWM) and follow along or code it yourself. We'll use the _SBUStoPWMconverter.ino_ for this test.

## Code Walkthrough
### Globals
We start with the familiar declaration of our SBUS object, an array to store the SBUS commands and storage for our failsafe and lost frame flags.

```C++
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
```

Next we define a structure for configuring each PWM output channel. This is simply a matter of conveinance and this information could be stored as separate variables.

```C++
/* Struct to store PWM settings */
struct PwmSetup {
  bool motor;
  unsigned int FailsafePosition;
  unsigned int Limits[2];
  float Gain[15];
  float Bias;
};
```

We would like to know if the particular PWM channel is a motor for implementing the motor safety logic. We would also like to know what position to command if the receiver goes into failsafe mode, this could be used to make your drone enter a spiraling descent, for instance. We'll also store the PWM endpoints and saturate the command at those values - useful for keeping your servos from running into linkage hard stops. Finally, each output channel is a sum of gains on the SBUS channels. These will operate on the normalized values, so a gain of 1 in the 4th index would mean that this output channel maps directly to SBUS channel 4. You can reverse the direction by using a negative gain and you can sum multiple channels (i.e. for elevon or V tail mixing) by mapping multiple input channels to a single output. Why a 15 size array for gains? Because we're going to use the first SBUS channel as our throttle safety switch. There is also a bias, which is useful for trimming the surface.

We create an array of 7 to store all of our PWM Settings and then add the familiar PWM output setup.

```C++
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
```

### Helper Function
We can use a helper funtion to make setting up each PWM channel a little easier. It simply takes the different options as parameters and returns the built struct.

```C++
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
```

### Setup
In _setup_ we'll start our SBUS communication, set all of our PWM pins to output, and setup our servos. For this example, I am setting up a motor on PWM 1 and elevons on PWM 2 and PWM 3. Notice how I use the helper function to set each of these up.

```C++
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
```

### Loop
In _loop_ we'll look for a valid SBUS packet. Then we'll loop through each PWM channel and check whether we're in failsafe mode or if the channel is a motor and the motor is safed. If either of these are the case, we output the failsafe or minimum value, respectively. Otherwise, we use the gain information to create the PWM command, scale the output to a PWM value and saturate it against the defined limits. Finally, we send the commands to the servos.

```C++
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
```

# Experiment
Upload your code to Teensy, turn on your transmitter, and power your servos. You should see them move according to the configuration provided. Carefully, try testing the throttle safety switch to see how it prevents the motors from activating unless they are armed. Try turning off your transmitter and notice how the servos move to their failsafe positions. Try playing around a little with different gain and bias settings.

[![Experiment Video](https://img.youtube.com/vi/exdONVAs6Qo/0.jpg)](https://www.youtube.com/watch?v=exdONVAs6Qo)

# Wrap Up
In this project, we created an SBUS to PWM converter. Instead of just simply converting an SBUS channel directly to PWM, we can mix different inputs to a single output. We also added failsafe commands, end points, a throttle safety switch, and a bias feature for trimming individual surfaces.

# Next Steps
With all of these tools, you could try:
1. Setting up and flying your own RC aircraft or drone using this SBUS to PWM converter.
2. Adding exponential to the PWM configuration for nonlinear stick mapping.
3. Using a JSON file to define the configuration for each PWM channel instead of needing to define them in _setup_. [ArduinoJSON](https://arduinojson.org/) is a JSON utility that works on Teensy. [Protobufs](https://developers.google.com/protocol-buffers/), and specifically [Nanopb](https://jpa.kapsi.fi/nanopb/) is another good option.
4. Creating a GUI to configure each PWM channel and visualize the configuration and output.
5. Modifying this software to work as an input to your favorite aircraft simulator so that you can fly wirelessly with your transmitter. 

# Tutorials in the Series
1. [PWM Introduction](https://github.com/bolderflight/PWM-Intro): Learn about the de facto standard for controlling servos, PWM!
2. [Reading PWM](https://github.com/bolderflight/Reading-PWM): Gain experience measuring pulse widths!
3. [SBUS Introduction](https://github.com/bolderflight/SBUS-Intro): Learn about one of the newest, coolest methods of commanding servos, SBUS!
4. [Reading SBUS](https://github.com/bolderflight/Reading-SBUS): Gain experience reading SBUS packets!
5. [SBUS to PWM Converter](https://github.com/bolderflight/SBUS-to-PWM): Use your knowledge of PWM and SBUS to create your own SBUS to PWM converter!
