/****************************************************************************************************************************
  basic_pwm.ino
  For RP2040 boards
  Written by Dr. Benjamin Bird

  A basic example to get you up and running.

  Library by Khoi Hoang https://github.com/khoih-prog/RP2040_PWM
  Licensed under MIT license

  The RP2040 PWM block has 8 identical slices. Each slice can drive two PWM output signals, or measure the frequency
  or duty cycle of an input signal. This gives a total of up to 16 controllable PWM outputs. All 30 GPIO pins can be driven
  by the PWM block
*****************************************************************************************************************************/

#include <Arduino.h>    
#include <vector>
#include "RP2040_PWM.h"
#define _PWM_LOGLEVEL_ 3 


RP2040_PWM* PWM_instances[4];

float frequency=50;

#define Servo1pin 0
#define Servo2pin 1
#define Servo3pin 2 
#define Servo4pin 3

struct Servo_t {
  int pin;
  float dc_min, dc_max;
};


Servo_t Servo1 = {Servo1pin, 2, 13};
Servo_t Servo2 = {Servo2pin, 2, 13};
Servo_t Servo3 = {Servo3pin, 2, 13};
Servo_t Servo4 = {Servo4pin, 3, 14};

void initServos(){

  PWM_instances[0] = new RP2040_PWM(Servo1.pin, frequency, Servo1.dc_min);
  PWM_instances[1] = new RP2040_PWM(Servo2.pin, frequency, Servo2.dc_min);
  PWM_instances[2] = new RP2040_PWM(Servo3.pin, frequency, Servo3.dc_min);
  PWM_instances[3] = new RP2040_PWM(Servo4.pin, frequency, Servo4.dc_min);
}

void setServo(Servo_t Servo, int angle){  //angle is between 0 and 180
  
  float dutyCycle = Servo.dc_min + (Servo.dc_max - Servo.dc_min) * (angle/180.0); //calculates dutyCycle from angle
  Serial.print("dutyCycle: ");
  Serial.println(dutyCycle);
  PWM_instances[Servo.pin]->setPWM(Servo.pin, frequency, dutyCycle); //sets dutyCycle in servo
}

void setup()
{
  Serial.begin(115200);
  initServos();
}

void loop()
{
  
  setServo(Servo1, 90);
  setServo(Servo2, 90);
  setServo(Servo3, 90);
  setServo(Servo4, 90);
  
}