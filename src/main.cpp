#include <Arduino.h> 
#include "RP2040_PWM.h"
#define _PWM_LOGLEVEL_ 3 

#define Servo1pin 0
#define Servo2pin 1
#define Servo3pin 2 
#define Servo4pin 3

struct Servo_t {
  int pin;
  float dc_min, dc_max;
  RP2040_PWM* PWM_instance;
};

float frequency=50; //50Hz

Servo_t Servo1 = {Servo1pin, 2, 12};
Servo_t Servo2 = {Servo2pin, 2.7, 12.8};
Servo_t Servo3 = {Servo3pin, 2, 12};
Servo_t Servo4 = {Servo4pin, 4, 14};

void initServos(){

  Servo1.PWM_instance = new RP2040_PWM(Servo1.pin, frequency, Servo1.dc_min);
  Servo2.PWM_instance = new RP2040_PWM(Servo2.pin, frequency, Servo2.dc_min);
  Servo3.PWM_instance = new RP2040_PWM(Servo3.pin, frequency, Servo3.dc_min);
  Servo4.PWM_instance = new RP2040_PWM(Servo4.pin, frequency, Servo4.dc_min);

}

void setServo(Servo_t Servo, int angle){  //angle is between 0 and 180
  
  float dutyCycle = (Servo.dc_max - Servo.dc_min)/180 * angle + Servo.dc_min; //calculates dutyCycle from angle
  //Serial.print("dutyCycle: ");
  //Serial.println(dutyCycle);
  Servo.PWM_instance->setPWM(Servo.pin, frequency, dutyCycle); //sets dutyCycle in servo
}

void setup()
{
  Serial.begin(115200);
  initServos();
}

void loop()
{
  /*
  float angle = 0;
  angle = Serial.read();
  */

  setServo(Servo1, 90);
  setServo(Servo2, 90);
  setServo(Servo3, 90);
  setServo(Servo4, 90);

  /*
  delay(1000);

  setServo(Servo1, 0);
  setServo(Servo2, 0);
  setServo(Servo3, 0);
  setServo(Servo4, 0);

  delay(1000);
  */
  
}