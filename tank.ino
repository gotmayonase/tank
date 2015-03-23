#include <SPI.h>
#include <EEPROM.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include "Boards.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define PROTOCOL_MAJOR_VERSION   0 //
#define PROTOCOL_MINOR_VERSION   0 //
#define PROTOCOL_BUGFIX_VERSION  2 // bugfix

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // turn on motor
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  ble_begin();
}


void loop() {
  while(ble_available())
  {
    byte cmd = ble_read();
    byte motor_direction = ble_read();
    byte motor_speed = ble_read();  
    switch(cmd) {
      
      case 0x01:
        {
          set_motor_speed(motor_direction, rightMotor, motor_speed);
        }
        break;
        
      case 0x00:
        {
          set_motor_speed(motor_direction, leftMotor, motor_speed);
        }
        break;
    }
    
    delay(10);    
    return;

  }
  
  if(!ble_connected())
  {
    set_motor_speed(0x01, leftMotor, 0);
    set_motor_speed(0x01, rightMotor, 0);
  }

  ble_do_events();
  delay(50);
}

void set_motor_speed(char motor_direction, Adafruit_DCMotor *motor, byte motor_speed)
{
  if(motor_direction == 0x01)
  {
    motor->run(FORWARD);
  } else {
    motor->run(BACKWARD);
  }
  
  motor->setSpeed(motor_speed);
}
