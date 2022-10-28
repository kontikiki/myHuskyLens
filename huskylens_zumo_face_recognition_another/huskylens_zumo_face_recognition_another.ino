//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
#include "HUSKYLENS.h"
#include <PIDLoop.h>
#include <ZumoMotors.h>

// this limits how fast Zumo travels forward (400 is max possible for Zumo)
#define MAX_TRANSLATE_VELOCITY  250

HUSKYLENS huskylens;
ZumoMotors motors;

//PIDLoop panLoop(350, 0, 600,true);
PIDLoop tiltLoop(500, 0, 700, true);

PIDLoop rotateLoop(300, 400, 300, false);
PIDLoop translateLoop(400, 800, 300, false);

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");

  // initialize motor objects
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  servoReady();
  servoOperation(50);

  Wire.begin();
  while (!huskylens.begin(Wire))
  {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }

  huskylens.writeAlgorithm(ALGORITHM_FACE_RECOGNITION); //Switch the algorithm to object tracking
}

void servoReady()
{
  DDRD |= (1 << PD3); //enable pin3 output

  TCCR2A |= (1 << WGM20) | (1 << WGM21);  //set fast pwm mode
  TCCR2B |= (1 << WGM22); //set fast pwm mode
  TCCR2A |= (1 << COM2B1);  //set not invert mode
  TCCR2B |= (1 << CS02) | (1 << CS01) | (1 << CS00);  //set prescaler 1024

  OCR2A = 156;  //set top value
}

void servoOperation(float commmand)
{
  OCR2B = map(1000+commmand, 0, 20000, 0, 156);  //set compare value (set duty cycle)
}

void loop()
{
  float panOffset, tiltOffset, headingOffset, left, right;

  huskylens.request();
  HUSKYLENSResult result = huskylens.read();
  if (result.width != -1) {

    ///////////////////////////////////////////////
    panOffset = 160 - result.xCenter;
    headingOffset = result.height;
    tiltOffset = result.yCenter - 120;

    rotateLoop.update(panOffset);
    translateLoop.update(headingOffset);
    tiltLoop.update(tiltOffset);

    ////////////////////////////////////////////
     // calculate left and right wheel velocities based on rotation and translation velocities
    left = -rotateLoop.m_command;
    right = rotateLoop.m_command;
    Serial.println("leftSpeed:"+String(left));
    Serial.println("rightSpeed:"+String(right));
    // set wheel velocities
    motors.setLeftSpeed(left * 1.5);
    motors.setRightSpeed(right * 1.5);

    ////////////////////////////////////////////////////
    // keep translation velocity below maximum
    if (translateLoop.m_command > MAX_TRANSLATE_VELOCITY)
      translateLoop.m_command = MAX_TRANSLATE_VELOCITY;

    if (headingOffset < 100)
    {
      motors.setLeftSpeed(translateLoop.m_command);
      motors.setRightSpeed(translateLoop.m_command);
    }
    else if (headingOffset > 130)
    {
      motors.setLeftSpeed(-translateLoop.m_command);
      motors.setRightSpeed(-translateLoop.m_command);
    }
    /////////////////////////////////////////////////////
    
   //OCR2B = map(1000 + tiltLoop.m_command, 0, 20000, 0, 156);
    servoOperation(tiltLoop.m_command);
    
    delay(10);
    //////////////////////////////////////////////////////
  }
  else {
    rotateLoop.reset();
    //    tiltLoop.reset();
    translateLoop.reset();
    motors.setLeftSpeed(0);
    motors.setRightSpeed(0);
    //servoOperation(30);
    Serial.println("no_object_detected");
  }
}
