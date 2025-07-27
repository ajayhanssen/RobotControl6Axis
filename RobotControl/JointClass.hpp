#pragma once

#include <AccelStepper.h>

// class representing singular joint/stepper incl gearbox
class Joint{
  private:
    AccelStepper __stepper;       // AccelStepper object controlling steppermom
    float __gearRatio;            // gearratio from motor to output
    float __stepSize;             // step size of stepper motor (1.8, 0.9, 0.45, 0.225, 0.1125)
    float __upperBoundDEG;        // upper boundary limiting the output position
    float __lowerBoundDEG;        // lower boundary limiting the output position
    int __homePin;                // pin holding the input of the homing sensordsd
    enum HomingState{
      IDLE,
      SEEK_HOME,
      BACKOFF,
      APPROACH,
      HOMED
    };

  public:
    Joint(int stepPin, int dirPin, int homePin, float gearRatio, float stepSize, float upperBoundDEG, float lowerBoundDEG)
    : __stepper(AccelStepper::DRIVER, stepPin, dirPin),
      __gearRatio(gearRatio),
      __stepSize(stepSize),
      __upperBoundDEG(upperBoundDEG),
      __lowerBoundDEG(lowerBoundDEG),
      __homePin(homePin)
    {
      pinMode(__homePin, INPUT);  // set homing pin to input
    }

    // calculate a position in steps from given position in degrees
    int degPos2stepPos(float deg){
      return (deg * __gearRatio) / __stepSize;
    }

    // calculate a position in degrees from given position in stepmoms
    float stepPos2degPos(int steps){
      return (steps * __stepSize) / __gearRatio;
    }

    // setting the target rotation of the stepper in steps from degs given
    void setTargetRotJoint(float deg){
      int targetPos = degPos2stepPos(deg);
      int upperBoundSTEP = degPos2stepPos(__upperBoundDEG);
      int lowerBoundSTEP = degPos2stepPos(__lowerBoundDEG);
      if (targetPos <= upperBoundSTEP && targetPos >= lowerBoundSTEP){
        __stepper.moveTo(targetPos);
      }
    }

    // function for homing, blocking leider
    void home(){
      
      // move towards sensor with vollgas
      __stepper.setMaxSpeed(500);
      __stepper.setAcceleration(300);
      __stepper.move(-100000);

      // while sensor is not triggered, run
      while(digitalRead(__homePin) == LOW){
        __stepper.run();
      }

      // stop as soon as sens reached
      __stepper.stop();

      // wart gach
      delay(100);

      // move back a little
      __stepper.move(500);
      while(__stepper.distanceToGo() != 0){
        __stepper.run();
      }

      // wart nomal gach
      delay(100);

      // move towards sensor with halbgas
      __stepper.setMaxSpeed(100);
      __stepper.setAcceleration(50);
      __stepper.move(-10000);

      // while sensor not triggered, run again
      while(digitalRead(__homePin) == LOW){
        __stepper.run();
      }

      // stop if sensor reached
      __stepper.stop();

      // set position to zero
      __stepper.setCurrentPosition(0);
    }

    // setter for setting stepper speed gwaungi
    void setSpeed(int speed){ __stepper.setMaxSpeed(speed); }

    // setter for setting stepper acceleration
    void setAcceleration(int acc){ __stepper.setAcceleration(acc); }

    // setter for setting current position counter to certain schall-weli
    void setPosition(int position){ __stepper.setCurrentPosition(position); }

    // getter for current pos of stepper
    long getPosition(){ return __stepper.currentPosition(); }

    void run(){
      __stepper.run();
    }
};

void moveJointsTo(float targets[], int numJoints, Joint* joints[], int refSpeed, int refAccel, int minSpeed, int minAccel){
  long deltas[numJoints];
  long maxDelta = 0;

  for (int i=0; i < numJoints; i++){
    long currentPos = joints[i]->getPosition();
    long targetPos = joints[i]->degPos2stepPos(targets[i]);
    long delta = abs(targetPos - currentPos);
    deltas[i] = delta;
    if (delta > maxDelta) {
      maxDelta = delta;
    }
    joints[i]->setTargetRotJoint(targets[i]);
  }

  for (int i=0; i < numJoints; i++){
    if (maxDelta == 0){
      joints[i]->setSpeed(0);
      joints[i]->setAcceleration(0);
    } else {
      float scale = (float)deltas[i] / (float)maxDelta;
      int speed = max(minSpeed, (int)(refSpeed * scale));
      int accel = max(minAccel, (int)(refAccel * scale));
      joints[i]->setSpeed(speed);
      joints[i]->setAcceleration(accel);
    }
  }
}


