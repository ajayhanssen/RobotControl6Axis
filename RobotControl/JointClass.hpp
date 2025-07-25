#pragma once

#include <AccelStepper.h>

class Joint{
  private:
    AccelStepper __stepper;
    float __gearRatio;
    float __positionDEG;
    float __stepSize;
    float __upperBoundDEG;
    float __lowerBoundDEG;
    int __homePin;

  public:
    Joint(int stepPin, int dirPin, int homePin, int acc, float gearRatio, float stepSize, float upperBoundDEG, float lowerBoundDEG)
    : __stepper(AccelStepper::DRIVER, stepPin, dirPin),
      __gearRatio(gearRatio),
      __stepSize(stepSize),
      __upperBoundDEG(upperBoundDEG),
      __lowerBoundDEG(lowerBoundDEG),
      __homePin(homePin)
    {
      __stepper.setAcceleration(acc);
      pinMode(__homePin, INPUT);
    }

    int degPos2stepPos(float deg){
      return (deg * __gearRatio) / __stepSize;
    }

    float stepPos2degPos(int steps){
      return (steps * __stepSize) / __gearRatio;
    }

    void setSpeed(int speed){
      __stepper.setMaxSpeed(speed);
    }

    void setAcceleration(int acc){
      __stepper.setAcceleration(acc);
    }

    void setTargetRotJoint(float deg){
      int targetPos = degPos2stepPos(deg);
      int upperBoundSTEP = degPos2stepPos(__upperBoundDEG);
      int lowerBoundSTEP = degPos2stepPos(__lowerBoundDEG);
      if (targetPos <= upperBoundSTEP && targetPos >= lowerBoundSTEP){
        __stepper.moveTo(targetPos);
      }
    }

    long getPosition(){ return __stepper.currentPosition(); }

    float getGearRatio(){ return __gearRatio; }

    float getStepSize(){ return __stepSize; }

    void setPosition(int position){
      __stepper.setCurrentPosition(position);
    }

    void run(){
      __stepper.run();
    }
};

void moveJointsTo(float targets[], int numJoints, Joint* joints[], int refSpeed){
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
    } else {
      float scale = (float)deltas[i] / (float)maxDelta;
      joints[i]->setSpeed(refSpeed * scale);
    }
  }
}

// J1 = Joint(1, 2, 16, 20.0f, 0.45f, 180.0f, 0.0f);