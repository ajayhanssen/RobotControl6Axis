#include "pinDefinitions.hpp"
#include "JointClass.hpp"

#define NUM_JOINTS 6

int refSpeed = 1000;
int refAccel = 400;
int minSpeed = 0;
int minAccel = 0;

// Joint(int stepPin, int dirPin, int homePin, float gearRatio, float stepSize, float upperBoundDEG, float lowerBoundDEG)
Joint J1 = Joint(J1STEP, J1DIR, J1HOME, J1GEARRATIO, STEPSIZE, J1UPPERBOUNDDEG, J1LOWERBOUNDDEG);
Joint J2 = Joint(J2STEP, J2DIR, J2HOME, J2GEARRATIO, STEPSIZE, J2UPPERBOUNDDEG, J2LOWERBOUNDDEG);
Joint J3 = Joint(J3STEP, J3DIR, J3HOME, J3GEARRATIO, STEPSIZE, J3UPPERBOUNDDEG, J3LOWERBOUNDDEG);
Joint J4 = Joint(J4STEP, J4DIR, J4HOME, J4GEARRATIO, STEPSIZE, J4UPPERBOUNDDEG, J4LOWERBOUNDDEG);
Joint J5 = Joint(J5STEP, J5DIR, J5HOME, J5GEARRATIO, STEPSIZE, J5UPPERBOUNDDEG, J5LOWERBOUNDDEG);
Joint J6 = Joint(J6STEP, J6DIR, J6HOME, J6GEARRATIO, STEPSIZE, J6UPPERBOUNDDEG, J6LOWERBOUNDDEG);

float targets[NUM_JOINTS] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
int16_t receivedVals[NUM_JOINTS];

// joint pointer array
Joint* joints[] = {&J1, &J2, &J3, &J4, &J5, &J6};

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  moveJointsTo(targets, 6, joints, 1000, 400, 0, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  runJoints(joints, NUM_JOINTS);
}
