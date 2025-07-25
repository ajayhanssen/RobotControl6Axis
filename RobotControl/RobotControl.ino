#include "pinDefinitions.hpp"
#include "JointClass.hpp"

int refSpeed = 1000;

// Joint(int stepPin, int dirPin, int homePin, int acc, float gearRatio, float stepSize, float upperBoundDEG, float lowerBoundDEG)
Joint J1 = Joint(J1STEP, J1DIR, J1HOME, 20.0f, 0.45f, 360.0f, 0.0f);
Joint J2 = Joint(J2STEP, J2DIR, J2HOME, 20.0f, 0.45f, 360.0f, 0.0f);

Joint J5 = Joint(J5STEP, J5DIR, J5HOME, 20.0f, 0.45f, 360.0f, 0.0f);
Joint J6 = Joint(J6STEP, J6DIR, J6HOME, 20.0f, 0.45f, 360.0f, 0.0f);

void setup() {
  // set target rotations of joint in degrees (brabr)
  float targets[] = {90.0f, 180.0f, 90.0f, 180.0f};

  // joint pointer array
  Joint* joints[] = {&J1, &J2, &J5, &J6};

  // set vel and accel according to travel dist, with max v (also set position)
  // moveJointsTo(float targets[], int numJoints, Joint* joints[], int refSpeed, int refAccel, int minSpeed, int minAccel)
  moveJointsTo(targets, 4, joints, 1000, 400, 0, 0);
}

void loop() {
  // Call all stepper rumn methods
  J1.run();
  J2.run();
  J5.run();
  J6.run();
}
