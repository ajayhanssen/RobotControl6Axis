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

float targets[NUM_JOINTS] = {0.0f};
int16_t receivedVals[NUM_JOINTS];

// joint pointer array
Joint* joints[] = {&J1, &J2, &J3, &J4, &J5, &J6};

enum ProdState{ // State machine for robot in general
      IDLE,
      HOME123,
      STDCONFIG123,
      HOME4,
      STDCONFIG4,
      HOME6,
      HOMINGPOS5,
      HOME5,
      STDCONFIG56,
      LISTENING,
      MOVING
    }
prodState = LISTENING;

void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void setup() {

  Serial.begin(9600);
  // set target rotations of joint in degrees (brabr)
  //float targets[] = {90.0f, 180.0f, 90.0f, 180.0f, 90.0f, 180.0f};

  // set vel and accel according to travel dist, with max v (also set position)
  // moveJointsTo(float targets[], int numJoints, Joint* joints[], int refSpeed, int refAccel, int minSpeed, int minAccel)
  //moveJointsTo(targets, 6, joints, 1000, 400, 0, 0);
}

void loop() {

  switch (prodState){
    case IDLE: // Do nothing, wait for start signal
      if (true){ // do start button later
        prodState = HOME123;
        J1.startHoming();
        J2.startHoming();
        J3.startHoming();
      }
      break;
    
    //////////////
    //- homing -//
    //////////////

    // home the first three joints
    case HOME123:
      J1.updateHoming();
      J2.updateHoming();
      J3.updateHoming();
      if (J1.isHomed() && J2.isHomed() && J3.isHomed()){
        prodState = STDCONFIG123;
        Joint* joints123[] = {&J1, &J2, &J3};
        float targets123[] = {J1HOME2STDDEG, J2HOME2STDDEG, J3HOME2STDDEG};
        moveJointsTo(targets123, 3, joints123, refSpeed, refAccel, minSpeed, minAccel);
      }
      break;

    // move first three joints to standard config
    case STDCONFIG123:
      J1.run();
      J2.run();
      J3.run();
      if (J1.getPosition() == J1.degPos2stepPos(J1HOME2STDDEG) && J2.getPosition() == J2.degPos2stepPos(J2HOME2STDDEG)
       && J3.getPosition() == J3.degPos2stepPos(J3HOME2STDDEG)){
        prodState = HOME4;
        J4.startHoming();
      }
      break;

    // home joint 4
    case HOME4:
      J4.updateHoming();
      if (J4.isHomed()){
        prodState = STDCONFIG4;
        Joint* joints4[] = {&J4};
        float targets4[] = {J4HOME2STDDEG};
        moveJointsTo(targets4, 1, joints4, refSpeed, refAccel, minSpeed, minAccel);
      }
      break;

    // move to joint 4 standard config
    case STDCONFIG4:
      J4.run();
      if(J4.getPosition() == J4.degPos2stepPos(J4HOME2STDDEG)){
        prodState = HOME6;
        J6.startHoming();
      }
      break;
    
    // home joint 6
    case HOME6:
      J6.updateHoming();
      if (J6.isHomed()){
        prodState = HOMINGPOS5;
        Joint* joints6[] = {&J6};
        float targets6[] = {J6HOMEPOS5};
        moveJointsTo(targets6, 1, joints6, refSpeed, refAccel, minSpeed, minAccel);
      }
      break;
    
    // move joint 6 to position to be able to home joint 5
    case HOMINGPOS5:
      J6.run();
      if (J6.getPosition() == J6.degPos2stepPos(J6HOMEPOS5)){
        prodState = HOME5;
        J5.startHoming();
      }
      break;

    // home joint 5
    case HOME5:
      J5.updateHoming();
      if (J5.isHomed()){
        prodState = STDCONFIG56;
        Joint* joints56[] = {&J5, &J6};
        float targets56[] = {J5HOME2STDDEG, J6HOME2STDDEG};
        moveJointsTo(targets56, 2, joints56, refSpeed, refAccel, minSpeed, minAccel);
      }
      break;

    // move joints 5 and 6 to standard config
    case STDCONFIG56:
      J5.run();
      J6.run();
      if (J5.getPosition() == J5.degPos2stepPos(J5HOME2STDDEG) && J6.getPosition() == J6.degPos2stepPos(J6HOME2STDDEG)){
        prodState = LISTENING;
      }
      break;


    ////////////////////////
    //- normal operation -//
    ////////////////////////

    case LISTENING:
      
      if (Serial.available() >= NUM_JOINTS * 2){
        uint8_t buffer[NUM_JOINTS * 2];
        Serial.readBytes(buffer, NUM_JOINTS * 2);

        for (int i=0; i < NUM_JOINTS; i++){
          // Matlab schickt Little-Endian -> erst niederwertiges Byte (buffer[2*i]), dann höherwertiges (buffer[2*i + 1])
        	// Bitverschiebung des höherwertigen Bytes um 8 nach links, bitweises ODER zum Hinzufügen des niederwertigen Bytes; casten zu int16_t
          receivedVals[i] = (int16_t)(buffer[2 * i + 1] << 8 | buffer[2 * i]);
          targets[i] = (float)receivedVals[i] / 100.0f;
        }
        prodState = MOVING;
        moveJointsTo(targets, NUM_JOINTS, joints, refSpeed, refAccel, minSpeed, minAccel);
      }
      break;
    
    case MOVING:
      runJoints(joints, NUM_JOINTS);
      if (JointsReachedTargets(targets, NUM_JOINTS, joints)){
        prodState = LISTENING;
        clearSerialBuffer();
      }
      break;
  }
}
