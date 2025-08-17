#pragma once

#define STEPSIZE 0.45f

#define ENABLEPIN 18

#define J1STEP 27                   // Pin connected to STEP pulse 
#define J1DIR 14                    // Pin connected to DIR input
#define J1HOME 36                   // Pin connected to homing sensor
#define J1GEARRATIO 6.4f            // Gear ratio of connected gear
#define J1UPPERBOUNDDEG 123.04f     // Upper boundary for joint
#define J1LOWERBOUNDDEG -123.04f    // Lower boundary for joint
#define J1HOME2STDDEG 0.0f          // Degrees from homing position to standard config
#define J1HOMEDIR NEGATIVE          // which direction to move for homing

#define J2STEP 12
#define J2DIR 13
#define J2HOME 39
#define J2GEARRATIO 20.0f
#define J2UPPERBOUNDDEG -3.37f
#define J2LOWERBOUNDDEG -145.0f
#define J2HOME2STDDEG -90.0f
#define J2HOMEDIR NEGATIVE

#define J3STEP 25
#define J3DIR 26
#define J3HOME 5
#define J3GEARRATIO 18.095f
#define J3UPPERBOUNDDEG 287.86f
#define J3LOWERBOUNDDEG 107.86f
#define J3HOME2STDDEG 180.0f
#define J3HOMEDIR POSITIVE

#define J4STEP 32
#define J4DIR 33
#define J4HOME 17
#define J4GEARRATIO 4.0f
#define J4UPPERBOUNDDEG 105.46f
#define J4LOWERBOUNDDEG -105.47f
#define J4HOME2STDDEG 0.0f
#define J4HOMEDIR POSITIVE

#define J5STEP 21
#define J5DIR 19
#define J5HOME 16
#define J5GEARRATIO 4.0f
#define J5UPPERBOUNDDEG 90.0f
#define J5LOWERBOUNDDEG -90.0f
#define J5HOME2STDDEG 0.0f
#define J5HOMEDIR POSITIVE

#define J6STEP 23
#define J6DIR 22
#define J6HOME 4
#define J6GEARRATIO 10.0f
#define J6UPPERBOUNDDEG 360.0f
#define J6LOWERBOUNDDEG 0.0f
#define J6HOME2STDDEG 180.0f
#define J6HOMEPOS5 45.0f
#define J6HOMEDIR POSITIVE

#define POSITIVE 1
#define NEGATIVE 0