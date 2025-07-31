#pragma once

#define STEPSIZE 0.45f

#define J1STEP 12                   // Pin connected to STEP pulse 
#define J1DIR 13                    // Pin connected to DIR input
#define J1HOME 21                   // Pin connected to homing sensor
#define J1GEARRATIO 6.4f            // Gear ratio of connected gear
#define J1UPPERBOUNDDEG 123.04f     // Upper boundary for joint
#define J1LOWERBOUNDDEG -123.04f    // Lower boundary for joint
#define J1HOME2STDDEG 0.0f         // Degrees from homing position to standard config

#define J2STEP 10
#define J2DIR 11
#define J2HOME 21
#define J2GEARRATIO 20.0f
#define J2UPPERBOUNDDEG -3.37f
#define J2LOWERBOUNDDEG -145.0f
#define J2HOME2STDDEG -90.0f

#define J3STEP 8
#define J3DIR 9
#define J3HOME 21
#define J3GEARRATIO 18.095f
#define J3UPPERBOUNDDEG 287.86f
#define J3LOWERBOUNDDEG 107.86f
#define J3HOME2STDDEG 180.0f

#define J4STEP 6
#define J4DIR 7
#define J4HOME 21
#define J4GEARRATIO 4.0f
#define J4UPPERBOUNDDEG 105.46f
#define J4LOWERBOUNDDEG -105.47f
#define J4HOME2STDDEG 0.0f

#define J5STEP 4
#define J5DIR 5
#define J5HOME 21
#define J5GEARRATIO 4.0f
#define J5UPPERBOUNDDEG 90.0f
#define J5LOWERBOUNDDEG -90.0f
#define J5HOME2STDDEG 0.0f

#define J6STEP 2
#define J6DIR 3
#define J6HOME 21
#define J6GEARRATIO 10.0f
#define J6UPPERBOUNDDEG 360.0f
#define J6LOWERBOUNDDEG 0.0f
#define J6HOME2STDDEG 180.0f
#define J6HOMEPOS5 45.0f