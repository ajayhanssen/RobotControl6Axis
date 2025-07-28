#pragma once

#define STEPSIZE 0.45f

#define J1STEP 10                   // Pin connected to STEP pulse 
#define J1DIR 11                    // Pin connected to DIR input
#define J1HOME 12                   // Pin connected to homing sensor
#define J1GEARRATIO 20.0f           // Gear ratio of connected gear
#define J1UPPERBOUNDDEG 360.0f      // Upper boundary for joint
#define J1LOWERBOUNDDEG 0.0f        // Lower boundary for joint
#define J1HOME2STDDEG 0.0f          // Degrees from homing position to standard config

#define J2STEP 8
#define J2DIR 9
#define J2HOME 12
#define J2GEARRATIO 20.0f
#define J2UPPERBOUNDDEG 360.0f
#define J2LOWERBOUNDDEG 0.0f
#define J2HOME2STDDEG 0.0f

#define J3STEP 6
#define J3DIR 7
#define J3HOME 12
#define J3GEARRATIO 20.0f
#define J3UPPERBOUNDDEG 360.0f
#define J3LOWERBOUNDDEG 0.0f
#define J3HOME2STDDEG 0.0f

#define J4STEP 4
#define J4DIR 5
#define J4HOME 15
#define J4GEARRATIO 20.0f
#define J4UPPERBOUNDDEG 360.0f
#define J4LOWERBOUNDDEG 0.0f
#define J4HOME2STDDEG 0.0f

#define J5STEP 2
#define J5DIR 3
#define J5HOME 12
#define J5GEARRATIO 20.0f
#define J5UPPERBOUNDDEG 360.0f
#define J5LOWERBOUNDDEG 0.0f
#define J5HOME2STDDEG 0.0f

#define J6STEP 0
#define J6DIR 1
#define J6HOME 15
#define J6GEARRATIO 20.0f
#define J6UPPERBOUNDDEG 360.0f
#define J6LOWERBOUNDDEG 0.0f
#define J6HOME2STDDEG 0.0f
#define J6HOMEPOS5 45.0f