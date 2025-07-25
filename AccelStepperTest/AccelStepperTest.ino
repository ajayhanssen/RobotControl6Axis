#include <AccelStepper.h>

#define stepPin1 2
#define dirPin1 3

#define stepPin2 4
#define dirPin2 5

#define stepsize 0.45

AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
//AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);

void setup() {

  Serial.begin(9600);

  stepper1.setMaxSpeed(1000); // 400 alt
  stepper1.setAcceleration(400); //240 alt
  stepper1.moveTo(400);

  //stepper1.setMinPulseWidth(5);

  //stepper2.setMaxSpeed(1000);
  //stepper2.setAcceleration(100);
  //stepper2.moveTo(800);
}

void loop() {
  stepper1.run();

  if(stepper1.currentPosition()==0){
    stepper1.moveTo(400);
  }

  if(stepper1.currentPosition()==400){
    stepper1.moveTo(0);
  }
  
  
}
