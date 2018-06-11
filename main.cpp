/*
Written by Peng Hoe Hor for our Warman Team, Professional Mechanical Teletubbies.
It is designed to be run in an Arduino UNO, however other versions of Arduino should
work fine. (Apologies for the spagetti code, I didnt have much time to do it)

MIT LICENSE
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in the
Software without restriction, including without limitation the rights to use, copy,
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

/*  Note : Int data type only carries 16 bits, same with short.
Long, double data types carries 32 bits.
*/

#include "AccelStepper.h"


//Declare pins  note only 1-7 have pulse width modulation needed to drive the driver.
const unsigned int RIGHT_MOTOR_STEP = 2;
const unsigned int LEFT_MOTOR_STEP = 3;

const unsigned int RIGHT_MOTOR_DIR = 8;
const unsigned int LEFT_MOTOR_DIR = 9;

const unsigned int LIFT_RELAY = 10;
const unsigned int LIFT_REVERSE_PIN = 11;

const unsigned int SLIDE_MOTOR = 7;
const unsigned int GRAB_MOTOR = 6; //only one pin as both outputs to the same thing.

const unsigned short PIN_ARRAY[] = {2,3,6,7,8,9,10,11}; //for pin init at setup()

//Motor properties
const unsigned int MAX_SPEED = 200; //200 Steps per second, 1 RPS
const unsigned int MAX_ACCELERATION = 400;  //v^2=u^2+2at, 400 Steps per second^2
const unsigned int STEPS_PER_REV = 200;

//Wheel properties
const unsigned int WHEEL_DIAMETER = 100; //100mm
const unsigned int DISTANCE_BETWEEN_WHEELS = 140; //140mm

//USER DEFINED CONSTANTS

//Moving distance in mm
const float DISTANCE_1 = 10.0;
const float DISTANCE_2 = 10.0;
const float DISTANCE_3 = 10.0;
const float DISTANCE_4 = 10.0;
const float DISTANCE_5 = 10.0;

//Lift properties
const unsigned int LIFT_EXTEND_TIME = 20000; //time to extend the lift to needed length, unit ms

//Time taken for motor to grab the plate
const unsigned int GRAB_TIME = 2000; //unit ms

//Time taken for slide to activate
const unsigned int SLIDE_TIME = 2000; //unit ms


//Create AccelStepper object

AccelStepper rightMotor(
  1,
  RIGHT_MOTOR_STEP,
  RIGHT_MOTOR_DIR
);

AccelStepper leftMotor(
  1,
  LEFT_MOTOR_STEP,
  LEFT_MOTOR_DIR
);

//Create Direction enum
enum Direction {
  CLOCKWISE,
  ANTI_CLOCKWISE
};

//CODE START BELOW

void lift_extend(){
  //activates the relay by LIFT_EXTEND_TIME duration.
  //The relay is set to positive voltage and extends
  digitalWrite(LIFT_RELAY, HIGH);
  delay(LIFT_EXTEND_TIME);
  digitalWrite(LIFT_RELAY, LOW);
}

void lift_retract(){
  //activates the relay by LIFT_EXTEND_TIME duration.
  //The relay is set to negative voltage and retracts
  digitalWrite(LIFT_REVERSE_PIN, HIGH);
  delay(LIFT_EXTEND_TIME);
  digitalWrite(LIFT_REVERSE_PIN, LOW);
}

void grab_payload(){
  //Activate grab motor for the GRAB_TIME long.
  digitalWrite(GRAB_MOTOR, HIGH);
  delay(GRAB_TIME);
  digitalWrite(GRAB_MOTOR, LOW);
}

void activate_slide(){
  //Activate slide motor for SLIDE_TIME long.
  digitalWrite(SLIDE_MOTOR, HIGH);
  delay(SLIDE_TIME);
  digitalWrite(SLIDE_MOTOR, LOW);
}

void move_until(){
  //See if the stepper motor needs moving, then loop until the motor does need to move.
  //This function do not take in any parameter so please define step distance before calling this function
  while( (rightMotor.isRunning() == 1 ) || ( leftMotor.isRunning() == 1 ) ){
    //isRunning is used instead of distanceToGo as comparing bool is faster
    rightMotor.run();
    leftMotor.run();
  }
}

long distance_mm_to_step(float distance_mm){
//Convert distance mm to distance step
long distance_step = round( (float)distance_mm / ( ( PI * (float)WHEEL_DIAMETER ) / (float)STEPS_PER_REV ) );

return distance_step;
}

void move_forward(float distance_mm){
  //Triggers both motors to move forward sent distance
  leftMotor.move(distance_mm_to_step(distance_mm));
  rightMotor.move(distance_mm_to_step(distance_mm));
  move_until();
}


void rotate(Direction direction){
  // rotates the robot 90 degrees in the specified direction
  float turn_distance = 1.0/2.0 * PI * (float)DISTANCE_BETWEEN_WHEELS;

  //change motor direction if going anti clockwise
  if (direction == ANTI_CLOCKWISE){
    turn_distance = -turn_distance;
  }

  leftMotor.move(distance_mm_to_step(turn_distance));
  move_until();
};

void setup(){
  //set max speeds
  rightMotor.setMaxSpeed( MAX_SPEED );
  leftMotor.setMaxSpeed( MAX_SPEED );

  // set acceleration
  rightMotor.setAcceleration( MAX_ACCELERATION );
  leftMotor.setAcceleration( MAX_ACCELERATION );

  //init all pins
  for( int i = 0; i< 8; i++ ){
    pinMode( PIN_ARRAY[i], OUTPUT );
  }

  //unit test,
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);

  //set data rate to 9600 baud per second
  Serial.begin( 9600 );
}

void loop(){
  //Get into lift into position
  move_forward(DISTANCE_1);
  digitalWrite(LED_BUILTIN,LOW);
  rotate(CLOCKWISE);
  move_forward(DISTANCE_2);

  //Raise lift and grab, then retract lift
  lift_extend();
  grab_payload();
  lift_retract();

  //Move into final position
  move_forward(DISTANCE_3);
  rotate(CLOCKWISE);
  move_forward(DISTANCE_4);
  rotate(ANTI_CLOCKWISE);
  move_forward(DISTANCE_5);

  //release payload
  activate_slide();

  delay(1000000000); //Make robot stay there for 278 hrs... More than enough time to retrive it.
}
