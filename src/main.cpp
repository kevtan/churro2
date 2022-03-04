#include <Arduino.h>
#include <Metro.h>
#include <Churro.h>

/**
 * What we have achieved by thursday/early friday morning
 * -- ultrasonic sensor wired and working!
 * -- FSM for success
 * -- onboard electronics + chassis: can power all necessary components with our
 *      ``OG'' batteries. swag. 
 * -- hugeeee switch for deciding team red/blue
 * -- working intake + outtake mechanism  
 * 
 * TO DOs
 * 0. [INTAKE/HOPPER] TODO:#00 ADD THE HOPPER
 * 1. [MECHANICAL] Fix wheel imbalance
 *      TODO:#0 Loosen the tight wheels: one motor/wheel/axle is MUCH tighter 
 *      than the other, which causes the robot to TURN instead of drive forward.
 * 2. [SENSOR/CODE] 
 *      TODO:#1 Experiment with keeping/removing the delay at end of calculating
 *      the sensor distance
 *      TODO:#2 Implement ``smooth_IR:'' using a running average of (10) 
 *      ultrasonic distance readings. 
 *      TODO:#3 Calibrate/make sure sensor is accurate. (We kinda checked this) 
 * 3. [FSM/CODE]
 *      TODO:#4 Figure out how LONG the timer needs to be, for turning 90 degrees
 *      CLOCKWISE and COUNTER-CLOCKWISE. 
 *      TODO:#5 Figure out the THRESHOLD for when the robot SEES the wall.
 *      TODO:#6 Code up the rest of the FSM.
 * 4. [ELECTRICAL]
 *      TODO:#8 figure out WHY the regulator gets so HOT (14V -> 5V). Is it really
 *      because we've been using it for too long? 
 *      TODO:#9 figure out if it's OKAY/SAFE to use the motor driver's onboard 5V
 *      SUPPLY to drive our (1) Arduino and (2) our ultrasonic sensor. 
 * 5. Style (for da bonus points)
 *      TODO:#7 use `ifdef/endif' for SERIAL_ON instead of `if (SERIAL_ON)'
 */


/*---------------Threshold Definitions--------------------------*/
#define TURN_WALL_THRESHOLD 5//17
#define TURN_SCORE_THRESHOLD 8.5
#define TURN_WALL_DURATION 1000 // SET THIS LATER
#define TURN_SCORING_DURATION 1000 // SET THIS LATER

/*---------------General Constants --------------------------*/
#define SERIAL_ON true
#define SONAR_DELAY 0
// TODO: Keep array of 10 values and use that value (smooth IR)
#define MOTOR_SPEED 255

/*---------------Pin Definitions--------------------------*/
#define MOTOR_R_ENABLE 2
#define MOTOR_R_DIR_1 4//3
#define MOTOR_R_DIR_2 3//4
#define MOTOR_L_ENABLE 7
#define MOTOR_L_DIR_1 5//6
#define MOTOR_L_DIR_2 6//5
#define MOTOR_HOPPER_ENABLE 8
#define MOTOR_HOPPER_DIR_1 9
#define MOTOR_HOPPER_DIR_2 10
#define TRIGGER 18
#define ECHO 17
#define TEAM_PIN 14
#define MAX_INDEX 10

/*---------------State Definitions--------------------------*/
typedef enum
{
  MOVING_TOWARD_WALL,
  TURN_WALL,
  MOVING_TOWARD_SCORING,
  TURN_SCORING,
  DONE
} States_t;

/*---------------Module Variables---------------------------*/
Team team;
bool passed_junction_1, passed_junction_2;

volatile States_t state;
volatile int curr_dist;

static Metro turnWallTimer = Metro(TURN_WALL_DURATION);  // 90 degree turn one-way
static Metro turnScoringTimer = Metro(TURN_SCORING_DURATION);  // 90 degree turn the-other-way
static Metro distanceTimer = Metro(SONAR_DELAY); // delay between computing sonar_distance
volatile int sonarvalues[MAX_INDEX+1];
volatile int sonar_index = 0;

/*---------------Function Prototypes---------------------------*/
void determine_team();
void initialize_motor_pins();
void motor_l_stop();
void motor_r_stop();
void motor_r_forward();
void motor_l_forward();
void motor_r_backward();
void motor_l_backward();
String print_state(int state);
void firstTurn();
void secondTurn();
void sonar_distance(int pingPin, int echoPin);

/* GOAL: Code up the FSM for the robot

   States: 
   - MOVING_FWD: starting state, move forward with same speed on both motors
   - TURN_WALL: moving towards side wall/other loading zone
   - TURN_SCORING: moving towards scoring zone
   - DONE: beat brick!
 */

void setup() {
  // put your setup code here, to run once:
  if (SERIAL_ON) Serial.begin(9600);

  initialize_motor_pins();
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);

  curr_dist = 84;
  delay(8000);

  determine_team();

  distanceTimer.reset();

  state = MOVING_TOWARD_WALL;
  passed_junction_1 = false;
  passed_junction_2 = false;
}

void determine_team()
{
  team = digitalRead(TEAM_PIN) ? Team::RED : Team::BLUE;
  if (SERIAL_ON) Serial.print("GO ");
  if (SERIAL_ON) Serial.print(team == Team::RED ? "RED" : "BLUE");
  if (SERIAL_ON) Serial.println(" TEAM");
}

void initialize_motor_pins()
{
  pinMode(MOTOR_R_ENABLE, OUTPUT);
  pinMode(MOTOR_R_DIR_1, OUTPUT);
  pinMode(MOTOR_R_DIR_2, OUTPUT);
  pinMode(MOTOR_L_ENABLE, OUTPUT);
  pinMode(MOTOR_L_DIR_1, OUTPUT);
  pinMode(MOTOR_L_DIR_2, OUTPUT);
}

void motor_r_stop()
{
  digitalWrite(MOTOR_R_DIR_1, LOW);
  digitalWrite(MOTOR_R_DIR_2, LOW);
}

void motor_l_stop()
{
  // analogWrite(MOTOR_L_ENABLE, 150);
  digitalWrite(MOTOR_L_DIR_1, LOW);
  digitalWrite(MOTOR_L_DIR_2, LOW);
}

void motor_r_forward()
{
  // analogWrite(MOTOR_R_ENABLE, 150);
  digitalWrite(MOTOR_R_DIR_1, LOW);
  digitalWrite(MOTOR_R_DIR_2, HIGH);
}

void motor_l_forward()
{
  // analogWrite(MOTOR_L_ENABLE, 150);
  digitalWrite(MOTOR_L_DIR_1, LOW);
  digitalWrite(MOTOR_L_DIR_2, HIGH);
}

void motor_r_backward()
{
  // analogWrite(MOTOR_R_ENABLE, 150);
  digitalWrite(MOTOR_R_DIR_1, HIGH);
  digitalWrite(MOTOR_R_DIR_2, LOW);
}

void motor_l_backward()
{
  // analogWrite(MOTOR_L_ENABLE, 150);
  digitalWrite(MOTOR_L_DIR_1, HIGH);
  digitalWrite(MOTOR_L_DIR_2, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  // if (SERIAL_ON) Serial.print("State: ");
  // if (SERIAL_ON) Serial.println(print_state(state));

  sonar_distance(TRIGGER, ECHO);

  if (SERIAL_ON) Serial.print("Distance : ");
  if (SERIAL_ON) Serial.println(curr_dist);

  switch (state) {
    case MOVING_TOWARD_WALL:
      // if we're 12 rad 2 = 17ish in. away from opposite wall
      if (!passed_junction_1 && curr_dist <= TURN_WALL_THRESHOLD) { // time to do an in-place turn
        if (SERIAL_ON) Serial.println("MOVING_TOWARD_WALL => TURN_WALL");
        state = TURN_WALL;
        turnWallTimer.reset();
        firstTurn();
        passed_junction_1 = true;
      // } else if (passed_junction_1 && curr_dist <= TURN_SCORE_THRESHOLD) { // time to do an in-place turn
      //   if (SERIAL_ON) Serial.println("MOVING_FWD => TURN_SCORING");
      //   state = TURN_SCORING;
      //   turnScoringTimer.reset();
      //   secondTurn();
      //   passed_junction_2 = true;
      } else { // still just moving forward
        // driveMotorsForward();
        motor_l_forward();
        motor_r_forward();
      }
      break;
    
    case TURN_WALL:
      if (turnWallTimer.check()) {
        // motor_l_forward();
        // motor_r_forward();
        motor_l_stop();
        motor_r_stop();
        state = MOVING_TOWARD_SCORING;
        if (SERIAL_ON) Serial.println("TURN_WALL => MOVING_TOWARD_SCORING");
      } else {
        // keep turning;
        firstTurn();
      }
      break;
    
    case MOVING_TOWARD_SCORING:
      motor_l_stop();
      motor_r_stop();
      break;
  
    default: 
      // if (SERIAL_ON) Serial.println("AHHHHHHHHH");
      motor_l_stop();
      motor_r_stop();
      break;
  }

  delay(500);
}

void firstTurn() {
  if (team == Team::RED){
    // turn right for X secs
    motor_l_forward();
    motor_r_backward();
  }
  else {
    // turn left for X secs 
    // driveLeftMotor(BACKWARD);
    motor_l_backward();
    // driveRightMotor(FORWARD);
    motor_r_forward();
  }
}

void secondTurn() {
  if (team == Team::RED) {
    // turn left for X secs
    motor_l_backward();
    motor_r_forward();
  }
  else {
    // turn right for X secs 
    motor_l_forward();
    motor_r_backward();
  }
}

String print_state(int state) {
  switch (state){
    case MOVING_TOWARD_WALL: 
      return "MOVING_FWD";

    case TURN_WALL: 
      return "TURN_WALL";
      
    case MOVING_TOWARD_SCORING: 
      return "MOVING_TOWARD_SCORING";

    /*  
    case TURN_SCORING: 
      return "TURN_SCORING";

    case DONE:
      return "DONE";
    */
   default:
    return "UNKNOWN";
  }
  return "UNKNOWN";
}

/* From https://www.tutorialspoint.com/arduino/arduino_ultrasonic_sensor.htm */
void sonar_distance(int pingPin, int echoPin) {
  if (distanceTimer.check()) {
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pingPin, LOW);
    long duration = pulseIn(echoPin, HIGH);

   if (sonar_index < MAX_INDEX) {
    sonarvalues[sonar_index] = (duration) / 74 / 2;
   }
   else {
      //verage of arry
      int sum = 0;
      for (int i = 0; i < MAX_INDEX; i++) {
        sum += sonarvalues[i];

      }
      curr_dist = sum/MAX_INDEX;
      sonar_index  =  0;
   }

    // cm = (duration) / 29 / 2;
    if (SERIAL_ON) Serial.print(curr_dist);
    if (SERIAL_ON) Serial.print("in, ");
    // if (SERIAL_ON) Serial.print(cm);
    // if (SERIAL_ON) Serial.print("cm");
    if (SERIAL_ON) Serial.println();
    distanceTimer.reset();
  }
}