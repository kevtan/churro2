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
 * 0. [INTAKE/HOPPER+CODE] 
 *      TODO:#0 ADD THE HOPPER PHYSICALLY
 *      TODO:#1 ADD HOPPER TO FSM CODE
 * 1. [MECHANICAL] Fix wheel imbalance
 *      TODO:#2 Loosen the tight wheels: one motor/wheel/axle is MUCH tighter 
 *      than the other, which causes the robot to TURN instead of drive forward.
 * 2. [SENSOR/CODE] 
 *      TODO:#3 Experiment with keeping/removing the delay at end of calculating
 *      the sensor distance
 *      TODO:#4 Implement ``smooth_IR:'' using a running average of (10) 
 *      ultrasonic distance readings. 
 *      TODO:#5 Calibrate/make sure sensor is accurate. (We kinda checked this) 
 * 3. [FSM/CODE]
 *      TODO:#6 Figure out how LONG the timer needs to be, for turning 90 degrees
 *      CLOCKWISE and COUNTER-CLOCKWISE. 
 *      TODO:#7 Figure out the THRESHOLD for when the robot SEES the wall.
 *    DONE:#8 Code up the rest of the FSM.
 * 4. [ELECTRICAL]
 *      TODO:#9 figure out WHY the regulator gets so HOT (14V -> 5V). Is it really
 *      because we've been using it for too long? 
 *      TODO:#10 figure out if it's OKAY/SAFE to use the motor driver's onboard 5V
 *      SUPPLY to drive our (1) Arduino and (2) our ultrasonic sensor. 
 * 5. Style (for da bonus points)
 *    DONE:#11 use `ifdef/endif' for SERIAL_ON instead of `if (SERIAL_ON)'
 */


/*---------------Threshold Definitions--------------------------*/
#define TURN_WALL_THRESHOLD 45
#define TRAVEL_TO_LONG_WALL_THRESHOLD 8
#define TURN_SCORE_THRESHOLD 8.5
#define TURN_WALL_DURATION 465 // SET THIS LATER
#define TURN_SCORING_DURATION 1000 // SET THIS LATER
#define HOPPER_DURATION 10

/*---------------USER_DEFINED THINGS --------------------------*/
#define SERIAL_ON
#define TESTING_ROBOT
// #define HOPPER

#define SONAR_DELAY 0
#define MOTOR_SPEED 255
#define MAX_INDEX 10
#define INITIAL_WALL_DISTANCE 85
#define INITIAL_DELAY 2000

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
#define MAX_INDEX 1

/*---------------State Definitions--------------------------*/
typedef enum
{
  LOADING,
  MOVING_TO_WALL,
  TURN_WALL,
  MOVING_TO_SCORING,
  TURN_SCORING,
  MOVING_TO_DONE,
  DONE,
  UNLOADING,
} States_t;

/*---------------Module Variables---------------------------*/
Team team;
bool passed_junction_1, passed_junction_2;

volatile States_t state;
volatile int curr_dist;

static Metro hopperTimer = Metro(HOPPER_DURATION);
static Metro turnWallTimer = Metro(TURN_WALL_DURATION);  // 90 degree turn one-way
static Metro turnScoringTimer = Metro(TURN_SCORING_DURATION);  // 90 degree turn the-other-way

static Metro distanceTimer = Metro(SONAR_DELAY); // delay between computing sonar_distance
volatile int sonarvalues[MAX_INDEX+1];
volatile int sonar_index = 0;

/*---------------Function Prototypes---------------------------*/
void determine_team();
String print_state(int state);

void initialize_motor_pins();
void stop_motors();
void motor_l_stop();
void motor_r_stop();
void drive_forward_motors();
void motor_r_forward();
void motor_l_forward();
void drive_backward_motors();
void motor_r_backward();
void motor_l_backward();

void turnTowardWall();
void turnTowardScoring();

void initialize_sonar_pins();
void sonar_distance(int pingPin, int echoPin);

void initialize_hopper_pins();
void hopper_intake();
void hopper_outtake();
void hopper_stop();

/*---------------Setup and Loop---------------------------*/

void setup() {
  // put your setup code here, to run once:
  #ifdef SERIAL_ON
  Serial.begin(9600);
  #endif

  initialize_motor_pins();
  initialize_sonar_pins();
  #ifdef HOPPER
  initialize_hopper_pins();
  #endif

  determine_team();
  state = MOVING_TO_WALL;

  delay(INITIAL_DELAY);
  curr_dist = INITIAL_WALL_DISTANCE;
  // distanceTimer.reset();

  hopperTimer.reset();
}

void loop() {
  // put your main code here, to run repeatedly:
  bool close_to_wall = false;

  sonar_distance(TRIGGER, ECHO);

  #ifdef SERIAL_ON
  Serial.print("State: ");
  Serial.print(print_state(state));
  Serial.print(", Distance : ");
   Serial.println(curr_dist);
  #endif

  switch (state) {
    case LOADING:
      if (hopperTimer.check()) {
        break;
      }

    case MOVING_TO_WALL:
      // if we're 12 rad 2 = 17ish in. away from opposite wall
      close_to_wall = (curr_dist <= TURN_WALL_THRESHOLD);
      if (close_to_wall) {  // time to do an in-place turn
        state = TURN_WALL;
        turnWallTimer.reset();
        turnTowardWall();
      } else { // still just moving forward
        motor_l_forward();
        motor_r_forward();
      }
      break;
    
    case TURN_WALL:
      if (turnWallTimer.check()) {
        #ifndef TESTING_ROBOT
        state = MOVING_TO_SCORING;
        motor_l_forward();
        motor_r_forward();
        #endif

        #ifdef TESTING_ROBOT
        state = DONE;
        motor_l_stop();
        motor_r_stop();
        #endif

      } else { // keep turning
        turnTowardWall();
      }
      break;
    
    case MOVING_TO_SCORING:
      close_to_wall = (curr_dist <= TURN_SCORE_THRESHOLD);
      if (close_to_wall) {
        state = TURN_SCORING;
        turnScoringTimer.reset();
        turnTowardScoring();
      } else {    
        motor_l_forward();
        motor_r_forward();
      }
      break;
    
    case TURN_SCORING:
      if (turnScoringTimer.check()) {
        state = MOVING_TO_DONE;
        motor_l_forward();
        motor_r_forward();
      } else {
        turnTowardScoring();
      }
      break;
    
    case MOVING_TO_DONE:
      close_to_wall = (curr_dist <= DONE_THRESHOLD);
      if (close_to_wall) {
        state = DONE;
        motor_l_stop();
        motor_r_stop();
      } else {
        motor_l_forward();
        motor_r_forward();
      }
      break;
    
    case DONE:
      motor_l_stop();
      motor_r_stop();
      break;
  
    default: 
      motor_l_stop();
      motor_r_stop();
      break;
  }

  delay(500);
}

/*---------------General Auxiliary Functions---------------------------*/

void determine_team()
{
  team = digitalRead(TEAM_PIN) ? Team::RED : Team::BLUE;

  #ifdef SERIAL_ON
  Serial.print("GO ");
  Serial.print(team == Team::RED ? "RED" : "BLUE");
  Serial.println(" TEAM");
  #endif
}

String print_state(int state) 
{
  switch (state){
    case MOVING_TO_WALL: 
      return "MOVING_TO_WALL";

    case TURN_WALL: 
      return "TURN_WALL";
      
    case MOVING_TO_SCORING: 
      return "MOVING_TO_SCORING";

    case TURN_SCORING: 
      return "TURN_SCORING";

    case MOVING_TO_DONE: 
      return "MOVING_TO_DONE";
      
    case DONE:
      return "DONE";

    default:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

/*---------------Motor Functions---------------------------*/

void initialize_motor_pins()
{
  pinMode(MOTOR_R_ENABLE, OUTPUT);
  pinMode(MOTOR_R_DIR_1, OUTPUT);
  pinMode(MOTOR_R_DIR_2, OUTPUT);
  pinMode(MOTOR_L_ENABLE, OUTPUT);
  pinMode(MOTOR_L_DIR_1, OUTPUT);
  pinMode(MOTOR_L_DIR_2, OUTPUT);
}

void stop_motors() 
{
  motor_r_stop();
  motor_l_stop();
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

void drive_forward_motors()
{
  motor_r_forward();
  motor_l_forward();
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

void drive_backward_motors()
{
  motor_r_backward();
  motor_l_backward();
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

void turn_left_motors()
{
  motor_l_backward();
  motor_r_forward();
}

void turn_right_motors()
{
  motor_l_forward();
  motor_r_backward();
}

void turnTowardWall() 
{
  if (team == Team::RED) {
    turn_right_motors();
  } else { 
    turn_left_motors();
  }
}

void turnTowardScoring() 
{
  if (team == Team::RED) {
    turn_left_motors();
  } else { 
    turn_right_motors();
  }
}

/*---------------Sonar Functions---------------------------*/

void initialize_sonar_pins() 
{
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
}

/* From https://www.tutorialspoint.com/arduino/arduino_ultrasonic_sensor.htm */
void sonar_distance() 
{
  // NOTE: Removed 100ms timer/delay 
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);

  long duration = pulseIn(ECHO, HIGH);


  if (sonar_index < MAX_INDEX) {
    sonarvalues[sonar_index] = (duration) / 74 / 2;
  } else {
    //verage of arry
    int sum = 0;
    for (int i = 0; i < MAX_INDEX; i++) {
      sum += sonarvalues[i];

    }
    curr_dist = sum/MAX_INDEX;
    sonar_index  =  0;
  }
  
  #ifdef SERIAL_ON
  Serial.print(curr_dist);
  Serial.print("in, ");
  #endif

/* calculate in cm
  cm = (duration) / 29 / 2;
  #ifdef
  Serial.print(cm);
  Serial.print("cm");
  #endif
*/

  Serial.println();
}

void initialize_hopper_pins() 
{
  pinMode(MOTOR_HOPPER_ENABLE, OUTPUT);
  pinMode(MOTOR_HOPPER_DIR_1, OUTPUT);
  pinMode(MOTOR_HOPPER_DIR_2, OUTPUT);
}

void hopper_intake() 
{
  analogWrite(MOTOR_HOPPER_ENABLE, 200);
  digitalWrite(MOTOR_HOPPER_DIR_1, LOW);
  digitalWrite(MOTOR_HOPPER_DIR_2, HIGH);
}

void hopper_stop() 
{
  analogWrite(MOTOR_HOPPER_ENABLE, 0);
}

void hopper_outtake() 
{
  analogWrite(MOTOR_HOPPER_ENABLE, 255);
  digitalWrite(MOTOR_HOPPER_DIR_1, HIGH);
  digitalWrite(MOTOR_HOPPER_DIR_2, LOW);
}
