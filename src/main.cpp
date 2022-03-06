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
#define TURN_WALL_THRESHOLD 30
// #define TURN_WALL_THRESHOLD 11
#define TRAVEL_TO_LONG_WALL_THRESHOLD 17
#define SCORING_ZONE_THRESHOLD 2
#define TURN_WALL_DURATION 525 //veer right a lil //515 (lil too short) //490 (orig)
#define TURN_SCORING_DURATION 450 // 480//490
#define HOPPER_DURATION 10

/*---------------General Constants --------------------------*/
#define SERIAL_ON true
// #define HOPPER

#define INITIAL_WALL_DISTANCE 85 
#define INITIAL_DELAY 5000
#define SONAR_DELAY 0
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
#define TRIGGER_1 18
#define ECHO_1 17
#define TRIGGER_2 22
#define ECHO_2 21
#define TEAM_PIN 14
#define MAX_INDEX 1

/*---------------State Definitions--------------------------*/
typedef enum
{
  LOADING,
  MOVING_TOWARD_WALL,
  TURN_WALL,
  MOVING_TOWARD_LONG_WALL,
  TURN_SCORING,
  MOVING_TOWARD_SCORING,
  DONE, 
  UNLOADING,
} States_t;

/*---------------Module Variables---------------------------*/
Team team;
bool passed_junction_1, passed_junction_2;

volatile States_t state;
volatile int curr_dist_fwd;
volatile int curr_dist_bwd;

#define MOVING_AVERAGE_WIDTH 10
int recent_sonar_distances_index = 0;
int recent_sonar_distances[MOVING_AVERAGE_WIDTH];

static Metro hopperTimer = Metro(HOPPER_DURATION); // delay between computing sonar_distance
static Metro turnWallTimer = Metro(TURN_WALL_DURATION);  // 90 degree turn one-way
static Metro turnScoringTimer = Metro(TURN_SCORING_DURATION);  // 90 degree turn the-other-way
// static Metro distanceTimer = Metro(SONAR_DELAY); // delay between computing sonar_distance
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

void initialize_sonar_pins();
int sonar_distance(int TRIG, int EC);

void initialize_hopper_pins();
void hopper_intake();
void hopper_outtake();
void hopper_stop();

/* GOAL: Code up the FSM for the robot

   States: 
   - MOVING_FWD: starting state, move forward with same speed on both motors
   - TURN_WALL: moving towards side wall/other loading zone
   - TURN_SCORING: moving towards scoring zone
   - DONE: beat brick!
 */

void setup() {
  // put your setup code here, to run once:
  #ifdef SERIAL_ON
  Serial.begin(9600);
  #endif

  for (int i = 0; i < MOVING_AVERAGE_WIDTH; i++)
    recent_sonar_distances[i] = INITIAL_WALL_DISTANCE;

  initialize_motor_pins();
  pinMode(TRIGGER_1, OUTPUT);
  pinMode(ECHO_1, INPUT);
  pinMode(TRIGGER_2, OUTPUT);
  pinMode(ECHO_2, INPUT);

  Serial.println("STARTING");

  curr_dist_fwd = INITIAL_WALL_DISTANCE;
  delay(INITIAL_DELAY);
  determine_team();
  //hopper_intake();

  // *FOR BLUE TEAM ONLY*
  // doing initial veer inside loading zone to compensate for mechanical issues
  /*
  motor_l_forward();
  motor_r_backward();
  delay(125);
  motor_l_stop();
  motor_r_stop();
  */

  // distanceTimer.reset();
  state = LOADING;
}

void determine_team()
{
  team = digitalRead(TEAM_PIN) ? Team::RED : Team::BLUE;

  #ifdef SERIAL_ON
  Serial.print("GO ");
  Serial.print(team == Team::RED ? "RED" : "BLUE");
  Serial.println(" TEAM");
  #endif
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

int get_sonar_distance() {
  // Get the instantaneous distance.
  int distance = sonar_distance(TRIGGER_1, ECHO_1);
  Serial.println(distance);
  // Write it to the appropriate location.
  recent_sonar_distances[recent_sonar_distances_index] = distance;
  recent_sonar_distances_index = (++recent_sonar_distances_index) % MOVING_AVERAGE_WIDTH;
  int sum = 0;
  for (int i = 0; i < MOVING_AVERAGE_WIDTH; i++)
    sum += recent_sonar_distances[i];
  return sum / MOVING_AVERAGE_WIDTH;
}

void loop() {
  // put your main code here, to run repeatedly:
  curr_dist_fwd = get_sonar_distance();
  // Serial.println(curr_dist_fwd);
  delay(100);
  curr_dist_bwd = sonar_distance(TRIGGER_2, ECHO_2);

  // #ifdef SERIAL_ON
  // Serial.print("State: ");
  // Serial.print(print_state(state));
  // Serial.print(", Forward Distance : ");
  // Serial.println(curr_dist_fwd);
  // Serial.print(", Backward Distance : ");
  // Serial.println(curr_dist_bwd);
  // #endif

  switch (state) {
    case LOADING:

      state = MOVING_TOWARD_WALL;
      break;
    
    case MOVING_TOWARD_WALL:
      // if we're 12 rad 2 = 17ish in. away from opposite wall
      if (curr_dist_fwd <= TURN_WALL_THRESHOLD) { //curr_dist_bwd >= 96 - TURN_WALL_THRESHOLD) { // time to do an in-place turn
        state = TURN_WALL;
        turnWallTimer.reset();
        firstTurn();

        #ifdef SERIAL_ON
        Serial.println("MOVING_TOWARD_WALL => TURN_WALL");
        #endif
      } else { 
        motor_l_forward();
        motor_r_forward();
      }
      break;
    
    case TURN_WALL:
      if (turnWallTimer.check()) {
        motor_l_stop();
        motor_r_stop();
        state = MOVING_TOWARD_LONG_WALL;

        #ifdef SERIAL_ON
        Serial.println("TURN_WALL => MOVING_TOWARD_LONG_WALL");
        #endif
        
      } else {
        firstTurn();
      }
      break;
    
    case MOVING_TOWARD_LONG_WALL:
      if (curr_dist_fwd <= TRAVEL_TO_LONG_WALL_THRESHOLD) { // time to do an in-place turn
        state = TURN_SCORING;
        turnScoringTimer.reset();
        secondTurn();

        #ifdef SERIAL_ON
        Serial.println("MOVING_TOWARD_LONG_WALL => TURN_SCORING");
        #endif
      } else { 
        motor_l_forward();
        motor_r_forward();
      }
      break;
    
    case TURN_SCORING:
      if (turnScoringTimer.check()) {
        motor_l_stop();
        motor_r_stop();
        state = MOVING_TOWARD_SCORING;

        #ifdef SERIAL_ON
        Serial.println("TURN_SCORING => MOVING_TOWARD_SCORING");
        #endif
      } else {
        secondTurn();
      }
      break;
    
    case MOVING_TOWARD_SCORING:

      motor_l_forward();
      motor_r_forward();
      delay(2500);
      motor_l_stop();
      motor_r_stop();
      state = DONE;
    /*
      if (sonar_distance() <= SCORING_ZONE_THRESHOLD) 
      {
        state = DONE;
        motor_l_stop();
        motor_r_stop();
      } else {
        motor_l_forward();
        motor_r_forward();
      }
      */
      break;
    
    case DONE:
      motor_l_stop();
      motor_r_stop();
      //hopper_outtake();
      break;
  
    // case MOVING_TOWARD_LONG_WALL:
    //   while (sonar_distance() > TRAVEL_TO_LONG_WALL_THRESHOLD) {
    //     Serial.print("Moving to long wall, ");
    //     motor_l_forward();
    //     motor_r_forward();
    //   }
    //   Serial.println();
    //   state = TURN_SCORING;
    //   Serial.println("Starting turn scoring timer");
    //   turnScoringTimer.reset();
    //   break;

    // case TURN_SCORING:
    //   Serial.println("HELLO");
    //   if (turnScoringTimer.check()) {
    //     motor_l_stop();
    //     motor_r_stop();
    //     state = MOVING_TOWARD_SCORING;
    //   } else {
    //     secondTurn();    
    //     Serial.print("turning");
    //   }
    //   break;

    default: 
      motor_l_stop();
      motor_r_stop();
      hopper_outtake();
      break;
  }

  delay(10);
}

void firstTurn() {
  if (team == Team::RED){
    // turn right for X secs
    motor_l_forward();
    motor_r_backward();
  }
  else {
    motor_l_backward();
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

String print_state(int state) 
{
  switch (state){
    case LOADING: 
      return "LOADING";
    
    case MOVING_TOWARD_WALL: return "MOVING_TOWARD_WALL";

    case TURN_WALL: 
      return "TURN_WALL";
      
    case MOVING_TOWARD_LONG_WALL: 
      return "MOVING_TOWARD_LONG_WALL";

    case TURN_SCORING: 
      return "TURN_SCORING";

    case MOVING_TOWARD_SCORING: 
      return "MOVING_TOWARD_SCORING";
      
    case DONE:
      return "DONE";

    case UNLOADING: return "UNLOADING";

    default:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}


/*---------------Sonar Functions---------------------------*/

void initialize_sonar_pins() 
{
  pinMode(TRIGGER_1, OUTPUT);
  pinMode(ECHO_1, INPUT);
  pinMode(TRIGGER_2, OUTPUT);
  pinMode(ECHO_2, INPUT);
}

/* From https://www.tutorialspoint.com/arduino/arduino_ultrasonic_sensor.htm */
int sonar_distance(int TRIG, int EC) {
  // if (distanceTimer.check()) {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    long duration = pulseIn(EC, HIGH);

    int dist = (duration) / 74 / 2;
 //averaging code
 /*
   if (sonar_index < MAX_INDEX) {
    sonarvalues[sonar_index] = (duration) / 74 / 2;
    sonar_index++;
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
   */

    // cm = (duration) / 29 / 2;
    //if (SERIAL_ON) Serial.print(curr_dist);
    //if (SERIAL_ON) Serial.println("in, ");
    // if (SERIAL_ON) Serial.print(cm);
    // if (SERIAL_ON) Serial.print("cm");
    //if (SERIAL_ON) Serial.println();
    // distanceTimer.reset();
    return dist;
}


/*---------------Hopper Functions---------------------------*/

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
