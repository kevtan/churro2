#include <Arduino.h>
#include <Metro.h>
#include <Churro.h>

// Miscellaneous
#define BAUD_RATE 9600

// Motor Pin Definitions
#define MOTOR_R_ENABLE 2
#define MOTOR_R_DIR_1 3
#define MOTOR_R_DIR_2 4
#define MOTOR_L_ENABLE 7
#define MOTOR_L_DIR_1 6
#define MOTOR_L_DIR_2 5

// Miscellaneous Pin Definitions
#define TEAM_PIN 14

// Status LED Definitions
#define STATUS_LED_PIN LED_BUILTIN
#define STATUS_LED_TIME_INTERVAL_MS 1000
bool status_led_is_on = true;
Metro status_led_timer = Metro(STATUS_LED_TIME_INTERVAL_MS);

// Infrared Reflectance Sensor Definitions
#define LINE_THRESHOLD_CL 800
#define LINE_THRESHOLD_CC 830
#define LINE_THRESHOLD_CR 870
#define IR_SENSOR_CL 18 // Center Left
#define IR_SENSOR_CC 16 // Center Center
#define IR_SENSOR_CR 14 // Center Right

// Global Variables
bool on_line = false;
Team team;

// Function Prototypes
void initialize_motor_pins();
void determine_team();
void motor_r_forward();
void motor_l_forward();
void motor_r_stop();
void motor_l_stop();
bool test_status_led_timer_expired();
void resp_status_led_timer_expired();
bool test_on_line();
bool test_off_line();
void resp_on_line();
void resp_off_line();

void setup()
{
  Serial.begin(BAUD_RATE);
  initialize_motor_pins();
  determine_team();
}

void loop()
{
  if (test_status_led_timer_expired())
    resp_status_led_timer_expired();
  if (test_on_line())
    resp_on_line();
  if (test_off_line())
    resp_off_line();
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

void determine_team()
{
  team = digitalRead(TEAM_PIN) ? Team::RED : Team::BLUE;
}

void motor_r_forward()
{
  analogWrite(MOTOR_R_ENABLE, 150);
  digitalWrite(MOTOR_R_DIR_1, LOW);
  digitalWrite(MOTOR_R_DIR_2, HIGH);
}

void motor_l_forward()
{
  analogWrite(MOTOR_L_ENABLE, 150);
  digitalWrite(MOTOR_L_DIR_1, LOW);
  digitalWrite(MOTOR_L_DIR_2, HIGH);
}

void motor_r_stop()
{
  analogWrite(MOTOR_R_ENABLE, 0);
}

void motor_l_stop()
{
  analogWrite(MOTOR_L_ENABLE, 0);
}

bool test_status_led_timer_expired()
{
  return status_led_timer.check();
}

void resp_status_led_timer_expired()
{
  status_led_timer.reset();
  status_led_is_on = not status_led_is_on;
  digitalWrite(STATUS_LED_PIN, status_led_is_on);
}

bool test_on_line()
{
  if (not on_line and analogRead(IR_SENSOR_CC) < LINE_THRESHOLD_CC)
  {
    on_line = true;
    return true;
  }
  return false;
}

bool test_off_line()
{
  if (on_line and analogRead(IR_SENSOR_CC) >= LINE_THRESHOLD_CC)
  {
    on_line = false;
    return true;
  }
  return false;
}

void resp_on_line()
{
  Serial.println("Off -> On");
  motor_l_stop();
  motor_r_stop();
}

void resp_off_line()
{
  Serial.println("On -> Off");
  motor_l_forward();
  motor_r_forward();
}