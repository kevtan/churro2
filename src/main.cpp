#include <Arduino.h>
#include <Metro.h>

// Motor Pin Definitions
#define MOTOR_R_ENABLE 2
#define MOTOR_R_DIR_1 3
#define MOTOR_R_DIR_2 4
#define MOTOR_L_ENABLE 7
#define MOTOR_L_DIR_1 6
#define MOTOR_L_DIR_2 5

// Status LED Definitions
#define STATUS_LED_PIN LED_BUILTIN
#define STATUS_LED_TIME_INTERVAL_MS 1000
bool status_led_is_on = true;
Metro status_led_timer = Metro(STATUS_LED_TIME_INTERVAL_MS);

// Function Prototypes
void initialize_motor_pins();
void motor_r_forward();
void motor_l_forward();
bool test_status_led_timer_expired();
bool resp_status_led_timer_expired();

void setup()
{
  initialize_motor_pins();
  motor_r_forward();
  motor_l_forward();
}

void loop()
{
  if (test_status_led_timer_expired())
    resp_status_led_timer_expired();
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

void motor_r_forward()
{
  digitalWrite(MOTOR_R_ENABLE, HIGH);
  digitalWrite(MOTOR_R_DIR_1, LOW);
  digitalWrite(MOTOR_R_DIR_2, HIGH);
}

void motor_l_forward()
{
  digitalWrite(MOTOR_L_ENABLE, HIGH);
  digitalWrite(MOTOR_L_DIR_1, LOW);
  digitalWrite(MOTOR_L_DIR_2, HIGH);
}

bool test_status_led_timer_expired()
{
  return status_led_timer.check();
}

bool resp_status_led_timer_expired()
{
  status_led_timer.reset();
  status_led_is_on = not status_led_is_on;
  digitalWrite(STATUS_LED_PIN, status_led_is_on);
}