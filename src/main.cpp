#include <Arduino.h>

// Motor Pin Definitions
#define MOTOR_R_ENABLE 2
#define MOTOR_R_DIR_1 3
#define MOTOR_R_DIR_2 4
#define MOTOR_L_ENABLE 7
#define MOTOR_L_DIR_1 6
#define MOTOR_L_DIR_2 5

// Function Prototypes
void initialize_motor_pins();
void motor_r_forward();
void motor_l_forward();

void setup() {
  initialize_motor_pins();
  motor_r_forward();
  motor_l_forward();
}

void loop() {
}

void initialize_motor_pins() {
  pinMode(MOTOR_R_ENABLE, OUTPUT);
  pinMode(MOTOR_R_DIR_1, OUTPUT);
  pinMode(MOTOR_R_DIR_2, OUTPUT);
  pinMode(MOTOR_L_ENABLE, OUTPUT);
  pinMode(MOTOR_L_DIR_1, OUTPUT);
  pinMode(MOTOR_L_DIR_2, OUTPUT);
}

void motor_r_forward() {
  digitalWrite(MOTOR_R_ENABLE, HIGH);
  digitalWrite(MOTOR_R_DIR_1, LOW);
  digitalWrite(MOTOR_R_DIR_2, HIGH);
}

void motor_l_forward() {
  digitalWrite(MOTOR_L_ENABLE, HIGH);
  digitalWrite(MOTOR_L_DIR_1, LOW);
  digitalWrite(MOTOR_L_DIR_2, HIGH);
}