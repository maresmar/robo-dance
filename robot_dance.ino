#include <Arduino.h>
#include "ctl_common.h"
#include "execution_task.h"
#include "Planner.h"

// Control
#define LED_PORT 11
#define BUTTON_PORT 2
// Mothors
#define LEFT_MOTOR_PORT 12
#define RIGHT_MOTOR_PORT 13
#define MAX_SPEED_PERCENT 33
// Sensors
#define LEFT_DIR_SENSOR_PORT 3
#define LEFT_LINE_SENSOR_PORT 4
#define MIDLE_LINE_SENSOR_PORT 5
#define RIGHT_LINE_SENSOR_PORT 6
#define RIGHT_DIR_SENSOR_PORT 7


enum state_t {
  stoped,
  running,
  returning
};

static Motor left_motor, right_motor;
static state_t state, last_state;
static sensors_t sensors;
static ExecutionTask exe_task;

static bool button_pressed, already_pressed;

void setup() {
  // Contorl
  pinMode(BUTTON_PORT, INPUT_PULLUP);
  pinMode(LED_PORT, OUTPUT);
  Serial.begin(9600);

  // Motors
  left_motor.attach(LEFT_MOTOR_PORT, 500, 2500);
  left_motor.setDirection(true);
  right_motor.attach(RIGHT_MOTOR_PORT, 500, 2500);
  right_motor.setDirection(false);

  // Sensors
  pinMode(LEFT_DIR_SENSOR_PORT, INPUT);
  pinMode(LEFT_LINE_SENSOR_PORT, INPUT);
  pinMode(MIDLE_LINE_SENSOR_PORT, INPUT);
  pinMode(RIGHT_LINE_SENSOR_PORT, INPUT);
  pinMode(RIGHT_DIR_SENSOR_PORT, INPUT);

  state = stoped;
  button_pressed = false;

  Planner::loadDefault();
  exe_task.start(Planner::getActivePlan());
}

void readButton() {
  bool pressed = !digitalRead(2);

  if (pressed && !already_pressed) {
    already_pressed = true;

    // Toggle enabled mode
    if (state == stoped) {
      state = last_state;
    } else {
      last_state = state;
      state = stoped;
    }
  }
  if (!pressed)
    already_pressed = false;
}

void readSensors() {
  sensors.left_dir = !digitalRead(LEFT_DIR_SENSOR_PORT);
  sensors.left_line = !digitalRead(LEFT_LINE_SENSOR_PORT);
  sensors.midle_line = !digitalRead(MIDLE_LINE_SENSOR_PORT);
  sensors.right_line = !digitalRead(RIGHT_LINE_SENSOR_PORT);
  sensors.right_dir = !digitalRead(RIGHT_DIR_SENSOR_PORT);
}

void setLed(bool enabled) {
  digitalWrite(LED_PORT, enabled);
}

void loop() {
  readButton();
  readSensors();

  if(state != stoped) {
    exe_task.tick(sensors, left_motor, right_motor);
  }
}
