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
  waiting,
  starting,
  executing,
  going_home
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

  state = waiting;
  button_pressed = false;

  // Loads the default plan - either from hardcoded string or a previously
  // saved plan stored in EEPROM.
  Planner::loadDefault();
  // Any plan can be explicitly loaded from string for debugging purposes.
  // Planner::loadFromString("<PLAN>");
}

void readButton() {
  bool pressed = !digitalRead(2);

  if (pressed && !already_pressed) {
    already_pressed = true;

    // Change state
    switch (state)
    {
    case waiting:
      state = starting;
      break;
    case executing:
      state = going_home;
      break;
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

  switch(state) {
    case waiting:
      setLed(false);
      Planner::processRemoteRequests();
      break;
    case starting:
      Serial.println("main: Starting");
      setLed(true);
      exe_task.start(Planner::getActivePlan());
      state = executing;
    case executing:
      exe_task.tick(sensors, left_motor, right_motor);
      break;
    case going_home:
      Serial.println("main: Going home");
      exe_task.tick(sensors, left_motor, right_motor, true);
      if(exe_task.isFinished())
        state = waiting;
      break;
    default:
      Serial.println("main: ERROR: state");
      break;
  }
}

/*
Robots stores up to X plans in its EEPROM. On startup the robot tries to load
its default plan = specified by the default slot. If its empty, it loads a plan
from hardcoded string. Otherwise it loads the plan saved at that default slot.
I.e. by default the robots always executes a non-empty plan.

The default slot can be changed to other numbers.

Robot remote commands before pressing the button:
  CX - clears EEPROM memory belonging to X-th slot.
  DX - sets slot X as the default slot.
  SX <plan> - Saves transmitted plan to slot X. Does NOT set this slot as
default. LX - Loads plan stored at slot X.

Hence a command is represented by two letters - {C,D,S,L} and a digit.
These letters must be the first two bytes of serial input, no leading or
intermediate whitespace is allowed.

After getting a new robot it is recommended to boot it up, clear all slots using
CX commands and set the default slot with DX, then perform a restart. Otherwise
the memory of X-th slot wil be interpretted as correct plan and fail if that
will not be the case.
  = If one gets unreasonable ParsingError on startup, this
is the reason.
  - If default slot is incorrect, no plan will be loaded and warning will be
printed.

EEPROM contents:
0     byte  - the default slot, [0,N=NumEEPROMSlots)
Nx4   bytes - EEPROMSlotInfo = slot's header with num of entries and starting
              RobotConfig.
Nx(Bx3)bytes- For each slot, block of B=MaxPlanLength CompPlanEntries.
*/
