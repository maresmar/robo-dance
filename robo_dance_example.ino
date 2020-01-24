#include "Planner.h"

// Starts empty
Plan plan;
// Starts the plan
bool button_pushed;
// Robot should go home
bool button_pushed_twice;

void setup() {
  // Loads the default plan - either from hardcoded string or a previously
  // saved plan stored in EEPROM.
  Planner::loadDefault();
  // Any plan can be explicitly loaded from string for debugging purposes.
  Planner::loadFromString("<PLAN>");
}
void loop() {
  if (!button_pushed) {
    bool anythingReceived = Planner::processRemoteRequests();
    // Maybe someone loaded a new plan, so update ours.
    // Creating/copying plans is cheap, so bool is not really needed.
    if (anythingReceived)
      plan = Planner::getActivePlan();
  }
  // Executing the current plan
  else {
    PlanEntry cmd =
        !button_pushed_twice ? plan.getNext(millis()) : plan.goHome();

    // Act accordingly
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