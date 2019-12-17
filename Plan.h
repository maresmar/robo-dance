#ifndef PLANNER_HEADER
#define PLANNER_HEADER

#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <stdio.h>

enum PlanEntry : uint8_t {
  Left,
  Go, // Go straight through one junction
  Right,
  Wait,
  Finished,
};

class Plan {
public:
  // Returns next command that should be executed by the robot given the time
  // elapsed from the beginning.
  PlanEntry getNext(unsigned long miliTimeElapsed) {}
  // Reset the plan to the first instruction
  void reset() {}
  // Get the plan that will return the robot to the initial position
  Plan getReverse();
};
#endif