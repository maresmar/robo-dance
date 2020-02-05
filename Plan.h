#ifndef PLANNER_HEADER
#define PLANNER_HEADER

#include <ctype.h>
#include <stdint.h>

enum PlanEntry : uint8_t {
  Left,
  Go, // Go straight through one junction
  Right,
  Wait,
  Finished,
};
enum class Direction : char {
  EAST = 'E',
  NORTH = 'N',
  SOUTH = 'S',
  WEST = 'W',
};
struct RobotConfig {
  RobotConfig() : column(1), row(1), curr_dir(Direction::NORTH) {}
  uint8_t column, row;
  Direction curr_dir;
};

struct CompPlanEntry {
  uint16_t timePoint : 15;
  bool rowFirst : 1;
  uint8_t row : 4;
  uint8_t column : 4;
};
static_assert(
    sizeof(CompPlanEntry) == 3,
    "The program was designed with 3 bytes entries in mind. If changed, ensure "
    "that all structures will fit into both RAM and EEPROM.");

// An interface that can execute plans loaded via Planner class.
class Plan {
public:
  // Create an empty plan that is always finished and robot is always home.
  Plan();
  Plan(uint8_t num_entries, CompPlanEntry *entries, RobotConfig init_pos);
  // Returns next command that should be executed by the robot given the time
  // elapsed from the beginning.
  PlanEntry getNext(unsigned long miliTimeElapsed);
  // Returns next command that will eventually guide the robot to its initial
  // position.
  PlanEntry goHome();

private:
  // Returns next instruction that will eventually satisfy the passed command.
  // Returns Finished when that happens.
  PlanEntry getNextStep(const CompPlanEntry &cmd,
                        unsigned long miliTimeElapsed);
  // Returns the direction that the robot would like to face to complete the
  // passed command.
  Direction desired_dir(const CompPlanEntry &cmd);
  // If the robot wants to rotate in order to fulfill its command, it returns
  // either LEFT or RIGHT and rotates the current position. Otherwise it returns
  // Go but does not move the robot.
  PlanEntry rotate(const CompPlanEntry &cmd);
  // Returns whether the robot wanted to and went straight.
  // It assumes that the robot tried to rotate and did not want to. I.e. rotate
  // returned Go.
  bool goStraight(const CompPlanEntry &cmd);

  CompPlanEntry *entries;
  uint8_t num_entries;
  RobotConfig curr_pos;
  uint8_t curr_entry_i;
  // ORDER DEPENDENT - must come after curr_pos.
  CompPlanEntry to_home_cmd;
};

#endif