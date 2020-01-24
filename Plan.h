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
  PlanEntry getNextStep(CompPlanEntry &cmd, unsigned long miliTimeElapsed);

  Direction desired_dir();
  // Returns whether the robot rotated and if so, which way.
  bool rotate(PlanEntry &rot_cmd);
  // Returns whether the robot went straight
  bool goStraight();

  CompPlanEntry *entries;
  uint8_t num_entries;
  RobotConfig curr_pos;
  uint8_t curr_entry_i;
  // ORDER DEPENDENT - must come after curr_pos.
  CompPlanEntry to_home_cmd;
};

#endif