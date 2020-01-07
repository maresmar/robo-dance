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
enum class CurrDir : char {
  EAST = 'E',
  NORTH = 'N',
  SOUTH = 'S',
  WEST = 'W',
};
struct RobotConfig {
  uint8_t column, row;
  CurrDir curr_dir;
};

struct CompPlanEntry {
  uint8_t column : 4;
  uint8_t row : 4;
  bool rowFirst : 1;
  uint16_t timePoint : 15;
};
static CurrDir rotateLeft(CurrDir c) {
  switch (c) {
  case CurrDir::EAST:
    return CurrDir::NORTH;
  case CurrDir::NORTH:
    return CurrDir::WEST;
  case CurrDir::WEST:
    return CurrDir::SOUTH;
  case CurrDir::SOUTH:
    return CurrDir::EAST;
  }
}
static CurrDir rotateRight(CurrDir c) {
  switch (c) {
  case CurrDir::EAST:
    return CurrDir::SOUTH;
  case CurrDir::NORTH:
    return CurrDir::EAST;
  case CurrDir::WEST:
    return CurrDir::NORTH;
  case CurrDir::SOUTH:
    return CurrDir::WEST;
  }
}
// An interface that executes the currently active plan stored in planner.
class Plan {
public:
  Plan(uint8_t num_entries, CompPlanEntry *entries, RobotConfig init_pos)
      : entries(entries), num_entries(num_entries), init_pos(init_pos),
        curr_pos(init_pos), curr_entry_i(0) {}

  // Returns next command that should be executed by the robot given the time
  // elapsed from the beginning.
  PlanEntry getNext(unsigned long miliTimeElapsed) {
    uint16_t deciTimeElapsed = miliTimeElapsed / 100;
    // In case of an empty plan
    if (curr_entry_i >= num_entries)
      return PlanEntry::Finished;

    CompPlanEntry &curr_cmd = entries[curr_entry_i];
    CurrDir c_dir = curr_pos.curr_dir;
    CurrDir d_dir = desired_dir();

    // Too early
    if (curr_cmd.timePoint > deciTimeElapsed)
      return PlanEntry::Wait;

    // Orientation incorrect
    if (d_dir != c_dir) {
      if (rotateRight(c_dir) == d_dir) {
        curr_pos.curr_dir = d_dir;
        return PlanEntry::Right;
      } else {
        curr_pos.curr_dir = rotateLeft(c_dir);
        return PlanEntry::Left;
      }
    }
    // Go straight
    if (curr_pos.row != curr_cmd.row || curr_pos.column != curr_cmd.column) {
      switch (c_dir) {
      case CurrDir::NORTH:
        curr_pos.row += 1;
        break;
      case CurrDir::SOUTH:
        curr_pos.row -= 1;
        break;
      case CurrDir::EAST:
        curr_pos.column += 1;
        break;
      case CurrDir::WEST:
        curr_pos.column -= 1;
        break;
      }
      return PlanEntry::Go;
    }

    // The command is already finished, fetch a new one.
    ++curr_entry_i;
    // Should be only one level-deep recursion since the next command is not
    // immediately completed.
    return getNext(miliTimeElapsed);
  }
  // Reset the plan to the first instruction
  // ENSURE that the robot is located at its initial position as specified
  // by the plan.
  void reset() { curr_entry_i = 0; }
  // Get the plan that will return the robot to the initial position
  Plan getReverse();

private:
  CurrDir desired_dir() {
    CompPlanEntry &curr_cmd = entries[curr_entry_i];
    int col_delta = (int)curr_cmd.column - curr_pos.column;
    int row_delta = (int)curr_cmd.row - curr_pos.row;
    if (col_delta == row_delta && col_delta == 0)
      return curr_pos.curr_dir;

    CurrDir row_orient = row_delta > 0 ? CurrDir::NORTH : CurrDir::SOUTH;
    CurrDir col_orient = col_delta > 0 ? CurrDir::EAST : CurrDir::WEST;
    if (curr_cmd.rowFirst)
      return row_delta != 0 ? row_orient : col_orient;
    else
      return col_delta != 0 ? col_orient : row_orient;
  }

  CompPlanEntry *entries;
  uint8_t num_entries;
  RobotConfig init_pos;
  RobotConfig curr_pos;
  uint8_t curr_entry_i;
};

#endif