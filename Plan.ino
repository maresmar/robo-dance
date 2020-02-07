#include "Plan.h"
#include <ctype.h>

namespace {
Direction rotateLeft(Direction c) {
  switch (c) {
  case Direction::EAST:
    return Direction::NORTH;
  case Direction::NORTH:
    return Direction::WEST;
  case Direction::WEST:
    return Direction::SOUTH;
  case Direction::SOUTH:
    return Direction::EAST;
  }
}

Direction rotateRight(Direction c) {
  switch (c) {
  case Direction::EAST:
    return Direction::SOUTH;
  case Direction::NORTH:
    return Direction::EAST;
  case Direction::WEST:
    return Direction::NORTH;
  case Direction::SOUTH:
    return Direction::WEST;
  }
}

bool areOpposite(Direction dir1, Direction dir2) {
  return rotateLeft(rotateLeft(dir1)) == dir2;
}

// Creates a plan entry that directs the robot to its initial position.
CompPlanEntry to_home(RobotConfig init_pos) {
  CompPlanEntry entry;
  entry.column = init_pos.column;
  entry.row = init_pos.row;
  entry.rowFirst = true; // Prefer rows just because.
  entry.timePoint = 0;   // Never wait
  return entry;
}
} // namespace

Plan::Plan()
    : entries(nullptr), num_entries(0), curr_entry_i(0),
      to_home_cmd(to_home(curr_pos)) {}

Plan::Plan(uint8_t num_entries, CompPlanEntry *entries, RobotConfig init_pos)
    : entries(entries), num_entries(num_entries), curr_pos(init_pos),
      curr_entry_i(0), to_home_cmd(to_home(init_pos)) {}

PlanEntry Plan::getNext(unsigned long miliTimeElapsed) {
  while (curr_entry_i < num_entries) {
    CompPlanEntry &curr_cmd = entries[curr_entry_i];
    PlanEntry next = getNextStep(curr_cmd, miliTimeElapsed);
    // Skips over finished commands ( e.g. if an instruction has been
    // duplicated).
    if (next != PlanEntry::Finished)
      return next;
    else
      ++curr_entry_i;
  }
  // We've completed the plan.
  return PlanEntry::Finished;
}

PlanEntry Plan::goHome() { return getNextStep(to_home_cmd, 0); }

PlanEntry Plan::getNextStep(const CompPlanEntry &cmd,
                            unsigned long miliTimeElapsed) {
  uint16_t deciTimeElapsed = miliTimeElapsed / 100;
  Direction c_dir = curr_pos.curr_dir;

  // Try to rotate
  PlanEntry rot_cmd = rotate(cmd);
  if (rot_cmd != PlanEntry::Go)
    return rot_cmd;
  // Did not want to rotate, try to move.
  if (goStraight(cmd))
    return PlanEntry::Go;
  // Too early
  if (deciTimeElapsed < cmd.timePoint)
  {
    Serial.print("TIME:");
    Serial.print(deciTimeElapsed);
    Serial.print("; ");
    Serial.println(cmd.timePoint);

    return PlanEntry::Wait;
  }
  // The command is already finished, fetch a new one.
  return PlanEntry::Finished;
}

Direction Plan::desired_dir(const CompPlanEntry &cmd) {
  int col_delta = (int)cmd.column - curr_pos.column;
  int row_delta = (int)cmd.row - curr_pos.row;
  if (col_delta == row_delta && col_delta == 0)
    return curr_pos.curr_dir;

  Direction row_orient = row_delta > 0 ? Direction::NORTH : Direction::SOUTH;
  Direction col_orient = col_delta > 0 ? Direction::EAST : Direction::WEST;
  if (cmd.rowFirst)
    return row_delta != 0 ? row_orient : col_orient;
  else
    return col_delta != 0 ? col_orient : row_orient;
}

// Returns whether the robot rotated and if so, which way.
PlanEntry Plan::rotate(const CompPlanEntry &cmd) {
  using C = Direction;
  using P = PlanEntry;
  C c_dir = curr_pos.curr_dir;
  C d_dir = desired_dir(cmd);
  if (d_dir == c_dir)
    return PlanEntry::Go;

  if (areOpposite(c_dir, d_dir)) {
    bool on_inner_edge = curr_pos.row == 1 || curr_pos.column == 1;
    bool east_south = c_dir == C::EAST || c_dir == C::SOUTH;
    // Always turn towards lower coordinates, except on the inner edges.
    if (on_inner_edge ^ east_south) {
      curr_pos.curr_dir = rotateRight(c_dir);
      return P::Right;
    } else {
      curr_pos.curr_dir = rotateLeft(c_dir);
      return P::Left;
    }
  }

  curr_pos.curr_dir = d_dir;
  return rotateRight(c_dir) == d_dir ? P::Right : P::Left;
}

// Returns whether the robot went straight
bool Plan::goStraight(const CompPlanEntry &cmd) {
  if (curr_pos.row == cmd.row && curr_pos.column == cmd.column)
    return false;

  Direction c_dir = curr_pos.curr_dir;
  switch (c_dir) {
  case Direction::NORTH:
    curr_pos.row += 1;
    break;
  case Direction::SOUTH:
    curr_pos.row -= 1;
    break;
  case Direction::EAST:
    curr_pos.column += 1;
    break;
  case Direction::WEST:
    curr_pos.column -= 1;
    break;
  }
  return true;
}