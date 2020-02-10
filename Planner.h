#ifndef PLAN_PARSER_HEADER
#define PLAN_PARSER_HEADER
// Only include one please, because of the static variables that should really
// be in .cpp files
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "Plan.h"

enum class ParseError {
  OK,
  InvalidOrient,
  InvalidCoords,
  InvalidTime,
  UnexpectedEnd,
  PlanTooLong
};

// Parses two-letter string into coordinates.
ParseError parseCoords(const char str[2], bool &col_first, uint8_t &row,
                       uint8_t &col);

// Parsers string of len 3: 'XYZ', X\in{1..9}, Y\in{A...I}, Z==CurrDir
ParseError parseCommand(const char str[2 + 5 + 1], CompPlanEntry &newEntry);

// Parse null terminated string of length max 8, str[0,1]==Coords,
// str[2+]==timepoint integer.
ParseError parseInitPos(const char *str, RobotConfig &initConfig);
// Parses one command from the input stream into the array.
// The cmd in the array has a format of: 'XY12345\0'
template <typename CharGetter>
ParseError preparseCmd(CharGetter &getter, char command_str[2 + 5 + 1]);

void printErrMsg(ParseError e);
// Consumes 3B*64 * 2 RAM = 384B, 1+4*5B+960B EEPROM
constexpr const static uint8_t MaxPlanLength = 64;
constexpr const static uint8_t NumEEPROMSlots = 5;
// Responsible for loading and saving plans.
class Planner {
public:
  // Loads the plan either from non-empty default EEPROM slot or from a backup string.
  static bool loadDefault();
  static bool loadFromString(const char *str);
  // Checks whether someones sent any remote commands and processes them if so.
  static bool processRemoteRequests();

  // The returned PLan is valid until the next loadXXX or processRemotecommands
  // methods are called.
  static Plan getActivePlan();

private:
  Planner() {}
  // Load a plan from the serial connection and sets it as active.
  static bool loadFromSerial();
  // Load a stored plan and sets it as active.
  static void loadFromEEPROM(int slot);
  // Saves the currently active plan to the chosen slot.
  static void saveToEEPROM(int slot);
  static void clearEEPROM(int slot);

  template <typename CharGetter> static ParseError load(CharGetter &getter);

  // Currently active plan loaded into the memory
  static CompPlanEntry active_entries[MaxPlanLength];
  // Scratch space for loading a new plan
  // Seems wastefull but it ensures that if the plan fails in the middle of
  // loading then the original plan is preserved.
  static CompPlanEntry loaded_entries[MaxPlanLength];
  static int num_active_entries;
  static RobotConfig active_init_config;
};

template <typename CharGetter>
ParseError preparseCmd(CharGetter &getter, char command_str[2 + 5 + 1]) {
  command_str[0] = getter.getNext();
  command_str[1] = getter.getNext();
  char t = tolower(getter.getNext());
  if (t == '\0')
    return ParseError::UnexpectedEnd;
  if (t != 't')
    return ParseError::InvalidCoords;

  // Read the time, only skip the first whitespace
  //  = between 'T' and the first digit
  // Time is missing
  if (getter.peek_next() == '\0')
    return ParseError::UnexpectedEnd;

  for (int i = 0; i < 6; ++i) {
    char last = getter.peek_next(false);
    // The number ended
    if (last == '\0' || isspace(last) || isalpha(last)) {
      command_str[2 + i] = '\0';
      break;
    } else if (i == 5) // Time won't fit into 16bits
      return ParseError::InvalidTime;
    else
      command_str[2 + i] = getter.getNext(false);
  }
  return ParseError::OK;
}

template <typename CharGetter> ParseError Planner::load(CharGetter &getter) {
  char init_pos[3];
  init_pos[0] = getter.getNext();
  init_pos[1] = getter.getNext();
  init_pos[2] = getter.getNext();
  if (init_pos[2] == '\0')
    return ParseError::UnexpectedEnd;

  RobotConfig new_init;
  ParseError err = parseInitPos(init_pos, new_init);
  if (err != ParseError::OK)
    return err;

  char new_cmd_str[2 + 5 + 1];
  // Parse commands
  int n = 0;
  for (; n < MaxPlanLength; ++n) {
    // Input ended
    if (getter.peek_next() == '\0') {
      break;
    }

    err = preparseCmd(getter, new_cmd_str);
    if (err != ParseError::OK)
      return err;
    err = parseCommand(new_cmd_str, loaded_entries[n]);
    if (err != ParseError::OK)
      return err;
  }
  if (getter.getNext() != '\0')
    return ParseError::PlanTooLong;

  // Plan loaded successfully -> set as active

  for (int i = 0; i < n; ++i)
    active_entries[i] = loaded_entries[i];
  num_active_entries = n;
  active_init_config = new_init;
  return ParseError::OK;
}
#endif