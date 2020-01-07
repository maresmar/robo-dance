#ifndef PLAN_PARSER_HEADER
#define PLAN_PARSER_HEADER
// Only include one please, because of the static variables that should really
// be in .cpp files
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "Plan.h"

#ifndef RUNNING_TESTS
#define PL_PRINT_PARSE_ERR_STR(x) (Serial.println(x))
#else
#define F(x) x
#define PL_PRINT_PARSE_ERR_STR(x) printf("ERR: %s\n", (x));
#endif

enum class ParseError {
  OK,
  InvalidOrient,
  InvalidCoords,
  InvalidTime,
  UnexpectedEnd,
  PlanTooLong
};

struct StringGetter {
  StringGetter(const char *str) : str(str) {}
  // Will return '\0' on error and keep returning it
  char getNext(bool skip_whitespace = true) {
    char c = peek_next(skip_whitespace);
    if (c != '\0')
      ++str;
    return c;
  }
  void skip_whitespace() {
    while (str && isspace(*str))
      ++str;
  }
  char peek_next(bool skip_whitespace = true) {
    if (skip_whitespace)
      this->skip_whitespace();
    char c = str ? *str : '\0';
    return c;
  }
  const char *str;
};

// Parses two-letter string into coordinates.
static ParseError parseCoords(const char str[2], bool &col_first, uint8_t &row,
                              uint8_t &col);
// Parsers string of len 3: 'XYZ', X\in{1..9}, Y\in{A...I}, Z==CurrDir
static ParseError parseCommand(const char str[2 + 5 + 1],
                               CompPlanEntry &newEntry);
// Parse null terminated string of length max 8, str[0,1]==Coords,
// str[2+]==timepoint integer.
static ParseError parseInitPos(const char *str, RobotConfig &initConfig);
template<typename CharGetter>
static ParseError preparseCmd(CharGetter &getter,
                              char command_str[2 + 5 + 1]) {
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
    char last = command_str[2 + i] = getter.getNext(false);
    // The number ended
    if (last == '\0' || isspace(last)) {
      command_str[2 + i] = '\0';
      break;
    } else if (i == 5) // Time won't fit into 16bits
      return ParseError::InvalidTime;
  }
  return ParseError::OK;
}

// Consumes 3B*64 * 2 RAM = 384B, 768B EEPROM
constexpr const static uint8_t MaxPlanLength = 64;
constexpr const static uint8_t NumEEPROMSlots = 4;

// Responsible for loading and saving plans. A plan can be loaded from a string
// or EEPROM.
class Planner {
public:
  static bool loadDefault() {
    static const char * const str = "B1N E1T150 b2T350 3At450 4CT567 D2T700";
    return loadFromString(str);
  }
  static bool loadFromString(const char *str) {
    StringGetter getter = StringGetter(str);
    ParseError err = load(getter);
    if (err != ParseError::OK)
      printErrMsg(err);
    return err == ParseError::OK;
  }
  static void loadFromSerial();
  static void loadFromEEPROM(int slot);
  static void saveToEEPROM(int slot);

  // The returned PLan is valid until next loadXXX method is called.
  static Plan getActivePlan() {
    return Plan(num_active_entries, active_entries, active_init_pos);
  }

private:
  Planner() {}
  static void printErrMsg(ParseError e) {
    switch (e) {
    case ParseError::OK:
      PL_PRINT_PARSE_ERR_STR(F("No errors."));
      break;
    case ParseError::InvalidOrient:
      PL_PRINT_PARSE_ERR_STR(F("Invalid orientation in the initial position."));
      break;
    case ParseError::InvalidCoords:
      PL_PRINT_PARSE_ERR_STR(F("Invalid coordinates."));
      break;
    case ParseError::InvalidTime:
      PL_PRINT_PARSE_ERR_STR(F("Invalid time."));
      break;
    case ParseError::PlanTooLong:
      PL_PRINT_PARSE_ERR_STR(F("Too many commands."));
      break;
    case ParseError::UnexpectedEnd:
      PL_PRINT_PARSE_ERR_STR(F("Unexpected end of input."));
      break;
    };
  }
	template<typename CharGetter>
  static ParseError load(CharGetter &getter) {
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
    active_init_pos = new_init;
    return ParseError::OK;
  }

  // Skips any whitespace symbols
  static void skip_whitespace(const char *&str) {
    while (str && isspace(*str))
      ++str;
  }
// For easier testing, do not  misuse!
#ifdef RUNNING_TESTS
public:
#endif
  // Currently active plan loaded into the memory
  static CompPlanEntry active_entries[MaxPlanLength];
  // Scratch space for loading a new plan
  // Seems wastefull but it ensures that if the plan fails in the middle of
  // loading then the original plan is preserved.
  static CompPlanEntry loaded_entries[MaxPlanLength];
  static int num_active_entries;
  static RobotConfig active_init_pos;
};

int Planner::num_active_entries = 0;
CompPlanEntry Planner::loaded_entries[MaxPlanLength];
CompPlanEntry Planner::active_entries[MaxPlanLength];
RobotConfig Planner::active_init_pos;

static ParseError parseCoords(const char str[2], bool &col_first, uint8_t &row,
                              uint8_t &col) {
  col_first = isalpha(str[0]);
  uint8_t col_index = col_first ? 0 : 1;
  uint8_t row_index = 1 - col_index;
  col = tolower(str[col_index]) - 'a' + 1;
  row = str[row_index] - '0';

  return (col < 1 || col > 9 || row < 1 || row > 9) ? ParseError::InvalidCoords
                                                    : ParseError::OK;
}

static ParseError parseInitPos(const char *str, RobotConfig &initConfig) {
  bool col_first;
  uint8_t row, col;

  ParseError coords_err = parseCoords(str, col_first, row, col);
  if (coords_err != ParseError::OK)
    return coords_err;
  CurrDir init_dir = (CurrDir)toupper(str[2]);

  if (init_dir != CurrDir::NORTH && init_dir != CurrDir::SOUTH &&
      init_dir != CurrDir::EAST && init_dir != CurrDir::WEST)
    return ParseError::InvalidOrient;

  initConfig.column = col;
  initConfig.row = row;
  initConfig.curr_dir = init_dir;
  return ParseError::OK;
}
static ParseError parseCommand(const char str[2 + 5 + 1],
                               CompPlanEntry &newEntry) {
  bool col_first;
  uint8_t row, col;
  ParseError coords_err = parseCoords(str, col_first, row, col);
  if (coords_err != ParseError::OK)
    return coords_err;

  int timePoint = atoi(str + 2);
  if (timePoint == 0)
    return ParseError::InvalidTime;

  newEntry.column = col & 0x0F;
  newEntry.row = row & 0x0F;
  newEntry.rowFirst = (!col_first) % 2;
  newEntry.timePoint = timePoint & 0x7FFF;
  return ParseError::OK;
}
#endif