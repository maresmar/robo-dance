#include "Planner.h"
#include <EEPROM.h>
#include <assert.h>

int Planner::num_active_entries = 0;
CompPlanEntry Planner::loaded_entries[MaxPlanLength];
CompPlanEntry Planner::active_entries[MaxPlanLength];
RobotConfig Planner::active_init_config = RobotConfig{};

namespace {
struct StringGetter {
  StringGetter(const char *str = nullptr) : str(str) {}

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

private:
  const char *str;
};
// Provides input string read from the serial interface.
struct SerialGetter {
  SerialGetter() {
    str = Serial.readString();
    getter = StringGetter(str.c_str());
  }
  char getNext(bool skip_whitespace = true) {
    return getter.getNext(skip_whitespace);
  }
  void skip_whitespace() { getter.skip_whitespace(); }
  char peek_next(bool skip_whitespace = true) {
    return getter.peek_next(skip_whitespace);
  }

private:
  String str;
  StringGetter getter;
};

// Info about one plan
struct EEPROMSlotInfo {
  RobotConfig config;
  uint8_t num_entries = 0;
};
static_assert(sizeof(EEPROMSlotInfo) == 4,
              "Program was designed with 4 bytes header per slot. If changed, "
              "ensure that all plans and their headers fit into the EEPROM.");

constexpr const int8_t magic_byte = 75;
// First byte is the default slot number.
constexpr const size_t EEPROM_header_size =
    1 + NumEEPROMSlots * sizeof(EEPROMSlotInfo);
constexpr const size_t EEPROM_plan_size = MaxPlanLength * sizeof(CompPlanEntry);
} // namespace

ParseError parseCoords(const char str[2], bool &col_first, uint8_t &row,
                       uint8_t &col) {
  col_first = isalpha(str[0]);
  uint8_t col_index = col_first ? 0 : 1;
  uint8_t row_index = 1 - col_index;
  col = tolower(str[col_index]) - 'a' + 1;
  row = str[row_index] - '0';

  return (col < 1 || col > 9 || row < 1 || row > 9) ? ParseError::InvalidCoords
                                                    : ParseError::OK;
}

ParseError parseInitPos(const char *str, RobotConfig &initConfig) {
  bool col_first;
  uint8_t row, col;

  ParseError coords_err = parseCoords(str, col_first, row, col);
  if (coords_err != ParseError::OK)
    return coords_err;
  Direction init_dir = (Direction)toupper(str[2]);

  if (init_dir != Direction::NORTH && init_dir != Direction::SOUTH &&
      init_dir != Direction::EAST && init_dir != Direction::WEST)
    return ParseError::InvalidOrient;

  initConfig.column = col;
  initConfig.row = row;
  initConfig.curr_dir = init_dir;
  return ParseError::OK;
}
ParseError parseCommand(const char str[2 + 5 + 1], CompPlanEntry &newEntry) {
  bool col_first;
  uint8_t row, col;
  ParseError coords_err = parseCoords(str, col_first, row, col);
  if (coords_err != ParseError::OK)
    return coords_err;

  int timePoint = atoi(str + 2);
  //atoi failed and time really is not a zero.
  if (timePoint == 0 && str[2]!='0')
    return ParseError::InvalidTime;

  newEntry.column = col & 0x0F;
  newEntry.row = row & 0x0F;
  newEntry.rowFirst = (!col_first) % 2;
  newEntry.timePoint = timePoint & 0x7FFF;
  return ParseError::OK;
}

void printErrMsg(ParseError e) {
  switch (e) {
  case ParseError::OK:
    Serial.println(F("No errors."));
    break;
  case ParseError::InvalidOrient:
    Serial.println(F("Invalid orientation in the initial position."));
    break;
  case ParseError::InvalidCoords:
    Serial.println(F("Invalid coordinates."));
    break;
  case ParseError::InvalidTime:
    Serial.println(F("Invalid time."));
    break;
  case ParseError::PlanTooLong:
    Serial.println(F("Too many commands."));
    break;
  case ParseError::UnexpectedEnd:
    Serial.println(F("Unexpected end of input."));
    break;
  };
}
bool Planner::loadDefault() {
  static const char *const str = "A1N E1T150 b2T350 3At450 4CT567 D2T700 ";
  int8_t byte;
  EEPROM.get(1023, byte);

  int8_t def_slot;
  EEPROM.get(0, def_slot);
  EEPROMSlotInfo info;
  if (def_slot < 0 || def_slot >= NumEEPROMSlots || magic_byte!=byte) {
    Serial.println(F("WARNING: Invalid default slot or ninitialized EEPROM? Fix "
                     "me with DX command and full EEPROM clear C0C1C2C3C4. Loading hardcoded string backup."));
    return loadFromString(str);
  }
  Serial.print(F("INFO: Loading the default plan from slot "));
  Serial.println(def_slot);

  EEPROM.get(1 + def_slot * sizeof(EEPROMSlotInfo), info);
  if (info.num_entries == 0 || info.num_entries>MaxPlanLength) {
    Serial.println(
        F("WARNING: The plan is empty or invalid, loading hardcoded string backup."));
    return loadFromString(str);
  } else {
    loadFromEEPROM(def_slot);
    Serial.print(F("INFO: Plan Loaded, number of entries:"));
    Serial.println(info.num_entries);
    return true;
  }
}
bool Planner::loadFromString(const char *str) {
  StringGetter getter = StringGetter(str);
  ParseError err = load(getter);
  if (err != ParseError::OK)
    printErrMsg(err);
  return err == ParseError::OK;
}
bool Planner::processRemoteRequests() {
  if (Serial.available() < 2)
    return false;

  char cmd = Serial.read();
  int slot = Serial.read() - '0';

  if (slot < 0 || slot >= NumEEPROMSlots) {
    Serial.print(F("ERROR: Wrong slot. Allowed slots are 0 to "));
    Serial.println(NumEEPROMSlots-1);
    return false;
  }

  if (cmd == 'C') {
    clearEEPROM(slot);
  } else if (cmd == 'S') {
    if (!loadFromSerial()){
      Serial.readString();
      return false;
    }
    Serial.print(F("INFO: Plan loaded, number of commands:"));
    Serial.println(num_active_entries);
    saveToEEPROM(slot);
  } else if (cmd == 'L') {
    loadFromEEPROM(slot);
    Serial.print(F("INFO: Plan loaded, number of commands:"));
    Serial.println(num_active_entries);
  } else if (cmd == 'D') {
    EEPROM.put(0, (int8_t)slot);
    EEPROM.put(1023,magic_byte);
    Serial.println(F("INFO: Default slot changed."));
  } else {
    Serial.println(F("ERROR: Unknown command."));
    return false;
  }
  return true;
}

Plan Planner::getActivePlan() {
  return Plan(num_active_entries, Planner::active_entries, active_init_config);
}

bool Planner::loadFromSerial() {
  SerialGetter getter;
  ParseError err = load(getter);
  if (err != ParseError::OK)
    printErrMsg(err);
  return err == ParseError::OK;
}

void Planner::loadFromEEPROM(int slot) {
  assert(slot >= 0 && slot < NumEEPROMSlots);
  EEPROMSlotInfo info;
  EEPROM.get(1 + sizeof(EEPROMSlotInfo) * slot, info);
  num_active_entries = info.num_entries;
  active_init_config = info.config;

  // Skip header and stored plans.
  int offset =
      EEPROM_header_size + slot * MaxPlanLength * sizeof(CompPlanEntry);
  for (int i = 0; i < num_active_entries; ++i)
    EEPROM.get(offset + i * sizeof(CompPlanEntry), active_entries[i]);
}
void Planner::saveToEEPROM(int slot) {
  assert(slot >= 0 && slot < NumEEPROMSlots);
  EEPROMSlotInfo info;
  info.num_entries = num_active_entries;
  info.config = active_init_config;

  EEPROM.put(1 + sizeof(EEPROMSlotInfo) * slot, info);
  // Skip header and stored plans.
  int offset = EEPROM_header_size + slot * EEPROM_plan_size;
  for (int i = 0; i < num_active_entries; ++i)
    EEPROM.put(offset + i * sizeof(CompPlanEntry), active_entries[i]);
}
void Planner::clearEEPROM(int slot) {
  assert(slot >= 0 && slot < NumEEPROMSlots);
  EEPROMSlotInfo empty;
  // First byte is the default slot number
  EEPROM.put(1 + slot * sizeof(EEPROMSlotInfo), empty);
  int offset = EEPROM_header_size + slot * EEPROM_plan_size;
  for (int i = 0; i < EEPROM_plan_size; i++)
    EEPROM.update(offset + i, 0);

  Serial.write("Slot cleared");
}
