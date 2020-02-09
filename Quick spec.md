# Robot dance

## Plan

- Go
  - 1 square position
- Turn
  - +/- 90 °
- Wait

```
PlanEntry:
tick()
isDone()
```

## Common

- Sensors
- Global state
  - Button press
  - Waiting / Executing / Going home
- Plan executor
- Plan loader
  - Load from
    - Load from EEPROM
    - Coded in C++
  - Save from serial to EEPROM
  - Reverse plan

  ## Control

  - Button 
      - Start
      - Reverse
