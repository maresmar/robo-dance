# Robot dance

## Plan

- Go
  - 1 square position
  - *Optional: Direction*
- Turn
  - +/- 90 °
- Wait
  - Time (absolute)

```
PlanEntry:
loop()
isFinished()
```

## Common

- Sensors
- Global state
  - Button press
  - Stop / Executing / Returning
- Plan executor
- Plan loader
  - Load from
    - File
    - Coded in C++
  - Save from serial
  - Reverse plan

  ## Control

  - Button 
      - Start
      - Reverse
