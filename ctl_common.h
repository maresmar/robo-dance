#ifndef CTL_COMMON
#define CTL_COMMON

#include <Servo.h>

class Motor : public Servo {
  private:
    int _dir;
  public:
    void motor(void) {
      _dir = 1;
    }

    void go(int percentage) {
      writeMicroseconds(1500 + _dir * percentage * 2); // or so
    }

    void setDirection(bool there) {
      if (there)
        _dir = 1;
      else
        _dir = -1;
    }
};

struct sensors_t {
    bool left_dir;
    bool left_line;
    bool midle_line;
    bool right_line;
    bool right_dir;
};

class RobotTask {
  public:
    virtual bool isFinished() = 0;
    virtual void tick(sensors_t& sensors, Motor& left_motor, Motor& right_motor) = 0;
};

#endif
