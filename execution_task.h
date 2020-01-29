#ifndef EXE_TASK
#define EXE_TASK

#include "ctl_common.h"
#include "Plan.h"
#include <limits.h> 

const int MAX_SPEED_PERCENT = 33;

class PlanStep {
public:
    virtual void tick(sensors_t& sensors, Motor& left_motor, Motor& right_motor) =  0;
    virtual bool isDone() = 0;
    virtual ~PlanStep() {}
};

class WaitStep : public PlanStep {
public:
    virtual void tick(sensors_t& sensors, Motor& left_motor, Motor& right_motor) {}
    virtual bool isDone() {
        return true;
    }
};

class GoStep : public PlanStep {
private:
    bool _done = false;
    unsigned long _line_time = ULONG_MAX;
    const unsigned long LINE_OFFSET_TIME = 1000;
public:
    virtual void tick(sensors_t& sensors, Motor& left_motor, Motor& right_motor) {
        // Detect line
        if((sensors.left_dir && sensors.left_line) || (sensors.right_line && sensors.right_dir))
            _line_time = millis();
        // Controll mothors
        if(millis() - _line_time <= LINE_OFFSET_TIME) {
            left_motor.go(MAX_SPEED_PERCENT);
            right_motor.go(MAX_SPEED_PERCENT);
        } else {
            left_motor.go(0);
            right_motor.go(0);
            _done = true;
        }
    }
    virtual bool isDone() {
        return _done;
    }
};

class TurnStep : public PlanStep {
private:
    enum {START, LOST_CENTER, LINE_OUTSIDE, FOUND_CENTER} _step = START;
    bool _left;
public:
    TurnStep(bool left): _left(left) {}

    virtual void tick(sensors_t& sensors, Motor& left_motor, Motor& right_motor) {
        switch(_step) {
            case START:
                if(_left) {
                    left_motor.go(-MAX_SPEED_PERCENT);
                    right_motor.go(MAX_SPEED_PERCENT);
                } else {
                    left_motor.go(MAX_SPEED_PERCENT);
                    right_motor.go(-MAX_SPEED_PERCENT);
                }
                if(!sensors.midle_line)
                    _step = LOST_CENTER;
                break;
            case LOST_CENTER:
                if(sensors.left_dir || sensors.right_dir)
                    _step = LINE_OUTSIDE;
                break;
            case LINE_OUTSIDE:
                if(sensors.midle_line) {
                    left_motor.go(0);
                    right_motor.go(0);
                    _step = FOUND_CENTER;
                }
                break;
            case FOUND_CENTER:
                break;
        }
    }

    virtual bool isDone() {
        return _step == FOUND_CENTER;
    }
};

class ExecutionTask : public RobotTask {
  private:
    Plan _plan;
    PlanStep&& _step = WaitStep();
    unsigned long _start;
    bool _finished = false;

    void featchNextStep() {
        switch (_plan.getNext(millis() - _start))
        {
        case Left:
            _step = TurnStep(true);
            break;
        case Right:
            _step = TurnStep(false);
            break;
        case Go: // Go straight through one junction
            _step = GoStep();
            break;
        case Wait:
            _step = WaitStep();
            break;
        case Finished:
            _finished  = true;
            break;
        default:
            break;
        }
    }
  public:
    ExecutionTask(): _plan() {
      featchNextStep();
    }

    virtual void start(Plan&& plan) {
      _plan = plan;
      _start = millis();
    }

    virtual bool isFinished() {
        return _finished;
    }
    
    virtual void tick(sensors_t& sensors, Motor& left_motor, Motor& right_motor) {
        if(!_finished) {
            _step.tick(sensors, left_motor, right_motor);
            if(_step.isDone()) {
                featchNextStep();
        }
        }
    }
};

#endif
