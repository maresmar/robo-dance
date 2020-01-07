#ifndef EXE_TASK
#define EXE_TASK

#include "ctl_common.h"
#include "Plan.h"

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
public:
    virtual void tick(sensors_t& sensors, Motor& left_motor, Motor& right_motor) {
        left_motor.go(MAX_SPEED_PERCENT);
        right_motor.go(MAX_SPEED_PERCENT);
    }
    virtual bool isDone() {
        return false;
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
            break;
        case Right:
        case Go: // Go straight through one junction
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

    virtual void start(Plan plan) {
      _plan = plan;
      _start = millis();
    }

    virtual bool isFinished() {
        return _finished;
    }
    
    virtual void tick(sensors_t& sensors, Motor& left_motor, Motor& right_motor) {
        if(!_finished) {
            _step.tick(sensors, left_motor, right_motor);
        }
        if(_step.isDone()) {
          featchNextStep();
        }
    }
};

#endif
