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
    enum {WAITING_LINE, WAITING_STOP, STOPED} _step = WAITING_LINE;
    bool _done = false;
    unsigned long _line_time = 0;
    const unsigned long LINE_OFFSET_TIME = 400;
public:
    virtual void tick(sensors_t& sensors, Motor& left_motor, Motor& right_motor) {
        switch (_step)
        {
        case WAITING_LINE:
            if((sensors.left_dir && sensors.left_line) || (sensors.right_line && sensors.right_dir)) {
                Serial.println("exe: GO: WAITING_STOP");
                // Fix if one wheel is already stoped
                left_motor.go(MAX_SPEED_PERCENT);
                right_motor.go(MAX_SPEED_PERCENT);
                // Update state
                _line_time = millis();
                _step = WAITING_STOP;
                break;
            }
            if(sensors.left_line) {
                left_motor.go(MAX_SPEED_PERCENT/2);
                right_motor.go(MAX_SPEED_PERCENT);
            } else if(sensors.right_line) {
                left_motor.go(MAX_SPEED_PERCENT);
                right_motor.go(MAX_SPEED_PERCENT/2);
            } else {
                left_motor.go(MAX_SPEED_PERCENT);
                right_motor.go(MAX_SPEED_PERCENT);
            }
            break;
        case WAITING_STOP:
            if(millis() - _line_time > LINE_OFFSET_TIME) {
                Serial.println("exe: G0: STOP");
                left_motor.go(0);
                right_motor.go(0);
                _step = STOPED;
            }
            break;
        case STOPED:
            break;
        }
    }
    virtual bool isDone() {
        return _step == STOPED;
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
                if(!sensors.midle_line) {
                    Serial.println("exe: Tx: LOST_CENTER");
                    _step = LOST_CENTER;
                }
                break;
            case LOST_CENTER:
                if(sensors.left_dir || sensors.right_dir) {
                    Serial.println("exe: Tx: LINE_OUTSIDE");
                    _step = LINE_OUTSIDE;
                }
                break;
            case LINE_OUTSIDE:
                if(sensors.midle_line) {
                    Serial.println("exe: Tx: FOUND_CENTER");
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

class ExecutionTask {
  private:
    Plan _plan;
    PlanStep* _step = 0;
    unsigned long _start;
    bool _finished;

    void featchNextStep(bool go_home) {
        // Time info
        Serial.print("exe(");
        unsigned long time = millis() - _start;
        Serial.print(time);
        Serial.print("): ");

        // Next step
        delete _step;
        switch (go_home ? _plan.goHome() : _plan.getNext(time))
        {
        case Left:
            Serial.println("TL");
            _step = new TurnStep(true);
            break;
        case Right:
            Serial.println("TR");
            _step = new TurnStep(false);
            break;
        case Go: // Go straight through one junction
            Serial.println("GO");
            _step = new GoStep();
            break;
        case Wait:
            Serial.println("wait");
            _step = new WaitStep();
            break;
        case Finished:
            Serial.println("DONE");
            _finished  = true;
            break;
        default:
            Serial.println("ERROR: featchNextStep()");
            break;
        }
    }
  public:
    ExecutionTask(): _plan() {
        _finished = false;
        _start = millis();
        featchNextStep(false);
    }

    void start(Plan&& plan) {
        _plan = plan;
        _finished = false;
        _start = millis();
        featchNextStep(false);
    }

    bool isFinished() {
        return _finished;
    }
    
    void tick(sensors_t& sensors, Motor& left_motor, Motor& right_motor, boolean go_home = false) {
        if(!_finished) {
            _step->tick(sensors, left_motor, right_motor);
            if(_step->isDone()) {
                featchNextStep(go_home);
            }
        }
    }
};

#endif
