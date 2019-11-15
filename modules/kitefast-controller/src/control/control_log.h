#ifndef _CONTROL_LOG_H_
#define _CONTROL_LOG_H_

#include <stdio.h>
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"

typedef struct {
    double time;
    ControlOutput controlOutputLog;
    StateEstimate stateEstLogPreStep;
    StateEstimate stateEstLogPostStep;
    MotorState motor_state;

} ControlLog;

void ControlLogInit(char* controllerVerNumber);
void ControlLogEntry(ControlLog* control_log);
#endif
