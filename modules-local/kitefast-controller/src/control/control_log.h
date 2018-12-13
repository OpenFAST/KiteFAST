#ifndef _CONTROL_LOG_H_
#define _CONTROL_LOG_H_

#include <stdio.h>
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"

typedef struct {

    ControlOutput controlOutputLog;
    StateEstimate stateEstLog;

} ControlLog;

void ControlLogInit(char* controllerVerNumber);
void ControlLogEntry(ControlLog* control_log);
#endif