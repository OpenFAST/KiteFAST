#ifndef CONTROL_GLOBSTRUCT_H
#define CONTROL_GLOBSTRUCT_H

#include "control/control_types.h"

typedef struct 
{
	StateEstimate state_est;
	ControlState state;
    ControlOutput raw_control_output;
	FlightStatus flight_status;
	
} ControlGlobal;
 
extern ControlGlobal controlglob;
#endif 