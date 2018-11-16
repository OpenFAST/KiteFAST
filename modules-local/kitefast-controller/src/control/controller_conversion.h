#ifndef _CONTROLLER_CONVERSION_H_
#define _CONTROLLER_CONVERSION_H_

#include <stdbool.h>

//#include "common\c_math\vec3.h"
//#include "c_math/vec3.h"
#include "control/control_types.h"

void AssignInputs(double dcm_g2b_c[], double pqr_c[], double *acc_norm_c,
	double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
	double Ab_c[], double *rho_c, double apparent_wind_c[],
	double tether_force_c[], double wind_g_c[],
	double kFlapA_c[], double Motor_c[],
	int *errStat, char *errMsg, StateEstimate *state_est);

void AssignOutputs(double kFlapA_c[], double Motor_c[],
	int *errStat, char *errMsg, ControlOutput* raw_control_output);

#endif
