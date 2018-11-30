#include "control/kfc.h"

#include "control/crosswind/crosswind.h"
#include "control/controller_conversion.h"
#include "control/controller_util.h"
#include "control/crosswind/crosswind_types.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/control_log.h"

void controller_init(int *errStat, char *errMsg)
{
	printf("   controller_init\n");

	// Controller Version Number
	// Version Log:
	// 0.0.1 - Inner Crosswind Step working
	// 0.0.2 - Power Loop Step working
	const char controllerVerNumber[] = "0.0.2"; // major.minor.[maintenance]
	printf("   controller_version: %s \n", controllerVerNumber);

	//loadcontroller(params);
	// Init Data structures and variables
	const StateEstimate state_est = {}; 
	ControlState state = {};
	const double flaps_z1[kNumFlaps] = {}; // Last flap command from previous mode - Can link up to previous 'delta' - Added Jmiller - STI
	
	CrosswindInit(&state_est, flaps_z1, 0.0, 0, &GetControlParams()->crosswind, &state.crosswind);

	// Init Control Logging
	ControlLogInit((char*)controllerVerNumber);

	char tmp[] = "   controller initializing";
	int i;
	for (i = 0; i < sizeof(tmp); i++)
	{
		errMsg[i] = tmp[i];
	}
}

void controller_step(double dcm_g2b_c[], double pqr_c[], double *acc_norm_c,
					 double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
					 double Ab_c[], double *rho_c, double apparent_wind_c[],
					 double tether_force_c[], double wind_g_c[],
					 double kFlapA_c[], double Motor_c[],
					 int *errStat, char *errMsg)
{
	printf("   controller_step\n");
	// this portion should be done in controller_init()
	// ControlParams params; // placeholder to avoid errors
	StateEstimate state_est; // placeholder to avoid errors
	ControlState state = {};
	ControlOutput raw_control_output = {};
	FlightStatus flight_status = {	.flight_mode = kFlightModeCrosswindNormal,
								  	.last_flight_mode = kFlightModeCrosswindNormal,
									.flight_mode_first_entry = false };
	ControlLog control_log;
	// loadcontroller(&params);
	ControlParams *params = GetControlParamsUnsafe();
	//Convert the inputs from controller_step and assins the values that correspond to the inputs of CSim
	AssignInputs(dcm_g2b_c, pqr_c, acc_norm_c,
				 Xg_c, Vg_c, Vb_c, Ag_c,
				 Ab_c, rho_c, apparent_wind_c,
				 tether_force_c, wind_g_c,
				 kFlapA_c, Motor_c,
				 errStat, errMsg, &state_est);

	control_log.stateEstLog = state_est; 
	// Other modes to be added here

	// flight_status -> struct FlightStatus
	// state_est -> struct StateEstimate
	// params -> struct ControlParams
	// state -> struct ControlState
	// control_output -> struct ControlOutput
	printf("   debug marker - pre crosswindstep \n");
	CrosswindStep(&flight_status, &state_est, &params->crosswind,
				  &state.crosswind, &raw_control_output);

	printf("   debug marker - post crosswindstep \n");

	char tmp[] = "   controller stepping";
	int i;
	for (i = 0; i < sizeof(tmp); i++)
	{
		errMsg[i] = tmp[i];
	}
	printf("   debug marker : made it out of CrosswindStep with Active Powerloop!\n");
	// Saves all variables for this time step in ControlLog struct to txt file for analysis
	control_log.controlOutputLog = raw_control_output;
	ControlLogEntry(&control_log);
	// Connects values that are in ControlOutput data struct to the final outputs that Kitefast is expecting.
	AssignOutputs(kFlapA_c, Motor_c,
	errStat, errMsg, &raw_control_output);
}

void controller_end(int *errStat, char *errMsg)
{
	printf("   controller_end\n");
	char tmp[] = "controller ending";
	int i;
	for (i = 0; i < sizeof(tmp); i++)
	{
		errMsg[i] = tmp[i];
	}
}
