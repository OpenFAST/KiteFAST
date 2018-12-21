#include "control/kfc.h"

#include "control/crosswind/crosswind.h"
#include "control/controller_conversion.h"
#include "control/controller_util.h"
#include "control/crosswind/crosswind_types.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/control_log.h"
#include "control/control_globstruct.h"
#include "control/physics/motor_param_types.h"
#include "control/rotor_control.h"
#include "control/physics/motor_params.h"
#include "control/system_params.h"

ControlGlobal controlglob = {	.flight_status = {	
									.flight_mode = kFlightModeCrosswindNormal,
								  	.last_flight_mode = kFlightModeCrosswindNormal,
									.flight_mode_first_entry = false },
								.state = {
									.motor_state = {
  										.rotor_omegas = {30.0, -30.0, 30.0, -30.0, 30.0, -30.0, 30.0, -30.0},
  										.rotor_torques = {0},
									}
								},
								.raw_control_output = {
									.rotors = {
										0,1,2,3,4,5,6,7
									}

								}
							}; 

void controller_init(int *errStat, char *errMsg)
{
	printf("   controller_init\n");

	// Controller Version Number
	// Version Log:
	// 0.1.0 - Inner Crosswind Step included and working
	// 0.2.0 - Power Loop Step included and working
	// 0.2.1 - added Global structs to handle passing between init and step functions
	// 0.3.0 - Path Loop inrotor_cmdscluded and working
	// 0.4.0 - Curvature Loop included and working 
	// 0.4.1 - Added pre-inner step preparation functions (CalcCrosswindPqr()/CalcCrosswindPqrDot()/CalcAeroForce())
	// 0.5.0 - Output Step icluded and working
	// 1.0.0 - First working draft of Controller - all minor steps are working
	// 1.1.0 - Motor Model added - hooks to kitefast still not connected - but controller is producing 
	const char controllerVerNumber[] = "1.1.0"; // major.minor.[maintenance]
	printf("   controller_version: %srotor_cmds \n", controllerVerNumber);

	// Init Data structures and variables
	const double flaps_z1[kNumFlaps] = {}; // Last flap command from previous mode - Can link up to previous 'delta' - Added Jmiller - STI
	
	CrosswindInit(&controlglob.state_est, flaps_z1, 0.0, 0, &GetControlParams()->crosswind, &controlglob.state.crosswind);
	
	// Motor Control
  	InitMotorControl(&controlglob.state.motor_state);

	ControlState controlstate_test = controlglob.state;
	StateEstimate stateest_test = controlglob.state_est;
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
	//Convert the inputs from controller_step and assins the values that correspond to the inputs of CSim
	AssignInputs(dcm_g2b_c, pqr_c, acc_norm_c,
				 Xg_c, Vg_c, Vb_c, Ag_c,
				 Ab_c, rho_c, apparent_wind_c,
				 tether_force_c, wind_g_c,
				 kFlapA_c, Motor_c,
				 errStat, errMsg, &controlglob.state_est,
				 &controlglob.state.motor_state);
	ControlLog control_log;
	control_log.stateEstLog = controlglob.state_est; 
	// Other modes to be added here
	#ifdef DEBUG
		printf("   debug marker - pre crosswindstep \n");
	#endif
	CrosswindStep(&controlglob.flight_status, &controlglob.state_est, &GetControlParamsUnsafe()->crosswind,
				  &controlglob.state.crosswind, &controlglob.raw_control_output);
	#ifdef DEBUG
		printf("   debug marker - post crosswindstep \n");
	#endif
	// Motor Control Step
	// double rotor_cmds[kNumMotors] = &controlglob->raw_control_output.rotors;
	double *ext_torques[kNumMotors] = {};
	SetMotorDirection(&controlglob.raw_control_output.rotors);
	MotorControlStep(GetMotorParamsUnsafe(), 
		&(GetSystemParamsUnsafe()->rotors[0]), 
		&GetSystemParamsUnsafe()->power_sys, 
		&controlglob.raw_control_output.rotors, ext_torques, 
		&controlglob.state.motor_state);

	char tmp[] = "   controller stepping";
	int i;
	for (i = 0; i < sizeof(tmp); i++)
	{
		errMsg[i] = tmp[i];
	}
	// Saves all variables for this time step in ControlLog struct to txt file for analysis
	control_log.controlOutputLog = controlglob.raw_control_output;
	control_log.motor_state = controlglob.state.motor_state;
	ControlLogEntry(&control_log);
	// Connects values that are in ControlOutput data struct to the final outputs that Kitefast is expecting.
	AssignOutputs(kFlapA_c, Motor_c,
	errStat, errMsg, &controlglob.raw_control_output, &controlglob.state.motor_state);
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
