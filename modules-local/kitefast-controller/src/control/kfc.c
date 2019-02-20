#include "control/kfc.h"
#include <math.h>
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

// ControlGlobal is a work around to avoid having to store variables in Kitefast
// The control global struct is defined in the init function, and then called throughout the step function
// Control Global comprises of existing CSim structrues (StateEstimate, ControlState, ControlOutput, FlightStatus)
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
// controller Init function -> Highest Level of Shared library
// controller_init
// 		- Initializes controller modules (crosswind, motor control) and loads in controller params
// Inputs:
// 		errStat - Error value
// 		errMsg - Error msg
// Output -> errStat & errMsg are pointers
//
// TODO 
// 		- add error checks for number of Flaps & number Pylons (done)
// 		- Connect with new inputs from Kitefast (waiting on Update)
// 		- setup passing of controller version to kitefast -> version of controller will be git hash
// NOTE:
// 		- for initial motor guesses -> try zeros
// 			- If that doesnt work try running MotorStep() within MotorInit()
// 
void controller_init(double Requested_dT, int numFlaps, int numPylons, double genTorq[], 
                  double rotorSpeed[], double rotorAccel[], double rotorBlPit[], 
                  double ctrlSettings[], double I_rot[], int *errStat, char *errMsg)
{
	printf("   controller_init\n");
	//==== Perform Checks ====// 
	// flap check
	assert(numFlaps == kNumFlaps);
	// pylonCheck
	assert(numPylons == 2); // TODO - JPM, find suitable input for pylons instead of hardcoded val
	// time step check
	assert(Requested_dT == *g_sys.ts);
	// check I of Rotors
	
	//==== Controller Version Number ====//
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
	printf("   controller_version: %s \n", controllerVerNumber);

	// Init Data structures and variables
	const double flaps_z1[kNumFlaps] = {}; // Last flap command from previous mode - Can link up to previous 'delta' - Added Jmiller - STI
	
	// Init Crosswind
	CrosswindInit(&controlglob.state_est, flaps_z1, 0.0, 0, &GetControlParams()->crosswind, &controlglob.state.crosswind);
	
	// Init Motor Control
  	InitMotorControl(&controlglob.state.motor_state);

	// Init Control Logging
	ControlLogInit((char*)controllerVerNumber);

	// Give inital guesses:
	for (int i=0; i<kNumMotors; i++){
		genTorq[i]	  = 0;
		rotorSpeed[i] = 0;
		rotorAccel[i] = 0;
		rotorBlPit[i] = 0;
	}
	for (int i=0; i<kNumFlaps; i++){
		ctrlSettings[i] = 0;
	}

	char tmp[] = "   controller initializing";
	int i;
	for (i = 0; i < sizeof(tmp); i++)
	{
		errMsg[i] = tmp[i];
	}
	*errStat = 0;
}

// Controller Step -> Highest Level of Shared library
// 		- performs a time step of the controller
// Inputs:
// 		dcm_g2b_c - "The DCM to go from the controller ground system to the kite body system"
// 		pqr_c - "The kite angular velocities expressed in the kite body system" rad/s
//		acc_norm_c - "Magnitude of the acceleration vector" m/s^2
// 		Xg_c - "Location of the Kite Fuselage reference point in the controller ground system" m
// 		Vg_c - "The kite translational velocities expressed in the controller ground system" m/s
// 		Vb_c - "The kite translational velocities expressed in the kite body system" m/s
//		Ag_c - "The kite accelerations expressed in the controller ground system" m/s^2
// 		Ab_c - "The kite accelerations expressed in the kite body system" m/s^2
// 		rho_c - "air density (constant in time and space)" kg/m^3
// 		apparent_wind_c - "relative wind velocity at the fuselage reference point expressed in the controller ground system" m/s
//		tether_force_c - "tether tension at bridle connection in the kite body system" N
// 		wind_g_c - "wind velocity at the ground station point expressed in the controller ground system" m/s
// 		kFlapA_c - 
// 		Motor_c -
// 		errStat - Error value
// 		errMsg - Error msg
// Outputs: 
// 		kFlapA_c as pointer
// 		Motor_c as pointer
// 		errSta as pointer
// 		errMsg as pointer
//
// TODO:
// 		- Fill in kFlapA_c summary above
// 		- Connect with new Inputs/Outputs from Kitefast (waiting on Update)


void controller_step(double t, double dcm_g2b_c[], double pqr_c[], double *acc_norm_c,
					 double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
					 double Ab_c[], double *rho_c, double apparent_wind_c[],
					 double tether_force_c[], double wind_g_c[],
					 double CtrlSettings[], double GenTorque[],
					 double RotorBladePitch[], double RotorAccel[], 
					 double RotorSpeed[], double AeroTorque[],
					 int *errStat, char *errMsg)
{

	#ifdef DEBUG //DEBUG preproc found in kfc.h
//		printf("   controller_step\n");
	#endif
	// // placeholders for new inputs:
	// double ext_torques[] = AeroTorque; //coming in as Aerotorque
	//Convert the inputs from controller_step and assins the values that correspond to the inputs of CSim

	printf(" debug - t = %f\n",t);
	
    printf(" debug - ACTUALLY STEPPING \n");
	AssignInputs(dcm_g2b_c, pqr_c, acc_norm_c,
				 Xg_c, Vg_c, Vb_c, Ag_c,
				 Ab_c, rho_c, apparent_wind_c,
				 tether_force_c, wind_g_c,
				 errStat, errMsg, &controlglob.state_est,
				 &controlglob.state.motor_state);

	ControlLog control_log;
	control_log.stateEstLog = controlglob.state_est; 
	// Other modes to be added here
	#if DEBUG
		printf("   debug marker - pre crosswindstep \n");
	#endif
	CrosswindStep(&controlglob.flight_status, &controlglob.state_est, &GetControlParamsUnsafe()->crosswind,
				  &controlglob.state.crosswind, &controlglob.raw_control_output);
	#if DEBUG
		printf("   debug marker - post crosswindstep \n");
	#endif
	// Motor Control Step
	SetMotorDirection(controlglob.raw_control_output.rotors);
	MotorControlStep(GetMotorParamsUnsafe(), 
		&(GetSystemParamsUnsafe()->rotors[0]), 
		&GetSystemParamsUnsafe()->power_sys, 
		controlglob.raw_control_output.rotors, AeroTorque, 
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
	// double Gen_Torque[kNumMotors]  = {}; // placeholder for new input
	// double Rotor_Accel[kNumMotors] = {}; // placeholder for new input
	// double Rotor_Speed[kNumMotors] = {}; // placeholder for new input
	// double Blade_Pitch[kNumMotors] = {}; // placeholder for new input

	AssignOutputs(CtrlSettings, GenTorque, RotorAccel, RotorSpeed, RotorBladePitch,
	errStat, errMsg, &controlglob.raw_control_output, &controlglob.state.motor_state);
}

void controller_end(int *errStat, char *errMsg)
{
	#ifdef DEBUG //DEBUG preproc found in kfc.h
		printf("   controller_end\n");
	#endif
	char tmp[] = "controller ending";
	int i;
	for (i = 0; i < sizeof(tmp); i++)
	{
		errMsg[i] = tmp[i];
	}
}
