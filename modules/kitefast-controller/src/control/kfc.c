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

// KFC.C -> top level of shared library (binary file)
// This is The functions in this file are the callable

// ControlGlobal is a work around to avoid having to store controller specific variables in Kitefast
// The control global struct is defined in the init function, and then called throughout the step function
// Control Global comprises of existing CSim structrues (StateEstimate, ControlState, ControlOutput, FlightStatus)
ControlGlobal controlglob = {	.state_est = {
    								.dcm_g2b = {-0.8641, 0.2557, 0.4335, 0.1019, -0.7546, 0.6482, 0.4929, 0.6043, 0.6260}, //first step of crosswind
    								.pqr = {-0.011701762580859, -0.010291828869872, -0.390627923692243},
									.pqr_f = {-0.011701762580859, -0.010291828869872, -0.390627923692243},
									.pqr_f_lpf = {-0.011701762580859, -0.010291828869872, -0.390627923692243},
									.acc_norm_f = 28.735379499680100,
    								.Xg = {-1.230195825216543e+02, -3.808578802064749e+02, -1.737989235739968e+02},
    								.Vg_f = {-53.9007723439951, 3.32946946341383, 29.7069230287192},
									.Vg = {-53.9007723439951, 3.32946946341383, 29.7069230287192},
    								.Vb_f = {61.5573534961638,1.65877222531134, -2.61162923749005},
									.Vb = {61.5573534961638,1.65877222531134, -2.61162923749005},
    								.Ag = {-4.10429877138525, 18.8476886549997, -10.3879191238667},
    								.Ab_f = {0.160085873782321, -22.5059243248619, 0.695634407713604},
    								.rho = 1.0747,
    								.apparent_wind.sph_f.v = 54.4623229188392,
									.apparent_wind.sph_f.alpha = 0.0809881150728984,
									.apparent_wind.sph_f.beta = -0.0413704345112544,
									.apparent_wind.sph_f.alpha_lpf = 0.0809881150728984,
									.apparent_wind.sph_f.beta_lpf = -0.0413704345112544,
									.tether_force_b.vector_f.x = 4.436435166866530e+03,
									.tether_force_b.vector_f.y = -4.843583750096925e+04,
									.tether_force_b.vector_f.z = 1.312415108306612e+05,
									.tether_force_b.valid = true,
    								.tether_force_b.sph.tension = 1.385223443033077e+05,
									.tether_force_b.sph.roll = 0.353366385134087,
									.tether_force_b.sph.roll_lpf =0.353366385134087,
									// .tether_force_b.sph.pitch = 0.9995,
    								.wind_g.vector.x = -7.6604,
									.wind_g.vector.y = -6.4279,
									.wind_g.vector.z = 0,
									.wind_aloft_g.vector_f.x = -7.6604,
									.wind_aloft_g.vector_f.y = -6.4279,
									.wind_aloft_g.vector_f.z = 0,
									.wind_g.vector_f.x = -7.6604,
									.wind_g.vector_f.y = -6.4279,
									.wind_g.vector_f.z = 0,
									.wind_g.dir_f = -2.4435,
									.wind_aloft_g.speed_f_playbook = 10.0743,
									.wind_g.valid = true,
									//adding joystick throttle value: - 6/27/19
									.joystick.throttle_f = 0.81,
									.joystick.valid = 1,
									.joystick.pitch_f = 0,
									.joystick.data.throttle = 0.81,
									.joystick.data.roll = 0,
									.joystick.data.pitch = 0,
									.joystick.data.yaw = 0,
									.joystick.data.switch_position = 1,
									.joystick.data.release = 0,
									.joystick.data.engage_auto_glide = 0,
									.stacking_state = 0,
									.gps_active = true,
									.perch_azi.valid = true,
									.perch_azi.angle = 0.8698,
									.perch_azi.angle_vel_f = 0,
								},
								.flight_status = {	
									.flight_mode = kFlightModeCrosswindNormal,
								  	.last_flight_mode = kFlightModeCrosswindNormal,
									.flight_mode_first_entry = false },
								.state = {
									// these rotor torques/omegas/accel are the values that make up the initial guess to Kitefast (omegas taken from .h5 file)
									.motor_state = {
  										.rotor_torques = {341.8406, -376.9308, -414.5138, 455.5038, -374.0471, 348.2114, 326.1038, -306.6319},
  										.rotor_omegas = {-150, 144, 139, -134, 173, -178, -183, 188},
  										.rotor_accel = {-16.3256, 17.7546, 20.2884, -22.5797, 16.2031, -14.9828, -14.5061, 13.8764},
									}
								},
								.raw_control_output = {
									.flaps = {-0.2682,-0.2682, 0, 0, 0.0682, 0.0682, 0.0682},
									.rotors = {148.97, 144, 139, 134, 173, 178, 183, 188}
								}
							}; 

// controller Init function -> Highest Level of Shared library
// controller_init
// 		- Initializes controller modules (crosswind, motor control) and loads in controller params
// Inputs:
// 		Requested_dT 	- value of dt. this must agree with g_sys.ts or controller will throw assertion (double)
// 		numFlaps		- number of flaps the kitefast kite is currently using. this must agree with the enum kNumFlaps (int)
// 		numPylons		- number of pylons per wing of the kite is currently using. this must agree with the enum kNumFlaps (int)
// 		I_rot			- Moment of Inertia of each rotor (size = #ofRotors x 1) (double)
// 		errStat 		- Error value (int)
// 		errMsg 			- Error msg (char)
// Output -> all outputs below are passed as pointers 
//		genTorq
//		rotorSpeed
//		rotorAccel
//		rotorBlPit
//		ctrlSettings
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
#ifdef DEBUG									//DEBUG preproc found in kfc.h
	printf("   controller_version: %s \n", controllerVerNumber);
#endif

	// Init Data structures and variables
	const double flaps_z1[kNumFlaps] = {}; // Last flap command from previous mode - Can link up to previous 'delta' - Added Jmiller - STI
	
	// Init Crosswind
	CrosswindInit(&controlglob.state_est, flaps_z1, 0.0, 0, &GetControlParams()->crosswind, &controlglob.state.crosswind);
	
	// Init Motor Control  ### RRD modified the global variables here  <<<<<<<<<<< START
	for (int i = 0; i < kNumMotors; i++) {
      controlglob.state.motor_state.rotor_torques[i] = genTorq[i];
      controlglob.state.motor_state.rotor_omegas[i]  = rotorSpeed[i];
      controlglob.state.motor_state.rotor_accel[i]   = rotorAccel[i];
	}
	 // Copy flaps data from initial values provided by user
	memcpy(controlglob.raw_control_output.flaps, ctrlSettings, sizeof(controlglob.raw_control_output.flaps));
	
	//>>>>>>>>>>> END
  	InitMotorControl(&controlglob.state.motor_state);

	// Init Control Logging
	ControlLogInit((char*)controllerVerNumber);

	// Give inital guesses:
		// convert outputs from controller to the kitefast frame
	AssignOutputs(ctrlSettings, genTorq, rotorAccel, rotorSpeed, rotorBlPit,
	errStat, errMsg, &controlglob.raw_control_output, &controlglob.state.motor_state);

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
	printf(" \n\ndebug - t = %f\n",t);
#endif
	// Convert the inputs of controller_step and assign the values that correspond to the inputs of CSim crosswind step (mainly state estimate)
	// the inputs of controller_step are copied to thie corresponding variables found within StateEstimate 8
	AssignInputs(dcm_g2b_c, pqr_c, acc_norm_c,
				 Xg_c, Vg_c, Vb_c, Ag_c,
				 Ab_c, rho_c, apparent_wind_c,
				 tether_force_c, wind_g_c, AeroTorque,
				 errStat, errMsg, &controlglob.state_est,
				 &controlglob.state.motor_state);

	// intialize controller logging function
	ControlLog control_log;
	control_log.time = t; // time
	
	// control_log is the structure that will be saved to the .csv file after each timestep (can change file name and variables being saved within control_log.c)
	// Here, the state estimate data struct is being saves as "stateEstLogPreStep". 
	// This was done when investigating if any state estimator variables are manipulated within the controller (a PostStep can be found after the CrosswindStep)
	control_log.stateEstLogPreStep = controlglob.state_est; // logging state estimate data struct

	// Call to crosswind step. This function is mostly staight from the crosswind controller in Csim.
	CrosswindStep(&controlglob.flight_status, &controlglob.state_est, &GetControlParamsUnsafe()->crosswind,
				  &controlglob.state.crosswind, &controlglob.raw_control_output);

	// Apply sign directions to rotors before calling the motor control step. control_output.rotors[] are magnitudes of the rotor velocities.
	// with the sign, the rotor magnitudes become rotor omegas.
	// TODO JPM -> clean this reindexing up
	controlglob.raw_control_output.rotors[kMotor1] = -controlglob.raw_control_output.rotors[kMotor1];
	controlglob.raw_control_output.rotors[kMotor2] = controlglob.raw_control_output.rotors[kMotor2];
	controlglob.raw_control_output.rotors[kMotor3] = controlglob.raw_control_output.rotors[kMotor3];
	controlglob.raw_control_output.rotors[kMotor4] = -controlglob.raw_control_output.rotors[kMotor4];
	controlglob.raw_control_output.rotors[kMotor5] = controlglob.raw_control_output.rotors[kMotor5];
	controlglob.raw_control_output.rotors[kMotor6] = -controlglob.raw_control_output.rotors[kMotor6];
	controlglob.raw_control_output.rotors[kMotor7] = -controlglob.raw_control_output.rotors[kMotor7];
	controlglob.raw_control_output.rotors[kMotor8] = controlglob.raw_control_output.rotors[kMotor8];

	// Motor Control Step
	MotorControlStep(GetMotorParamsUnsafe(), 
		&(GetSystemParamsUnsafe()->rotors[0]), 
		&GetSystemParamsUnsafe()->power_sys, 
		controlglob.raw_control_output.rotors, controlglob.state.motor_state.aero_torque, 
		&controlglob.state.motor_state);

	// set motor direction was used to change the direction of the motors.
	// A new iteration saw that the rotors needed to be changed before going into MotorControlStep.
	// The function can be found in motors.c
	//SetMotorDirection(controlglob.state.motor_state.rotor_omegas, controlglob.state.motor_state.rotor_accel, 
	// controlglob.state.motor_state.rotor_omegas)

	// add default errMsg
	char tmp[] = "   controller stepping";
	int i;
	for (i = 0; i < sizeof(tmp); i++)
	{
		errMsg[i] = tmp[i];
	}
	// Saves all variables for this time step in ControlLog struct to txt file for analysis
	control_log.controlOutputLog = controlglob.raw_control_output;
	control_log.motor_state = controlglob.state.motor_state;
	control_log.stateEstLogPostStep = controlglob.state_est; // Post crosswind step saving
	// Call to control log
	ControlLogEntry(&control_log);

	// Assign outputs takes the outputs needed for kitefast, and extracts them from the corresponding variables within the CSim data structs
	// this function will also rearrange arrays to be in the correct indexing for kitefast. It was also flip some signs on the flaps. 
	AssignOutputs(CtrlSettings, GenTorque, RotorAccel, RotorSpeed, RotorBladePitch,
	errStat, errMsg, &controlglob.raw_control_output, &controlglob.state.motor_state);
}

// controller_end 
// 		- performs a end step of the controller (currently doesn't do anything that will affect simulation/controller)
// Inputs:
// 		errStat 		- Error value (currently a placeholder, no value sent from controller to kitefast will trigger anything)
// 		errMsg 			- Error msg (currently a placeholder, no value sent from controller to kitefast will trigger anything)
// Outputs: 
// 		errSta as pointer
// 		errMsg as pointer
//
// TODO:
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
