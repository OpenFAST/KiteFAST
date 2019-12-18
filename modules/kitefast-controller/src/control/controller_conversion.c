#include "control/controller_conversion.h"

#include <stdio.h>
#include <math.h>
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/ground_station_frame.h"
#include "kfc.h"

// AssignInputs
// 		- assigns input variables to kitefastcontroller and finds their corresponding variables in the csim data structures. 
// Inputs:
// 		dcm_g2b_c 	- "The DCM to go from the controller ground system to the kite body system"
// 		pqr_c 		- "The kite angular velocities expressed in the kite body system" rad/s
//		acc_norm_c 	- "Magnitude of the acceleration vector" m/s^2
// 		Xg_c 		- "Location of the Kite Fuselage reference point in the controller ground system" m
// 		Vg_c 		- "The kite translational velocities expressed in the controller ground system" m/s
// 		Vb_c 		- "The kite translational velocities expressed in the kite body system" m/s
//		Ag_c 		- "The kite accelerations expressed in the controller ground system" m/s^2
// 		Ab_c 		- "The kite accelerations expressed in the kite body system" m/s^2
// 		rho_c 		- "air density (constant in time and space)" kg/m^3
// 		apparent_wind_c - "relative wind velocity at the fuselage reference point expressed in the controller ground system" m/s
//		tether_force_c 	- "tether tension at bridle connection in the kite body system" N
// 		wind_g_c 		- "wind velocity at the ground station point expressed in the controller ground system" m/s
// 		AeroTorque 		- "control surface deflections" rad
// 		errStat 		- Error value (currently a placeholder, no value sent from controller to kitefast will trigger anything)
// 		errMsg 			- Error msg (currently a placeholder, no value sent from controller to kitefast will trigger anything)
// Outputs: 
// 		StateEstimate *state_est as pointer
// 		MotorState *motor_state as pointer
//
// TODO:
// 		- Evaluate chosen frequencies on low pass filters (tether_roll, alpha, beta, and pqr signals)
//			- M. Abraham mentioned that low pass bandwidth on the tether roll (0.2Hz) is smaller than the controller's bandwidth (thats a nono)
// 		- clean-up dcm_g2b transpose implementation
// 		- check if sign flip on AeroTorque is no longer be necessary due to new act disk tables
// 		- (done) update hard-coded low-pass filter func. argumetns from 0.01 to gsys.ts 

// optimizer flag is included below to turn off optimizer which is used for debugging
__attribute__((optimize(0)))  void AssignInputs(double dcm_g2b_c[], double pqr_c[], double *acc_norm_c,
	double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
	double Ab_c[], double *rho_c, double apparent_wind_c[],
	double tether_force_c[], double wind_g_c[], double AeroTorque[], int *errStat, char *errMsg,
	StateEstimate *state_est, MotorState *motor_state){

	// print statements to allow user to view streaming kite state data
	#if DEBUG
		printf("  dcm_g2b_c = [%0.4f, %0.4f, %0.4f],[%0.4f, %0.4f, %0.4f],[%0.4f, %0.4f, %0.4f] \n",
					dcm_g2b_c[0], dcm_g2b_c[1], dcm_g2b_c[2], dcm_g2b_c[3], dcm_g2b_c[4], dcm_g2b_c[5], dcm_g2b_c[6], dcm_g2b_c[7], dcm_g2b_c[8]);
		printf("  pqr_c = [%0.4f, %0.4f, %0.4f] \n",pqr_c[0],pqr_c[1],pqr_c[2]);
		//printf("  acc_norm_c = %0.4f \n", *(double*)acc_norm_c);
		printf("  acc_norm_c = %0.4f \n", acc_norm_c);
		printf("  Xg_c = [%0.4f, %0.4f, %0.4f] \n",Xg_c[0],Xg_c[1],Xg_c[2]);
		printf("  Vg_c = [%0.4f, %0.4f, %0.4f] \n",Vg_c[0],Vg_c[1],Vg_c[2]);
		printf("  Vb_c = [%0.4f, %0.4f, %0.4f] \n",Vb_c[0],Vb_c[1],Vb_c[2]);
		printf("  Ag_c = [%0.4f, %0.4f, %0.4f] \n",Ag_c[0],Ag_c[1],Ag_c[2]);
		printf("  Ab_c = [%0.4f, %0.4f, %0.4f] \n",Ab_c[0],Ab_c[1],Ab_c[2]);
		printf("  rho_c = %0.4f \n", *(double*)rho_c);
		printf("  apparent_wind_c = [%0.4f, %0.4f, %0.4f] \n",apparent_wind_c[0],apparent_wind_c[1],apparent_wind_c[2]);
		printf("  tether_force_c = [%0.4f, %0.4f, %0.4f] \n",tether_force_c[0],tether_force_c[1],tether_force_c[2]);
		printf("  wind_g_c = [%0.4f, %0.4f, %0.4f] \n",wind_g_c[0],wind_g_c[1],wind_g_c[2]);
		printf("  AeroTorque (csim) = [%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f] \n",
			AeroTorque[0],
			AeroTorque[1],
			AeroTorque[2],
			AeroTorque[3],
			AeroTorque[4],
			AeroTorque[5],
			AeroTorque[6],
			AeroTorque[7]);

	#endif
	//dcm_2gb - convert and copy value into state_est->dcm_g2b
	const Mat3 dcm_g2b_tmp = { { { dcm_g2b_c[0], dcm_g2b_c[1], dcm_g2b_c[2] },
		                     { dcm_g2b_c[3], dcm_g2b_c[4], dcm_g2b_c[5] },
		                     { dcm_g2b_c[6], dcm_g2b_c[7], dcm_g2b_c[8] } } };
			 
	Mat3 dcm_g2b_t;
	// dcm_g2b given from kitefast is actually the transpose of the required matrix.
	// It was discovered that Fortran is column major (i.e. a 2x1 array when defined in fortran is a 1 row 2 column matrix)
	Mat3Trans(&dcm_g2b_tmp, &dcm_g2b_t);
	memcpy(&state_est->dcm_g2b, &dcm_g2b_t, sizeof(state_est->dcm_g2b));

	//pqr_c - convert and copy value into state_est->pqr_c
	Vec3 pqr_c_tmp = { pqr_c[0], pqr_c[1], pqr_c[2] };
	memcpy(&state_est->pqr_f, &pqr_c_tmp, sizeof(state_est->pqr_f));
	memcpy(&state_est->pqr, &pqr_c_tmp, sizeof(state_est->pqr));

	// Low pass filter added to help with transients - currently using 5Hz
	state_est->pqr_f.x = Lpf(state_est->pqr_f.x, 5, *g_sys.ts, &state_est->pqr_f_lpf.x);
	state_est->pqr_f.y = Lpf(state_est->pqr_f.y, 5, *g_sys.ts, &state_est->pqr_f_lpf.y);
	state_est->pqr_f.z = Lpf(state_est->pqr_f.z, 5, *g_sys.ts, &state_est->pqr_f_lpf.z);

	// save values to pqr_f_lpf to use in next step (Lpf requires values from previous step)
	state_est->pqr_f_lpf.x = state_est->pqr_f.x; 
	state_est->pqr_f_lpf.y = state_est->pqr_f.y;	
	state_est->pqr_f_lpf.z = state_est->pqr_f.z;

	//acc_norm_c - convert and copy value into state_est->acc_norm_f
	double acc_norm_f_tmp = *acc_norm_c;
	memcpy(&state_est->acc_norm_f, &acc_norm_f_tmp, sizeof(state_est->acc_norm_f));
	
	// Xg - convert and copy value into state_est->Xg
	Vec3 Xg_tmp = { Xg_c[0] , Xg_c[1] , Xg_c[2] };
	memcpy(&state_est->Xg, &Xg_tmp, sizeof(state_est->Xg));
	
	// Vg - convert and copy value into state_est->Vg & state_est->Vg_f since no filtered values are provided
	Vec3 Vg_tmp = { Vg_c[0] , Vg_c[1] , Vg_c[2] };
	memcpy(&state_est->Vg, &Vg_tmp, sizeof(state_est->Vg));
	memcpy(&state_est->Vg_f, &Vg_tmp, sizeof(state_est->Vg_f));

	// Vb - convert and copy value into state_est->Vb & state_est->Vb_f since no filtered values are provided
	Vec3 Vb_tmp = { Vb_c[0] , Vb_c[1] , Vb_c[2] };
	memcpy(&state_est->Vb, &Vb_tmp, sizeof(state_est->Vb));
	memcpy(&state_est->Vb_f, &Vb_tmp, sizeof(state_est->Vb_f));

	// Ag - convert and copy value into state_est->Ag  // RRD-Why does it have to be converted? Ask JUSTIN. Also I removed the - signs
	Vec3 Ag_tmp = { Ag_c[0] , Ag_c[1], Ag_c[2] };
	// Vec3 Ag_tmp = { Ag_c[0] , Ag_c[1], Ag_c[2] };
	memcpy(&state_est->Ag, &Ag_tmp, sizeof(state_est->Ag));

	// Ab_f - convert and copy value into state_est->Ab_f  // RRD-Why does it have to be converted? Ask JUSTIN. Also I removed the - signs
	Vec3 Ab_tmp = { Ab_c[0] , Ab_c[1] , Ab_c[2] };
	// Vec3 Ab_tmp = { Ab_c[0] , Ab_c[1] , Ab_c[2] };
	memcpy(&state_est->Ab_f, &Ab_tmp, sizeof(state_est->Ab_f));
	// rho - convert and copy value into state_est->rho
	double rho_tmp = *rho_c;
	memcpy(&state_est->rho, &rho_tmp, sizeof(state_est->rho));

	// apparent wind (consists of x,y,z velocity components)
	// controller primarily uses spherical coordinates.
	// Alpha, beta and V are needed in stead of cart. coords. 
	// create temporary Vec3 to handle velocity in cart. coord.
	Vec3 V_wind_b; 
	const Vec3 V_wind_tmp = {apparent_wind_c[0],apparent_wind_c[1],apparent_wind_c[2]};

	// find wind velocity in body frame
	Mat3Vec3Mult(&dcm_g2b_t, &V_wind_tmp, &V_wind_b);
	// magnitude of velocity vector
	state_est->apparent_wind.sph_f.v = Sqrt(Square(V_wind_b.x)+Square(V_wind_b.y)+Square(V_wind_b.z));
	// angle of attack approximation
	state_est->apparent_wind.sph_f.alpha = atan(V_wind_b.z/V_wind_b.x); 
	// side-slip  approximation
	state_est->apparent_wind.sph_f.beta = -asin(V_wind_b.y/state_est->apparent_wind.sph_f.v); 
	// added solution type - 6/27/19 (found value using .h5)
	state_est->apparent_wind.solution_type = kApparentWindSolutionTypePitot; 
	// apply low-pass filters to alpha and beta approximations
	state_est->apparent_wind.sph_f.alpha = Lpf(state_est->apparent_wind.sph_f.alpha , 2.0, *g_sys.ts, &state_est->apparent_wind.sph_f.alpha_lpf);
	state_est->apparent_wind.sph_f.beta  = Lpf(state_est->apparent_wind.sph_f.beta  , 2.0, *g_sys.ts, &state_est->apparent_wind.sph_f.beta_lpf );
	// save low-pass filtered values for next time step
	state_est->apparent_wind.sph_f.alpha_lpf = state_est->apparent_wind.sph_f.alpha;
	state_est->apparent_wind.sph_f.beta_lpf = state_est->apparent_wind.sph_f.beta;
	
	//RRD: changed this one to keep CSIM order in here
	for (int i = 0; i < kNumMotors; i++) {
      motor_state->aero_torque[i] = AeroTorque[i];
	}
		 
	// tether_force_c
	// calculate tether roll angle using force vector components:
	double Tx = tether_force_c[0];
	double Ty = tether_force_c[1];
	double Tz = tether_force_c[2];
	// tether roll approximation using tether force
	state_est->tether_force_b.sph.roll = acos((pow(Tx,2)+pow(Tz,2))/(sqrt(pow(Tx,2)+pow(Tz,2))*sqrt(pow(Tx,2)+pow(Ty,2)+pow(Tz,2))));
	// add low-pass filter to filter out transients seen in force vectors
	state_est->tether_force_b.sph.roll = Lpf(state_est->tether_force_b.sph.roll, 0.2, *g_sys.ts, &state_est->tether_force_b.sph.roll_lpf);
	// save current tether roll angle to be using in next step.
	state_est->tether_force_b.sph.roll_lpf = state_est->tether_force_b.sph.roll;
	// approximation for tether pitch (unused in controller)
	state_est->tether_force_b.sph.pitch = Tz / (sqrt(pow(Tx,2)+pow(Tz,2))); // adding tether pitch - 6/27/19
	printf("  Tether Roll Angle (rad): = [%0.4f] \n",state_est->tether_force_b.sph.roll);
	state_est->tether_force_b.vector_f.x = Tx;
	state_est->tether_force_b.vector_f.y = Ty;
	state_est->tether_force_b.vector_f.z = Tz;
	state_est->tether_force_b.valid = true; 
	// adding addtional signals to be sure nothing is left unitinitialized - 6/27/19
	state_est->tether_force_b.vector.x = Tx;
	state_est->tether_force_b.vector.y = Ty;
	state_est->tether_force_b.vector.z = Tz;
	// assumed tension is magnitude of tether force inputs - Jmiller - STI
	state_est->tether_force_b.sph.tension = sqrt(pow(Tx,2) + pow(Ty,2) + pow(Tz,2));
	printf("  Tether Tension (N): = [%0.4f] \n",state_est->tether_force_b.sph.tension);

	//wind_g_c
	Vec3 wind_g_csim = {wind_g_c[0],wind_g_c[1],wind_g_c[2]};
	state_est->wind_g.vector.x = wind_g_csim.x;
	state_est->wind_g.vector.y = wind_g_csim.y;
	state_est->wind_g.vector.z = wind_g_csim.z;
	// calculate direction of widn using VecGToAzimuthTz
	state_est->wind_g.dir_f = VecGToAzimuth(&state_est->wind_g.vector);
	// adding filtered versions - 6/27/19
	state_est->wind_g.vector_f.x = wind_g_csim.x;
	state_est->wind_g.vector_f.y = wind_g_csim.y;
	state_est->wind_g.vector_f.z = wind_g_csim.z;
	state_est->wind_g.vector_f_slow.x = wind_g_csim.x;
	state_est->wind_g.vector_f_slow.y = wind_g_csim.y;
	state_est->wind_g.vector_f_slow.z = wind_g_csim.z;
	// holding at constant 10 due to wing_g.speed_f_playbook from csim data until origin on this variable is found
	// added 6/27/19
	state_est->wind_aloft_g.vector.x = wind_g_csim.x;
	state_est->wind_aloft_g.vector.y = wind_g_csim.y;
	state_est->wind_aloft_g.vector.z = wind_g_csim.z;
	state_est->wind_aloft_g.vector_f.x = wind_g_csim.x;
	state_est->wind_aloft_g.vector_f.y = wind_g_csim.y;
	state_est->wind_aloft_g.vector_f.z = wind_g_csim.z;
	state_est->wind_aloft_g.vector_f_slow.x = wind_g_csim.x;
	state_est->wind_aloft_g.vector_f_slow.y = wind_g_csim.y;
	state_est->wind_aloft_g.vector_f_slow.z = wind_g_csim.z;
	// estimate for playbook speed (magnitude of the wind_aloft vector)
	state_est->wind_aloft_g.speed_f_playbook = sqrt(pow(state_est->wind_aloft_g.vector_f.x,2) + pow(state_est->wind_aloft_g.vector_f.y,2) + pow(state_est->wind_aloft_g.vector_f.z,2));

	// adding joystick throttle value: - 6/27/19
	state_est->joystick.throttle_f = 0.81;
	state_est->joystick.valid = 1;
	state_est->joystick.pitch_f = 0;
	state_est->joystick.data.throttle = 0.81;
	state_est->joystick.data.roll = 0;
	state_est->joystick.data.pitch = 0;
	state_est->joystick.data.yaw = 0;
	state_est->joystick.data.switch_position = 1;
	state_est->joystick.data.release = 0;
	state_est->joystick.data.engage_auto_glide = 0;

	// adding valid flag - 6/27/19
	state_est->wind_g.valid = true;
	state_est->gps_active = true;
	
	// added perch_azi values from .h5 file
	state_est->perch_azi.valid = true;
	state_est->perch_azi.angle = 0.8698;
	state_est->perch_azi.angle_vel_f = 0;

	state_est->stacking_state = 0;

}

// AssignOutput
// 		- assigns output variables to kitefastcontroller and finds their corresponding variables in the csim data structures. 
// Inputs:
//		- ControlOutput csim data structure
//		- MotorState csim data structure
// Outputs: 
// 		- CtrlSettings 	- control surface deflections in kitefast frame (passed to function as pointer) (rad)
// 		- Gen_Torque 	- generator torque in kitefast frame (passed to function as pointer) (N-m)
// 		- Rotor_Accel 	- rotor accelerations in kitefast frame (passed to function as pointer) (m/s^2)
// 		- Rotor_Speed 	- (in kitefast frame) (passed to function as pointer) (m/s)
// 		- Blade_Pitch 	- (in kitefast frame) (passed to function as pointer) (rad)
// 		- errStat 		- Error value (currently a placeholder, no value sent from controller to kitefast will trigger anything)
// 		- errMsg 		- Error msg (currently a placeholder, no value sent from controller to kitefast will trigger anything)
// TODO:
// 		
void AssignOutputs(double CtrlSettings[], double Gen_Torque[],
	double Rotor_Accel[], double Rotor_Speed[], double Blade_Pitch[],
	int *errStat, char *errMsg, 
	ControlOutput* raw_control_output, MotorState* motor_state){

	// MAKANI FLAP convention (top-down view, nose facing up)
	// 
	// Port			           Center				Starboard
	// 							  ^	
	// 						      ||
	// -------------------------------------------------------- wing
	// // A1  //  A2  //  A3  //  ||  \\  A5  \\  A7  \\  A8  \\  
	// 							  ||
	// KFAST FLAP convention

	//RRD replace CtrlSettings above with this copy/pass-through; Could not figure out how to use membpy, ask JUSTIN
	//memcpy(&CtrlSettings, &raw_control_output->flaps, sizeof(CtrlSettings));
	for (int i = 0; i < kNumFlaps; i++) {
      CtrlSettings[i] = raw_control_output->flaps[i];
	}
	
	// Blade Pitch 
	// currently not apart of controller. Place holders can be found below:
	*Blade_Pitch    = 0.0; // Can this take care of all ?
	// Blade_Pitch[0] = 0.0;
	// Blade_Pitch[1] = 0.0; 
	// Blade_Pitch[2] = 0.0; 
	// Blade_Pitch[3] = 0.0; 
	// Blade_Pitch[4] = 0.0; 
	// Blade_Pitch[5] = 0.0; 
	// Blade_Pitch[6] = 0.0; 
	// Blade_Pitch[7] = 0.0; 

	//// MOTORS
	// Kitefast Motor Order -> Kitefast sign convention
	// 
	// [0] starboard-inboard-top      (-)
	// [1] starboard-inboard-bottom   (+)
	// [2] starboard-outboard-top     (+)
	// [3] starboard-outboard-bottom  (-)
	// [4] port-inboard-top           (-)
	// [5] port-inboard-bottom        (+)
	// [6] port-outboard-top          (+)
	// [7] port-outboard-bottom       (-)

	// Controller Motor Order -> Makani Sign Convention
	// 
	// Table:  location, #, name and rotational direction
	// (Pos or Neg) of each rotor, from the position of standing in front of and facing the kite.
	// 
	// 8. STo  | 7. STi|          | 6. PTi  | 5. PTo                       
	// Pos     | Neg   |          | Neg     | Pos
	// ----------------------------------------------
	// Starboard Wing  | Fuselage | Port Wing
	// ----------------------------------------------
	// 1. SBo  | 2. SBi|          | 3. PBi  | 4. PBo
	// Neg     | Pos   |          | Pos     | Neg
	// 
	// The propellers follow
	// the sign convention where positive means
	// that the propeller is rotating in a positive direction (i.e. right
	// hand rule) about the propeller axis, which is predominately in the
	// same direction as the body x-axis.
	// 
	// SORTED BY MAKANI 0-7			// SORTED BY KFAST 0-7	
	// ID   | Makani | KFast		ID   | KFast  | Makani	
	// -----------------------	   -----------------------	
	// SBo  |   1    |   3   		STi  |   0    |   7      			
	// SBi  |   2    |   1   		SBi  |   1    |   2      	
	// PBi  |   3    |   5   		STo  |   2    |   8      	
	// PBo  |   4    |   7   		SBo  |   3    |   1      	
	// PTo  |   5    |   6   		PTi  |   4    |   6      	
	// PTi  |   6    |   4   		PBi  |   5    |   3      	
	// STi  |   7    |   0   		PTo  |   6    |   5      	
	// STo  |   8    |   2   		PBo  |   7    |   4    

	// SUMMARY 
	// [0] Starboard inner Top = kMotor7 (-)
	// [1] Starboard inner Bot = kMotor2 (+)
	// [2] Starboard outer Top = kMotor8 (+)
	// [3] Starboard outer Bot = kMotor1 (-)
	// [4] Port      inner Top = kMotor6 (-)
	// [5] Port      inner Bot = kMotor3 (+)
	// [6] Port      outer Top = kMotor5 (+)
	// [7] Port      outer Bot = kMotor4 (-)

	//Generator Torques
	// RRD: Can we use memcpy to remove for loops?
	for (int i = 0; i < kNumMotors; i++) {
      Gen_Torque[i]  = motor_state->rotor_torques[i];
	  Rotor_Accel[i] = motor_state->rotor_accel[i];
	  Rotor_Speed[i] = motor_state->rotor_omegas[i];
	}


	// Print outputs of controller:
	#if DEBUG
		printf("  kFlapA_c (kfas frame) = [A1=%0.4f, A2=%0.4f, A4=%0.4f, A5=%0.4f, A7=%0.4f, A8=%0.4f, El=%0.4f, Ru=%0.4f] \n", //, E1=%0.4f, E2=%0.4f, E3=%0.4f, E4=%0.4f
				CtrlSettings[0],
				CtrlSettings[1],
				CtrlSettings[2],
				CtrlSettings[3],
				CtrlSettings[4],
				CtrlSettings[5],
				CtrlSettings[6],
				CtrlSettings[7]);
		printf("  kFlapA_c (csim frame) = [%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f] \n",
				raw_control_output->flaps[kFlapA1],
				raw_control_output->flaps[kFlapA2],
				raw_control_output->flaps[kFlapA4],
				raw_control_output->flaps[kFlapA5],
				raw_control_output->flaps[kFlapA7],
				raw_control_output->flaps[kFlapA8],
				-raw_control_output->flaps[kFlapEle],
				raw_control_output->flaps[kFlapRud]);
		printf("  Gen_Torque (csim frame) = [%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f] \n",
				motor_state->rotor_torques[kMotor1],
				motor_state->rotor_torques[kMotor2],
				motor_state->rotor_torques[kMotor3],
				motor_state->rotor_torques[kMotor4],
				motor_state->rotor_torques[kMotor5],
				motor_state->rotor_torques[kMotor6],
				motor_state->rotor_torques[kMotor7],
				motor_state->rotor_torques[kMotor8]);
		printf("  Rotor_Speed (csim frame) = [%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f] \n",
				motor_state->rotor_omegas[kMotor1],
				motor_state->rotor_omegas[kMotor2],
				motor_state->rotor_omegas[kMotor3],
				motor_state->rotor_omegas[kMotor4],
				motor_state->rotor_omegas[kMotor5],
				motor_state->rotor_omegas[kMotor6],
				motor_state->rotor_omegas[kMotor7],
				motor_state->rotor_omegas[kMotor8]);
		printf("  Rotor_Accel (csim frame) = [%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f] \n",
				motor_state->rotor_accel[kMotor1],
				motor_state->rotor_accel[kMotor2],
				motor_state->rotor_accel[kMotor3],
				motor_state->rotor_accel[kMotor4],
				motor_state->rotor_accel[kMotor5],
				motor_state->rotor_accel[kMotor6],
				motor_state->rotor_accel[kMotor7],
				motor_state->rotor_accel[kMotor8]);
	#endif
}
