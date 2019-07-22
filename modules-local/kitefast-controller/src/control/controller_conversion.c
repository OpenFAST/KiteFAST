#include "control/controller_conversion.h"

#include <stdio.h>
#include <math.h>
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/ground_station_frame.h"
#include "kfc.h"

void AssignInputs(double dcm_g2b_c[], double pqr_c[], double *acc_norm_c,
	double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
	double Ab_c[], double *rho_c, double apparent_wind_c[],
	double tether_force_c[], double wind_g_c[], double AeroTorque[], int *errStat, char *errMsg,
	StateEstimate *state_est, MotorState *motor_state){

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
			-AeroTorque[3],
			-AeroTorque[1],
			-AeroTorque[5],
			-AeroTorque[7],
			-AeroTorque[6],
			-AeroTorque[4],
			-AeroTorque[0],
			-AeroTorque[2]);
		// printf("  AeroTorque (kfas) = [%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f] \n",
		// 	AeroTorque[0],
		// 	AeroTorque[1],
		// 	AeroTorque[2],
		// 	AeroTorque[3],
		// 	AeroTorque[4],
		// 	AeroTorque[5],
		// 	AeroTorque[6],
		// 	AeroTorque[7]);
	#endif
	//dcm_2gb - convert and copy value into state_est->dcm_g2b
	const Mat3 dcm_g2b_tmp = { { { dcm_g2b_c[0], dcm_g2b_c[1], dcm_g2b_c[2] },
		                     { dcm_g2b_c[3], dcm_g2b_c[4], dcm_g2b_c[5] },
		                     { dcm_g2b_c[6], dcm_g2b_c[7], dcm_g2b_c[8] } } };
			 
	Mat3 dcm_g2b_t;
	Mat3Trans(&dcm_g2b_tmp, &dcm_g2b_t);
	memcpy(&state_est->dcm_g2b, &dcm_g2b_t, sizeof(state_est->dcm_g2b));

	//pqr_c - convert and copy value into state_est->pqr_c
	Vec3 pqr_c_tmp = { pqr_c[0], pqr_c[1], pqr_c[2] };
	memcpy(&state_est->pqr_f, &pqr_c_tmp, sizeof(state_est->pqr_f));
	memcpy(&state_est->pqr, &pqr_c_tmp, sizeof(state_est->pqr));

	state_est->pqr_f.x = Lpf(state_est->pqr_f.x, 5, 0.01, &state_est->pqr_f_lpf.x);
	state_est->pqr_f.y = Lpf(state_est->pqr_f.y, 5, 0.01, &state_est->pqr_f_lpf.y);
	state_est->pqr_f.z = Lpf(state_est->pqr_f.z, 5, 0.01, &state_est->pqr_f_lpf.z);

	state_est->pqr_f_lpf.x = state_est->pqr_f.x;
	state_est->pqr_f_lpf.y = state_est->pqr_f.y;	
	state_est->pqr_f_lpf.z = state_est->pqr_f.z;

	//acc_norm_c - convert and copy value into state_est->acc_norm_f
	double acc_norm_f_tmp = *acc_norm_c;
	memcpy(&state_est->acc_norm_f, &acc_norm_f_tmp, sizeof(state_est->acc_norm_f));

	// Xg - convert and copy value into state_est->Xg
	Vec3 Xg_tmp = { Xg_c[0] , Xg_c[1] , Xg_c[2] };
	memcpy(&state_est->Xg, &Xg_tmp, sizeof(state_est->Xg));
	
	// Vg - convert and copy value into state_est->Vg
	Vec3 Vg_tmp = { Vg_c[0] , Vg_c[1] , Vg_c[2] };
	//Vec3 Vg_tmp_csim;
	//Mat3Vec3Mult(&dcm_kfast2csim_ground, &Vg_tmp, &Vg_tmp_csim);
	memcpy(&state_est->Vg, &Vg_tmp, sizeof(state_est->Vg));
	memcpy(&state_est->Vg_f, &Vg_tmp, sizeof(state_est->Vg_f));

	// Vb - convert and copy value into state_est->Vb
	Vec3 Vb_tmp = { Vb_c[0] , Vb_c[1] , Vb_c[2] };
	memcpy(&state_est->Vb, &Vb_tmp, sizeof(state_est->Vb));
	memcpy(&state_est->Vb_f, &Vb_tmp, sizeof(state_est->Vb_f));

	// Ag - convert and copy value into state_est->Ag
	Vec3 Ag_tmp = { -Ag_c[0] , -Ag_c[1], -Ag_c[2] };
	// Vec3 Ag_tmp = { Ag_c[0] , Ag_c[1], Ag_c[2] };
	memcpy(&state_est->Ag, &Ag_tmp, sizeof(state_est->Ag));

	// Ab_f - convert and copy value into state_est->Ab_f
	Vec3 Ab_tmp = { -Ab_c[0] , -Ab_c[1] , -Ab_c[2] };
	// Vec3 Ab_tmp = { Ab_c[0] , Ab_c[1] , Ab_c[2] };
	memcpy(&state_est->Ab_f, &Ab_tmp, sizeof(state_est->Ab_f));
	// rho - convert and copy value into state_est->rho
	double rho_tmp = *rho_c;
	memcpy(&state_est->rho, &rho_tmp, sizeof(state_est->rho));

	Vec3 V_wind_b; 
	const Vec3 V_wind_tmp = {apparent_wind_c[0],apparent_wind_c[1],apparent_wind_c[2]};
	Mat3Vec3Mult(&dcm_g2b_t, &V_wind_tmp, &V_wind_b);
	state_est->apparent_wind.sph_f.v = Sqrt(Square(V_wind_b.x)+Square(V_wind_b.y)+Square(V_wind_b.z));
	state_est->apparent_wind.sph_f.alpha = atan(V_wind_b.z/V_wind_b.x);
	state_est->apparent_wind.sph_f.beta = -asin(V_wind_b.y/state_est->apparent_wind.sph_f.v); // remove (-) and test
	state_est->apparent_wind.solution_type = kApparentWindSolutionTypePitot; // added 6/27/19

	state_est->apparent_wind.sph_f.alpha = Lpf(state_est->apparent_wind.sph_f.alpha , 0.5, 0.01, &state_est->apparent_wind.sph_f.alpha_lpf);
	state_est->apparent_wind.sph_f.beta  = Lpf(state_est->apparent_wind.sph_f.beta  , 0.5, 0.01, &state_est->apparent_wind.sph_f.beta_lpf );

	state_est->apparent_wind.sph_f.alpha_lpf = state_est->apparent_wind.sph_f.alpha;
	state_est->apparent_wind.sph_f.beta_lpf = state_est->apparent_wind.sph_f.beta;

	// AeroTorque
	double AeroTorque_tmp[8];
	memcpy(&AeroTorque_tmp, &AeroTorque, sizeof(AeroTorque));
	// AeroTorque Must be in Makani
	AeroTorque[0] = -AeroTorque_tmp[3]; // SBo
	AeroTorque[1] = -AeroTorque_tmp[1]; // SBi
	AeroTorque[2] = -AeroTorque_tmp[5]; // PBi
	AeroTorque[3] = -AeroTorque_tmp[7]; // PBo
	AeroTorque[4] = -AeroTorque_tmp[6]; // PTo
	AeroTorque[5] = -AeroTorque_tmp[4]; // PTi
	AeroTorque[6] = -AeroTorque_tmp[0]; // STi
	AeroTorque[7] = -AeroTorque_tmp[2]; // STo
		 
	//tether_force_c
	// calculate tether roll angle using force vector components:
	double Tx = tether_force_c[0];
	double Ty = tether_force_c[1];
	double Tz = tether_force_c[2];
	
	//todo update 0.01 to gsys
	state_est->tether_force_b.sph.roll = acos((pow(Tx,2)+pow(Tz,2))/(sqrt(pow(Tx,2)+pow(Tz,2))*sqrt(pow(Tx,2)+pow(Ty,2)+pow(Tz,2))));
	state_est->tether_force_b.sph.roll = Lpf(state_est->tether_force_b.sph.roll, 0.2, 0.01, &state_est->tether_force_b.sph.roll_lpf);
	state_est->tether_force_b.sph.roll_lpf = state_est->tether_force_b.sph.roll;
	//state_est->tether_force_b.sph.pitch = Tz / (sqrt(pow(Tx,2)+pow(Tz,2))); // adding tether pitch - 6/27/19
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

	state_est->perch_azi.valid = true;
	state_est->perch_azi.angle = 0.8698;
	state_est->perch_azi.angle_vel_f = 0;

}

void AssignOutputs(double CtrlSettings[], double Gen_Torque[],
	double Rotor_Accel[], double Rotor_Speed[], double Blade_Pitch[],
	int *errStat, char *errMsg, 
	ControlOutput* raw_control_output, MotorState* motor_state){

	// Kite Fast control surfaces convention
	// starboard-wing-1
	// starboard-wing-2
	// port-wing-1
	// port-wing-2
	// starboard-horizontalstab-1
	// starboard-horizontalstab-2
	// port-horizontalstab-1
	// port-horizontalstab-2
	// verticalstab-1
	// verticalstab-2

	// MAKANI FLAP convention (top-down view, nose facing up)
	// 
	// Port			           Center				Starboard
	// 							  ^	
	// 						      ||
	// -------------------------------------------------------- wing
	// // A1  //  A2  //  A3  //  ||  \\  A5  \\  A7  \\  A8  \\  
	// 							  ||
	// KFAST FLAP convention

	//kFlap_A
	CtrlSettings[0] = 	raw_control_output->flaps[kFlapA5]; 	//	Starboard wing control ID 1
	CtrlSettings[1] = 	raw_control_output->flaps[kFlapA7]; 	//	Starboard wing control ID 2
	CtrlSettings[2] = 	raw_control_output->flaps[kFlapA8]; 	//	Starboard wing control ID 3
	CtrlSettings[3] = 	raw_control_output->flaps[kFlapA4]; 	//	Port      wing control ID 1
	CtrlSettings[4] = 	raw_control_output->flaps[kFlapA2]; 	//	Port      wing control ID 2
	CtrlSettings[5] = 	raw_control_output->flaps[kFlapA1]; 	//	Port      wing control ID 3
	CtrlSettings[6] = 	raw_control_output->flaps[kFlapRud]; 	//	Rudder
	CtrlSettings[7] = 	raw_control_output->flaps[kFlapRud];	//	Rudder
	CtrlSettings[8] = 	-raw_control_output->flaps[kFlapEle];	//	Elevators	
	CtrlSettings[9] = 	-raw_control_output->flaps[kFlapEle];	//	Elevators	
	CtrlSettings[10] = 	-raw_control_output->flaps[kFlapEle];	//	Elevators	
	CtrlSettings[11] = 	-raw_control_output->flaps[kFlapEle];	// 	Elevators
	
	// Blade Pitch
	Blade_Pitch[0] = 0.0;
	Blade_Pitch[1] = 0.0; 
	Blade_Pitch[2] = 0.0; 
	Blade_Pitch[3] = 0.0; 
	Blade_Pitch[4] = 0.0; 
	Blade_Pitch[5] = 0.0; 
	Blade_Pitch[6] = 0.0; 
	Blade_Pitch[7] = 0.0; 

	// //Motor speeds	
	// Motor_c[0] = motor_state->motor_speeds[kMotor1];
	// Motor_c[1] = motor_state->motor_speeds[kMotor2];
	// Motor_c[2] = motor_state->motor_speeds[kMotor3];
	// Motor_c[3] = motor_state->motor_speeds[kMotor4];
	// Motor_c[4] = motor_state->motor_speeds[kMotor5];
	// Motor_c[5] = motor_state->motor_speeds[kMotor6];
	// Motor_c[6] = motor_state->motor_speeds[kMotor7];
	// Motor_c[7] = motor_state->motor_speeds[kMotor8];

	//Generator Torques
	Gen_Torque[0] = motor_state->rotor_torques[kMotor7]; // Starboard inner Top = kMotor7
	Gen_Torque[1] = motor_state->rotor_torques[kMotor2]; // Starboard inner Bot = kMotor2
	Gen_Torque[2] = motor_state->rotor_torques[kMotor8]; // Starboard outer Top = kMotor8
	Gen_Torque[3] = motor_state->rotor_torques[kMotor1]; // Starboard outer Bot = kMotor1
	Gen_Torque[4] = motor_state->rotor_torques[kMotor6]; // Port      inner Top = kMotor6
	Gen_Torque[5] = motor_state->rotor_torques[kMotor3]; // Port      inner Bot = kMotor3
	Gen_Torque[6] = motor_state->rotor_torques[kMotor5]; // Port      outer Top = kMotor5
	Gen_Torque[7] = motor_state->rotor_torques[kMotor4]; // Port      outer Bot = kMotor4

	// Rotor Accelerations

	Rotor_Accel[0] = motor_state->rotor_accel[kMotor7];		// (-)
	Rotor_Accel[1] = motor_state->rotor_accel[kMotor2];		// (+)
	Rotor_Accel[2] = motor_state->rotor_accel[kMotor8];		// (+)
	Rotor_Accel[3] = motor_state->rotor_accel[kMotor1];		// (-)
	Rotor_Accel[4] = motor_state->rotor_accel[kMotor6];		// (-)
	Rotor_Accel[5] = motor_state->rotor_accel[kMotor3];		// (+)
	Rotor_Accel[6] = motor_state->rotor_accel[kMotor5];		// (+)
	Rotor_Accel[7] = motor_state->rotor_accel[kMotor4];		// (-)

	// Rotor Speeds
	Rotor_Speed[0] = motor_state->rotor_omegas[kMotor7];
	Rotor_Speed[1] = motor_state->rotor_omegas[kMotor2];
	Rotor_Speed[2] = motor_state->rotor_omegas[kMotor8];
	Rotor_Speed[3] = motor_state->rotor_omegas[kMotor1];
	Rotor_Speed[4] = motor_state->rotor_omegas[kMotor6];
	Rotor_Speed[5] = motor_state->rotor_omegas[kMotor3];
	Rotor_Speed[6] = motor_state->rotor_omegas[kMotor5];
	Rotor_Speed[7] = motor_state->rotor_omegas[kMotor4];

	#if DEBUG
		printf("  kFlapA_c (kfas frame) = [%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f] \n",
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
		printf("  Gen_Torque (kfast frame) = [%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f] \n",
					Gen_Torque[0],
					Gen_Torque[1],
					Gen_Torque[2],
					Gen_Torque[3],
					Gen_Torque[4],
					Gen_Torque[5],
					Gen_Torque[6],
					Gen_Torque[7]);
		printf("  Rotor_Speed (kfast frame) = [%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f] \n",
					Rotor_Speed[0],
					Rotor_Speed[1],
					Rotor_Speed[2],
					Rotor_Speed[3],
					Rotor_Speed[4],
					Rotor_Speed[5],
					Rotor_Speed[6],
					Rotor_Speed[7]);
	#endif
}
