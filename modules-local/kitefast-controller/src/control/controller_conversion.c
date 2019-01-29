#include "control/controller_conversion.h"

#include <stdio.h>
#include <math.h>
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/ground_station_frame.h"


void AssignInputs(double dcm_g2b_c[], double pqr_c[], double *acc_norm_c,
	double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
	double Ab_c[], double *rho_c, double apparent_wind_c[],
	double tether_force_c[], double wind_g_c[],
	double kFlapA_c[], double Motor_c[], double ext_torques[],
	int *errStat, char *errMsg, StateEstimate *state_est, MotorState *motor_state){

	#if DEBUG
		printf("  dcm_g2b_c = [%0.4f, %0.4f, %0.4f],[%0.4f, %0.4f, %0.4f],[%0.4f, %0.4f, %0.4f] \n",
					dcm_g2b_c[0], dcm_g2b_c[1], dcm_g2b_c[2], dcm_g2b_c[3], dcm_g2b_c[4], dcm_g2b_c[5], dcm_g2b_c[6], dcm_g2b_c[7], dcm_g2b_c[8]);
		printf("  pqr_c = [%0.4f, %0.4f, %0.4f] \n",pqr_c[0],pqr_c[1],pqr_c[2]);
		printf("  acc_norm_c = %0.4f \n", acc_norm_c);
		printf("  Xg_c = [%0.4f, %0.4f, %0.4f] \n",Xg_c[0],Xg_c[1],Xg_c[2]);
		printf("  Vg_c = [%0.4f, %0.4f, %0.4f] \n",Vg_c[0],Vg_c[1],Vg_c[2]);
		printf("  Vb_c = [%0.4f, %0.4f, %0.4f] \n",Vb_c[0],Vb_c[1],Vb_c[2]);
		printf("  Ag_c = [%0.4f, %0.4f, %0.4f] \n",Ag_c[0],Ag_c[1],Ag_c[2]);
		printf("  Ab_c = [%0.4f, %0.4f, %0.4f] \n",Ab_c[0],Ab_c[1],Ab_c[2]);
		printf("  rho_c = %0.4f \n", rho_c);
		printf("  apparent_wind_c = [%0.4f, %0.4f, %0.4f] \n",apparent_wind_c[0],apparent_wind_c[1],apparent_wind_c[2]);
		printf("  tether_force_c = [%0.4f, %0.4f, %0.4f] \n",tether_force_c[0],tether_force_c[1],tether_force_c[2]);
		printf("  wind_g_c = [%0.4f, %0.4f, %0.4f] \n",wind_g_c[0],wind_g_c[1],wind_g_c[2]);
	#endif

	//dcm_2gb - convert and copy value into state_est->dcm_g2b
	Mat3 dcm_g2b_tmp = { { { dcm_g2b_c[0], dcm_g2b_c[1], dcm_g2b_c[2] },
						   { dcm_g2b_c[3], dcm_g2b_c[4], dcm_g2b_c[5] },
						   { dcm_g2b_c[6], dcm_g2b_c[7], dcm_g2b_c[8] } } };;
	memcpy(&state_est->dcm_g2b, &dcm_g2b_tmp, sizeof(state_est->dcm_g2b));
	
	//pqr_c - convert and copy value into state_est->pqr_c
	Vec3 pqr_c_tmp = { pqr_c[0], pqr_c[1], pqr_c[2] };
	memcpy(&state_est->pqr_f, &pqr_c_tmp, sizeof(state_est->pqr_f));

	//acc_norm_c - convert and copy value into state_est->acc_norm_f
	double acc_norm_f_tmp = *acc_norm_c;
	memcpy(&state_est->acc_norm_f, &acc_norm_f_tmp, sizeof(state_est->acc_norm_f));

	// Xg - convert and copy value into state_est->Xg
	Vec3 Xg_tmp = { Xg_c[0] , Xg_c[1] , Xg_c[2] };
	memcpy(&state_est->Xg, &Xg_tmp, sizeof(state_est->Xg));

	// Vg - convert and copy value into state_est->Vg
	Vec3 Vg_tmp = { Vg_c[0] , Vg_c[1] , Vg_c[2] };
	memcpy(&state_est->Vg, &Vg_tmp, sizeof(state_est->Vg));

	// Vb - convert and copy value into state_est->Vb
	Vec3 Vb_tmp = { Vb_c[0] , Vb_c[1] , Vb_c[2] };
	memcpy(&state_est->Vb, &Vb_tmp, sizeof(state_est->Vb));

	// Ag - convert and copy value into state_est->Ag
	Vec3 Ag_tmp = { Ag_c[0] , Ag_c[1] , Ag_c[2] };
	memcpy(&state_est->Ag, &Ag_tmp, sizeof(state_est->Ag));

	// Ab_f - convert and copy value into state_est->Ab_f
	Vec3 Ab_tmp = { Ab_c[0] , Ab_c[1] , Ab_c[2] };
	memcpy(&state_est->Ab_f, &Ab_tmp, sizeof(state_est->Ab_f));

	// rho - convert and copy value into state_est->rho
	double rho_tmp = *rho_c;
	memcpy(&state_est->rho, &rho_tmp, sizeof(state_est->rho));

	//apparent_wind_c_v
	if(apparent_wind_c[0] < 0.0){  //added by Justin Miller - STI 
      apparent_wind_c[0] = -apparent_wind_c[0];
    } // TODO - Check reference frames of kitefast vs CSim- Airspeed is coming in (-), assertions fail if airspeed is (-)
	state_est->apparent_wind.sph_f.v = apparent_wind_c[0];
	state_est->apparent_wind.sph_f.alpha = apparent_wind_c[1];
	state_est->apparent_wind.sph_f.beta = apparent_wind_c[2];
	state_est->apparent_wind.solution_type = kApparentWindSolutionTypePitot; // Done to clear alpha/beta fault in GetFlags()
	//tether_force_c
	state_est->tether_force_b.vector_f.x = tether_force_c[0];
	state_est->tether_force_b.vector_f.y = tether_force_c[1];
	state_est->tether_force_b.vector_f.z = tether_force_c[2];
	state_est->tether_force_b.valid = false;
	// assumed tension is magnitude of tether force inputs - Jmiller - STI
	state_est->tether_force_b.sph.tension = sqrt(pow(tether_force_c[0],2) + pow(tether_force_c[1],2) + pow(tether_force_c[2],2));
	//wind_g_c
	state_est->wind_g.vector.x = wind_g_c[0];
	state_est->wind_g.vector.y = wind_g_c[1];
	state_est->wind_g.vector.z = wind_g_c[2];
	state_est->wind_g.dir_f = VecGToAzimuth(&state_est->wind_g.vector);
	// Motor_c
	for (int i = 0; i<kNumMotors; i++){
		motor_state->rotor_omegas[i] = Motor_c[i];
	}
	
}

void AssignOutputs(double kFlapA_c[], double Motor_c[], double Gen_Torque[],
	double Rotor_Accel[], double Rotor_Speed[], double Blade_Pitch[],
	int *errStat, char *errMsg, ControlOutput* raw_control_output, MotorState* motor_state){

	//kFlap_A
	kFlapA_c[0] = raw_control_output->flaps[kFlapA1];
	kFlapA_c[1] = raw_control_output->flaps[kFlapA2];
	kFlapA_c[2] = raw_control_output->flaps[kFlapA4];
	kFlapA_c[3] = raw_control_output->flaps[kFlapA5];
	kFlapA_c[4] = raw_control_output->flaps[kFlapA7];
	kFlapA_c[5] = raw_control_output->flaps[kFlapA8];
	kFlapA_c[6] = raw_control_output->flaps[kFlapEle];
	kFlapA_c[7] = raw_control_output->flaps[kFlapRud];

	//Motor speeds	
	Motor_c[0] = motor_state->motor_speeds[kMotor1];
	Motor_c[1] = motor_state->motor_speeds[kMotor2];
	Motor_c[2] = motor_state->motor_speeds[kMotor3];
	Motor_c[3] = motor_state->motor_speeds[kMotor4];
	Motor_c[4] = motor_state->motor_speeds[kMotor5];
	Motor_c[5] = motor_state->motor_speeds[kMotor6];
	Motor_c[6] = motor_state->motor_speeds[kMotor7];
	Motor_c[7] = motor_state->motor_speeds[kMotor8];

	//Generator Torques
	Gen_Torque[0] = motor_state->rotor_torques[kMotor1];
	Gen_Torque[1] = motor_state->rotor_torques[kMotor2];
	Gen_Torque[2] = motor_state->rotor_torques[kMotor3];
	Gen_Torque[3] = motor_state->rotor_torques[kMotor4];
	Gen_Torque[4] = motor_state->rotor_torques[kMotor5];
	Gen_Torque[5] = motor_state->rotor_torques[kMotor6];
	Gen_Torque[6] = motor_state->rotor_torques[kMotor7];
	Gen_Torque[7] = motor_state->rotor_torques[kMotor8];

	// Rotor Accelerations
	Rotor_Accel[0] = motor_state->rotor_accel[kMotor1];
	Rotor_Accel[1] = motor_state->rotor_accel[kMotor2];
	Rotor_Accel[2] = motor_state->rotor_accel[kMotor3];
	Rotor_Accel[3] = motor_state->rotor_accel[kMotor4];
	Rotor_Accel[4] = motor_state->rotor_accel[kMotor5];
	Rotor_Accel[5] = motor_state->rotor_accel[kMotor6];
	Rotor_Accel[6] = motor_state->rotor_accel[kMotor7];
	Rotor_Accel[7] = motor_state->rotor_accel[kMotor8];

	// Rotor Speeds
	Rotor_Speed[0] = motor_state->rotor_omegas[kMotor1];
	Rotor_Speed[1] = motor_state->rotor_omegas[kMotor2];
	Rotor_Speed[2] = motor_state->rotor_omegas[kMotor3];
	Rotor_Speed[3] = motor_state->rotor_omegas[kMotor4];
	Rotor_Speed[4] = motor_state->rotor_omegas[kMotor5];
	Rotor_Speed[5] = motor_state->rotor_omegas[kMotor6];
	Rotor_Speed[6] = motor_state->rotor_omegas[kMotor7];
	Rotor_Speed[7] = motor_state->rotor_omegas[kMotor8];
		
	#if DEBUG
		printf("  kFlapA_c = [%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f] \n",
					kFlapA_c[0],
					kFlapA_c[1],
					kFlapA_c[2],
					kFlapA_c[3],
					kFlapA_c[4],
					kFlapA_c[5],
					kFlapA_c[6],
					kFlapA_c[7]);
		printf("  Gen_Torque = [%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f] \n",
					Gen_Torque[0],
					Gen_Torque[1],
					Gen_Torque[2],
					Gen_Torque[3],
					Gen_Torque[4],
					Gen_Torque[5],
					Gen_Torque[6],
					Gen_Torque[7]);
	#endif
}
