#include "control/control_log.h"
#include <time.h>

void ControlLogInit(char* controllerVerNumber){
	//Init Data SavestateEstLog
    char timeStr[100];
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    strftime(timeStr, sizeof(timeStr)-1, "%d/%m/%Y %H:%M \n", t);

	FILE * fp;
    fp = fopen("controller_save_data.csv", "w+");
    fprintf(fp, "Controller Output Text File \nFile Created on: %s",timeStr);
    fprintf(fp, "controller version: %s \n", controllerVerNumber);
    fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,","time", "dcm0_0", "dcm0_1", "dcm0_2", "dcm1_0", "dcm1_1", "dcm1_2", "dcm2_0", "dcm2_1", "dcm2_2"); // 10
    fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,","acc_norm_f", "pqr_f.x", "pqr_f.y", "pqr_f.z", "Xg.x", "Xg.y", "Xg.z", "Vg.x", "Vg.y", "Vg.z"); // 20
	fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,","Vb.x", "Vb.y", "Vb.z", "Ag.x", "Ag.y", "Ag.z", "Ab_f.x", "Ab_f.y", "Ab_f.z", "rho"); // 30
    fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,", "wind_g.vector.x", "wind_g.vector.y", "wind_g.vector.z", "app_wnd_f.v", "app_wnd_f.alpha", "app_wnd_f.beta", "teth_vec_b.x", "teth_vec_b.y", "teth_vec_b.z"); // 39
    fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,","flaps[0]", "flaps[1]", "flaps[2]", "flaps[3]", "flaps[4]", "flaps[5]", "flaps[6]", "flaps[7]"); // 47
    fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,","rotors[0]", "rotors[1]", "rotors[2]", "rotors[3]", "rotors[4]", "rotors[5]", "rotors[6]", "rotors[7]"); // 55
    fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,","r_torque[0]", "r_torque[1]", "r_torque[2]", "r_torque[3]", "r_torque[4]", "r_torque[5]", "r_torque[6]", "r_torque[7]"); 
	fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,","aerotorque[0]", "aerotorque[1]", "aerotorque[2]", "aerotorque[3]", "aerotorque[4]", "aerotorque[5]", "aerotorque[6]", "aerotorque[7]");
	fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,","loop_angle", "path_center_g_x", "path_center_g_y", "path_center_g_z", "tether_roll_post", "tether_roll_pre", "tether_pit", "speed_f_playbook", "wind_g_dir_f");   // 64
	fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,","path_rad_play_out", "airspd_cmd_power_out", "d_airspd_loopang_pwr_out", "alpha_nom_pwr_out", "beta_nom_pwr_out", "k_aero_cmd_pth_out", "pqr_cmd_newcrs_pqr_out_x", "pqr_cmd_newcrs_pqr_out_y", "pqr_cmd_newcrs_pqr_out_z", "kite_accel_ff_loop_kin_out"); //74
    fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,","k_geom_cmd_pth_out", "k_aero_cur_pth_out", "k_geom_cur_pth_out", "pqr_cmd_crv_out_x", "pqr_cmd_crv_out_y", "pqr_cmd_crv_out_z", "flring_crv_out", "alpha_cmd_crv_out", "beta_cmd_crv_out", "tet_roll_cmd_crv_out"); // 84
	fprintf(fp, "%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s,%24s","dCL_cmd_curv_out", "aileron", "inboard_flap", "midboard_flap", "outboard_flap", "elevator", "rudder", "thrust", "moment_x", "moment_y", "moment_z"); // 85
	fprintf(fp, "\n");
	fclose(fp);
#ifdef DEBUG
    printf("   Control Logging Initiated: File saved as - controller_save_data.txt \n");
#endif
}

void ControlLogEntry(ControlLog* control_log){
    //assemble data for save file
#ifdef DEBUG
    printf("   Control Logging : Saving Step data\n");
#endif


    FILE *fp;
    fp = fopen("controller_save_data.csv", "a+");
    fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,",	//10	// time, dcm0_0, dcm0_1, dcm0_2, dcm1_0, dcm1_1, dcm1_2, dcm2_0, dcm2_1, dcm2_2, 
		control_log->time,
		control_log->stateEstLogPreStep.dcm_g2b.d[0][0],
        control_log->stateEstLogPreStep.dcm_g2b.d[0][1],
        control_log->stateEstLogPreStep.dcm_g2b.d[0][2],
        control_log->stateEstLogPreStep.dcm_g2b.d[1][0],
        control_log->stateEstLogPreStep.dcm_g2b.d[1][1],
        control_log->stateEstLogPreStep.dcm_g2b.d[1][2],
        control_log->stateEstLogPreStep.dcm_g2b.d[2][0],
        control_log->stateEstLogPreStep.dcm_g2b.d[2][1],
        control_log->stateEstLogPreStep.dcm_g2b.d[2][2]);
	fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,",	//20	// acc_norm_f, pqr_f.x, pqr_f.y, pqr_f.z, Xg.x, Xg.y, Xg.z, Vg.x, Vg.y, Vg.z, 
		control_log->stateEstLogPreStep.acc_norm_f,
		control_log->stateEstLogPreStep.pqr_f.x,
        control_log->stateEstLogPreStep.pqr_f.y,
        control_log->stateEstLogPreStep.pqr_f.z,
		control_log->stateEstLogPreStep.Xg.x,
		control_log->stateEstLogPreStep.Xg.y,
		control_log->stateEstLogPreStep.Xg.z,
		control_log->stateEstLogPreStep.Vg.x,
		control_log->stateEstLogPreStep.Vg.y,
		control_log->stateEstLogPreStep.Vg.z);
	fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,",	//30 // "Vb.x, Vb.y, Vb.z, Ag.x, Ag.y, Ag.z, Ab_f.x, Ab_f.y, Ab_f.z, rho,"); 
		control_log->stateEstLogPreStep.Vb.x,
		control_log->stateEstLogPreStep.Vb.y,
		control_log->stateEstLogPreStep.Vb.z,
		control_log->stateEstLogPreStep.Ag.x,
		control_log->stateEstLogPreStep.Ag.y,
		control_log->stateEstLogPreStep.Ag.z,
		control_log->stateEstLogPreStep.Ab_f.x,
		control_log->stateEstLogPreStep.Ab_f.y,
		control_log->stateEstLogPreStep.Ab_f.z,
		control_log->stateEstLogPreStep.rho);
	fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,", //39 //wind_g.vector.x, wind_g.vector.y, wind_g.vector.z, sph_f.v, sph_f.alpha, sph_f.beta, tether_force_b.vector_f.x, tether_force_b.vector_f.y, tether_force_b.vector_f.z, "	
		control_log->stateEstLogPreStep.wind_g.vector.x,
		control_log->stateEstLogPreStep.wind_g.vector.y,
		control_log->stateEstLogPreStep.wind_g.vector.z,
		control_log->stateEstLogPreStep.apparent_wind.sph_f.v,    
		control_log->stateEstLogPreStep.apparent_wind.sph_f.alpha,
		control_log->stateEstLogPreStep.apparent_wind.sph_f.beta,
		control_log->stateEstLogPreStep.tether_force_b.vector_f.x,
		control_log->stateEstLogPreStep.tether_force_b.vector_f.y,
		control_log->stateEstLogPreStep.tether_force_b.vector_f.z);
	fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,",	//47  // "flaps[0], flaps[1], flaps[2], flaps[3], flaps[4], flaps[5], flaps[6], flaps[7], ")
		control_log->controlOutputLog.flaps[kFlapA1],
		control_log->controlOutputLog.flaps[kFlapA2],
		control_log->controlOutputLog.flaps[kFlapA4],
		control_log->controlOutputLog.flaps[kFlapA5],
		control_log->controlOutputLog.flaps[kFlapA7],
		control_log->controlOutputLog.flaps[kFlapA8],
		control_log->controlOutputLog.flaps[kFlapEle],
		control_log->controlOutputLog.flaps[kFlapRud]);
	fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,",	//55	//"rotors[0], rotors[1], rotors[2], rotors[3], rotors[4], rotors[5], rotors[6], rotors[7], "); /
		control_log->motor_state.rotor_omegas[kMotor1],
		control_log->motor_state.rotor_omegas[kMotor2],
		control_log->motor_state.rotor_omegas[kMotor3],
		control_log->motor_state.rotor_omegas[kMotor4],
		control_log->motor_state.rotor_omegas[kMotor5],
		control_log->motor_state.rotor_omegas[kMotor6],
		control_log->motor_state.rotor_omegas[kMotor7],
		control_log->motor_state.rotor_omegas[kMotor8]);
	fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,",	//55	//"r_torque[0], r_torque[1], r_torque[2], r_torque[3], r_torque[4], r_torque[5], r_torque[6], r_torque[7], "); /
		control_log->motor_state.rotor_torques[kMotor1],
		control_log->motor_state.rotor_torques[kMotor2],
		control_log->motor_state.rotor_torques[kMotor3],
		control_log->motor_state.rotor_torques[kMotor4],
		control_log->motor_state.rotor_torques[kMotor5],
		control_log->motor_state.rotor_torques[kMotor6],
		control_log->motor_state.rotor_torques[kMotor7],
		control_log->motor_state.rotor_torques[kMotor8]);
	fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,",	//55	//"aerotorque[0], aerotorque[1], aerotorque[2], aerotorque[3], aerotorque[4], aerotorque[5], aerotorque[6], aerotorque[7], "); /
		control_log->motor_state.aero_torque[kMotor1],
		control_log->motor_state.aero_torque[kMotor2],
		control_log->motor_state.aero_torque[kMotor3],
		control_log->motor_state.aero_torque[kMotor4],
		control_log->motor_state.aero_torque[kMotor5],
		control_log->motor_state.aero_torque[kMotor6],
		control_log->motor_state.aero_torque[kMotor7],
		control_log->motor_state.aero_torque[kMotor8]);
	fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,",//64	//"loop_angle, path_center_g_x, path_center_g_y, path_center_g_z, tether_roll_post, tether_roll_pre, tether_pit, speed_f_playbook, wind_g.dir_f"// 	
		control_log->controlOutputLog.loop_angle,
		control_log->controlOutputLog.path_center_g.x,
		control_log->controlOutputLog.path_center_g.y,
		control_log->controlOutputLog.path_center_g.z,
		control_log->stateEstLogPostStep.tether_force_b.sph.roll,
		control_log->stateEstLogPreStep.tether_force_b.sph.roll,
		control_log->stateEstLogPostStep.tether_force_b.sph.pitch,
		control_log->stateEstLogPreStep.wind_aloft_g.speed_f_playbook,
		control_log->stateEstLogPreStep.wind_g.dir_f);
	fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,",//74 	//"path_radius_playbook_out, airspeed_cmd_power_out, d_airspeed_d_loopangle_power_out, alpha_nom_power_out, beta_nom_power_out, k_aero_cmd_path_out, pqr_cmd_new_cross_pqr_out_x, pqr_cmd_new_cross_pqr_out_y, pqr_cmd_new_cross_pqr_out_z, kite_accel_ff_loop_kin_out"
		control_log->controlOutputLog.path_radius_playbook_out,
  		control_log->controlOutputLog.airspeed_cmd_power_out,
  		control_log->controlOutputLog.d_airspeed_d_loopangle_power_out,
  		control_log->controlOutputLog.alpha_nom_power_out,
  		control_log->controlOutputLog.beta_nom_power_out,
  		control_log->controlOutputLog.k_aero_cmd_path_out,
		control_log->controlOutputLog.pqr_cmd_new_cross_pqr_out.x,
		control_log->controlOutputLog.pqr_cmd_new_cross_pqr_out.y,
		control_log->controlOutputLog.pqr_cmd_new_cross_pqr_out.z,
		control_log->controlOutputLog.kite_accel_ff_loop_kin_out);
	fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%d,%24.6f,%24.6f,%24.6f,",//84	//"k_geom_cmd_path_out, k_aero_curr_path_out, k_geom_curr_path_out, pqr_cmd_curv_out_x, pqr_cmd_curv_out_y, pqr_cmd_curv_out_z, flaring_curv_out, alpha_cmd_curv_out, beta_cmd_curv_out, tether_roll_cmd_curv_out"// 	
  		control_log->controlOutputLog.k_geom_cmd_path_out,
  		control_log->controlOutputLog.k_aero_curr_path_out,
  		control_log->controlOutputLog.k_geom_curr_path_out,
  		control_log->controlOutputLog.pqr_cmd_curv_out.x,
		control_log->controlOutputLog.pqr_cmd_curv_out.y,
		control_log->controlOutputLog.pqr_cmd_curv_out.z,
  		control_log->controlOutputLog.flaring_curv_out,
  		control_log->controlOutputLog.alpha_cmd_curv_out,
  		control_log->controlOutputLog.beta_cmd_curv_out,
  		control_log->controlOutputLog.tether_roll_cmd_curv_out);
	fprintf(fp, "%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,%24.6f,",    //"//85 dCL_cmd_curv_out"//
  		control_log->controlOutputLog.dCL_cmd_curv_out,
		control_log->controlOutputLog.delta_inner_out.aileron,
		control_log->controlOutputLog.delta_inner_out.inboard_flap,
		control_log->controlOutputLog.delta_inner_out.midboard_flap,
		control_log->controlOutputLog.delta_inner_out.outboard_flap,
		control_log->controlOutputLog.delta_inner_out.elevator,
		control_log->controlOutputLog.delta_inner_out.rudder,
        control_log->controlOutputLog.thrust_moment_inner_out.thrust,
        control_log->controlOutputLog.thrust_moment_inner_out.moment.x,
        control_log->controlOutputLog.thrust_moment_inner_out.moment.y,
        control_log->controlOutputLog.thrust_moment_inner_out.moment.z);
    fprintf(fp, "\n");
    fclose(fp);
#ifdef DEBUG
    printf("   Control Step Saved: File saved as - controller_save_data.csv \n");
#endif
}
