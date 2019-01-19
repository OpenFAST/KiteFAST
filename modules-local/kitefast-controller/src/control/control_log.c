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
    fprintf(fp, "dcm0_0, dcm0_1, dcm0_2, dcm1_0, dcm1_1, dcm1_2, dcm2_0, dcm2_1, dcm2_2, ");
    fprintf(fp, "acc_norm_f, pqr_f.x, pqr_f.y, pqr_f.z, Xg.x, Xg.y, Xg.z, Vg.x, Vg.y, Vg.z, Vb.x, Vb.y, Vb.z, Ag.x, Ag.y, Ag.z, ");
    fprintf(fp, "Ab_f.x, Ab_f.y, Ab_f.z, rho, sph_f.v, sph_f.alpha, sph_f.beta, tether_force_b.vector_f.x, tether_force_b.vector_f.y, tether_force_b.vector_f.z, ");
    fprintf(fp, "wind_g.vector.x, wind_g.vector.y, wind_g.vector.z, flaps[0], flaps[1], flaps[2], flaps[3], flaps[4], flaps[5], flaps[6], flaps[7], ");
    fprintf(fp, "rotors[0], rotors[1], rotors[2], rotors[3], rotors[4], rotors[5], rotors[6], rotors[7]");  
    fprintf(fp, "\n");
	fclose(fp);
    printf("   Control Logging Initiated: File saved as - controller_save_data.txt \n");
}

void ControlLogEntry(ControlLog* control_log){
    //assemble data for save file
    printf("   Control Logging : Saving Step data\n");
    char assembledStr[1024];
    char tempStr[80];
    // dcm 
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,",
        control_log->stateEstLog.dcm_g2b.d[0][0],
        control_log->stateEstLog.dcm_g2b.d[1][0],
        control_log->stateEstLog.dcm_g2b.d[2][0],
        control_log->stateEstLog.dcm_g2b.d[0][1],
        control_log->stateEstLog.dcm_g2b.d[1][1],
        control_log->stateEstLog.dcm_g2b.d[2][1],
        control_log->stateEstLog.dcm_g2b.d[0][2],
        control_log->stateEstLog.dcm_g2b.d[1][2],
        control_log->stateEstLog.dcm_g2b.d[2][2]);
    strcpy( assembledStr, tempStr);
    // pqr
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,",
        control_log->stateEstLog.pqr_f.x,
        control_log->stateEstLog.pqr_f.y,
        control_log->stateEstLog.pqr_f.z);
    strcat( assembledStr, tempStr);
    // acc_norm_c
    sprintf(tempStr, "%0.4f,", control_log->stateEstLog.acc_norm_f);
    strcat( assembledStr, tempStr);
    // Xg_c
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,",
        control_log->stateEstLog.Xg.x,        
        control_log->stateEstLog.Xg.y,
        control_log->stateEstLog.Xg.z);
    strcat( assembledStr, tempStr);
    // Vg_c
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,",
        control_log->stateEstLog.Vg.x,        
        control_log->stateEstLog.Vg.y,
        control_log->stateEstLog.Vg.z);
    strcat( assembledStr, tempStr);
    // Vb_c
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,",
        control_log->stateEstLog.Vb.x,        
        control_log->stateEstLog.Vb.y,
        control_log->stateEstLog.Vb.z);
    strcat( assembledStr, tempStr);
    // Ag_c
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,",
        control_log->stateEstLog.Ag.x,        
        control_log->stateEstLog.Ag.y,
        control_log->stateEstLog.Ag.z);
    strcat( assembledStr, tempStr);
    // Ab_c
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,",
        control_log->stateEstLog.Ab_f.x,        
        control_log->stateEstLog.Ab_f.y,
        control_log->stateEstLog.Ab_f.z);
    strcat( assembledStr, tempStr);
	// rho
    sprintf(tempStr, "%0.4f,", control_log->stateEstLog.rho);
    strcat( assembledStr, tempStr); 
	//apparent_wind_c_v
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,",
        control_log->stateEstLog.apparent_wind.sph_f.v,        
        control_log->stateEstLog.apparent_wind.sph_f.alpha,
        control_log->stateEstLog.apparent_wind.sph_f.beta);
    strcat( assembledStr, tempStr);
	//tether_force_c
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,",
        control_log->stateEstLog.tether_force_b.vector_f.x,        
        control_log->stateEstLog.tether_force_b.vector_f.y,
        control_log->stateEstLog.tether_force_b.vector_f.z);
    strcat( assembledStr, tempStr);
	//wind_g
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,",
        control_log->stateEstLog.wind_g.vector.x,        
        control_log->stateEstLog.wind_g.vector.y,
        control_log->stateEstLog.wind_g.vector.z);
    strcat( assembledStr, tempStr);   
    // kFlaps[]
    printf("   Control Logging : saving flaps\n");
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,",
        control_log->controlOutputLog.flaps[kFlapA1],    
        control_log->controlOutputLog.flaps[kFlapA2],
        control_log->controlOutputLog.flaps[kFlapA4],
        control_log->controlOutputLog.flaps[kFlapA5],
        control_log->controlOutputLog.flaps[kFlapA7],
        control_log->controlOutputLog.flaps[kFlapA8],
        control_log->controlOutputLog.flaps[kFlapEle],
        control_log->controlOutputLog.flaps[kFlapRud]);
    strcat( assembledStr, tempStr);
    printf("   Control Logging : saving rotors\n");
    // rotors 
    sprintf(tempStr, "%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,",
        control_log->controlOutputLog.rotors[kMotor1],    
        control_log->controlOutputLog.rotors[kMotor2],
        control_log->controlOutputLog.rotors[kMotor3],
        control_log->controlOutputLog.rotors[kMotor4],
        control_log->controlOutputLog.rotors[kMotor5],
        control_log->controlOutputLog.rotors[kMotor6],
        control_log->controlOutputLog.rotors[kMotor7],
        control_log->controlOutputLog.rotors[kMotor8]);
    strcat( assembledStr, tempStr); 
    
    FILE * fp;
	fp = fopen("controller_save_data.csv", "a+");
	fprintf(fp, "%s \n", assembledStr);
	fclose(fp);

    printf("   Control Step Saved: File saved as - controller_save_data.csv \n");
}