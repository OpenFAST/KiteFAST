#include <stdio.h>
#include "KiteFastController.h"
 
 int main(int argc, char *argv[])
 {
    int errStat = 1;
    char errMsg = 'l';
    double dT = 0.01;
    double t_c = dT;
    int numFlaps = 8;
    int numPylons = 2;
    double dcm_g2b_c[9] = {-0.8610, 0.2349, 0.4511,0.1192, -0.7690, 0.6280,0.4944, 0.5945, 0.6341}; //first step of crosswind
    double pqr_c[3]= {0.2524, 0.3759, -0.0398};
    double acc_norm_c = 21.00;
    double Xg_c[3]= {-125.4900, -378.4510, -171.1080};
    double Vg_c[3] = {-53.9010, 3.3290, 29.7070};
    double Vb_c[3] = {61.4940, 2.4386, -3.3843};
    double Ag_c[3] = {-0.2503,-2.3947,0.2594};
    double Ab_c[3] = {-0.4016,-0.6470,-1.0060};
    double rho_c = 1.0750;
    double apparent_wind_c[3]= {46.2406, -9.7569, -29.7070};
    double tether_force_c[3]= {2664.8407, -66441.2279, 157789.4866};
    double wing_g_c[3]= {-7.6604, -6.4279, 0.0000};
    double genTorq[8] = {1,2,3,4,5,6,7,8};
    // double AeroTorque[8] = {1,2,3,4,5,6,7,8};
    double rotorSpeed[8]={1,2,3,4,5,6,7,8};
    double rotorAccel[8]={1,2,3,4,5,6,7,8};
    double rotorBlPit[8]={1,2,3,4,5,6,7,8};
    double ctrlSettins[8]={1,2,3,4,5,6,7,8};
    double i_rot[8]={1,2,3,4,5,6,7,8};
    double CtrlSettings[3]= {0,0,0};
    // double Motor_c[8]= {0,0,0,0,0,0,0,0};
    double AeroTorque[8] = {0,0,0,0,0,0,0,0};

    kfc_dll_init(dT, numFlaps, numPylons, 
                  i_rot, genTorq, rotorSpeed,
                  rotorAccel, rotorBlPit, 
                  ctrlSettins,
                  &errStat, &errMsg); 

    kfc_dll_step(t_c, dcm_g2b_c, pqr_c, &acc_norm_c,
        Xg_c, Vg_c, Vb_c, Ag_c,
        Ab_c, &rho_c, apparent_wind_c,  
        tether_force_c, wing_g_c, AeroTorque, genTorq,
        rotorSpeed, rotorAccel, rotorBlPit,
        CtrlSettings,
        &errStat, &errMsg);

    kfc_dll_end(&errStat, &errMsg); 
    return 0;
 }

 