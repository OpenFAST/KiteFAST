#include <stdio.h>
#include "KiteFastController.h"
 
 int main(int argc, char *argv[])
 {
    int errStat = 1;
    char errMsg = 'l';
    double dT = 0.01;
    int numFlaps = 8;
    int numPylons = 2;
    double dcm_g2b_c[9] = {0.7213,0.0803,0.6880,-0.0426,0.9965,-0.0717,-0.6913,0.0224,0.722}; //first step of crosswind
    double pqr_c[3]= {-0.0017,-0.0243,-0.0604};
    double acc_norm_c = 9.8855;
    double Xg_c[3]= {-269.0570,6.5145,-323.9749};
    double Vg_c[3] = {18.1094,-2.6402,-22.5261};
    double Vb_c[3] = {28.7473,-1.6821,-3.6202};
    double Ag_c[3] = {-0.2503,-2.3947,0.2594};
    double Ab_c[3] = {-0.4016,-0.6470,-1.0060};
    double rho_c = 1.0747;
    double apparent_wind_c[3]= {34.6587,-0.0388,-0.0254};
    double tether_force_c[3]= {-4783.5,95.4776,8838.0};
    double wing_g_c[3]= {-5.8650, -0.1156, 2.2478};
    double genTorq[8] = {1,2,3,4,5,6,7,8};
    double rotorSpeed   [8]={1,2,3,4,5,6,7,8};
    double rotorAccel   [8]={1,2,3,4,5,6,7,8};
    double rotorBlPit   [8]={1,2,3,4,5,6,7,8};
    double ctrlSettins  [8]={1,2,3,4,5,6,7,8};
    double i_rot        [8]={1,2,3,4,5,6,7,8};
    double kFlapA_c[3]= {0,0,0};
    double Motor_c[8]= {0,0,0,0,0,0,0,0};
    double AeroTorque[8] = {0,0,0,0,0,0,0,0};

    kfc_dll_init(&dT, &numFlaps, &numPylons, 
                  genTorq, rotorSpeed,
                  rotorAccel, rotorBlPit, 
                  ctrlSettins, i_rot,
                  &errStat, &errMsg); 
    // kfc_dll_init(&errStat, &errMsg);
    kfc_dll_end(&errStat, &errMsg);

    kfc_dll_step(dcm_g2b_c, pqr_c, &acc_norm_c,
        Xg_c, Vg_c, Vb_c, Ag_c,
        Ab_c, &rho_c, apparent_wind_c,  
        tether_force_c, wing_g_c, AeroTorque, genTorq,
        rotorSpeed, rotorAccel, rotorBlPit,
        kFlapA_c, Motor_c,
        &errStat, &errMsg);

    return 0;
 }