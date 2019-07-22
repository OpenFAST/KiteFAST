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
    double dcm_g2b_c[9] = {-0.8610, 0.2591, 0.4334, 0.0989, -0.7549, 0.6483, 0.4952, 0.6025, 0.6260}; //first step of crosswind
    double pqr_c[3]= {-0.0120, 0.166, -0.4127};  
    double acc_norm_c = 28.7601;
    double Xg_c[3]= {-122.4417, -380.4510, -174.0961};
    double Vg_c[3] = {-53.8786, 3.2314, 29.6990};
    double Vb_c[3] = {61.5307, 1.4932, -2.6675};
    double Ag_c[3] = {5.6324, -17.8906 , 12.6850};
    double Ab_c[3] = {-0.1684, 22.5354, -0.9354};
    double rho_c = 1.0747;
    double apparent_wind_c[3]= {-54.2207, 2.2447, -4.4018};
    double tether_force_c[3]= {4158.6, -48016, 129764.69};
    double wing_g_c[3]= {-7.6604, -6.4279, 1.3732};
    double genTorq[8]       ={0,0,0,0,0,0,0,0};
    double rotorSpeed[8]    ={0,0,0,0,0,0,0,0};
    double rotorAccel[8]    ={0,0,0,0,0,0,0,0};
    double rotorBlPit[8]    ={0,0,0,0,0,0,0,0};
    double ctrlSettings[12]  ={0,0,0,0,0,0,0,0,0,0,0,0};
    double i_rot[8]={0,0,0,0,0,0,0,0};
    double AeroTorque[8] = {0,0,0,0,0,0,0,0};

    kfc_dll_init(dT, numFlaps, numPylons, 
        i_rot, genTorq, rotorSpeed,
        rotorAccel, rotorBlPit, 
        ctrlSettings,
        &errStat, &errMsg); 

    kfc_dll_step(t_c, dcm_g2b_c, pqr_c, &acc_norm_c,
        Xg_c, Vg_c, Vb_c, Ag_c,
        Ab_c, &rho_c, apparent_wind_c,  
        tether_force_c, wing_g_c, AeroTorque, genTorq,
        rotorSpeed, rotorAccel, rotorBlPit, ctrlSettings,
        &errStat, &errMsg);

    kfc_dll_end(&errStat, &errMsg); 
    return 0;
 }

 