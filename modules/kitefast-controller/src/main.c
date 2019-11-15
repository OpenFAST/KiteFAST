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
    double dcm_g2b_c[9] = {-0.8641, 0.2557, 0.4335, 0.1019, -0.7546, 0.6482, 0.4929, 0.6043, 0.6260};
    double pqr_c[3]= {-0.011701762580859, -0.010291828869872, -0.390627923692243};  
    double acc_norm_c = 28.735379499680100;
    double Xg_c[3]= {-1.230195825216543e+02, -3.808578802064749e+02, -1.737989235739968e+02};
    double Vg_c[3] = {-53.9007723439951, 3.32946946341383, 29.7069230287192};
    double Vb_c[3] = {61.5573534961638,1.65877222531134, -2.61162923749005};
    double Ag_c[3] = {-4.10429877138525, 18.8476886549997, -10.3879191238667};
    double Ab_c[3] = {0.160085873782321, -22.5059243248619, 0.695634407713604};
    double rho_c = 1.0747;
    double apparent_wind_c[3]= {-54.237362206593140, 2.252487308494521, -4.402210752771378};
    double tether_force_c[3]= {4.436435166866530e+03, -4.843583750096925e+04, 1.312415108306612e+05};
    double wing_g_c[3]= {-7.6604, -6.4279, 0};
    double genTorq[8]       ={0,0,0,0,0,0,0,0};
    double rotorSpeed[8]    ={0,0,0,0,0,0,0,0};
    double rotorAccel[8]    ={0,0,0,0,0,0,0,0};
    double rotorBlPit[8]    ={0,0,0,0,0,0,0,0};
    double ctrlSettings[12]  ={0,0,0,0,0,0,0,0,0,0,0,0};
    double i_rot[8]={0,0,0,0,0,0,0,0};
    double AeroTorque[8] = {-348.7333, 404.6280, 328.2791, -367.3086, -371.5845, 446.1638, 399.3240, -490.7282};

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

 