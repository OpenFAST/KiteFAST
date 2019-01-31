
#include "KiteFastController.h"
#include "control/kfc.h"

// KiteFastController.c
// Intermediary wrapper between Kitefast and Shared Library

void kfc_dll_init(double *dT, int *numFlaps, int *numPylons, 
                  double genTorq[], double rotorSpeed[],
                  double rotorAccel[], double rotorBlPit[], 
                  double ctrlSettins[], double i_rot[],
                  int *errStat, char *errMsg) 
{
    #ifdef DEBUG //DEBUG preproc found in kfc.h 
        printf("  debug marker - pre controller_init() \n");
    #endif

    // Init call to Shared Library
    controller_init(dT, numFlaps, numPylons, genTorq, 
                  rotorSpeed, rotorAccel, rotorBlPit, 
                  ctrlSettins, i_rot, errStat, errMsg); 
    
    #ifdef DEBUG //DEBUG preproc found in kfc.h
        printf("  debug marker - post controller_init() \n");
    #endif
}

void kfc_dll_end(int *errStat, char *errMsg)
{
    #ifdef DEBUG //DEBUG preproc found in kfc.h
        printf("  debug marker - pre controller_end() \n");
    #endif

    // End call to Shared Library
    controller_end(errStat, errMsg); 

    #ifdef DEBUG //DEBUG preproc found in kfc.h
        printf("  debug marker - post controller_end() \n");
    #endif
}

void kfc_dll_step(double dcm_g2b_c[], double pqr_c[], double *acc_norm_c,
                  double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
                  double Ab_c[], double *rho_c, double apparent_wind_c[],
                  double tether_force_c[], double wind_g_c[],
                  double AeroTorque[], double GenTorque[],
                  double RotorSpeed[], double RotorAccel[],
                  double RotorBladePitch[], double kFlapA_c[],
                  double Motor_c[],
                  int *errStat, char *errMsg)
{
    #ifdef DEBUG //DEBUG preproc found in kfc.h
        printf("  debug marker - pre controller_step() \n");
    #endif

    // Step call to Shared Library
    controller_step(dcm_g2b_c, pqr_c, acc_norm_c, 
                 Xg_c, Vg_c, Vb_c, Ag_c,
                 Ab_c, rho_c, apparent_wind_c,
                 tether_force_c, wind_g_c,
                 kFlapA_c, Motor_c, GenTorque,
                 RotorBladePitch, RotorAccel, 
                 RotorSpeed, AeroTorque,
                 errStat, errMsg);

    #ifdef DEBUG //DEBUG preproc found in kfc.h
        printf("  debug marker - post controller_step() \n");
    #endif
}
