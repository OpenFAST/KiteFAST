
#include "KiteFastController.h"
#include "control/kfc.h"

// KiteFastController.c
// Intermediary wrapper between Kitefast and Shared Library
// the fortran to C protocol will be looking explicitly for the functions in this file "kfc_dll_init", "kfc_dll_end", "kfc_dll_step"
// This script simply passes the information to the shared library.
// The callable functions of the shared library are controller_init, controller_step, and controller_end, which are found in control/kfc.c

// kfc_dll_init - passes information to controller_init
void kfc_dll_init(double dT, int numFlaps, int numPylons, 
                  double* I_rot, double* genTorq, double* rotorSpeed,
                  double* rotorAccel, double* rotorBlPit, 
                  double* ctrlSettings,
                  int *errStat, char *errMsg) 
{
    // Init call to Shared Library
    controller_init(dT, numFlaps, numPylons, genTorq, 
                  rotorSpeed, rotorAccel, rotorBlPit, 
                  ctrlSettings, I_rot, errStat, errMsg); 
}

// kfc_dll_end - passes information to controller_end
void kfc_dll_end(int *errStat, char *errMsg)
{
    // End call to Shared Library
    controller_end(errStat, errMsg); 
}

// kfc_dll_step - passes information to controller_step
void kfc_dll_step(double t_c, double dcm_g2b_c[], double pqr_c[], double *acc_norm_c,
                  double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
                  double Ab_c[], double *rho_c, double apparent_wind_c[],
                  double tether_force_c[], double wind_g_c[], double AeroTorque[],
                  double GenTorque[], double RotorSpeed[], 
                  double RotorAccel[], double RotorBladePitch[],
                  double CtrlSettings[],
                  int *errStat, char *errMsg)
{
    // Step call to Shared Library
    controller_step(t_c, dcm_g2b_c, pqr_c, acc_norm_c, 
                 Xg_c, Vg_c, Vb_c, Ag_c,
                 Ab_c, rho_c, apparent_wind_c,
                 tether_force_c, wind_g_c,
                 CtrlSettings, GenTorque,
                 RotorBladePitch, RotorAccel, 
                 RotorSpeed, AeroTorque,
                 errStat, errMsg);

}
