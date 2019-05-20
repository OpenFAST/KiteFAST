#ifndef __KFC_H__
#define __KFC_H__

#ifdef __cplusplus
extern "C" {
#endif

// No major difference between DEBUG mode and Normal mode
// When DEBUG is enabled, a series of print statements will be enabled.
 #define DEBUG 1 //Set Debug to 0 to take out of debug mode

#include <stdio.h>
void controller_init(double Requested_dT, int numFlaps, int numPylons, double genTorq[], 
                  double rotorSpeed[], double rotorAccel[], double rotorBlPit[], 
                  double ctrlSettins[], double I_rot[], int *errStat, char *errMsg);

void controller_end(int *errStat, char *errMsg);


void controller_step(double t, double dcm_g2b_c[], double pqr_c[], double *acc_norm_c,
					 double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
					 double Ab_c[], double *rho_c, double apparent_wind_c[],
					 double tether_force_c[], double wind_g_c[],
					 double CtrlSettings[], double GenTorque[],
					 double RotorBladePitch[], double RotorAccel[], 
					 double RotorSpeed[], double AeroTorque[],
					 int *errStat, char *errMsg);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif

