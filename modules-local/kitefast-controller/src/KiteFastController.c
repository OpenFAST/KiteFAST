
#include "KiteFastController.h"
#include "control/kfc.h"

void kfc_dll_init(int *errStat, char *errMsg)
{
    printf("  debug marker - pre controller_init() \n");
    controller_init(errStat, errMsg);
    printf("  debug marker - post controller_init() \n");
}

void kfc_dll_end(int *errStat, char *errMsg)
{
    printf("  debug marker - pre controller_end() \n");
    controller_end(errStat, errMsg);
    printf("  debug marker - post controller_end() \n");
}

void kfc_dll_step(double dcm_g2b_c[], double pqr_c[], double *acc_norm_c,
                  double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
                  double Ab_c[], double *rho_c, double apparent_wind_c[],
                  double tether_force_c[], double wind_g_c[],
                  double kFlapA_c[], double Motor_c[],
                  int *errStat, char *errMsg)
{
    printf("  debug marker - pre controller_step() \n");
    controller_step(dcm_g2b_c, pqr_c, acc_norm_c,
                 Xg_c, Vg_c, Vb_c, Ag_c,
                 Ab_c, rho_c, apparent_wind_c,
                 tether_force_c, wind_g_c,
                 kFlapA_c, Motor_c,
                 errStat, errMsg);
    printf("  debug marker - post controller_step() \n");
}
