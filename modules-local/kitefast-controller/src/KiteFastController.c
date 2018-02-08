
#include "KiteFastController.h"

void kfc_dll_init(int *errStat, char *errMsg) {
    *errStat = 1;
    char tmp[] = "this is a message from dll\'s init";
    for (int i = 0; i < sizeof(tmp); i++)
    {
        errMsg[i] = tmp[i];
    }
    printf(" KiteFastController_Controller in kfc_dll_init: returning ErrStat=1 ErrMsg=this is a message from dll\'s init\n");
}

void kfc_dll_end(int *errStat, char *errMsg) {
    *errStat = 3;
    char tmp[] = "this is a message from from dll\'s end";
    for (int i = 0; i < sizeof(tmp); i++)
    {
        errMsg[i] = tmp[i];
    }
    printf(" KiteFastController_Controller in kfc_dll_end: returning ErrStat=1 ErrMsg=this is a message from dll\'s end\n");
}

void kfc_dll_step(double dcm_g2b_c[], double pqr_c[], double acc_norm_c[],
                  double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
                  double Ab_c[], double *rho_c, double apparent_wind_c[],
                  double tether_force_c[], double wind_g_c[],
                  double SFlp_c[], double PFlp_c[], double Rudr_c[],
                  double SElv_c[], double PElv_c[], double GenSPyRtr_c[],
                  double GenPPyRtr_c[], int *errStat, char *errMsg) {
    *errStat = 2;
    char tmp[] = "this is a message from from dll\'s step";
    for (int i = 0; i < sizeof(tmp); i++)
    {
        errMsg[i] = tmp[i];
    }
    printf(" KiteFastController_Controller in kfc_dll_step: returning ErrStat=1 ErrMsg=this is a message from dll\'s step\n");
}