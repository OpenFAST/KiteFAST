
#include "KiteFastController.h"

void kfc_dll_init(int *errStat, char *errMsg)
{
    *errStat = 0;
    char tmp[] = "this is a message from dll\'s init";
    int i;
    for (i = 0; i < sizeof(tmp); i++)
    {
        errMsg[i] = tmp[i];
    }
    printf(" KiteFastController_Controller in kfc_dll_init: returning ErrStat=0 ErrMsg=this is a message from dll\'s init\n");
}

void kfc_dll_end(int *errStat, char *errMsg)
{
    *errStat = 0;
    char tmp[] = "this is a message from from dll\'s end";
    int i;
    for (i = 0; i < sizeof(tmp); i++)
    {
        errMsg[i] = tmp[i];
    }
    printf(" KiteFastController_Controller in kfc_dll_end: returning ErrStat=0 ErrMsg=this is a message from dll\'s end\n");
}

void kfc_dll_step(double dcm_g2b_c[], double pqr_c[], double *acc_norm_c,
                  double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[],
                  double Ab_c[], double *rho_c, double apparent_wind_c[],
                  double tether_force_c[], double wind_g_c[],
                  double kFlapA_c[], double Motor_c[],
                  int *errStat, char *errMsg)
{
    *errStat = 0;
    char tmp[] = "this is a message from from dll\'s step";
    int i;
    static int n = 0;
    for (i = 0; i < sizeof(tmp); i++)
    {
        errMsg[i] = tmp[i];
    }

    printf(" KiteFastController_Controller in kfc_dll_step: these are the values in dcm_g2b_c\n");
    for (i = 0; i < 9; i++)
    {
        printf("%f\n", dcm_g2b_c[i]);
    }

    printf(" KiteFastController_Controller in kfc_dll_step: these are the values in Motor_c \n");
    for (i = 0; i < 8; i++)
    {
        printf("%f\n", Motor_c[i]);
    }

    n++;
    for (i = 0; i < 10; i++)
    {
        kFlapA_c[i] = 0.0;
    }

    if (n > 100)
    {
        kFlapA_c[5] = 20.0;
        kFlapA_c[4] = 20.0;
        printf(" KiteFastController_Controller in kfc_dll_step: setting flaps\n");
    }

    printf(" KiteFastController_Controller in kfc_dll_step: returning ErrStat=0 ErrMsg=this is a message from dll\'s step\n");
}
