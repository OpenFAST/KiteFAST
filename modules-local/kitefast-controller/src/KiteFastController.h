#define DllExport   __declspec( dllexport )

// some constants (keep these synced with values in KiteFAST's fortran code)
#define INTERFACE_STRING_LENGTH 1025

#define ErrID_None 0 
#define ErrID_Info 1 
#define ErrID_Warn 2 
#define ErrID_Severe 3 
#define ErrID_Fatal 4 

static int AbortErrLev = ErrID_Fatal;      // abort error level; compare with NWTC Library

DllExport void __cdecl kfc_dll_init(int *errStat, char errMsg[]);
DllExport void __cdecl kfc_dll_end(int *errStat, char errMsg[]);
DllExport void __cdecl kfc_dll_step(double dcm_g2b_c[], double pqr_c[], double acc_norm_c[], 
                                    double Xg_c[], double Vg_c[], double Vb_c[], double Ag_c[], 
                                    double Ab_c[], double *rho_c, double apparent_wind_c[], 
                                    double tether_force_c[], double wind_g_c[],  
                                    double SFlp_c[], double PFlp_c[], double Rudr_c[], 
                                    double SElv_c[], double PElv_c[], double GenSPyRtr_c[], 
                                    double GenPPyRtr_c[], int *errStat, char errMsg[]);