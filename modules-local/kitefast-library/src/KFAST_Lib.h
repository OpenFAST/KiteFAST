// routines in KFAST_Lib.dll

#ifdef __cplusplus
#define EXTERNAL_ROUTINE extern "C"
#else
#define EXTERNAL_ROUTINE extern
#endif
EXTERNAL_ROUTINE void KFAST_Init(double *dt, int *numFlaps, int *numPylons, int *numComp, int *numCompNds, const char KAD_FileName[], const char IfW_FileName[], const char MD_FileName[],
    const char *outFileRoot, double *gravity, double FusODCM_c[], int *numRtrPtsElem_c, double rtrPts_c[], int *numRefPtElem_c, double refPts_c[], int *numDCMElem_c, double nodeDCMs_c[], int *errStat, char errMsg[]);
EXTERNAL_ROUTINE void KFAST_AssRes();
EXTERNAL_ROUTINE void KFAST_End();

// some constants (keep these synced with values in FAST's fortran code)
#define INTERFACE_STRING_LENGTH 1025

#define ErrID_None 0 
#define ErrID_Info 1 
#define ErrID_Warn 2 
#define ErrID_Severe 3 
#define ErrID_Fatal 4 

static int AbortErrLev = ErrID_Fatal;      // abort error level; compare with NWTC Library

#define CHANNEL_LENGTH 10  
