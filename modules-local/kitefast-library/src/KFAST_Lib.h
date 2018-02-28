// routines in KFAST_Lib.dll

#ifdef __cplusplus
#define EXTERNAL_ROUTINE extern "C"
#else
#define EXTERNAL_ROUTINE extern
#endif

EXTERNAL_ROUTINE void KFAST_Init(double *dt, int *numFlaps, int *numPylons, int *numComp, int numCompNds[], const char KAD_FileName[], const char IfW_FileName[], const char MD_FileName[], const char KFC_FileName[],
    const char outFileRoot[], double *gravity, double windPt[], double FusODCM[], int *numRtrPtsElem, double rtrPts[], int *numRefPtElem, double refPts[], 
    int *numNodePtElem, double nodePts[], int *numDCMElem, double nodeDCMs[], int *errStat, char errMsg[]);
EXTERNAL_ROUTINE void KFAST_AssRes(double *t, int *numRtSpdRtrElem, double RtSpd_PyRtr[], double WindPt[], double FusO[], double FusODCM[], double FusOv[], 
                                    double FusOomegas[], double FusOacc[], int *numNodePtElem, double *nodePts[],
                                    int *numNodeVelElem, double nodeVels[], int *numNodeOmegaElem, double nodeOmegas[], 
                                    int *numDCMElem, double nodeDCMs[], int *numRtrPtsElem, double rtrPts[], int *errStat, char errMsg[]);
EXTERNAL_ROUTINE void KFAST_AfterPredict(int *errStat, char errMsg[]);
EXTERNAL_ROUTINE void KFAST_Output(int *errStat, char errMsg[]);
EXTERNAL_ROUTINE void KFAST_End(int *errStat, char errMsg[]);

// some constants (keep these synced with values in FAST's fortran code)
#define INTERFACE_STRING_LENGTH 1025

#define ErrID_None 0 
#define ErrID_Info 1 
#define ErrID_Warn 2 
#define ErrID_Severe 3 
#define ErrID_Fatal 4 

static int AbortErrLev = ErrID_Fatal;      // abort error level; compare with NWTC Library

#define CHANNEL_LENGTH 10  
