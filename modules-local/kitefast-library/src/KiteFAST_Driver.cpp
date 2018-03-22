// KiteFAST_Driver.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "KFAST_Lib.h"
#include <windows.h> 
#include <stdio.h> 

//!typedef void(__cdecl *KFAST_INITPROC)(double *dt, int *numFlaps, int *numPylons, int *numComp, int *numCompNds, const char *KAD_FileName, const char *IfW_FileName, const char *MD_FileName,
//!    const char *outFileRoot, double *gravity, double *FusODCM_c, int *numRtrPtsElem_c, double *rtrPts_c, int *numRefPtElem_c, double *refPts_c, int *numDCMElem_c, double *nodeDCMs_c, int *errStat, char *errMsg);

int _tmain(int argc, _TCHAR* argv[])
{
//    HINSTANCE hinstLib;
//    KFAST_INITPROC ProcInit;
//    BOOL fFreeResult, fRunTimeLinkSuccess = FALSE;
    int i;
    int c;
  
    double dt = 0.01;
    int isInitialTime;
    int nFinal;          // Last time increment for this simulation
    int numFlaps = 3;
    int numPylons = 2;
    int numComp = 10;
    int *pNumCompNds = NULL;
    char KAD_FileName[INTERFACE_STRING_LENGTH];
    char IfW_FileName[INTERFACE_STRING_LENGTH];
    char MD_FileName[INTERFACE_STRING_LENGTH];
    char KFC_FileName[INTERFACE_STRING_LENGTH];
    char outFileRoot[INTERFACE_STRING_LENGTH];
    double gravity = 9.81;
    double *pWindPt;
    double *pFusODCM;
    double *pFusODCM_prev;
    int numRtrPtsElem;
    double *pRtrPts;
    int numRefPtElem;
    double *pRefPts;
    int numNodePtElem;
    double *pNodePts;
    int numDCMElem;
    double *pNodeDCMs;
    int errStat;
    char errMsg[INTERFACE_STRING_LENGTH];
    int n;
    double t;
    int numRtSpdRtrElem;
    double *pRtSpd_PyRtr;
    double *pFusO;
    double *pFusO_prev;
    double *pFusOv_prev;
    double *pFusOomegas_prev;
    double *pFusOacc_prev;
    double *pNodeVels;
    double *pNodeOmegas;
    int    *pModFlags;
    double *pRtrVels;
    double *pRtrDCMs;
    int numNodeLoadsElem;
    double *pNodeLoads;
    int numRtrLoadsElem;
    double *pRtrLoads;
    double *pKiteOffset;


   // Set module flags 0 = off, 1=on
    pModFlags = (int *)malloc(4*sizeof(int));
    pModFlags[0] = 1;  // use KAD module
    pModFlags[1] = 1;  // use InflowWind module
    pModFlags[2] = 0;  // use MoorDyn module
    pModFlags[3] = 0;  // no KiteFAST controller

    // Set input file names
    strcpy(KAD_FileName, "C:\\Dev\\makani\\kitefast_models\\simple_m600_model.inp");
    strcpy(IfW_FileName, "C:\\Dev\\makani\\kitefast_models\\kiteInflowWind.dat");
    strcpy(MD_FileName , "C:\\Dev\\makani\\kitefast_models\\kiteTether.dat");
    strcpy(KFC_FileName, "Kite-controller.dll");
    strcpy(outFileRoot , "KiteTest");


    // Set up the number of nodes per kite component
    pNumCompNds = (int *)malloc(numComp*sizeof(int));
    pNumCompNds[0] = 3; // Fuselage nodes
    pNumCompNds[1] = 2; //  Starboard wing nodes
    pNumCompNds[2] = 2; //  Port wing nodes
    pNumCompNds[3] = 2; //  vertical stabilizer nodes
    pNumCompNds[4] = 2; //  starboard horizontal stabilizer nodes
    pNumCompNds[5] = 2; //  port horizontal stabilizer nodes
    pNumCompNds[6] = 2; //  starboard inboard pylon nodes
    pNumCompNds[7] = 2; //  starboard outboard pylon nodes
    pNumCompNds[8] = 2; //  port inboard pylon nodes
    pNumCompNds[9] = 2; //  port outboard pylon nodes

    // Set the ground station point where the wind is measured
    pWindPt = (double *)malloc(3 * sizeof(double));
    pWindPt[0] = 0.0;
    pWindPt[1] = 0.0;
    pWindPt[2] = 10.0;

    //Test the FusODCM as a 1D array instead of a 2D array
    // The kite is aligned with the Global Coordinate system
    pFusODCM = (double *)malloc(9*sizeof(double));
    pFusODCM[0] =  -1.0;
    pFusODCM[1] =  0.0;
    pFusODCM[2] =  0.0;
    pFusODCM[3] =  0.0;
    pFusODCM[4] =  1.0;
    pFusODCM[5] =  0.0;
    pFusODCM[6] =  0.0;
    pFusODCM[7] =  0.0;
    pFusODCM[8] =  -1.0;

    // Offset of kit in global coordinates (m)
    // This offset needs to be added to all the regerence points.

    pKiteOffset = (double *)malloc(3 * sizeof(double));
    pKiteOffset[0] = 0.0;
    pKiteOffset[1] = 0.0;
    pKiteOffset[2] = 300.0;

    // Rotor points
    numRtrPtsElem = numPylons * 4 * 3;
    pRtrPts = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrVels = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrDCMs = (double *)malloc(numRtrPtsElem * 3 * sizeof(double));
    
    numRtrLoadsElem = numRtrPtsElem*2;
    pRtrLoads = (double *)malloc(numRtrLoadsElem * sizeof(double));

    // Starboard inner top
    pRtrPts[0] = -1.861 + pKiteOffset[0];
    pRtrPts[1] = 1.213 + pKiteOffset[1];
    pRtrPts[2] = 1.221 + pKiteOffset[2];
    // Starboard inner bottom
    pRtrPts[3] = -1.515 + pKiteOffset[0];
    pRtrPts[4] = 1.213 + pKiteOffset[1];
    pRtrPts[5] = -1.593 + pKiteOffset[2];
    // Starboard outer top
    pRtrPts[6] = -1.861 + pKiteOffset[0];
    pRtrPts[7] = 3.640 + pKiteOffset[1];
    pRtrPts[8] = 1.221 + pKiteOffset[2];
    // Starboard outer bottom
    pRtrPts[9] = -1.515 + pKiteOffset[0];
    pRtrPts[10] = 3.640 + pKiteOffset[1];
    pRtrPts[11] = -1.593 + pKiteOffset[2];
    // Port inner top
    pRtrPts[12] = -1.861 + pKiteOffset[0];
    pRtrPts[13] = -1.213 + pKiteOffset[1];
    pRtrPts[14] = 1.221 + pKiteOffset[2];
    // Port inner bottom
    pRtrPts[15] = -1.515 + pKiteOffset[0];
    pRtrPts[16] = -1.213 + pKiteOffset[1];
    pRtrPts[17] = -1.593 + pKiteOffset[2];
    // Port outer top
    pRtrPts[18] = -1.861 + pKiteOffset[0];
    pRtrPts[19] = -3.639 + pKiteOffset[1];
    pRtrPts[20] = 1.221 + pKiteOffset[2];
    // Port outer bottom
    pRtrPts[21] = -1.515 + pKiteOffset[0];
    pRtrPts[22] = -3.639 + pKiteOffset[1];
    pRtrPts[23] = -1.593 + pKiteOffset[2];



    // Reference points
    numRefPtElem = numComp * 3;
    pRefPts = (double *)malloc(numRefPtElem * sizeof(double));

    for (i = 0; i < numRefPtElem; i++)
    {
        pRefPts[i] = 0.0;
    }
    c = 0;
    //Fuselage
    pRefPts[c + 0] = 0.0 + pKiteOffset[0];
    pRefPts[c + 1] = 0.0 + pKiteOffset[1];
    pRefPts[c + 2] = 0.0 + pKiteOffset[2];
    c = c + 3;
    // Starboard Wing
    pRefPts[c + 0] = 0.0 + pKiteOffset[0];
    pRefPts[c + 1] = 0.0 + pKiteOffset[1];
    pRefPts[c + 2] = 0.0 + pKiteOffset[2];
    c = c + 3;
    // Port Wing
    pRefPts[c + 0] = 0.0 + pKiteOffset[0];
    pRefPts[c + 1] = 0.0 + pKiteOffset[1];
    pRefPts[c + 2] = 0.0 + pKiteOffset[2];
    c = c + 3;
    // Vertical Stabilizer

    pRefPts[c + 0] = 6.891 + pKiteOffset[0];
    pRefPts[c + 1] = 0.0 + pKiteOffset[1];
    pRefPts[c + 2] = 0.0 + pKiteOffset[2];
    c = c + 3;
    // Starboard Horizontal Stabilizer
    pRefPts[c + 0] = 6.555 + pKiteOffset[0];
    pRefPts[c + 1] = 0.0 + pKiteOffset[1];
    pRefPts[c + 2] = -0.817 + pKiteOffset[2];
    c = c + 3;
    // Port Horizontal Stabilizer
    pRefPts[c + 0] = 6.555 + pKiteOffset[0];
    pRefPts[c + 1] = 0.0 + pKiteOffset[1];
    pRefPts[c + 2] = -0.817 + pKiteOffset[2];
    c = c + 3;
    // Starboard Pylons
    pRefPts[c + 0] = -0.857 + pKiteOffset[0];
    pRefPts[c + 1] = 1.0 + pKiteOffset[1];
    pRefPts[c + 2] = 0.0 + pKiteOffset[2];
    c = c + 3;

    pRefPts[c + 0] = -0.857 + pKiteOffset[0];
    pRefPts[c + 1] = 3.5 + pKiteOffset[1];
    pRefPts[c + 2] = 0.0 + pKiteOffset[2];
    c = c + 3;

    // Port Pylons
    pRefPts[c + 0] = -0.857 + pKiteOffset[0];
    pRefPts[c + 1] = -1.0 + pKiteOffset[1];
    pRefPts[c + 2] = 0.0 + pKiteOffset[2];
    c = c + 3;

    pRefPts[c + 0] = -0.857 + pKiteOffset[0];
    pRefPts[c + 1] = -3.5 + pKiteOffset[1];
    pRefPts[c + 2] = 0.0 + pKiteOffset[2];
    c = c + 3;

    // nodal DCMs
    numDCMElem = 0;
    numNodePtElem = 0;
    for (i = 0; i < numComp; i++) 
    {
        numDCMElem    = numDCMElem + 9 * pNumCompNds[i];
        numNodePtElem = numNodePtElem + 3 * pNumCompNds[i];
    }
    pNodeDCMs = (double *)malloc(numDCMElem * sizeof(double));
    pNodePts  = (double *)malloc(numNodePtElem * sizeof(double));
    numNodeLoadsElem = numNodePtElem*2;
    pNodeLoads = (double *)malloc(numNodeLoadsElem * sizeof(double));

    // Set all DCMs to FusO DCM
    for (i = 0; i < numDCMElem; i=i+9)
    {
        pNodeDCMs[i] = pFusODCM[0];
        pNodeDCMs[i + 1] = pFusODCM[1];
        pNodeDCMs[i + 2] = pFusODCM[2];
        pNodeDCMs[i + 3] = pFusODCM[3];
        pNodeDCMs[i + 4] = pFusODCM[4];
        pNodeDCMs[i + 5] = pFusODCM[5];
        pNodeDCMs[i + 6] = pFusODCM[6];
        pNodeDCMs[i + 7] = pFusODCM[7];
        pNodeDCMs[i + 8] = pFusODCM[8];
    }
    
    n = 0;
    // Fuselage node positions
    pNodePts[0] = 0.027 +pRefPts[n + 0];
    pNodePts[1] = 0.000 +pRefPts[n + 1];
    pNodePts[2] = -0.170 +pRefPts[n + 2];
    pNodePts[3] = 3.400 +pRefPts[n + 0];
    pNodePts[4] = 0.000 +pRefPts[n + 1];
    pNodePts[5] = 0.00  +pRefPts[n + 2];
    pNodePts[6] = 6.917   +pRefPts[n + 0];
    pNodePts[7] = 0.000   +pRefPts[n + 1];
    pNodePts[8] = -0.039  +pRefPts[n + 2];
    c = 9;
    //  Starboard wing nodes
    n = 3;
    pNodePts[c + 0] = 0.000  + pRefPts[n+0];
    pNodePts[c + 1] = 0.000  + pRefPts[n+1];
    pNodePts[c + 2] = 0.000  + pRefPts[n+2];
    pNodePts[c + 3] = 0.000  + pRefPts[n+0];
    pNodePts[c + 4] = 12.831 + pRefPts[n+1];
    pNodePts[c + 5] = 0.384  + pRefPts[n+2];
    n = n + 3;
    c = c + 6;
    //  Port wing nodes
    pNodePts[c + 0] = 0.000 + pRefPts[n + 0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 2] = 0.000 + pRefPts[n + 2];
    pNodePts[c + 3] = 0.000 + pRefPts[n + 0];
    pNodePts[c + 4] = -12.831 + pRefPts[n + 1];
    pNodePts[c + 5] = 0.384 + pRefPts[n + 2];
    n = n + 3;
    c = c + 6;
    //  vertical stabilizer nodes
    pNodePts[c + 0] = 0.123 + pRefPts[n + 0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 2] = 2.850 + pRefPts[n + 2];
    pNodePts[c + 3] = 0.044 + pRefPts[n + 0];
    pNodePts[c + 4] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 5] = -0.712 + pRefPts[n + 2];
    n = n + 3;
    c = c + 6;
    //  starboard horizontal stabilizer nodes
    pNodePts[c + 0] = 0.000 + pRefPts[n + 0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 2] = 0.000 + pRefPts[n + 2];
    pNodePts[c + 3] = 0.417 + pRefPts[n + 0];
    pNodePts[c + 4] = 2.447 + pRefPts[n + 1];
    pNodePts[c + 5] = 0.000 + pRefPts[n + 2];
    n = n + 3;
    c = c + 6;
    //  port horizontal stabilizer nodes
    pNodePts[c + 0] = 0.000 + pRefPts[n + 0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 2] = 0.000 + pRefPts[n + 2];
    pNodePts[c + 3] = 0.417 + pRefPts[n + 0];
    pNodePts[c + 4] = -2.447 + pRefPts[n + 1];
    pNodePts[c + 5] = 0.000 + pRefPts[n + 2];
    n = n + 3;
    c = c + 6;
    //  starboard inboard pylon nodes
    pNodePts[c + 0] = -0.729 + pRefPts[n + 0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 2] = 1.470 + pRefPts[n + 2];
    pNodePts[c + 3] = -0.510 + pRefPts[n + 0];
    pNodePts[c + 4] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 5] = -1.832 + pRefPts[n + 2];
    n = n + 3;
    c = c + 6;
    //  starboard outboard pylon nodes
    pNodePts[c + 0] = -0.729 + pRefPts[n + 0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 2] = 1.470 + pRefPts[n + 2];
    pNodePts[c + 3] = -0.510 + pRefPts[n + 0];
    pNodePts[c + 4] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 5] = -1.832 + pRefPts[n + 2];
    n = n + 3;
    c = c + 6;
    //  port inboard pylon nodes
    pNodePts[c + 0] = -0.729 + pRefPts[n + 0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 2] = 1.470 + pRefPts[n + 2];
    pNodePts[c + 3] = -0.510 + pRefPts[n + 0];
    pNodePts[c + 4] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 5] = -1.832 + pRefPts[n + 2];
    n = n + 3;
    c = c + 6;

    //  port outboard pylon nodes
    pNodePts[c + 0] = -0.729 + pRefPts[n + 0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 2] = 1.470 + pRefPts[n + 2];
    pNodePts[c + 3] = -0.510 + pRefPts[n + 0];
    pNodePts[c + 4] = 0.000 + pRefPts[n + 1];
    pNodePts[c + 5] = -1.832 + pRefPts[n + 2];


    // This is called as part of the user module constructor
    KFAST_Init(&dt, &numFlaps, &numPylons, &numComp, pNumCompNds, pModFlags, KAD_FileName, IfW_FileName, MD_FileName, KFC_FileName, outFileRoot, &gravity, pWindPt,
        pFusODCM, &numRtrPtsElem, pRtrPts, &numRefPtElem, pRefPts, &numNodePtElem, pNodePts, &numDCMElem, pNodeDCMs, &errStat, errMsg);
    
    if (errStat != 0)
    {
        printf("%s\n", errMsg);
        return errStat;
    }


    // The outputs for the first timestep need to be obtained
    // TODO: Is call part of the user module constructor or part of InitialAssRes??? GJH

    t = 0.0; // initial time
    numRtSpdRtrElem = 8;
    pFusODCM_prev = pFusODCM;
    pRtSpd_PyRtr = (double *)malloc(numRtSpdRtrElem*sizeof(double));   
    for (i = 0; i < numRtSpdRtrElem; i++)
    {
        pRtSpd_PyRtr[i] = 0.0;
    }
    pFusO        = (double *)malloc(3*sizeof(double));
    pFusO[0] = pRefPts[0];
    pFusO[1] = pRefPts[1];
    pFusO[2] = pRefPts[2];

    pFusO_prev = (double *)malloc(3 * sizeof(double));
    pFusO_prev[0] = pRefPts[0];
    pFusO_prev[1] = pRefPts[1];
    pFusO_prev[2] = pRefPts[2];


    pFusOv_prev = (double *)malloc(3 * sizeof(double));
    pFusOv_prev[0] = -50.0;
    pFusOv_prev[1] =  0.0;
    pFusOv_prev[2] =  0.0;

    pFusOomegas_prev = (double *)malloc(3 * sizeof(double));
    pFusOomegas_prev[0] = 0.0;
    pFusOomegas_prev[1] = 0.0;
    pFusOomegas_prev[2] = 0.0;

    pFusOacc_prev    = (double *)malloc(3*sizeof(double));
    pFusOacc_prev[0] = 0.0;
    pFusOacc_prev[1] = 0.0;
    pFusOacc_prev[2] = 0.0;

    pNodeVels   = (double *)malloc(numNodePtElem * sizeof(double));
    pNodeOmegas = (double *)malloc(numNodePtElem * sizeof(double));
    i = 0;
    for (n = 0; n < numNodePtElem / 3; n++)
    {
        pNodeVels[n * 3] = -50.0;
        pNodeVels[n * 3 + 1] =  0.0;
        pNodeVels[n * 3 + 2] =  0.0;
        pNodeOmegas[n * 3] = 0.0;
        pNodeOmegas[n * 3 + 1] = 0.0;
        pNodeOmegas[n * 3 + 2] = 0.0;
    }
    
    // Need to set rotor point velocities and orientations
    for (n = 0; n < numRtrPtsElem; n=n+3)
    {
        pRtrVels[n]     = -50.0;
        pRtrVels[n + 1] =   0.0;
        pRtrVels[n + 2] =   0.0;
    }

    for (i = 0; i < numDCMElem; i = i + 9)
    {
        pRtrDCMs[i] = pFusODCM[0];
        pRtrDCMs[i + 1] = pFusODCM[1];
        pRtrDCMs[i + 2] = pFusODCM[2];
        pRtrDCMs[i + 3] = pFusODCM[3];
        pRtrDCMs[i + 4] = pFusODCM[4];
        pRtrDCMs[i + 5] = pFusODCM[5];
        pRtrDCMs[i + 6] = pFusODCM[6];
        pRtrDCMs[i + 7] = pFusODCM[7];
        pRtrDCMs[i + 8] = pFusODCM[8];
    }

    // This returns the loads from KiteFAST at the initial time
    isInitialTime = 1;  // we do not advance the states for the first call to KFAST_AssRes(), we only want to compute the output loads
    KFAST_AssRes(&t, &isInitialTime, &numRtSpdRtrElem, pRtSpd_PyRtr, pWindPt, pFusO_prev, pFusO, pFusODCM_prev, pFusOv_prev,
                    pFusOomegas_prev, pFusOacc_prev, &numNodePtElem, pNodePts,
                    &numNodePtElem, pNodeVels, &numNodePtElem, pNodeOmegas,
                    &numDCMElem, pNodeDCMs, &numRtrPtsElem, pRtrPts, pRtrVels, pRtrDCMs, 
                    &numNodeLoadsElem, pNodeLoads, &numRtrLoadsElem, pRtrLoads, & errStat, errMsg);
    if (errStat != 0)
    {
        printf("%s\n", errMsg);
        return errStat;
    }

    // Output()
    KFAST_Output(&t, &errStat, errMsg);
    if (errStat != 0)
    {
        printf("%s\n", errMsg);
        printf("%s\n", "Quitting due to error in KFAST_OUTPUT");
        return errStat;
    }



    //Now we begin calls associated with the time marching loop of MBDyn
    isInitialTime = 0;  // Now we do advance the states for the each call to KFAST_AssRes()
    nFinal = 1;
    for (n = 2; n < nFinal; n++) 
    {
        t = t + dt;

        // Modify positions and velocities and accelerations

        // For now we will simulate constant positions, velocities, and accelerations

        // AssRes()
        KFAST_AssRes(&t, &isInitialTime, &numRtSpdRtrElem, pRtSpd_PyRtr, pWindPt, pFusO_prev, pFusO, pFusODCM_prev, pFusOv_prev,
            pFusOomegas_prev, pFusOacc_prev, &numNodePtElem, pNodePts,
            &numNodePtElem, pNodeVels, &numNodePtElem, pNodeOmegas,
            &numDCMElem, pNodeDCMs, &numRtrPtsElem, pRtrPts, pRtrVels, pRtrDCMs, 
            &numNodeLoadsElem, pNodeLoads, &numRtrLoadsElem, pRtrLoads, &errStat, errMsg);
        if (errStat != 0)
        {
            printf("%s\n", errMsg);
            return errStat;
        }

        // AfterPredict()
        KFAST_AfterPredict(&errStat, errMsg);
        if (errStat != 0)
        {
            printf("%s\n", errMsg);
            return errStat;
        }

        // Output()
        KFAST_Output(&t, &errStat, errMsg);
        if (errStat != 0)
        {
            printf("%s\n", errMsg);
            return errStat;
        }

    }


    // End()
    KFAST_End(&errStat, errMsg);
    if (errStat != 0)
    {
        printf("%s\n", errMsg);
        return errStat;
    }


    //free(pNumCompNds);
    //free(pWindPt);
    //free(pFusODCM);
    ////free(pRtrPts);
    ////free(pRefPts);
    //free(pNodePts);
    //free(pNodeDCMs);
    //free(pRtSpd_PyRtr);
    //free(pFusO);
    //free(pFusO_prev);
    //free(pFusOv_prev);
    //free(pFusOomegas_prev);
    //free(pFusOacc_prev);
    //free(pNodeVels);
    //free(pNodeOmegas);
    //free(pModFlags);
    //free(pRtrVels);
    //free(pRtrDCMs);
    //free(pNodeLoads);
    //free(pRtrLoads);
    //free(pKiteOffset);


    return errStat;

}

