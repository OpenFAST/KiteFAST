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
    int doUpdateStates;
    int nFinal = 5;   // Last time increment for this simulation
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
    pFusODCM[0] =  1.0;
    pFusODCM[1] =  0.0;
    pFusODCM[2] =  0.0;
    pFusODCM[3] =  0.0;
    pFusODCM[4] =  1.0;
    pFusODCM[5] =  0.0;
    pFusODCM[6] =  0.0;
    pFusODCM[7] =  0.0;
    pFusODCM[8] =  1.0;

    // Rotor points
    numRtrPtsElem = numPylons * 4 * 3;
    pRtrPts = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrVels = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrDCMs = (double *)malloc(numRtrPtsElem * 3 * sizeof(double));
    
    numRtrLoadsElem = numRtrPtsElem*2;
    pRtrLoads = (double *)malloc(numRtrLoadsElem * sizeof(double));

    // Starboard inner top
    pRtrPts[0] = -1.861;
    pRtrPts[1] = 1.213;
    pRtrPts[2] = 1.221;
    // Starboard inner bottom
    pRtrPts[3] = -1.515;
    pRtrPts[4] = 1.213;
    pRtrPts[5] = -1.593;
    // Starboard outer top
    pRtrPts[6] = -1.861;
    pRtrPts[7] = 3.640;
    pRtrPts[8] = 1.221;
    // Starboard outer bottom
    pRtrPts[9]  = -1.515;
    pRtrPts[10] = 3.640;
    pRtrPts[11] = -1.593;
    // Port inner top
    pRtrPts[12] = -1.861;
    pRtrPts[13] = -1.213;
    pRtrPts[14] = 1.221;
    // Port inner bottom
    pRtrPts[15] = -1.515;
    pRtrPts[16] = -1.213;
    pRtrPts[17] = -1.593;
    // Port outer top
    pRtrPts[18] = -1.861;
    pRtrPts[19] = -3.639;
    pRtrPts[20] = 1.221;
    // Port outer bottom
    pRtrPts[21] = -1.515;
    pRtrPts[22] = -3.639;
    pRtrPts[23] = -1.593;



    // Reference points
    numRefPtElem = numComp * 3;
    pRefPts = (double *)malloc(numRefPtElem * sizeof(double));
    
    for (i = 0; i < numRefPtElem; i++)
    {
        pRefPts[i] = 0.0;
    }

    //Fuselage
    pRefPts[0] = 0.0;
    pRefPts[1] = 0.0;
    pRefPts[2] = 0.0;
    // Starboard Wing
    pRefPts[3] = 0.0;
    pRefPts[4] = 0.0;
    pRefPts[5] = 0.0;
    // Port Wing
    pRefPts[6] = 0.0;
    pRefPts[7] = 0.0;
    pRefPts[8] = 0.0;
    // Vertical Stabilizer
    pRefPts[6] = 6.891;
    pRefPts[7] = 98.0;
    pRefPts[8] = 100.0;
    // Starboard Horizontal Stabilizer
    pRefPts[9] = 6.555;
    pRefPts[10] = 0.0;
    pRefPts[11] = -0.817;
    // Port Horizontal Stabilizer
    pRefPts[12] = 6.555;
    pRefPts[13] = 0.0;
    pRefPts[14] = -0.817;
    // Starboard Pylons
    pRefPts[15] = -0.857;
    pRefPts[16] = 1.0;
    pRefPts[17] = 0.0;

    pRefPts[18] = -0.857;
    pRefPts[19] = 3.5;
    pRefPts[20] = 0.0;

    // Port Pylons
    pRefPts[21] = -0.857;
    pRefPts[22] = -1.0;
    pRefPts[23] = 0.0;

    pRefPts[24] = -0.857;
    pRefPts[25] = -3.5;
    pRefPts[26] = 0.0;

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

    // Set all DCMs to Identity
    for (i = 0; i < numDCMElem; i=i+9)
    {
        pNodeDCMs[i]   = 1.0;
        pNodeDCMs[i+1] = 0.0;
        pNodeDCMs[i+2] = 0.0;
        pNodeDCMs[i+3] = 0.0;
        pNodeDCMs[i+4] = 1.0;
        pNodeDCMs[i+5] = 0.0;
        pNodeDCMs[i+6] = 0.0;
        pNodeDCMs[i+7] = 0.0;
        pNodeDCMs[i+8] = 1.0;
    }
   
    // Fuselage node positions
    pNodePts[0] = 0.027;
    pNodePts[1] = 0.000;
    pNodePts[2] = -0.170;
    pNodePts[3] = 3.400;
    pNodePts[4] = 0.000;
    pNodePts[5] = 0.00;
    pNodePts[6] = 6.917;
    pNodePts[7] = 0.000;
    pNodePts[8] = -0.039;
    c = 9;
    //  Starboard wing nodes
    pNodePts[c + 0] = 0.000;
    pNodePts[c + 1] = 0.000;
    pNodePts[c + 2] = 0.000;
    pNodePts[c + 3] = 0.000;
    pNodePts[c + 4] = 12.831;
    pNodePts[c + 5] = 0.384;
    c = c + 6;
    //  Port wing nodes
    pNodePts[c + 0] = 0.000;
    pNodePts[c + 1] = 0.000;
    pNodePts[c + 2] = 0.000;
    pNodePts[c + 3] = 0.000;
    pNodePts[c + 4] = -12.831;
    pNodePts[c + 5] = 0.384;
    c = c + 6;
    //  vertical stabilizer nodes
    pNodePts[c + 0] = 0.123 + pRefPts[6];
    pNodePts[c + 1] = 0.000 + pRefPts[7];
    pNodePts[c + 2] = 2.850 + pRefPts[8];
    pNodePts[c + 3] = 0.044 + pRefPts[6];
    pNodePts[c + 4] = 0.000 + pRefPts[7];
    pNodePts[c + 5] = -0.712 + pRefPts[8];
    c = c + 6;
    //  starboard horizontal stabilizer nodes
    pNodePts[c + 0] = 0.000 + pRefPts[9];
    pNodePts[c + 1] = 0.000 + pRefPts[10];
    pNodePts[c + 2] = 0.000 + pRefPts[11];
    pNodePts[c + 3] = 0.417 + pRefPts[9];
    pNodePts[c + 4] = 2.447 + pRefPts[10];
    pNodePts[c + 5] = 0.000 + pRefPts[11];
    c = c + 6;
    //  port horizontal stabilizer nodes
    pNodePts[c + 0] = 0.000 + pRefPts[12];
    pNodePts[c + 1] = 0.000 + pRefPts[13];
    pNodePts[c + 2] = 0.000 + pRefPts[14];
    pNodePts[c + 3] = 0.417 + pRefPts[12];
    pNodePts[c + 4] = -2.447 + pRefPts[13];
    pNodePts[c + 5] = 0.000 + pRefPts[14];
    c = c + 6;
    //  starboard inboard pylon nodes
    pNodePts[c + 0] = -0.729 + pRefPts[15];
    pNodePts[c + 1] = 0.000 + pRefPts[16];
    pNodePts[c + 2] = 1.470 + pRefPts[17];
    pNodePts[c + 3] = -0.510 + pRefPts[15];
    pNodePts[c + 4] = 0.000 + pRefPts[16];
    pNodePts[c + 5] = -1.832 + pRefPts[17];
    c = c + 6;
    //  starboard outboard pylon nodes
    pNodePts[c + 0] = -0.729 + pRefPts[18];
    pNodePts[c + 1] = 0.000 + pRefPts[19];
    pNodePts[c + 2] = 1.470 + pRefPts[20];
    pNodePts[c + 3] = -0.510 + pRefPts[18];
    pNodePts[c + 4] = 0.000 + pRefPts[19];
    pNodePts[c + 5] = -1.832 + pRefPts[20];
    c = c + 6;
    //  port inboard pylon nodes
    pNodePts[c + 0] = -0.729 + pRefPts[21];
    pNodePts[c + 1] = 0.000 + pRefPts[22];
    pNodePts[c + 2] = 1.470 + pRefPts[23];
    pNodePts[c + 3] = -0.510 + pRefPts[21];
    pNodePts[c + 4] = 0.000 + pRefPts[22];
    pNodePts[c + 5] = -1.832 + pRefPts[23];
    c = c + 6;

    //  port outboard pylon nodes
    pNodePts[c + 0] = -0.729 + pRefPts[24];
    pNodePts[c + 1] = 0.000 + pRefPts[25];
    pNodePts[c + 2] = 1.470 + pRefPts[26];
    pNodePts[c + 3] = -0.510 + pRefPts[24];
    pNodePts[c + 4] = 0.000 + pRefPts[25];
    pNodePts[c + 5] = -1.832 + pRefPts[26];


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
    pFusO[0] = pRefPts[0]-50.0;
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
    for (n = 0; n < numRtrPtsElem; n++)
    {
        pRtrVels[n] = 0.0;
    }

    for (n = 0; n < numRtrPtsElem; n++)
    {
        pRtrVels[n] = 0.0;
    }
    for (n = 0; n < numRtrPtsElem*3; n++)
    {
        pRtrDCMs[n] = 0.0;
    }
    for (n = 0; n < numRtrPtsElem * 3; n=n+9)
    {
        pRtrDCMs[n] = 1.0;
    }
    for (n = 4; n < numRtrPtsElem * 3; n = n + 9)
    {
        pRtrDCMs[n] = 1.0;
    }
    for (n = 8; n < numRtrPtsElem * 3; n = n + 9)
    {
        pRtrDCMs[n] = 1.0;
    }

    // This returns the loads from KiteFAST at the initial time
    doUpdateStates = 0;  // we do not advance the states for the first call to KFAST_AssRes(), we only want to compute the output loads
    KFAST_AssRes(&t, &doUpdateStates, &numRtSpdRtrElem, pRtSpd_PyRtr, pWindPt, pFusO_prev, pFusO, pFusODCM_prev, pFusOv_prev,
                    pFusOomegas_prev, pFusOacc_prev, &numNodePtElem, pNodePts,
                    &numNodePtElem, pNodeVels, &numNodePtElem, pNodeOmegas,
                    &numDCMElem, pNodeDCMs, &numRtrPtsElem, pRtrPts, pRtrVels, pRtrDCMs, 
                    &numNodeLoadsElem, pNodeLoads, &numRtrLoadsElem, pRtrLoads, & errStat, errMsg);
    if (errStat != 0)
    {
        printf("%s\n", errMsg);
        return errStat;
    }

    //Now we begin calls associated with the time marching loop of MBDyn
    doUpdateStates = 1;  // Now we do advance the states for the each call to KFAST_AssRes()

    for (n = 1; n < nFinal; n++) 
    {
        t = t + dt;

        // Modify positions and velocities and accelerations

        // For now we will simulate constant positions, velocities, and accelerations

        // AssRes()
        KFAST_AssRes(&t, &doUpdateStates, &numRtSpdRtrElem, pRtSpd_PyRtr, pWindPt, pFusO_prev, pFusO, pFusODCM_prev, pFusOv_prev,
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
        KFAST_Output(&errStat, errMsg);
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


    free(pNumCompNds);
    free(pFusODCM);
    free(pRtrPts);
    free(pRefPts);
    free(pNodePts);
    free(pNodeDCMs);

    free(pRtSpd_PyRtr);
    free(pFusO);
    free(pFusOv_prev);
    free(pNodeDCMs);


    return 0;

}

