// KiteFAST_Driver.cpp : Defines the entry point for the console application.
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
// #include "stdafx.h"
// #include <windows.h> 
#include "KFAST_Lib.h"

//!typedef void(__cdecl *KFAST_INITPROC)(double *dt, int *numFlaps, int *numPylons, int *numComp, int *numCompNds, const char *KAD_FileName, const char *IfW_FileName, const char *MD_FileName,
//!    const char *outFileRoot, double *gravity, double *FusODCM_c, int *numRtrPtsElem_c, double *rtrPts_c, int *numRefPtElem_c, double *refPts_c, int *numDCMElem_c, double *nodeDCMs_c, int *errStat, char *errMsg);

int main(int argc, char *argv[])
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
    int numPylons = 1;
    int numComp = 8;
    int *pNumCompNds = NULL;
    char KAD_FileName[INTERFACE_STRING_LENGTH];
    char IfW_FileName[INTERFACE_STRING_LENGTH];
    char MD_FileName[INTERFACE_STRING_LENGTH];
    char KFC_FileName[INTERFACE_STRING_LENGTH];
    char outFileRoot[INTERFACE_STRING_LENGTH];
    int  printSum = 1;
    double gravity = 9.81;
	int  KAD_InterpOrder = 0;  // 0 = hold KAD outputs between KAD calls, 1 = Linearly interpolate outputs, 2 = 2nd order interpolation of outputs
    double *pWindPt;
    double *pFusODCM;
    double *pFusODCM_prev;
    int numRtrPts;
    int numRtrPtsElem;
    double *pRtrPts;
    int numRefPtElem;
    double *pRefPts;
    int numNodePts;
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
    double *pFusOalphas;
    double *pFusOv;
    double *pFusOomegas;
    double *pFusOacc;
    double *pNodeVels;
    double *pNodeOmegas;
    double *pNodeAccs;
    int    *pModFlags;
    double *pRtrVels;
    double *pRtrDCMs;
    double *pRtrMass;
    double *pRtrI_Rot;
    double *pRtrI_Trans;
    double *pRtrXcm;
    double *pRtrOmegas;
    double *pRtrAccs;
    double *pRtrAlphas;
    int numNodeLoadsElem;
    double *pNodeLoads;
    int numRtrLoadsElem;
    double *pRtrLoads;
    double *pKiteOffset;
    int numGaussLoadPts;
    int numGaussPtLoadsElem;
    double *pGaussPtLoads;

    //Outputs
    int nFusOuts = 2;
    int FusOutNd[] = { 2, 3 };
    int nSWnOuts = 3;
    int SWnOutNd[] = { 1, 2, 3 };
    int nPWnOuts = 1;
    int PWnOutNd[] = { 3 };
    int nVSOuts  = 0;
    int VSOutNd[] = { 1 };
    int nSHSOuts = 0;
    int SHSOutNd[] = { 1 };
    int nPHSOuts = 0;
    int PHSOutNd[] = { 1 };
    int nPylOuts = 2;
    int PylOutNd[] = { 1,2 };

    int numOutChan = 7;
    //char* outChanList[] = { "Fus1TDx   ", "Fus1TDy   ", "Fus1TDz   ", "SWn1TDx   ", "SWn1TDy   ", "SWn1TDz   ", "PP12FRc   " };
    char* outChanList[] = { "Fus1TDx", "Fus1TDy", "Fus1TDz", "dddddd", "eeeeeee", "ffffffff", "PP12FRc" };
    int*  pChanList_len;
    pChanList_len = (int *)malloc(numOutChan * sizeof(int));
    pChanList_len[0] = 7;
    pChanList_len[1] = 7;
    pChanList_len[2] = 7;
    pChanList_len[3] = 6;
    pChanList_len[4] = 7;
    pChanList_len[5] = 8;
    pChanList_len[6] = 7;
    

    // Set module flags 0 = off, 1=on
    pModFlags = (int *)malloc(4*sizeof(int));
    pModFlags[0] = 1;  // use KAD module
    pModFlags[1] = 1;  // use InflowWind module
    pModFlags[2] = 0;  // use MoorDyn module
    pModFlags[3] = 0;  // no KiteFAST controller

    // Set input file names
    // NOTE: All the data further below is directly tied to the KAD file listed here.
    strcpy(KAD_FileName, "D:\\DEV\\makani\\google-repo\\sandbox\\glue-codes\\kitefast\\test_cases\\m000\\simple_m000_model_AD.txt");
    strcpy(IfW_FileName, "D:\\DEV\\makani\\google-repo\\sandbox\\glue-codes\\kitefast\\test_cases\\m000\\kiteInflowWind.dat");
    strcpy(MD_FileName, "D:\\DEV\\makani\\google-repo\\sandbox\\glue-codes\\kitefast\\test_cases\\m000\\kiteTether.dat");
    strcpy(KFC_FileName, "D:\\DEV\\makani\\google-repo\\sandbox\\glue-codes\\kitefast\\test_cases\\m000\\libkitefastcontroller_controller.so");
    strcpy(outFileRoot, "KiteTest");

    // Set up the number of nodes per kite component
    pNumCompNds = (int *)malloc(numComp*sizeof(int));
    pNumCompNds[0] = 3; // Fuselage nodes
    pNumCompNds[1] = 3; //  Starboard wing nodes
    pNumCompNds[2] = 3; //  Port wing nodes
    pNumCompNds[3] = 3; //  vertical stabilizer nodes
    pNumCompNds[4] = 3; //  starboard horizontal stabilizer nodes
    pNumCompNds[5] = 3; //  port horizontal stabilizer nodes
    pNumCompNds[6] = 3; //  starboard inboard pylon nodes
    pNumCompNds[7] = 3; //  port inboard pylon nodes

    
    // Set the ground station point where the wind is measured
    pWindPt = (double *)malloc(3 * sizeof(double));
    pWindPt[0] = 100.0;
    pWindPt[1] = 20.0;
    pWindPt[2] = 0.0;

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
    /*pFusODCM[0] = 1.0;
    pFusODCM[1] = 2.0;
    pFusODCM[2] = 3.0;
    pFusODCM[3] = 4.0;
    pFusODCM[4] = 5.0;
    pFusODCM[5] = 6.0;
    pFusODCM[6] = 7.0;
    pFusODCM[7] = 8.0;
    pFusODCM[8] = 9.0;*/

    // Offset of kite in global coordinates (m)
    // This offset needs to be added to all the reference points.

    pKiteOffset = (double *)malloc(3 * sizeof(double));
    pKiteOffset[0] = 0.0;
    pKiteOffset[1] = 0.0;
    pKiteOffset[2] = 100.0;

    // Rotor reference points and associated nacelle quantities
    numRtrPts     = numPylons * 4;
    numRtrPtsElem = numRtrPts * 3;
    pRtrPts     = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrVels    = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrOmegas  = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrAccs    = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrAlphas  = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrDCMs    = (double *)malloc(numRtrPtsElem * 3 * sizeof(double));
    pRtrMass    = (double *)malloc(numRtrPts     * sizeof(double));
    pRtrI_Rot   = (double *)malloc(numRtrPts     * sizeof(double));
    pRtrI_Trans = (double *)malloc(numRtrPts     * sizeof(double));
    pRtrXcm     = (double *)malloc(numRtrPts     * sizeof(double));

    numRtrLoadsElem = numRtrPtsElem*2;
    pRtrLoads = (double *)malloc(numRtrLoadsElem * sizeof(double));

    // Starboard inner top in global
    pRtrPts[0] = -1.861 + pKiteOffset[0];
    pRtrPts[1] = 1.213 + pKiteOffset[1];
    pRtrPts[2] = 1.221 + pKiteOffset[2];
    // Starboard inner bottom
    pRtrPts[3] = -1.515 + pKiteOffset[0];
    pRtrPts[4] = 1.213 + pKiteOffset[1];
    pRtrPts[5] = -1.593 + pKiteOffset[2];
    //// Starboard outer top
    //pRtrPts[6] = -1.861 + pKiteOffset[0];
    //pRtrPts[7] = 3.640 + pKiteOffset[1];
    //pRtrPts[8] = 1.221 + pKiteOffset[2];
    //// Starboard outer bottom
    //pRtrPts[9] = -1.515 + pKiteOffset[0];
    //pRtrPts[10] = 3.640 + pKiteOffset[1];
    //pRtrPts[11] = -1.593 + pKiteOffset[2];
    // Port inner top
    pRtrPts[6] = -1.861 + pKiteOffset[0];
    pRtrPts[7] = -1.213 + pKiteOffset[1];
    pRtrPts[8] = 1.221 + pKiteOffset[2];
    // Port inner bottom
    pRtrPts[9] = -1.515 + pKiteOffset[0];
    pRtrPts[10] = -1.213 + pKiteOffset[1];
    pRtrPts[11] = -1.593 + pKiteOffset[2];
    //// Port outer top
    //pRtrPts[18] = -1.861 + pKiteOffset[0];
    //pRtrPts[19] = -3.639 + pKiteOffset[1];
    //pRtrPts[20] = 1.221 + pKiteOffset[2];
    //// Port outer bottom
    //pRtrPts[21] = -1.515 + pKiteOffset[0];
    //pRtrPts[22] = -3.639 + pKiteOffset[1];
    //pRtrPts[23] = -1.593 + pKiteOffset[2];

    //Set all the rotor masses and inertias and CM offsets
    for (i = 0; i < numRtrPts; i++)
    {
       pRtrMass[i]  = 20.0;
       pRtrI_Rot[i] = 200.0;
       pRtrI_Trans[i] = 15.0;
       pRtrXcm[i] = 0.1;
    }

    // Reference points  specified in the Kite Coordinate System
    numRefPtElem = numComp * 3;
    pRefPts = (double *)malloc(numRefPtElem * sizeof(double));

    for (i = 0; i < numRefPtElem; i++)
    {
        pRefPts[i] = 0.0;
    }
    c = 0;

    //Fuselage in global 
    pRefPts[c + 0] = pKiteOffset[0];
    pRefPts[c + 1] = pKiteOffset[1];
    pRefPts[c + 2] = pKiteOffset[2];
    c = c + 3;

    // Starboard Wing in kite coords
    pRefPts[c + 0] = 0.0;
    pRefPts[c + 1] = 0.5;
    pRefPts[c + 2] = 0.0;
    c = c + 3;

    // Port Wing in kite coords
    pRefPts[c + 0] = 0.0;
    pRefPts[c + 1] = -0.50;
    pRefPts[c + 2] = 0.0;
    c = c + 3;

    // Vertical Stabilizer in kite coords
    pRefPts[c + 0] = -5.1;
    pRefPts[c + 1] = 0.0  ;
    pRefPts[c + 2] = -2.0  ;
    c = c + 3;

    // Starboard Horizontal Stabilizer in kite coords
    pRefPts[c + 0] = -5.1  ;
    pRefPts[c + 1] = 0.1    ;
    pRefPts[c + 2] = -2.0 ;
    c = c + 3;

    // Port Horizontal Stabilizer in kite coords
    pRefPts[c + 0] = -5.1 ;
    pRefPts[c + 1] = -0.1   ;
    pRefPts[c + 2] = -2.0;
    c = c + 3;

    // Starboard Pylons in kite coords
    pRefPts[c + 0] = 0.1 ;
    pRefPts[c + 1] = 3.0 ;
    pRefPts[c + 2] = -0.5 ;
    c = c + 3;

   /* pRefPts[c + 0] = 0.857 ;
    pRefPts[c + 1] = 3.5    ;
    pRefPts[c + 2] = 0.0    ;
    c = c + 3;*/

    // Port Pylons in kite coords
    pRefPts[c + 0] = 0.1;
    pRefPts[c + 1] = -3.0 ;
    pRefPts[c + 2] = -0.5 ;
    c = c + 3;

   /* pRefPts[c + 0] = 0.857 ;
    pRefPts[c + 1] = -3.5;
    pRefPts[c + 2] = 0.0 ;
    c = c + 3;*/

    // nodal DCMs
    numDCMElem = 0;
    numNodePts = 0;
    numNodePtElem = 0;
    numGaussPtLoadsElem = 0;

    for (i = 0; i < numComp; i++) 
    {
        numDCMElem          = numDCMElem + 9 * pNumCompNds[i];
        numNodePts          = numNodePts + pNumCompNds[i];
        numNodePtElem       = numNodePtElem + 3 * pNumCompNds[i];
        numGaussPtLoadsElem = numGaussPtLoadsElem + 6 * (pNumCompNds[i] - 1);
    }
    numGaussLoadPts = numGaussPtLoadsElem / 6;

    pNodeDCMs = (double *)malloc(numDCMElem * sizeof(double));
    pNodePts  = (double *)malloc(numNodePtElem * sizeof(double));
    numNodeLoadsElem = numNodePtElem*2;
    pNodeLoads = (double *)malloc(numNodeLoadsElem * sizeof(double));
    pGaussPtLoads = (double *)malloc(numGaussPtLoadsElem * sizeof(double));

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

    // The node positions are in Global, but the pRefPts are in Kite, pKiteOffset is in Global


    // Fuselage node positions
    pNodePts[0] =-5.000 + pKiteOffset[0];
    pNodePts[1] = 0.000 + pKiteOffset[1];
    pNodePts[2] = 0.000 + pKiteOffset[2];
    pNodePts[3] = 0.000 + pKiteOffset[0];
    pNodePts[4] = 0.000 + pKiteOffset[1];
    pNodePts[5] = 0.000 + pKiteOffset[2];
    pNodePts[6] = 5.000 + pKiteOffset[0];
    pNodePts[7] = 0.000 + pKiteOffset[1];
    pNodePts[8] = 0.000 + pKiteOffset[2];
    c = 9;
    //  Starboard wing nodes
    n = 3;
    pNodePts[c + 0] = 0.000 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 2] = 0.000 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 3] = 0.000 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 4] = 2.500 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 5] = 0.000 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 6] = 0.000 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 7] = 5.500 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 8] = 0.000 - pRefPts[n + 2] + pKiteOffset[2];
    n = n + 3;
    c = c + 9;
    //  Port wing nodes
    pNodePts[c + 0] = 0.000 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 2] = 0.000 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 3] = 0.000 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 4] = -2.5  + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 5] = 0.0   - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 6] = 0.000 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 7] = -5.5  + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 8] = 0.0   - pRefPts[n + 2] + pKiteOffset[2];
    n = n + 3;
    c = c + 9;
    //  vertical stabilizer nodes
    pNodePts[c + 0] = 0.000 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 2] = 0.000 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 3] = 0.000 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 4] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 5] = -1    - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 6] = 0.00  - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 7] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 8] = -2    - pRefPts[n + 2] + pKiteOffset[2];
    n = n + 3;
    c = c + 9;
    //  starboard horizontal stabilizer nodes
    pNodePts[c + 0] = 0.000 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 2] = 0.000 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 3] = 0.0 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 4] = .4 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 5] = 0.000 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 6] = 0.0 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 7] = .9 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 8] = 0.000 - pRefPts[n + 2] + pKiteOffset[2];
    n = n + 3;
    c = c + 9;
    //  port horizontal stabilizer nodes
    pNodePts[c + 0] = 0.000 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 2] = 0.000 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 3] = 0 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 4] = -.4 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 5] = 0.000 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 6] = 0 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 7] = -.9 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 8] = 0.000 - pRefPts[n + 2] + pKiteOffset[2];
    n = n + 3;
    c = c + 9;
    //  starboard inboard pylon nodes
    pNodePts[c + 0] = - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 1] = + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 2] = - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 3] = - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 4] = + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 5] = -0.5 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 6] = - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 7] = + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 8] = -1 - pRefPts[n + 2] + pKiteOffset[2];
    n = n + 3;
    c = c + 9;
    //  starboard outboard pylon nodes
   /* pNodePts[c + 0] = -0.729 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 2] = 1.470 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 3] = -0.510 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 4] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 5] = -1.832 - pRefPts[n + 2] + pKiteOffset[2];
    n = n + 3;
    c = c + 6;*/
    //  port inboard pylon nodes
    pNodePts[c + 0] =  - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 1] =  + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 2] =  - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 3] =  - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 4] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 5] = -0.5 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 6] =  - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 7] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 8] = -1 - pRefPts[n + 2] + pKiteOffset[2];
    n = n + 3;
    c = c + 9;

    //  port outboard pylon nodes
    /*pNodePts[c + 0] = -0.729 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 1] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 2] = 1.470 - pRefPts[n + 2] + pKiteOffset[2];
    pNodePts[c + 3] = -0.510 - pRefPts[n + 0] + pKiteOffset[0];
    pNodePts[c + 4] = 0.000 + pRefPts[n + 1] + pKiteOffset[1];
    pNodePts[c + 5] = -1.832 - pRefPts[n + 2] + pKiteOffset[2];
*/


    // This is called as part of the user module constructor
    KFAST_Init(&dt, &numFlaps, &numPylons, &numComp, pNumCompNds, pModFlags, KAD_FileName, IfW_FileName, MD_FileName, KFC_FileName, outFileRoot, &printSum, &gravity, &KAD_InterpOrder,
       pFusODCM, &numRtrPts, pRtrPts, pRtrMass, pRtrI_Rot, pRtrI_Trans, pRtrXcm, pRefPts, &numNodePts, pNodePts, pNodeDCMs,
       &nFusOuts, FusOutNd, &nSWnOuts, SWnOutNd, &nPWnOuts, PWnOutNd, &nVSOuts, VSOutNd, &nSHSOuts, SHSOutNd, &nPHSOuts, PHSOutNd, &nPylOuts, PylOutNd, &numOutChan, outChanList, pChanList_len, &errStat, errMsg);

   
    if (errStat == 4)
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

   /* pFusO_prev = (double *)malloc(3 * sizeof(double));
    pFusO_prev[0] = pRefPts[0];
    pFusO_prev[1] = pRefPts[1];
    pFusO_prev[2] = pRefPts[2];


    pFusOv_prev = (double *)malloc(3 * sizeof(double));
    pFusOv_prev[0] = -50.0;
    pFusOv_prev[1] =  0.0;
    pFusOv_prev[2] =  0.0;*/

    pFusOv = (double *)malloc(3 * sizeof(double));
    pFusOv[0] = -50.0;
    pFusOv[1] = 0.0;
    pFusOv[2] = 0.0;

    /*pFusOomegas_prev = (double *)malloc(3 * sizeof(double));
    pFusOomegas_prev[0] = 0.0;
    pFusOomegas_prev[1] = 0.0;
    pFusOomegas_prev[2] = 0.0;*/

    pFusOomegas = (double *)malloc(3 * sizeof(double));
    pFusOomegas[0] = 0.0;
    pFusOomegas[1] = 0.0;
    pFusOomegas[2] = 0.0;

   /* pFusOacc_prev    = (double *)malloc(3*sizeof(double));
    pFusOacc_prev[0] = 0.0;
    pFusOacc_prev[1] = 0.0;
    pFusOacc_prev[2] = 0.0;*/

    pFusOacc = (double *)malloc(3 * sizeof(double));
    pFusOacc[0] = 0.0;
    pFusOacc[1] = 0.0;
    pFusOacc[2] = 0.0;

    pFusOalphas = (double *)malloc(3 * sizeof(double));
    pFusOalphas[0] = 0.0;
    pFusOalphas[1] = 0.0;
    pFusOalphas[2] = 0.0;

    pNodeVels   = (double *)malloc(numNodePtElem * sizeof(double));
    pNodeOmegas = (double *)malloc(numNodePtElem * sizeof(double));
    pNodeAccs   = (double *)malloc(numNodePtElem * sizeof(double));
    i = 0;
    for (n = 0; n < numNodePtElem / 3; n++)
    {
        pNodeVels[n * 3] = -50.0;
        pNodeVels[n * 3 + 1] =  0.0;
        pNodeVels[n * 3 + 2] =  0.0;
        pNodeOmegas[n * 3] = 0.0;
        pNodeOmegas[n * 3 + 1] = 0.0;
        pNodeOmegas[n * 3 + 2] = 0.0;
        pNodeAccs[n * 3] = 1.0;
        pNodeAccs[n * 3 + 1] = 2.0;
        pNodeAccs[n * 3 + 2] = 3.0;
    }
    
    // Need to set rotor point velocities and orientations
    for (n = 0; n < numRtrPtsElem; n=n+3)
    {
        pRtrVels[n]     = -50.0;
        pRtrVels[n + 1] = 0.0;
        pRtrVels[n + 2] = 0.0;
        pRtrOmegas[n ] = 0.0;
        pRtrOmegas[n  + 1] = 0.0;
        pRtrOmegas[n  + 2] = 0.0;
        pRtrAccs[n ] = 0.0;
        pRtrAccs[n  + 1] = 0.0;
        pRtrAccs[n  + 2] = 0.0;
        pRtrAlphas[n ] = 0.0;
        pRtrAlphas[n  + 1] = 0.0;
        pRtrAlphas[n  + 2] = 0.0;
    }

    for (i = 0; i < numRtrPtsElem*3; i = i + 9)
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
    KFAST_AssRes(&t, &isInitialTime, pWindPt, pFusO, pFusODCM,  pFusOv, 
       pFusOomegas,  pFusOacc, pFusOalphas, &numNodePts, pNodePts, pNodeDCMs,
                    pNodeVels, pNodeOmegas, pNodeAccs,
                    &numRtrPts, pRtrPts, pRtrDCMs, pRtrVels, pRtrOmegas, pRtrAccs, pRtrAlphas,
                    pNodeLoads, pRtrLoads, &errStat, errMsg);

  

    for (n = 0; n < numNodeLoadsElem; n = n + 6)
    {
        if (fabs(pNodeLoads[n ]) > 0 || fabs(pNodeLoads[n  + 1]) > 0 || fabs(pNodeLoads[n + 2]) > 0 || fabs(pNodeLoads[n +3]) > 0 || fabs(pNodeLoads[n  + 4]) > 0 || fabs(pNodeLoads[n  + 5]) > 0)
        {
            printf("Node %d loads = %10.3e,%10.3e,%10.3e,%10.3e,%10.3e,%10.3e\n", n / 6, pNodeLoads[n], pNodeLoads[n  + 1], pNodeLoads[n + 2], pNodeLoads[n +3], pNodeLoads[n  + 4], pNodeLoads[n + 5]);
        }
    }
    if (errStat != 0)
    {
        printf("%s\n", errMsg);
        return errStat;
    }
    for (n = 0; n < numGaussPtLoadsElem; n = n + 6)
    {
       pGaussPtLoads[n] = 1.0;
       pGaussPtLoads[n+1] = 2.0;
       pGaussPtLoads[n+2] = 3.0;
       pGaussPtLoads[n+3] = 4.0;
       pGaussPtLoads[n+4] = 5.0;
       pGaussPtLoads[n+5] = 6.0;
    }
    // Output()
    KFAST_Output(&t, &numGaussLoadPts, pGaussPtLoads, &errStat, errMsg);
    if (errStat != 0)
    {
        printf("%s\n", errMsg);
        printf("%s\n", "Quitting due to error in KFAST_OUTPUT");
        return errStat;
    }
    t = t + dt;  // MBDyn increments time before AfterPredict()
    
    // AfterPredict()
    KFAST_AfterPredict(&t, &errStat, errMsg);
    if (errStat != 0)
    {
       printf("%s\n", errMsg);
       return errStat;
    }


    //Now we begin calls associated with the time marching loop of MBDyn
    isInitialTime = 0;  // Now we do advance the states for the each call to KFAST_AssRes()
    nFinal = 2;
    for (n = 2; n <= nFinal; n++) 
    {
        

        // Modify positions and velocities and accelerations

        // For now we will simulate constant positions, velocities, and accelerations

        // AssRes()
        KFAST_AssRes(&t, &isInitialTime, pWindPt, pFusO, pFusODCM, pFusOv,
           pFusOomegas, pFusOacc, pFusOalphas, &numNodePts, pNodePts, pNodeDCMs,
           pNodeVels, pNodeOmegas, pNodeAccs,
           &numRtrPts, pRtrPts, pRtrDCMs, pRtrVels, pRtrOmegas, pRtrAccs, pRtrAlphas,
           pNodeLoads, pRtrLoads, &errStat, errMsg);
         

        if (errStat != 0)
        {
            printf("%s\n", errMsg);
            return errStat;
        }

        // Output()
        KFAST_Output(&t, &numGaussPtLoadsElem, pGaussPtLoads, &errStat, errMsg);
        if (errStat != 0)
        {
           printf("%s\n", errMsg);
           return errStat;
        }

        t = t + dt;  // MBDyn increments time before AfterPredict()

       
        // AfterPredict()
        KFAST_AfterPredict(&t, &errStat, errMsg);
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

