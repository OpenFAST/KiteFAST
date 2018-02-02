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
    double dt = 0.1;
    int numFlaps = 2;
    int numPylons = 2;
    int numComp = 10;
    int *pNumCompNds = NULL;
    char KAD_FileName[INTERFACE_STRING_LENGTH];
    char IfW_FileName[INTERFACE_STRING_LENGTH];
    char MD_FileName[INTERFACE_STRING_LENGTH];
    char outFileRoot[INTERFACE_STRING_LENGTH];
    double gravity = 9.81;
    double *pFusODCM;
    int numRtrPtsElem;
    double *pRtrPts;
    int numRefPtElem;
    double *pRefPts;
    int numDCMElem;
    double *pNodeDCMs;
    int errStat;
    char errMsg[INTERFACE_STRING_LENGTH];

    // Set input file names
    strcpy(KAD_FileName, "Kite-m600.inp");
    strcpy(IfW_FileName, "steady-wind.inp");
    strcpy(MD_FileName , "Kite-tethers.inp");
    strcpy(outFileRoot , "KiteTest");


    // Set up the number of nodes per kite component
    pNumCompNds = (int *)malloc(numComp*sizeof(int));
    pNumCompNds[0] = 15; // Fuselage nodes
    pNumCompNds[1] = 9; //  Starboard wing nodes
    pNumCompNds[2] = 9; //  Port wing nodes
    pNumCompNds[3] = 5; //  vertical stabilizer nodes
    pNumCompNds[4] = 4; //  starboard horizontal stabilizer nodes
    pNumCompNds[5] = 4; //  port horizontal stabilizer nodes
    pNumCompNds[6] = 6; //  starboard inboard pylon nodes
    pNumCompNds[7] = 6; //  starboard outboard pylon nodes
    pNumCompNds[8] = 6; //  port inboard pylon nodes
    pNumCompNds[9] = 6; //  port outboard pylon nodes

    //Test the FusODCM as a 1D array instead of a 2D array
    pFusODCM = (double *)malloc(9*sizeof(double));
    pFusODCM[0] = -1.0;
    pFusODCM[1] =  0.0;
    pFusODCM[2] =  0.0;
    pFusODCM[3] =  0.0;
    pFusODCM[4] =  1.0;
    pFusODCM[5] =  0.0;
    pFusODCM[6] =  0.0;
    pFusODCM[7] =  0.0;
    pFusODCM[8] = -1.0;

    // Rotor points
    numRtrPtsElem = numPylons * 4 * 3;
    pRtrPts = (double *)malloc(numRtrPtsElem * sizeof(double));
    // Starboard inner top
    pRtrPts[0] = 104.0;
    pRtrPts[1] = 108.0;
    pRtrPts[2] = 110.0;
    // Starboard inner bottom
    pRtrPts[3] = 104.0;
    pRtrPts[4] = 108.0;
    pRtrPts[5] =  96.0;
    // Starboard outer top
    pRtrPts[6] = 104.0;
    pRtrPts[7] = 115.0;
    pRtrPts[8] = 110.0;
    // Starboard outer bottom
    pRtrPts[9] = 104.0;
    pRtrPts[10] = 115.0;
    pRtrPts[11] =  96.0;
    // Port inner top
    pRtrPts[12] = 104.0;
    pRtrPts[13] =  92.0;
    pRtrPts[14] = 110.0;
    // Port inner bottom
    pRtrPts[15] = 104.0;
    pRtrPts[16] =  92.0;
    pRtrPts[17] = 96.0;
    // Port outer top
    pRtrPts[18] = 104.0;
    pRtrPts[19] =  92.0;
    pRtrPts[20] = 110.0;
    // Port outer bottom
    pRtrPts[21] = 104.0;
    pRtrPts[22] =  92.0;
    pRtrPts[23] = 96.0;



    // Reference points
    numRefPtElem = numComp * 3;
    pRefPts = (double *)malloc(numRefPtElem * sizeof(double));
    for (i = 0; i < numRefPtElem; i++)
    {
        pRefPts[i] = 0.0;
    }
    //Fuselage
    pRefPts[0] = 100.0;
    pRefPts[1] = 100.0;
    pRefPts[2] = 100.0;
    // Starboard Wing
    pRefPts[3] = 105.0;
    pRefPts[4] = 102.0;
    pRefPts[5] = 100.0;
    // Port Wing
    pRefPts[6] = 105.0;
    pRefPts[7] = 98.0;
    pRefPts[8] = 100.0;
    // Vertical Stabilizer


    // nodal DCMs
    numDCMElem = 0;
    for (i = 0; i < numComp; i++) 
    {
        numDCMElem = numDCMElem + 9 * pNumCompNds[i];
    }
    pNodeDCMs = (double *)malloc(numDCMElem * sizeof(double));
    for (i = 0; i < numDCMElem; i++)
    {
        pNodeDCMs[i] = 1.0;
    }
    KFAST_Init(&dt, &numFlaps, &numPylons, &numComp, pNumCompNds, KAD_FileName, IfW_FileName, MD_FileName, outFileRoot, &gravity,
        pFusODCM, &numRtrPtsElem, pRtrPts, &numRefPtElem, pRefPts, &numDCMElem, pNodeDCMs, &errStat, errMsg);

    free(pNumCompNds);
    free(pFusODCM);
    free(pRtrPts);
    free(pRefPts);
    free(pNodeDCMs);


    // Get a handle to the DLL module.

    //hinstLib = LoadLibrary(TEXT("KiteFASTlib.dll"));

    //// If the handle is valid, try to get the function address.

    //if (hinstLib != NULL)
    //{
    //    ProcInit = (KFAST_INITPROC)GetProcAddress(hinstLib, "KFAST_Init");

    //    // If the function address is valid, call the function.

    //    if (NULL != ProcInit)
    //    {
    //        fRunTimeLinkSuccess = TRUE;
    //        (ProcInit)(&dt, &numFlaps, &numPylons, &numComp, numCompNds, KAD_FileName, IfW_FileName, MD_FileName, outFileRoot, &gravity, 
    //            FusODCM, &numRtrPtsElem, rtrPts, &numRefPtElem, refPts, &numDCMElem, nodeDCMs, &errStat, errMsg);
    //    }
    //    // Free the DLL module.

    //    fFreeResult = FreeLibrary(hinstLib);
    //}

    //// If unable to call the DLL function, use an alternative.
    //if (!fRunTimeLinkSuccess)
    //    printf("Message printed from executable\n");

    return 0;

}

