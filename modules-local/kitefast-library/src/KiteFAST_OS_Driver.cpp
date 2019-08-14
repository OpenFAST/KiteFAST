// KiteFAST_Driver.cpp : Defines the entry point for the console application.
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
// #include "stdafx.h"
// #include <windows.h> 
#include "KFAST_Lib.h"
#include "KFAST_OS_Lib.h"

//!typedef void(__cdecl *KFAST_INITPROC)(double *dt, int *numFlaps, int *numPylons, int *numComp, int *numCompNds, const char *KAD_FileName, const char *IfW_FileName, const char *MD_FileName,
//!    const char *outFileRoot, double *gravity, double *FusODCM_c, int *numRtrPtsElem_c, double *rtrPts_c, int *numRefPtElem_c, double *refPts_c, int *numDCMElem_c, double *nodeDCMs_c, int *errStat, char *errMsg);
int PlaceInGlobal(double& pX, double& pY, double& pZ, double refX, double refY, double refZ, double pFusODCM[], double pKiteOffset[])
{
   double pXtemp = pX + refX;
   double pYtemp = pY + refY;
   double pZtemp = pZ + refZ;
   /*pX = pKiteOffset[0] + pXtemp*pFusODCM[0] + pYtemp*pFusODCM[3] + pZtemp*pFusODCM[6];
   pY = pKiteOffset[1] + pXtemp*pFusODCM[1] + pYtemp*pFusODCM[4] + pZtemp*pFusODCM[7];
   pZ = pKiteOffset[2] + pXtemp*pFusODCM[2] + pYtemp*pFusODCM[5] + pZtemp*pFusODCM[8];*/

   pX = pKiteOffset[0] + pXtemp*pFusODCM[0] + pYtemp*pFusODCM[1] + pZtemp*pFusODCM[2];
   pY = pKiteOffset[1] + pXtemp*pFusODCM[3] + pYtemp*pFusODCM[4] + pZtemp*pFusODCM[5];
   pZ = pKiteOffset[2] + pXtemp*pFusODCM[6] + pYtemp*pFusODCM[7] + pZtemp*pFusODCM[8];

   return 0;
}

int Setup_m000_Geom(int& numPylons, int& numComp, double** pFusODCM, double** pKiteOffset, int** pNumCompNds, double** pRefPts, int& numNodePts, double** pNodeDCMs, double **pNodePts, int& numRtrPts, double** pRtrPts, double** pRtrMass, double** pRtrI_Rot, double** pRtrI_Trans, double** pRtrXcm, int& numNodePtElem, int& numRtrPtsElem, int& numGaussPtLoadsElem)
{

   int i;

   numPylons = 1;
   numComp = 8;
   // The kite is aligned with the Global Coordinate system
   *pFusODCM = (double *)malloc(9 * sizeof(double));
   (*pFusODCM)[0] = 0.8610;
   (*pFusODCM)[1] = -0.2349;
   (*pFusODCM)[2] = -0.4511;

   (*pFusODCM)[3] = 0.1192;
   (*pFusODCM)[4] = -0.7690;
   (*pFusODCM)[5] = 0.6280;

   (*pFusODCM)[6] = -0.4944;
   (*pFusODCM)[7] = -0.5945;
   (*pFusODCM)[8] = -0.6341;

   // Offset of kite in global coordinates (m)
   // This offset needs to be added to all the reference points.

   *pKiteOffset = (double *)malloc(3 * sizeof(double));
   (*pKiteOffset)[0] = 126.6398; //0.0; 
   (*pKiteOffset)[1] = -379.6509; // 0.0;
   (*pKiteOffset)[2] = 172.8097; // 100.0;


   // Set up the number of nodes *per kite component
   *pNumCompNds = (int *)malloc(numComp*sizeof(int));
   (*pNumCompNds)[0] = 3; // Fuselage nodes
   (*pNumCompNds)[1] = 3; //  Starboard wing nodes
   (*pNumCompNds)[2] = 3; //  port wing nodes
   (*pNumCompNds)[3] = 3; //  vertical stabilizer nodes
   (*pNumCompNds)[4] = 3; //  starboard horizontal stabilizer nodes
   (*pNumCompNds)[5] = 3; //  port horizontal stabilizer nodes
   (*pNumCompNds)[6] = 3; //  starboard inboard *pylon nodes
   (*pNumCompNds)[7] = 3; //  port inboard pylon nodes


   // Reference *points  specified in the Kite Coordinate System
   int numRefPtElem = numComp * 3;
   *pRefPts = (double *)malloc(numRefPtElem * sizeof(double));

   for (i = 0; i < numRefPtElem; i++)
   {
      (*pRefPts)[i] = 0.0;
   }
   int c = 0;

   //Fuselage in global 
   (*pRefPts)[c + 0] = (*pKiteOffset)[0];
   (*pRefPts)[c + 1] = (*pKiteOffset)[1];
   (*pRefPts)[c + 2] = (*pKiteOffset)[2];
   c = c + 3;

   // Starboard Wing in kite coords
   (*pRefPts)[c + 0] = 0.0;
   (*pRefPts)[c + 1] = 0.5;
   (*pRefPts)[c + 2] = 0.0;
   c = c + 3;

   // *port Wing in kite coords
   (*pRefPts)[c + 0] = 0.0;
   (*pRefPts)[c + 1] = -0.50;
   (*pRefPts)[c + 2] = 0.0;
   c = c + 3;

   // Vertical Stabilizer in kite coords
   (*pRefPts)[c + 0] = -5.1;
   (*pRefPts)[c + 1] = 0.0;
   (*pRefPts)[c + 2] = -2.0;
   c = c + 3;

   // Starboard Horizontal Stabilizer in kite coords
   (*pRefPts)[c + 0] = -5.1;
   (*pRefPts)[c + 1] = 0.1;
   (*pRefPts)[c + 2] = -2.0;
   c = c + 3;

   // *port Horizontal Stabilizer in kite coords
   (*pRefPts)[c + 0] = -5.1;
   (*pRefPts)[c + 1] = -0.1;
   (*pRefPts)[c + 2] = -2.0;
   c = c + 3;

   // Starboard *pylons in kite coords
   (*pRefPts)[c + 0] = 0.1;
   (*pRefPts)[c + 1] = 3.0;
   (*pRefPts)[c + 2] = -0.5;
   c = c + 3;

   /* (*pRefPts)[c + 0] = 0.857 ;
   (*pRefPts)[c + 1] = 3.5    ;
   (*pRefPts)[c + 2] = 0.0    ;
   c = c + 3;*/

   // *port *pylons in kite coords
   (*pRefPts)[c + 0] = 0.1;
   (*pRefPts)[c + 1] = -3.0;
   (*pRefPts)[c + 2] = -0.5;
   c = c + 3;

   /* (*pRefPts)[c + 0] = 0.857 ;
   (*pRefPts)[c + 1] = -3.5;
   (*pRefPts)[c + 2] = 0.0 ;
   c = c + 3;*/

   // nodal DCMs
   int numDCMElem = 0;
   numNodePts = 0;
   numNodePtElem = 0;
   numGaussPtLoadsElem = 0;

   for (i = 0; i < numComp; i++)
   {
      numDCMElem = numDCMElem + 9 * (*pNumCompNds)[i];
      numNodePts = numNodePts + (*pNumCompNds)[i];
      numNodePtElem = numNodePtElem + 3 * (*pNumCompNds)[i];
      numGaussPtLoadsElem = numGaussPtLoadsElem + 6 * ((*pNumCompNds)[i] - 1);
   }


   *pNodeDCMs = (double *)malloc(numDCMElem * sizeof(double));
   *pNodePts = (double *)malloc(numNodePtElem * sizeof(double));


   // Set all DCMs to FusO DCM
   for (i = 0; i < numDCMElem; i = i + 9)
   {
      (*pNodeDCMs)[i] = (*pFusODCM)[0];
      (*pNodeDCMs)[i + 1] = (*pFusODCM)[1];
      (*pNodeDCMs)[i + 2] = (*pFusODCM)[2];
      (*pNodeDCMs)[i + 3] = (*pFusODCM)[3];
      (*pNodeDCMs)[i + 4] = (*pFusODCM)[4];
      (*pNodeDCMs)[i + 5] = (*pFusODCM)[5];
      (*pNodeDCMs)[i + 6] = (*pFusODCM)[6];
      (*pNodeDCMs)[i + 7] = (*pFusODCM)[7];
      (*pNodeDCMs)[i + 8] = (*pFusODCM)[8];
   }

   int n = 0;

   // The node *positions are in Global, but the (*pRefPts) are in Kite, (*pKiteOffset) is in Global


   // Fuselage node *positions
   (*pNodePts)[0] = -5.000 + (*pKiteOffset)[0];
   (*pNodePts)[1] = 0.000 + (*pKiteOffset)[1];
   (*pNodePts)[2] = 0.000 + (*pKiteOffset)[2];
   (*pNodePts)[3] = 0.000 + (*pKiteOffset)[0];
   (*pNodePts)[4] = 0.000 + (*pKiteOffset)[1];
   (*pNodePts)[5] = 0.000 + (*pKiteOffset)[2];
   (*pNodePts)[6] = 5.000 + (*pKiteOffset)[0];
   (*pNodePts)[7] = 0.000 + (*pKiteOffset)[1];
   (*pNodePts)[8] = 0.000 + (*pKiteOffset)[2];
   c = 9;
   //  Starboard wing nodes
   n = 3;
   (*pNodePts)[c + 0] = 0.000 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 1] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 2] = 0.000 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 3] = 0.000 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 4] = 2.500 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 5] = 0.000 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 6] = 0.000 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 7] = 5.500 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 8] = 0.000 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   n = n + 3;
   c = c + 9;
   //  *port wing nodes
   (*pNodePts)[c + 0] = 0.000 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 1] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 2] = 0.000 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 3] = 0.000 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 4] = -2.5 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 5] = 0.0 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 6] = 0.000 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 7] = -5.5 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 8] = 0.0 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   n = n + 3;
   c = c + 9;
   //  vertical stabilizer nodes
   (*pNodePts)[c + 0] = 0.000 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 1] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 2] = 0.000 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 3] = 0.000 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 4] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 5] = -1 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 6] = 0.00 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 7] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 8] = -2 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   n = n + 3;
   c = c + 9;
   //  starboard horizontal stabilizer nodes
   (*pNodePts)[c + 0] = 0.000 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 1] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 2] = 0.000 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 3] = 0.0 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 4] = .4 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 5] = 0.000 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 6] = 0.0 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 7] = .9 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 8] = 0.000 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   n = n + 3;
   c = c + 9;
   //  *port horizontal stabilizer nodes
   (*pNodePts)[c + 0] = 0.000 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 1] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 2] = 0.000 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 3] = 0 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 4] = -.4 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 5] = 0.000 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 6] = 0 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 7] = -.9 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 8] = 0.000 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   n = n + 3;
   c = c + 9;
   //  starboard inboard *pylon nodes
   (*pNodePts)[c + 0] = -(*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 1] = +(*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 2] = -(*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 3] = -(*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 4] = +(*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 5] = -0.5 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 6] = -(*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 7] = +(*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 8] = -1 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   n = n + 3;
   c = c + 9;
   //  starboard outboard *pylon nodes
   /* (*pNodePts)[c + 0] = -0.729 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 1] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 2] = 1.470 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 3] = -0.510 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 4] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 5] = -1.832 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   n = n + 3;
   c = c + 6;*/
   //  *port inboard *pylon nodes
   (*pNodePts)[c + 0] = -(*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 1] = +(*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 2] = -(*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 3] = -(*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 4] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 5] = -0.5 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 6] = -(*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 7] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 8] = -1 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   n = n + 3;
   c = c + 9;

   /*port outboard *pylon nodes
   (*pNodePts)[c + 0] = -0.729 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 1] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 2] = 1.470 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   (*pNodePts)[c + 3] = -0.510 - (*pRefPts)[n + 0] + (*pKiteOffset)[0];
   (*pNodePts)[c + 4] = 0.000 + (*pRefPts)[n + 1] + (*pKiteOffset)[1];
   (*pNodePts)[c + 5] = -1.832 - (*pRefPts)[n + 2] + (*pKiteOffset)[2];
   */

   // Rotor reference *points and associated nacelle quantities
   numRtrPts = numPylons * 4;
   numRtrPtsElem = numRtrPts * 3;
   *pRtrPts = (double *)malloc(numRtrPtsElem * sizeof(double));


   *pRtrMass = (double *)malloc(numRtrPts     * sizeof(double));
   *pRtrI_Rot = (double *)malloc(numRtrPts     * sizeof(double));
   *pRtrI_Trans = (double *)malloc(numRtrPts     * sizeof(double));
   *pRtrXcm = (double *)malloc(numRtrPts     * sizeof(double));

   // Starboard inner top in global
   (*pRtrPts)[0] = -1.861 + (*pKiteOffset)[0];
   (*pRtrPts)[1] = 1.213 + (*pKiteOffset)[1];
   (*pRtrPts)[2] = 1.221 + (*pKiteOffset)[2];
   // Starboard inner bottom
   (*pRtrPts)[3] = -1.515 + (*pKiteOffset)[0];
   (*pRtrPts)[4] = 1.213 + (*pKiteOffset)[1];
   (*pRtrPts)[5] = -1.593 + (*pKiteOffset)[2];
   //// Starboard outer top
   //pRtrPts[6] = -1.861 + (*pKiteOffset)[0];
   //pRtrPts[7] = 3.640 + (*pKiteOffset)[1];
   //pRtrPts[8] = 1.221 + (*pKiteOffset)[2];
   //// Starboard outer bottom
   //pRtrPts[9] = -1.515 + (*pKiteOffset)[0];
   //pRtrPts[10] = 3.640 + (*pKiteOffset)[1];
   //pRtrPts[11] = -1.593 + (*pKiteOffset)[2];
   // *port inner top
   (*pRtrPts)[6] = -1.861 + (*pKiteOffset)[0];
   (*pRtrPts)[7] = -1.213 + (*pKiteOffset)[1];
   (*pRtrPts)[8] = 1.221 + (*pKiteOffset)[2];
   // *port inner bottom
   (*pRtrPts)[9] = -1.515 + (*pKiteOffset)[0];
   (*pRtrPts)[10] = -1.213 + (*pKiteOffset)[1];
   (*pRtrPts)[11] = -1.593 + (*pKiteOffset)[2];
   //// *port outer top
   //pRtrPts[18] = -1.861 + (*pKiteOffset)[0];
   //pRtrPts[19] = -3.639 + (*pKiteOffset)[1];
   //pRtrPts[20] = 1.221 + (*pKiteOffset)[2];
   //// *port outer bottom
   //pRtrPts[21] = -1.515 + (*pKiteOffset)[0];
   //pRtrPts[22] = -3.639 + (*pKiteOffset)[1];
   //pRtrPts[23] = -1.593 + (*pKiteOffset)[2];

   //Set all the rotor masses and inertias and CM offsets
   for (i = 0; i < numRtrPts; i++)
   {
      (*pRtrMass)[i] = 20.0;
      (*pRtrI_Rot)[i] = 200.0;
      (*pRtrI_Trans)[i] = 15.0;
      (*pRtrXcm)[i] = 0.1;
   }

   return 0;
}


int Setup_m600_Geom(int& numPylons, int& numComp, double** pFusODCM, double** pKiteOffset, int** pNumCompNds, double** pRefPts, int& numNodePts, double** pNodeDCMs, double **pNodePts, int& numRtrPts, double** pRtrPts, double** pRtrMass, double** pRtrI_Rot, double** pRtrI_Trans, double** pRtrXcm, int& numNodePtElem, int& numRtrPtsElem, int& numGaussPtLoadsElem)
{

   int i;

   numPylons = 2;
   numComp = 10;
   // The kite is aligned with the Global Coordinate system
   *pFusODCM = (double *)malloc(9 * sizeof(double));
   (*pFusODCM)[0] = 0.8610;
   (*pFusODCM)[1] = -0.2349;
   (*pFusODCM)[2] = -0.4511;

   (*pFusODCM)[3] = 0.1192;
   (*pFusODCM)[4] = -0.7690;
   (*pFusODCM)[5] = 0.6280;

   (*pFusODCM)[6] = -0.4944;
   (*pFusODCM)[7] = -0.5945;
   (*pFusODCM)[8] = -0.6341;

   // Offset of kite in global coordinates (m)
   // This offset needs to be added to all the reference points.

   *pKiteOffset = (double *)malloc(3 * sizeof(double));
   (*pKiteOffset)[0] = 126.6398; //0.0; 
   (*pKiteOffset)[1] = -379.6509; // 0.0;
   (*pKiteOffset)[2] = 172.8097; // 100.0;


   // Set up the number of nodes *per kite component
   *pNumCompNds = (int *)malloc(numComp*sizeof(int));
   (*pNumCompNds)[0] = 3; // Fuselage nodes
   (*pNumCompNds)[1] = 5; //  Starboard wing nodes
   (*pNumCompNds)[2] = 5; //  port wing nodes
   (*pNumCompNds)[3] = 3; //  vertical stabilizer nodes
   (*pNumCompNds)[4] = 5; //  starboard horizontal stabilizer nodes
   (*pNumCompNds)[5] = 5; //  port horizontal stabilizer nodes
   (*pNumCompNds)[6] = 3; //  starboard inboard pylon nodes
   (*pNumCompNds)[7] = 3; //  starboard outboard pylon nodes
   (*pNumCompNds)[8] = 3; //  port inboard pylon nodes
   (*pNumCompNds)[9] = 3; //  port outboard pylon nodes


   // Reference *points  specified in the Kite Coordinate System
   int numRefPtElem = numComp * 3;
   *pRefPts = (double *)malloc(numRefPtElem * sizeof(double));

   for (i = 0; i < numRefPtElem; i++)
   {
      (*pRefPts)[i] = 0.0;
   }
   int c = 0;

   //Fuselage in kite coords (temporarily) for transformation work to follow 
   (*pRefPts)[c + 0] = 0.0;
   (*pRefPts)[c + 1] = 0.0;
   (*pRefPts)[c + 2] = 0.0;
   c = c + 3;

   // Starboard Wing in kite coords
   (*pRefPts)[c + 0] = 0.0;
   (*pRefPts)[c + 1] = 0.5;
   (*pRefPts)[c + 2] = 0.0;
   c = c + 3;

   // *port Wing in kite coords
   (*pRefPts)[c + 0] = 0.0;
   (*pRefPts)[c + 1] = -0.50;
   (*pRefPts)[c + 2] = 0.0;
   c = c + 3;

   // Vertical Stabilizer in kite coords
   (*pRefPts)[c + 0] = -6.891;
   (*pRefPts)[c + 1] = 0.0;
   (*pRefPts)[c + 2] = 0.0;
   c = c + 3;

   // Starboard Horizontal Stabilizer in kite coords
   (*pRefPts)[c + 0] = -6.555;
   (*pRefPts)[c + 1] = 0.0;
   (*pRefPts)[c + 2] = 0.817;
   c = c + 3;

   // *port Horizontal Stabilizer in kite coords
   (*pRefPts)[c + 0] = -6.555;
   (*pRefPts)[c + 1] = 0.0;
   (*pRefPts)[c + 2] = 0.817;
   c = c + 3;

   // Starboard *pylons in kite coords
   (*pRefPts)[c + 0] = 0.857;
   (*pRefPts)[c + 1] = 1.060;
   (*pRefPts)[c + 2] = 0.0;
   c = c + 3;

   (*pRefPts)[c + 0] = 0.857;
   (*pRefPts)[c + 1] = 3.486;
   (*pRefPts)[c + 2] = 0.0;
   c = c + 3;

   // *port *pylons in kite coords
   (*pRefPts)[c + 0] = 0.857;
   (*pRefPts)[c + 1] = -1.367;
   (*pRefPts)[c + 2] = 0.0;
   c = c + 3;

   (*pRefPts)[c + 0] = 0.857;
   (*pRefPts)[c + 1] = -3.793;
   (*pRefPts)[c + 2] = 0.0;
   c = c + 3;

   // nodal DCMs
   int numDCMElem = 0;
   numNodePts = 0;
   numNodePtElem = 0;
   numGaussPtLoadsElem = 0;

   for (i = 0; i < numComp; i++)
   {
      numDCMElem = numDCMElem + 9 * (*pNumCompNds)[i];
      numNodePts = numNodePts + (*pNumCompNds)[i];
      numNodePtElem = numNodePtElem + 3 * (*pNumCompNds)[i];
      numGaussPtLoadsElem = numGaussPtLoadsElem + 6 * ((*pNumCompNds)[i] - 1);
   }

   // Set all DCMs to FusO DCM
   *pNodeDCMs = (double *)malloc(numDCMElem * sizeof(double));
   for (i = 0; i < numDCMElem; i = i + 9)
   {
      (*pNodeDCMs)[i] = (*pFusODCM)[0];
      (*pNodeDCMs)[i + 1] = (*pFusODCM)[1];
      (*pNodeDCMs)[i + 2] = (*pFusODCM)[2];
      (*pNodeDCMs)[i + 3] = (*pFusODCM)[3];
      (*pNodeDCMs)[i + 4] = (*pFusODCM)[4];
      (*pNodeDCMs)[i + 5] = (*pFusODCM)[5];
      (*pNodeDCMs)[i + 6] = (*pFusODCM)[6];
      (*pNodeDCMs)[i + 7] = (*pFusODCM)[7];
      (*pNodeDCMs)[i + 8] = (*pFusODCM)[8];
   }


   *pNodePts = (double *)malloc(numNodePtElem * sizeof(double));
   int n = 0;

   // The node *positions are in Global, but the (*pRefPts) are in Kite, (*pKiteOffset) is in Global

   for (i = 0; i < numNodePtElem; i++)
   {
      (*pNodePts)[i] = 0.0;
   }

   // Fuselage node positions in kite coordinates relative to component reference point:  (0,0,0) for fuselage
   n = 0;
   double fusXloc[] = { 0.0, -1.553, -6.917 };
   for (i = 0; i < 3 * (*pNumCompNds)[0]; i = i + 3)
   {
      (*pNodePts)[i] = fusXloc[n];
      double x = (*pNodePts)[i];
      double y = (*pNodePts)[i + 1];
      double z = (*pNodePts)[i + 2];
      int result = PlaceInGlobal(x, y, z, (*pRefPts)[0], (*pRefPts)[1], (*pRefPts)[2], *pFusODCM, *pKiteOffset);
      (*pNodePts)[i] = x;
      (*pNodePts)[i + 1] = y;
      (*pNodePts)[i + 2] = z;
      n++;
   }

   // Starboard wing node positions in kite coordinates relative to component reference point
   n = 0;
   c = 3 * (*pNumCompNds)[0];
   double yloc[] = { 0.0, 1.060, 3.486, 6.432, 12.831 };
   for (i = 0; i < 3 * (*pNumCompNds)[1]; i = i + 3)
   {
      (*pNodePts)[c + i + 1] = yloc[n];
      double x = (*pNodePts)[c + i];
      double y = (*pNodePts)[c + i + 1];
      double z = (*pNodePts)[c + i + 2];
      int result = PlaceInGlobal(x, y, z, (*pRefPts)[3], (*pRefPts)[4], (*pRefPts)[5], *pFusODCM, *pKiteOffset);
      (*pNodePts)[c + i] = x;
      (*pNodePts)[c + i + 1] = y;
      (*pNodePts)[c + i + 2] = z;
      n++;
   }

   // Port wing node positions in kite coordinates relative to component reference point
   n = 0;
   c = c + 3 * (*pNumCompNds)[1];
   double yloc2[] = { 0.0, -1.060, -3.486, -6.432, -12.831 };
   for (i = 0; i < 3 * (*pNumCompNds)[2]; i = i + 3)
   {
      (*pNodePts)[c + i + 1] = yloc2[n];
      double x = (*pNodePts)[c + i];
      double y = (*pNodePts)[c + i + 1];
      double z = (*pNodePts)[c + i + 2];
      int result = PlaceInGlobal(x, y, z, (*pRefPts)[6], (*pRefPts)[7], (*pRefPts)[8], *pFusODCM, *pKiteOffset);
      (*pNodePts)[c + i] = x;
      (*pNodePts)[c + i + 1] = y;
      (*pNodePts)[c + i + 2] = z;
      n++;
   }

   // Vertical stab node positions in kite coordinates relative to component reference point
   n = 0;
   c = c + 3 * (*pNumCompNds)[2];
   double zloc[] = { -2.850, 0.0, 0.712 };
   for (i = 0; i < 3 * (*pNumCompNds)[3]; i = i + 3)
   {
      (*pNodePts)[c + i + 1] = zloc[n];
      double x = (*pNodePts)[c + i];
      double y = (*pNodePts)[c + i + 1];
      double z = (*pNodePts)[c + i + 2];
      int result = PlaceInGlobal(x, y, z, (*pRefPts)[9], (*pRefPts)[10], (*pRefPts)[11], *pFusODCM, *pKiteOffset);
      (*pNodePts)[c + i] = x;
      (*pNodePts)[c + i + 1] = y;
      (*pNodePts)[c + i + 2] = z;
      n++;
   }


   // Starboard stab node positions in kite coordinates relative to component reference point
   n = 0;
   c = c + 3 * (*pNumCompNds)[3];
   double yloc3[] = { 0.0, 0.5, 1.0, 1.724, 2.447 };
   for (i = 0; i < 3 * (*pNumCompNds)[4]; i = i + 3)
   {
      (*pNodePts)[c + i + 1] = yloc3[n];
      double x = (*pNodePts)[c + i];
      double y = (*pNodePts)[c + i + 1];
      double z = (*pNodePts)[c + i + 2];
      int result = PlaceInGlobal(x, y, z, (*pRefPts)[12], (*pRefPts)[13], (*pRefPts)[14], *pFusODCM, *pKiteOffset);
      (*pNodePts)[c + i] = x;
      (*pNodePts)[c + i + 1] = y;
      (*pNodePts)[c + i + 2] = z;
      n++;
   }


   // Port stab node positions in kite coordinates relative to component reference point
   n = 0;
   c = c + 3 * (*pNumCompNds)[4];
   double yloc4[] = { 0.0, -0.5, -1.0, -1.724, -2.447 };
   for (i = 0; i < 3 * (*pNumCompNds)[5]; i = i + 3)
   {
      (*pNodePts)[c + i + 1] = yloc4[n];
      double x = (*pNodePts)[c + i];
      double y = (*pNodePts)[c + i + 1];
      double z = (*pNodePts)[c + i + 2];
      int result = PlaceInGlobal(x, y, z, (*pRefPts)[15], (*pRefPts)[16], (*pRefPts)[17], *pFusODCM, *pKiteOffset);
      (*pNodePts)[c + i] = x;
      (*pNodePts)[c + i + 1] = y;
      (*pNodePts)[c + i + 2] = z;
      n++;
   }

   // Starboard inner pylon node positions in kite coordinates relative to component reference point
   n = 0;
   c = c + 3 * (*pNumCompNds)[5];
   double zloc2[] = { -1.470, -0.120, 1.832 };
   for (i = 0; i < 3 * (*pNumCompNds)[6]; i = i + 3)
   {
      (*pNodePts)[c + i + 1] = zloc2[n];
      double x = (*pNodePts)[c + i];
      double y = (*pNodePts)[c + i + 1];
      double z = (*pNodePts)[c + i + 2];
      int result = PlaceInGlobal(x, y, z, (*pRefPts)[18], (*pRefPts)[19], (*pRefPts)[20], *pFusODCM, *pKiteOffset);
      (*pNodePts)[c + i] = x;
      (*pNodePts)[c + i + 1] = y;
      (*pNodePts)[c + i + 2] = z;
      n++;
   }

   // Starboard inner pylon node positions in kite coordinates relative to component reference point
   n = 0;
   c = c + 3 * (*pNumCompNds)[6];
   double zloc3[] = { -1.470, -0.120, 1.832 };
   for (i = 0; i < 3 * (*pNumCompNds)[7]; i = i + 3)
   {
      (*pNodePts)[c + i + 1] = zloc3[n];
      double x = (*pNodePts)[c + i];
      double y = (*pNodePts)[c + i + 1];
      double z = (*pNodePts)[c + i + 2];
      int result = PlaceInGlobal(x, y, z, (*pRefPts)[21], (*pRefPts)[22], (*pRefPts)[23], *pFusODCM, *pKiteOffset);
      (*pNodePts)[c + i] = x;
      (*pNodePts)[c + i + 1] = y;
      (*pNodePts)[c + i + 2] = z;
      n++;
   }

   // Port inner pylon node positions in kite coordinates relative to component reference point
   n = 0;
   c = c + 3 * (*pNumCompNds)[7];
   double zloc4[] = { -1.470, -0.120, 1.832 };
   for (i = 0; i < 3 * (*pNumCompNds)[8]; i = i + 3)
   {
      (*pNodePts)[c + i + 1] = zloc4[n];
      double x = (*pNodePts)[c + i];
      double y = (*pNodePts)[c + i + 1];
      double z = (*pNodePts)[c + i + 2];
      int result = PlaceInGlobal(x, y, z, (*pRefPts)[24], (*pRefPts)[25], (*pRefPts)[26], *pFusODCM, *pKiteOffset);
      (*pNodePts)[c + i] = x;
      (*pNodePts)[c + i + 1] = y;
      (*pNodePts)[c + i + 2] = z;
      n++;
   }

   // Port inner pylon node positions in kite coordinates relative to component reference point
   n = 0;
   c = c + 3 * (*pNumCompNds)[8];
   double zloc5[] = { -1.470, -0.120, 1.832 };
   for (i = 0; i < 3 * (*pNumCompNds)[9]; i = i + 3)
   {
      (*pNodePts)[c + i + 1] = zloc5[n];
      double x = (*pNodePts)[c + i];
      double y = (*pNodePts)[c + i + 1];
      double z = (*pNodePts)[c + i + 2];
      int result = PlaceInGlobal(x, y, z, (*pRefPts)[27], (*pRefPts)[28], (*pRefPts)[29], *pFusODCM, *pKiteOffset);
      (*pNodePts)[c + i] = x;
      (*pNodePts)[c + i + 1] = y;
      (*pNodePts)[c + i + 2] = z;
      n++;
   }

   //// Place into global coordinates
   //for (i = 0; i < numNodePtElem; i = i + 3)
   //{
   //   double x = (*pNodePts)[i];
   //   double y = (*pNodePts)[i + 1]; 
   //   double z = (*pNodePts)[i + 2];
   //   int result = PlaceInGlobal(x, y, z, (*pRefPts)[0], (*pRefPts)[1], (*pRefPts)[2], *pFusODCM, *pKiteOffset);
   //   (*pNodePts)[i    ] = x;
   //   (*pNodePts)[i + 1] = y;
   //   (*pNodePts)[i + 2] = z;
   //}

   //Set Fuselage ref point in global 
   (*pRefPts)[0] = (*pKiteOffset)[0];
   (*pRefPts)[1] = (*pKiteOffset)[1];
   (*pRefPts)[2] = (*pKiteOffset)[2];

   // Rotor reference *points and associated nacelle quantities
   numRtrPts = numPylons * 4;
   numRtrPtsElem = numRtrPts * 3;
   *pRtrPts = (double *)malloc(numRtrPtsElem * sizeof(double));


   *pRtrMass = (double *)malloc(numRtrPts     * sizeof(double));
   *pRtrI_Rot = (double *)malloc(numRtrPts     * sizeof(double));
   *pRtrI_Trans = (double *)malloc(numRtrPts     * sizeof(double));
   *pRtrXcm = (double *)malloc(numRtrPts     * sizeof(double));

   // Starboard inner top in global
   (*pRtrPts)[0] = 1.861;
   (*pRtrPts)[1] = 1.213;
   (*pRtrPts)[2] = -1.221;
   // Starboard inner bottom
   (*pRtrPts)[3] = 1.515;
   (*pRtrPts)[4] = 1.213;
   (*pRtrPts)[5] = 1.593;
   // Starboard outer top
   (*pRtrPts)[6] = 1.861;
   (*pRtrPts)[7] = 3.640;
   (*pRtrPts)[8] = -1.221;
   // Starboard outer bottom
   (*pRtrPts)[9] = 1.515;
   (*pRtrPts)[10] = 3.640;
   (*pRtrPts)[11] = 1.593;
   // port inner top
   (*pRtrPts)[12] = 1.861;
   (*pRtrPts)[13] = -1.213;
   (*pRtrPts)[14] = -1.221;
   // port inner bottom
   (*pRtrPts)[15] = 1.515;
   (*pRtrPts)[16] = -1.213;
   (*pRtrPts)[17] = 1.593;
   // port outer top
   (*pRtrPts)[18] = 1.861;
   (*pRtrPts)[19] = -3.639;
   (*pRtrPts)[20] = -1.221;
   // port outer bottom
   (*pRtrPts)[21] = 1.515;
   (*pRtrPts)[22] = -3.639;
   (*pRtrPts)[23] = 1.593;

   // Place into global coordinates
   for (i = 0; i < numRtrPtsElem; i = i + 3)
   {
      double x = (*pRtrPts)[i];
      double y = (*pRtrPts)[i + 1];
      double z = (*pRtrPts)[i + 2];
      int result = PlaceInGlobal(x, y, z, 0.0, 0.0, 0.0, *pFusODCM, *pKiteOffset);
      (*pRtrPts)[i] = x;
      (*pRtrPts)[i + 1] = y;
      (*pRtrPts)[i + 2] = z;
   }

   //Set all the rotor masses and inertias and CM offsets
   for (i = 0; i < numRtrPts; i++)
   {
      (*pRtrMass)[i] = 7.7;
      (*pRtrI_Rot)[i] = 1.61;
      (*pRtrI_Trans)[i] = 0.805;
      (*pRtrXcm)[i] = 0.0;
   }

   return 0;
}
int main(int argc, char *argv[])
{
//    HINSTANCE hinstLib;
//    KFAST_INITPROC ProcInit;
//    BOOL fFreeResult, fRunTimeLinkSuccess = FALSE;
    int i;
    int c;
  
    double dt = 0.001;
    int isInitialTime;
    int nFinal;          // Last time increment for this simulation
    int numFlaps = 3;
    int numPylons;
    int numComp;
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
    
    int result = Setup_m600_Geom(numPylons, numComp, &pFusODCM, &pKiteOffset, &pNumCompNds, &pRefPts, numNodePts, &pNodeDCMs, &pNodePts, numRtrPts, &pRtrPts, &pRtrMass, &pRtrI_Rot, &pRtrI_Trans, &pRtrXcm, numNodePtElem, numRtrPtsElem, numGaussPtLoadsElem);

    
    // Set the ground station point where the wind is measured
    pWindPt = (double *)malloc(3 * sizeof(double));
    pWindPt[0] = 100.0;
    pWindPt[1] = 20.0;
    pWindPt[2] = 0.0;

    

    //Setup Offshore simulation data
    char MD_Mooring_FileName[INTERFACE_STRING_LENGTH];
    char HD_FileName[INTERFACE_STRING_LENGTH];
    int  simMod = 2;  //3 = Offshore, no kite simulation model
    double tmax = 60.0;
    
    double* pWindPtVel = (double *)malloc(3 * sizeof(double));
   
    double* pPtfmO = (double *)malloc(3 * sizeof(double));
    double* pPtfmODCM = (double *)malloc(9 * sizeof(double));
    double* pPtfmOv = (double *)malloc(3 * sizeof(double));
    double* pPtfmOomegas = (double *)malloc(3 * sizeof(double));
    double* pPtfmOacc = (double *)malloc(3 * sizeof(double));
    double* pPtfmOalphas = (double *)malloc(3 * sizeof(double));

    double* pPtfmIMUPt = (double *)malloc(3 * sizeof(double));
    double* pPtfmIMUDCM = (double *)malloc(3 * sizeof(double));
    double* pPtfmIMUv = (double *)malloc(3 * sizeof(double));
    double* pPtfmIMUomegas = (double *)malloc(3 * sizeof(double));
    double* pPtfmIMUacc = (double *)malloc(3 * sizeof(double));

    double* pGSRefPt = (double *)malloc(3 * sizeof(double));
    double* pGSRefDCM = (double *)malloc(9 * sizeof(double));
    double* pGSRefv = (double *)malloc(3 * sizeof(double));
    double* pGSRefomegas = (double *)malloc(3 * sizeof(double));
    double* pGSRefacc = (double *)malloc(3 * sizeof(double));
    double* pPtfmLoads = (double *)malloc(6 * sizeof(double));

    // Set module flags 0 = off, 1=on
    pModFlags = (int *)malloc(6 * sizeof(int));
    pModFlags[0] = 1;  // use KAD module
    pModFlags[1] = 1;  // use InflowWind module
    pModFlags[2] = 1;  // use MoorDyn tether module
    pModFlags[3] = 0;  // no KiteFAST controller
    pModFlags[4] = 1;  // use HydroDyn module
    pModFlags[5] = 1;  // use MoorDyn mooring module

    // Set input file names
    // NOTE: All the data further below is directly tied to the KAD file listed here.
    strcpy(outFileRoot, "D:\\DEV\\makani\\google-repo\\sandbox\\glue-codes\\kitefast\\test_cases\\5MW_OC3Spar_DLL_WTurb_WavesIrr\\KiteOSTest");
    strcpy(KAD_FileName, "D:\\DEV\\makani\\google-repo\\sandbox\\glue-codes\\kitefast\\test_cases\\kiteaerodyn\\simple_m600_model.inp");
    strcpy(IfW_FileName, "D:\\DEV\\makani\\google-repo\\sandbox\\glue-codes\\kitefast\\test_cases\\kiteinflow\\kiteInflowWind.dat");
    strcpy(MD_FileName, "D:\\DEV\\makani\\google-repo\\sandbox\\glue-codes\\kitefast\\test_cases\\kitemooring\\m600-MoorDyn_coarse.dat");
    strcpy(KFC_FileName, "D:\\DEV\\makani\\google-repo\\sandbox\\glue-codes\\kitefast\\test_cases\\m600\\libkitefastcontroller_controller.so");
    strcpy(MD_Mooring_FileName, "D:\\DEV\\makani\\google-repo\\sandbox\\glue-codes\\kitefast\\test_cases\\kitemooring\\moordyn_mooring.inp");
    strcpy(HD_FileName,         "D:\\DEV\\makani\\google-repo\\sandbox\\glue-codes\\kitefast\\test_cases\\hydrodyn\\NRELOffshrBsline5MW_OC3Hywind_HydroDyn.dat");
    // strcpy(outFileRoot, "/Users/rmudafor/Development/makani/makani_openfast/glue-codes/kitefast/test_cases/5MW_OC3Spar_DLL_WTurb_WavesIrr/KiteOSTest");
    // strcpy(KAD_FileName, "/Users/rmudafor/Development/makani/makani_openfast/glue-codes/kitefast/test_cases/m000/simple_m000_model_AD.txt");
    // strcpy(IfW_FileName, "/Users/rmudafor/Development/makani/makani_openfast/glue-codes/kitefast/test_cases/m000/kiteInflowWind.dat");
    // strcpy(MD_FileName, "/Users/rmudafor/Development/makani/makani_openfast/glue-codes/kitefast/test_cases/m000/kiteTether.dat");
    // strcpy(KFC_FileName, "/Users/rmudafor/Development/makani/makani_openfast/glue-codes/kitefast/test_cases/m000/libkitefastcontroller_controller.so");
    // strcpy(MD_Mooring_FileName, "/Users/rmudafor/Development/makani/makani_openfast/glue-codes/kitefast/test_cases/5MW_OC3Spar_DLL_WTurb_WavesIrr/NRELOffshrBsline5MW_OC3Hywind_MoorDyn.dat");
    // strcpy(HD_FileName, "/Users/rmudafor/Development/makani/makani_openfast/glue-codes/kitefast/test_cases/5MW_OC3Spar_DLL_WTurb_WavesIrr/NRELOffshrBsline5MW_OC3Hywind_HydroDyn.dat");
    pPtfmO[0] = 0.0;
    pPtfmO[1] = 0.0;
    pPtfmO[2] = 0.0;

    for (i = 0; i < 9; i++)
    {
       pPtfmODCM[i] = 0.0;
       pPtfmIMUDCM[i] = 0.0;
       pGSRefDCM[i] = 0.0;

    }

    pPtfmODCM[0]   = 1.0;
    pPtfmIMUDCM[0] = 1.0;
    pGSRefDCM[0]   = 1.0;
    pPtfmODCM[4]   = 1.0;
    pPtfmIMUDCM[4] = 1.0;
    pGSRefDCM[4] = 1.0;
    pPtfmODCM[8] = 1.0;
    pPtfmIMUDCM[8] = 1.0;
    pGSRefDCM[8] = 1.0;

    
    pGSRefPt[0] = 0.0;
    pGSRefPt[1] = 0.0;
    pGSRefPt[2] = 2.0;

    for (i = 0; i < 3; i++)
    {
       pWindPtVel[i] = 0.0;
       pPtfmOv[i] = 0.0;
       pPtfmOomegas[i] = 0.0;
       pPtfmOacc[i] = 0.0;
       pPtfmOalphas[i] = 0.0;     
       pPtfmIMUPt[i] = 0.0;
       pPtfmIMUv[i] = 0.0;
       pPtfmIMUomegas[i] = 0.0;
       pPtfmIMUacc[i] = 0.0;
       pGSRefv[i] = 0.0;
       pGSRefomegas[i] = 0.0;
       pGSRefacc[i] = 0.0;
       pPtfmLoads[i] = 0.0;
    }
    for (i = 3; i < 6; i++)
    {
       pPtfmLoads[i] = 0.0;
    }

    // This is called as part of the user module constructor
    //KFAST_Init(&dt, &numFlaps, &numPylons, &numComp, pNumCompNds, pModFlags, KAD_FileName, IfW_FileName, MD_FileName, KFC_FileName, outFileRoot, &printSum, &gravity, &KAD_InterpOrder,
    //   pFusODCM, &numRtrPts, pRtrPts, pRtrMass, pRtrI_Rot, pRtrI_Trans, pRtrXcm, pRefPts, &numNodePts, pNodePts, pNodeDCMs,
    //   &nFusOuts, FusOutNd, &nSWnOuts, SWnOutNd, &nPWnOuts, PWnOutNd, &nVSOuts, VSOutNd, &nSHSOuts, SHSOutNd, &nPHSOuts, PHSOutNd, &nPylOuts, PylOutNd, &numOutChan, outChanList, pChanList_len, &errStat, errMsg);
    printf("Calling Init\n");
    KFAST_OS_Init(&simMod, &dt, &tmax, &numFlaps, &numPylons, &numComp, pNumCompNds, pModFlags, KAD_FileName, IfW_FileName, MD_FileName, KFC_FileName, HD_FileName, MD_Mooring_FileName, outFileRoot, &printSum, &gravity, &KAD_InterpOrder,
       pFusODCM, &numRtrPts, pRtrPts, pRtrMass, pRtrI_Rot, pRtrI_Trans, pRtrXcm, pRefPts, &numNodePts, pNodePts, pNodeDCMs,
       &nFusOuts, FusOutNd, &nSWnOuts, SWnOutNd, &nPWnOuts, PWnOutNd, &nVSOuts, VSOutNd, &nSHSOuts, SHSOutNd, &nPHSOuts, PHSOutNd, &nPylOuts, PylOutNd, pPtfmO, pPtfmODCM, pGSRefPt, &numOutChan, outChanList, pChanList_len, &errStat, errMsg);

    if (errStat == 4)
    {
        printf("%s\n", errMsg);
        return errStat;
    }


    // The outputs for the first timestep need to be obtained
    // TODO: Is call part of the user module constructor or part of InitialAssRes??? GJH
    numRtrLoadsElem = numRtrPtsElem * 2;
    pRtrLoads = (double *)malloc(numRtrLoadsElem * sizeof(double));
    numGaussLoadPts = numGaussPtLoadsElem / 6;
    numNodeLoadsElem = numNodePtElem * 2;
    pNodeLoads = (double *)malloc(numNodeLoadsElem * sizeof(double));
    pGaussPtLoads = (double *)malloc(numGaussPtLoadsElem * sizeof(double));
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
    pRtrVels = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrOmegas = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrAccs = (double *)malloc(numRtrPtsElem * sizeof(double));
    pRtrAlphas = (double *)malloc(numRtrPtsElem * sizeof(double));
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

    pRtrDCMs = (double *)malloc(3*numRtrPtsElem * sizeof(double));

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
    //KFAST_AssRes(&t, &isInitialTime, pWindPt, pFusO, pFusODCM,  pFusOv, 
    //   pFusOomegas,  pFusOacc, pFusOalphas, &numNodePts, pNodePts, pNodeDCMs,
    //                pNodeVels, pNodeOmegas, pNodeAccs,
    //                &numRtrPts, pRtrPts, pRtrDCMs, pRtrVels, pRtrOmegas, pRtrAccs, pRtrAlphas,
    //                pNodeLoads, pRtrLoads, &errStat, errMsg);
    printf("Calling KFAST_OS_AssRes\n");
    KFAST_OS_AssRes(&t, &isInitialTime, pWindPt, pWindPtVel, pFusO, pFusODCM, pFusOv,
       pFusOomegas, pFusOacc, pFusOalphas, &numNodePts, pNodePts, pNodeDCMs,
       pNodeVels, pNodeOmegas, pNodeAccs,
       &numRtrPts, pRtrPts, pRtrDCMs, pRtrVels, pRtrOmegas, pRtrAccs, pRtrAlphas,
       pPtfmO, pPtfmODCM, pPtfmOv, pPtfmOomegas, pPtfmOacc, pPtfmOalphas,
       pPtfmIMUPt, pPtfmIMUDCM, pPtfmIMUv, pPtfmIMUomegas, pPtfmIMUacc,
       pGSRefPt, pGSRefDCM, pGSRefv, pGSRefomegas, pGSRefacc,
       pNodeLoads, pRtrLoads, pPtfmLoads, &errStat, errMsg);
    n = 0;
    if (fabs(pPtfmLoads[n]) > 0 || fabs(pPtfmLoads[n + 1]) > 0 || fabs(pPtfmLoads[n + 2]) > 0 || fabs(pPtfmLoads[n + 3]) > 0 || fabs(pPtfmLoads[n + 4]) > 0 || fabs(pPtfmLoads[n + 5]) > 0)
    {
       printf("Platform node loads = %10.3e,%10.3e,%10.3e,%10.3e,%10.3e,%10.3e\n", pPtfmLoads[n], pPtfmLoads[n + 1], pPtfmLoads[n + 2], pPtfmLoads[n + 3], pPtfmLoads[n + 4], pPtfmLoads[n + 5]);
    }
    /*for (n = 0; n < numNodeLoadsElem; n = n + 6)
    {
        if (fabs(pNodeLoads[n ]) > 0 || fabs(pNodeLoads[n  + 1]) > 0 || fabs(pNodeLoads[n + 2]) > 0 || fabs(pNodeLoads[n +3]) > 0 || fabs(pNodeLoads[n  + 4]) > 0 || fabs(pNodeLoads[n  + 5]) > 0)
        {
            printf("Node %d loads = %10.3e,%10.3e,%10.3e,%10.3e,%10.3e,%10.3e\n", n / 6, pNodeLoads[n], pNodeLoads[n  + 1], pNodeLoads[n + 2], pNodeLoads[n +3], pNodeLoads[n  + 4], pNodeLoads[n + 5]);
        }
    }
    n = 0;
    if (fabs(pPtfmLoads[n]) > 0 || fabs(pPtfmLoads[n  + 1]) > 0 || fabs(pPtfmLoads[n + 2]) > 0 || fabs(pPtfmLoads[n +3]) > 0 || fabs(pPtfmLoads[n  + 4]) > 0 || fabs(pPtfmLoads[n  + 5]) > 0)
    {
        printf("Platform node loads = %10.3e,%10.3e,%10.3e,%10.3e,%10.3e,%10.3e\n", pPtfmLoads[n], pPtfmLoads[n  + 1], pPtfmLoads[n + 2], pPtfmLoads[n +3], pPtfmLoads[n  + 4], pPtfmLoads[n + 5]);
    } */
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
    printf("Calling KFAST_OS_Output\n");
    KFAST_OS_Output(&t, &numGaussLoadPts, pGaussPtLoads, &errStat, errMsg);
    if (errStat != 0)
    {
        printf("%s\n", errMsg);
        printf("%s\n", "Quitting due to error in KFAST_OUTPUT");
        return errStat;
    }
    t = t + dt;  // MBDyn increments time before AfterPredict()
    
    // AfterPredict()
    printf("Calling KFAST_OS_AfterPredict\n");
    KFAST_OS_AfterPredict(&t, &errStat, errMsg);
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
        //KFAST_AssRes(&t, &isInitialTime, pWindPt, pFusO, pFusODCM, pFusOv,
        //   pFusOomegas, pFusOacc, pFusOalphas, &numNodePts, pNodePts, pNodeDCMs,
        //   pNodeVels, pNodeOmegas, pNodeAccs,
        //   &numRtrPts, pRtrPts, pRtrDCMs, pRtrVels, pRtrOmegas, pRtrAccs, pRtrAlphas,
        //   pNodeLoads, pRtrLoads, &errStat, errMsg);
       printf("Calling KFAST_OS_AssRes\n");
        KFAST_OS_AssRes(&t, &isInitialTime, pWindPt, pWindPtVel, pFusO, pFusODCM, pFusOv,
           pFusOomegas, pFusOacc, pFusOalphas, &numNodePts, pNodePts, pNodeDCMs,
           pNodeVels, pNodeOmegas, pNodeAccs,
           &numRtrPts, pRtrPts, pRtrDCMs, pRtrVels, pRtrOmegas, pRtrAccs, pRtrAlphas,
           pPtfmO, pPtfmODCM, pPtfmOv,pPtfmOomegas,pPtfmOacc,pPtfmOalphas,
           pPtfmIMUPt, pPtfmIMUDCM, pPtfmIMUv, pPtfmIMUomegas, pPtfmIMUacc,
           pGSRefPt, pGSRefDCM, pGSRefv, pGSRefomegas, pGSRefacc,
           pNodeLoads, pRtrLoads, pPtfmLoads, &errStat, errMsg);

        if (errStat != 0)
        {
            printf("%s\n", errMsg);
            return errStat;
        }

        // Output()
        printf("Calling KFAST_OS_Output\n");
        KFAST_OS_Output(&t, &numGaussPtLoadsElem, pGaussPtLoads, &errStat, errMsg);
        if (errStat != 0)
        {
           printf("%s\n", errMsg);
           return errStat;
        }

        t = t + dt;  // MBDyn increments time before AfterPredict()

       
        // AfterPredict()
        printf("Calling KFAST_OS_AfterPredict\n");
        KFAST_OS_AfterPredict(&t, &errStat, errMsg);
        if (errStat != 0)
        {
            printf("%s\n", errMsg);
            return errStat;
        }

       

    }


    // End()
    KFAST_OS_End(&errStat, errMsg);
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
