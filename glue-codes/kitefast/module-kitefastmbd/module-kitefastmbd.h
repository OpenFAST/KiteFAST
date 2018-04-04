/* $Header: /var/cvs/mbdyn/mbdyn/mbdyn-1.0/modules/module-kitefastmbd/module-kitefastmbd.h,v 1.8 2017/01/12 14:47:15 masarati Exp $ */
/*
 * MBDyn (C) is a multibody analysis code.
 * http://www.mbdyn.org
 *
 * Copyright (C) 1996-2017
 *
 * Pierangelo Masarati  <masarati@aero.polimi.it>
 *
 * Dipartimento di Ingegneria Aerospaziale - Politecnico di Milano
 * via La Masa, 34 - 20156 Milano, Italy
 * http://www.aero.polimi.it
 *
 * Changing this copyright notice is forbidden.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License).
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "mbconfig.h" /* This goes first in every *.c,*.cc file */

#include <iostream>
#include <cfloat>
#include <math.h>

#include "dataman.h"
#include "userelem.h"
#include "drive_.h"

#define USE_SINGLE_PRECISION

#ifndef KiteFAST_MBD_H
#define KiteFAST_MBD_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if defined(USE_SINGLE_PRECISION)
typedef float f_real;
#elif defined(USE_DOUBLE_PRECISION)
typedef double f_real;
#else
#error "define either USE_SINGLE_PRECISION or USE_DOUBLE_PRECISION"
#endif

typedef long int f_integer;

#define ErrID_None 0
#define ErrID_Info 1
#define ErrID_Warn 2
#define ErrID_Severe 3
#define ErrID_Fatal 4
#define CHANNEL_LENGTH 10
#define INTERFACE_STRING_LENGTH 1025

extern int KFAST_Init(double *dt,
                      int *numFlaps,
                      int *numPylons,
                      int *numComp,
                      int numCompNds[],
                      int modFlags[],
                      const char KAD_FileName[],
                      const char IfW_FileName[],
                      const char MD_FileName[],
                      const char KFC_FileName[],
                      const char outFileRoot[],
                      double *gravity,
                      double windPt[],
                      double FusODCM[],
                      int *numRtrPtsElem,
                      double rtrPts[],
                      int *numRefPtElem,
                      double refPts[],
                      int *numNodePtElem,
                      double nodePts[],
                      int *numDCMElem,
                      double nodeDCMs[],
                      int *errStat,
                      char errMsg[]);
extern int KFAST_AssRes(double *t,
                        int *isInitialTime,
                        int *numRtSpdRtrElem,
                        double RtSpd_PyRtr[],
                        double WindPt[],
                        double FusO_prev[],
                        double FusO[],
                        double FusODCM_prev[],
                        double FusOv_prev[],
                        double FusOomegas_prev[],
                        double FusOacc_prev[],
                        int *numNodePtElem,
                        double nodePts[],
                        int *numNodeVelElem,
                        double nodeVels[],
                        int *numNodeOmegaElem,
                        double nodeOmegas[],
                        int *numDCMElem,
                        double nodeDCMs[],
                        int *numRtrPtsElem,
                        double rtrPts[],
                        double rtrVels[],
                        double rtrDCMs[],
                        int *numNodeLoadsElem,
                        double nodeLoads[],
                        int *numRtrLoadsElem,
                        double rtrLoads[],
                        int *errStat,
                        char errMsg[]);
extern int KFAST_AfterPredict(int *errStat, char errMsg[]);
extern int KFAST_Output(double *t, int *errStat, char errMsg[]);
extern int KFAST_End(int *errStat, char errMsg[]);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* KiteFAST_MBD_H */

class ModuleKiteFAST : virtual public Elem, public UserDefinedElem
{
private:
  struct KiteFASTNode
  {
    StructNode *pNode;
  };

  const static int AbortErrLev = ErrID_Fatal; // abort error level; compare with NWTC Library
  
  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];

  // class data
  doublereal ground_station_point[3];
  integer first_iteration;  // 0 no - 1 yes
  integer n_pylons_per_wing;
  integer numNodePtElem;
  integer node_count_no_rotors;
  integer numDCMElem;
  integer rotor_node_count;
  integer numRtrPtsElem;
  doublereal *node_points;
  doublereal *node_dcms;
  doublereal *node_velocities;
  doublereal *node_omegas;
  doublereal *rotor_points;
  doublereal *rotor_velocities;
  doublereal *rotor_dcms;

  std::vector<KiteFASTNode> nodes;
  std::vector<KiteFASTNode> nodes_fuselage;
  std::vector<KiteFASTNode> nodes_portwing;
  std::vector<KiteFASTNode> nodes_starwing;
  std::vector<KiteFASTNode> nodes_vstab;
  std::vector<KiteFASTNode> nodes_porthstab;
  std::vector<KiteFASTNode> nodes_starhstab;
  std::vector<KiteFASTNode> nodes_portpylon1;
  std::vector<KiteFASTNode> nodes_starpylon1;
  std::vector<KiteFASTNode> nodes_portrotors;
  std::vector<KiteFASTNode> nodes_starrotors;

  mutable std::ofstream outputfile;
  DriveOwner Time;

public:
  ModuleKiteFAST(unsigned uLabel, const DofOwner *pDO, DataManager *pDM, MBDynParser &HP);
  virtual ~ModuleKiteFAST(void);
  void SetValue(DataManager *pDM, VectorHandler &X, VectorHandler &XP, SimulationEntity::Hints *ph);
  void ValidateInputKeyword(MBDynParser &HP, const char *keyword);
  void BuildComponentNodeArray(DataManager *pDM, MBDynParser &HP, const char *keyword, std::vector<KiteFASTNode> &node_array);
  void InitOutputFile(std::string output_file_name);
  virtual void Output(OutputHandler &OH) const;
  int iGetNumConnectedNodes(void) const;
  virtual void WorkSpaceDim(integer *piNumRows, integer *piNumCols) const;
  VariableSubMatrixHandler &AssJac(VariableSubMatrixHandler &WorkMat, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr);
  void Update(const VectorHandler &XCurr, const VectorHandler &XPrimeCurr);
  SubVectorHandler &AssRes(SubVectorHandler &WorkVec, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr);
  void BeforePredict(VectorHandler & /* X */, VectorHandler & /* XP */, VectorHandler & /* XPrev */, VectorHandler & /* XPPrev */) const;
  void AfterPredict(VectorHandler &X, VectorHandler &XP);
  void AfterConvergence(const VectorHandler &X, const VectorHandler &XP);

  // helper functions while in development
  void printdebug(std::string debugstring) const;
  void PrintNodeLocations(KiteFASTNode node);

  // these are specific for mbdyn, not used by us or KiteFAST
  unsigned int iGetNumPrivData(void) const;
  void GetConnectedNodes(std::vector<const Node *> &connectedNodes) const;
  std::ostream &Restart(std::ostream &out) const;
  virtual unsigned int iGetInitialNumDof(void) const;
  virtual void InitialWorkSpaceDim(integer *piNumRows, integer *piNumCols) const;
  VariableSubMatrixHandler &InitialAssJac(VariableSubMatrixHandler &WorkMat, const VectorHandler &XCurr);
  SubVectorHandler &InitialAssRes(SubVectorHandler &WorkVec, const VectorHandler &XCurr);
};
