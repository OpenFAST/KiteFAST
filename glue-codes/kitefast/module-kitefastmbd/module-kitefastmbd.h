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
#include <string>

#include "dataman.h"
#include "userelem.h"
#include "drive_.h"
#include "beam.h"

#include <sstream>

#define SSTR(x) static_cast<std::ostringstream &>(           \
                    (std::ostringstream() << std::dec << x)) \
                    .str()

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
                      int *printSum,
                      double *gravity,
                      double windPt[],
                      double FusODCM[],
                      int *numRtrPts,
                      double rtrPts[],
                      double rtrMass[],
                      double rtrI_Rot[],
                      double rtrI_trans[],
                      double rtrXcm[],
                      double refPts[],
                      int *numNodePts,
                      double nodePts[],
                      double nodeDCMs[],
                      int *nFusOuts,
                      int FusOutNd[],
                      int *nSWnOuts,
                      int SWnOutNd[],
                      int *nPWnOuts,
                      int PWnOutNd[],
                      int *nVSOuts,
                      int VSOutNd[],
                      int *nSHSOuts,
                      int SHSOutNd[],
                      int *nPHSOuts,
                      int PHSOutNd[],
                      int *nPylOuts,
                      int PylOutNd[],
                      int *numOutChan,
                      char *chanList[],
                      int *errStat,
                      char errMsg[]);
extern int KFAST_AssRes(double *t,
                        int *isInitialTime,
                        double WindPt[],
                        double FusO_prev[],
                        double FusO[],
                        double FusODCM_prev[],
                        double FusODCM[],
                        double FusOv_prev[],
                        double FusOv[],
                        double FusOomegas_prev[],
                        double FusOomegas[],
                        double FusOacc_prev[],
                        double FusOacc[],
                        double FusOalphas[],
                        int *numNodePts,
                        double nodePts[],
                        double nodeDCMs[],
                        double nodeVels[],
                        double nodeOmegas[],
                        double nodeAccs[],
                        int *numRtrPts,
                        double rtrPts[],
                        double rtrDCMs[],
                        double rtrVels[],
                        double rtrOmegas[],
                        double rtrAccs[],
                        double rtrAlphas[],
                        double nodeLoads[],
                        double rtrLoads[],
                        int *errStat,
                        char errMsg[]);
extern int KFAST_AfterPredict(int *errStat, char errMsg[]);
extern int KFAST_Output(double *t,             // simulation time for the current timestep (s)
                        int *numGaussLoadPts,  // Total number of gauss points in the MBDyn model
                        double gaussPtLoads[], // Array of loads in the global coordinate system (3 forces + 3 moments) corresponding to each MBDyn gauss point. ( N, N-m )
                        int *errStat,
                        char errMsg[]);
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

  struct KiteFASTBeam
  {
    Beam *pBeam;
  };

  const static int AbortErrLev = ErrID_Fatal; // abort error level; compare with NWTC Library

  // KFAST_Init interface variables
  doublereal time_step;                                               // dt
  integer n_flaps_per_wing;                                           // numFlaps
  integer n_pylons_per_wing;                                          // numPylons
  integer n_components;                                               // numComp
  integer *component_node_counts;                                     // numCompNds
  integer kitefast_module_flags[4];                                   // modFlags
  char kiteaerodyn_filename[INTERFACE_STRING_LENGTH];                 // KAD_FileName
  char inflowwind_filename[INTERFACE_STRING_LENGTH];                  // IfW_FileName
  char moordyn_filename[INTERFACE_STRING_LENGTH];                     // MD_FileName
  char controller_filename[INTERFACE_STRING_LENGTH];                  // KFC_FileName
  char output_file_root[INTERFACE_STRING_LENGTH];                     // outFileRoot
  integer print_summary_file;                                         // printSum
  doublereal gravity;                                                 // gravity
  doublereal ground_station_point[3];                                 // windPt
  doublereal fuselage_dcm[9];                                         // FusODCM
  integer n_rotor_points;                                             // numRtrPts
  doublereal *rotor_points;                                           // rtrPts
  doublereal *rotor_masses;                                           // rtrMass
  doublereal *rotor_rotational_inertias;                              // rtrI_Rot
  doublereal *rotor_translational_inertias;                           // rtrI_trans
  doublereal *rotor_cm_offsets;                                       // rtrXcm
  doublereal *reference_points;                                       // refPts
  integer node_count_no_rotors;                                       // numNodePts
  doublereal *node_points;                                            // nodePts
  doublereal *node_dcms;                                              // nodeDCMs
  integer n_fuselage_outputs;                                         // nFusOuts
  std::vector<integer> fuselage_output_nodes;                         // FusOutNd
  integer n_starboard_wing_outputs;                                   // nSWnOuts
  std::vector<integer> starboard_wing_output_nodes;                   // SWnOutNd
  integer n_port_wing_outputs;                                        // nPWnOuts
  std::vector<integer> port_wing_output_nodes;                        // PWnOutNd
  integer n_vertical_stabilizer_outputs;                              // nVSOuts
  std::vector<integer> vertical_stabilizer_output_nodes;              // VSOutNd
  integer n_starboard_horizontal_stabilizer_outputs;                  // nSHSOuts
  std::vector<integer> starboard_horizontal_stabilizer_output_nodes;  // SHSOutNd
  integer n_port_horizontal_stabilizer_outputs;                       // nPHSOuts
  std::vector<integer> port_horizontal_stabilizer_output_nodes;       // PHSOutNd
  integer n_pylon_outputs;                                            // nPylOuts
  std::vector<integer> pylon_output_nodes;                            // PylOutNd
  integer n_output_channels;                                          // numOutChan
  std::vector<std::string> output_channels;                           // chanList
  // error_status                                                        errStat - local variable
  // error message                                                       errMsg - local variable

  // KFAST_AssRes interface variables
  // commented variables are already declared above
  doublereal t;                          // t
  integer first_iteration;               // isInitialTime
  // doublereal ground_station_point[3]; // WindPt
  doublereal fuselage_position_prev[3];  // FusO_prev
  doublereal fuselage_position[3];       // FusO
  doublereal fuselage_dcm_prev[9];       // FusODCM_prev
  // doublereal fuselage_dcm[9];         // FusODCM
  doublereal fuselage_vels_prev[3];      // FusOv_prev
  doublereal fuselage_vels[3];           // FusOv
  doublereal fuselage_omegas_prev[3];    // FusOomegas_prev
  doublereal fuselage_omegas[3];         // FusOomegas
  doublereal fuselage_accs_prev[3];      // FusOacc_prev
  doublereal fuselage_accs[3];           // FusOacc
  doublereal fuselage_alphas[3];         // FusOalphas
  // integer node_count_no_rotors;       // numNodePts
  // doublereal *node_points;            // nodePts
  // doublereal *node_dcms;              // nodeDCMs
  doublereal *node_vels;                 // nodeVels
  doublereal *node_omegas;               // nodeOmegas
  doublereal *node_accs;                 // nodeAccs
  // integer n_rotor_points;             // numRtrPts
  // doublereal *rotor_points;           // rtrPts
  doublereal *rotor_dcms;                // nodeDCMs
  doublereal *rotor_vels;                // nodeVels
  doublereal *rotor_omegas;              // rtrOmegas
  doublereal *rotor_accs;                // rtrAccs
  doublereal *rotor_alphas;              // rtrAlphas
  doublereal *node_loads;                // nodeLoads
  doublereal *rotor_loads;               // rtrLoads
  // error_status                           errStat - local variable
  // error message                          errMsg - local variable

  // KFAST_Output interface variables - all are local
  // doublereal t;                   // t
  // integer n_gauss_load_points;    // numGaussLoadPts
  // doublereal *gauss_point_loads;  // gaussPtLoads
  // error_status                       errStat
  // error message                      errMsg

  // other data
  doublereal initial_time;
  integer total_beam_count;
  KiteFASTNode mip_node;
  std::vector<KiteFASTNode> nodes;
  std::vector<KiteFASTNode> nodes_fuselage;
  std::vector<KiteFASTNode> nodes_portwing;
  std::vector<KiteFASTNode> nodes_starwing;
  std::vector<KiteFASTNode> nodes_vstab;
  std::vector<KiteFASTNode> nodes_porthstab;
  std::vector<KiteFASTNode> nodes_starhstab;
  std::vector< std::vector<KiteFASTNode> > nodes_starpylons;
  std::vector< std::vector<KiteFASTNode> > nodes_portpylons;
  std::vector<KiteFASTNode> nodes_portrotors;
  std::vector<KiteFASTNode> nodes_starrotors;

  std::vector<KiteFASTBeam> beams;
  std::vector<KiteFASTBeam> beams_fuselage;
  std::vector<KiteFASTBeam> beams_portwing;
  std::vector<KiteFASTBeam> beams_starwing;
  std::vector<KiteFASTBeam> beams_vstab;
  std::vector<KiteFASTBeam> beams_porthstab;
  std::vector<KiteFASTBeam> beams_starhstab;
  std::vector< std::vector<KiteFASTBeam> > beams_starpylons;
  std::vector< std::vector<KiteFASTBeam> > beams_portpylons;
  std::vector<KiteFASTBeam> beams_throwaway;

  mutable std::ofstream outputfile;
  DriveOwner Time;
  const DataManager *data_manager;

public:
  ModuleKiteFAST(unsigned uLabel, const DofOwner *pDO, DataManager *pDM, MBDynParser &HP);
  virtual ~ModuleKiteFAST(void);
  void SetValue(DataManager *pDM, VectorHandler &X, VectorHandler &XP, SimulationEntity::Hints *ph);
  void ValidateInputKeyword(MBDynParser &HP, const char *keyword);
  void BuildComponentArrays(DataManager *pDM, MBDynParser &HP, const char *keyword, std::vector<KiteFASTNode> &node_array, std::vector<KiteFASTBeam> &beam_array);
  void BuildComponentNodeArray(DataManager *pDM, MBDynParser &HP, std::vector<KiteFASTNode> &node_array);
  void BuildComponentBeamArray(DataManager *pDM, MBDynParser &HP, std::vector<KiteFASTBeam> &beam_array);
  void BuildComponentOutputArray(MBDynParser &HP, const char *keyword, integer &n_outputs, std::vector<int> &output_nodes);
  void InitOutputFile(std::string output_file_name);
  doublereal _GetPrivateData(KiteFASTBeam beam, const char *private_data);
  virtual void Output(OutputHandler &OH); // const;
  int iGetNumConnectedNodes(void) const;
  virtual void WorkSpaceDim(integer *piNumRows, integer *piNumCols) const;
  VariableSubMatrixHandler &AssJac(VariableSubMatrixHandler &WorkMat, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr);
  void Update(const VectorHandler &XCurr, const VectorHandler &XPrimeCurr);
  void _AssRes(doublereal *nodeLoads, doublereal *rotorLoads);
  SubVectorHandler &AssRes(SubVectorHandler &WorkVec, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr);
  void BeforePredict(VectorHandler & /* X */, VectorHandler & /* XP */, VectorHandler & /* XPrev */, VectorHandler & /* XPPrev */) const;
  void AfterPredict(VectorHandler &X, VectorHandler &XP);
  void AfterConvergence(const VectorHandler &X, const VectorHandler &XP);

  // helper functions while in development
  void printdebug(std::string debugstring) const;
  void PrintNodeLocations(KiteFASTNode node);

  // these are specific for mbdyn, not used by KiteFASTMBD or KiteFAST
  unsigned int iGetNumPrivData(void) const;
  void GetConnectedNodes(std::vector<const Node *> &connectedNodes) const;
  std::ostream &Restart(std::ostream &out) const;
  virtual unsigned int iGetInitialNumDof(void) const;
  virtual void InitialWorkSpaceDim(integer *piNumRows, integer *piNumCols) const;
  VariableSubMatrixHandler &InitialAssJac(VariableSubMatrixHandler &WorkMat, const VectorHandler &XCurr);
  SubVectorHandler &InitialAssRes(SubVectorHandler &WorkVec, const VectorHandler &XCurr);
};
