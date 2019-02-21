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
#define INTERFACE_STRING_LENGTH 1025

// *** Mapping from kitefastmbd to KiteFAST variable names ***

extern int KFAST_Init(
  double *time_step,                     // dt
  int *n_flaps_per_wing,                 // numFlaps
  int *n_pylons_per_wing,                // numPylons
  int *n_components,                     // numComp
  int component_node_counts[],           // numCompNds
  int kitefast_module_flags[],           // modFlags
  const char kiteaerodyn_filename[],     // KAD_FileName
  const char inflowwind_filename[],      // IfW_FileName
  const char moordyn_filename[],         // MD_FileName
  const char controller_filename[],      // KFC_FileName
  const char output_file_root[],         // outFileRoot
  int *print_summary_file,               // printSum
  double *gravity,                       // gravity
  double mip_dcm[],                      // FusODCM
  int *n_rotor_points,                   // numRtrPts
  double rotor_points[],                 // rtrPts
  double rotor_masses[],                 // rtrMass
  double rotor_rotational_inertias[],    // rtrI_Rot
  double rotor_translational_inertias[], // rtrI_trans
  double rotor_cm_offsets[],             // rtrXcm
  double reference_points[],             // refPts
  int *node_count_no_rotors,             // numNodePts
  double node_points[],                  // nodePts
  double node_dcms[],                    // nodeDCMs
  int *n_fuselage_outputs,               // nodePts
  int fuselage_output_nodes[],           // nodeDCMs
  int *n_starboard_wing_outputs,         // nFusOuts
  int starboard_wing_output_nodes[],     // FusOutNd
  int *n_port_wing_outputs,              // nSWnOuts
  int port_wing_output_nodes[],          // SWnOutNd
  int *n_vertical_stabilizer_outputs,                 // nPWnOuts
  int vertical_stabilizer_output_nodes[],             // PWnOutNd
  int *n_starboard_horizontal_stabilizer_outputs,     // nVSOuts
  int starboard_horizontal_stabilizer_output_nodes[], // VSOutNd
  int *n_port_horizontal_stabilizer_outputs,          // nSHSOuts
  int port_horizontal_stabilizer_output_nodes[],      // SHSOutNd
  int *n_pylon_outputs,                               // nPHSOuts
  int pylon_output_nodes[],              // PHSOutNd
  int *n_output_channels,                // nPylOuts
  char *output_channel_array[],          // PylOutNd
  int output_channel_lengths[],          // numOutChan
  int *error_status,                     // chanList
  char error_message[]                   // ChanList_len
);
extern int KFAST_AssRes(
  double *t,                     // t
  int *first_iteration,          // isInitialTime
  double ground_station_point[], // WindPt
  double mip_position[],         // FusO
  double mip_dcm[],              // FusODCM
  double mip_vels[],             // FusOv
  double mip_omegas[],           // FusOomegas
  double mip_accs[],             // FusOacc
  double mip_alphas[],           // FusOalphas
  int *node_count_no_rotors,     // numNodePts
  double node_points[],          // nodePts
  double node_dcms[],            // nodeDCMs
  double node_vels[],            // nodeVels
  double node_omegas[],          // nodeOmegas
  double node_accs[],            // nodeAccs
  int *n_rotor_points,           // numRtrPts
  double rotor_points[],         // rtrPts
  double rotor_dcms[],           // nodeDCMs
  double rotor_vels[],           // nodeVels
  double rotor_omegas[],         // rtrOmegas
  double rotor_accs[],           // rtrAccs
  double rotor_alphas[],         // rtrAlphas
  double node_loads[],           // nodeLoads
  double rotor_loads[],          // rtrLoads
  int *error_status,             // errStat
  char error_message[]           // errMsg
);
extern int KFAST_AfterPredict(
  double *t,                     // t
  int *error_status,             // errStat
  char error_message[]           // errMsg
);
extern int KFAST_Output(
  double *current_time,          // simulation time for the current timestep (s)
  int *n_gauss_load_points,      // Total number of gauss points in the MBDyn model
  double gauss_point_loads[],    // Array of loads in the global coordinate system (3 forces + 3 moments) corresponding to each MBDyn gauss point. ( N, N-m )
  int *error_status,             // errStat
  char error_message[]           // errMsg
);
extern int KFAST_End(
  int *error_status,             // errStat
  char error_message[]           // errMsg
);

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

  doublereal time_step;
  doublereal ground_station_point[3];
  integer n_rotor_points;
  integer node_count_no_rotors;
  integer first_iteration;
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
  std::vector< std::vector<KiteFASTNode> > nodes_starpylons; // < size - n pylons < size - n nodes in pylon >>
  std::vector< std::vector<KiteFASTNode> > nodes_portpylons; // < size - n pylons < size - n nodes in pylon >>
  std::vector<KiteFASTNode> nodes_starrotors;
  std::vector<KiteFASTNode> nodes_portrotors;

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
  doublereal GetPrivateData(KiteFASTBeam beam, const char *private_data) const;
  virtual void Output(OutputHandler &OH) const;
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
