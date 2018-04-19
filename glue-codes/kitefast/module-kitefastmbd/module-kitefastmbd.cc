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

#include "module-kitefastmbd.h"

#define DEBUGUDE

ModuleKiteFAST::ModuleKiteFAST(unsigned uLabel, const DofOwner *pDO, DataManager *pDM, MBDynParser &HP) : Elem(uLabel, flag(0)), UserDefinedElem(uLabel, pDO)
{
  if (!HP.IsArg())
  {
    silent_cout("No Arguments provided for ModuleKiteFAST for UDE IDE=" << uLabel << std::endl);
    throw NoErr(MBDYN_EXCEPT_ARGS);
  }

  if (HP.IsKeyWord("help"))
  {
    silent_cout(std::endl
                << "Module: KiteFastMBD" << std::endl
                << "Author: Rick Damiani <rick.damiani@nrel.gov> and Rafael Mudafort <rafael.mudafort@nrel.gov>" << std::endl
                << "Organization:" << std::endl
                << "    National Renewable Energy Lab" << std::endl
                << "    National Wind Technology Center" << std::endl
                << "    15013 Denver W Pkwy" << std::endl
                << "    Golden, CO 80401" << std::endl
                << "" << std::endl
                << std::endl);
  }

  // setup the time drive
  Time.Set(new TimeDriveCaller(pDM->pGetDrvHdl()));
  silent_cout("initial time: " << Time.dGet() << std::endl);

  // *** input parsing ***

  // parse the kitefast module flags
  // 0 = off, 1 = on
  ValidateInputKeyword(HP, "fast_submodule_flags");
  int kitefast_module_flags[4];
  kitefast_module_flags[0] = HP.GetInt(); // kiteaerodyn
  kitefast_module_flags[1] = HP.GetInt(); // inflowwind
  kitefast_module_flags[2] = HP.GetInt(); // moordyn
  kitefast_module_flags[3] = HP.GetInt(); // controller

  // parse the kitefast module input files
  ValidateInputKeyword(HP, "fast_submodule_input_files");
  char kiteaerodyn_filename[INTERFACE_STRING_LENGTH];
  char inflowwind_filename[INTERFACE_STRING_LENGTH];
  char moordyn_filename[INTERFACE_STRING_LENGTH];
  char controller_filename[INTERFACE_STRING_LENGTH];
  strcpy(kiteaerodyn_filename, HP.GetFileName());
  strcpy(inflowwind_filename, HP.GetFileName());
  strcpy(moordyn_filename, HP.GetFileName());
  strcpy(controller_filename, HP.GetFileName());

  // parse the output file settings and initialize the output file
  ValidateInputKeyword(HP, "output_file_root");
  char output_file_root[INTERFACE_STRING_LENGTH];
  strcpy(output_file_root, HP.GetFileName());
  std::string output_file_name;
  output_file_name = strcat(output_file_root, "MBD.out");
  InitOutputFile(output_file_name);

  // parse the time step
  ValidateInputKeyword(HP, "time_step");
  doublereal dt = HP.GetReal();

  // parse the gravity
  ValidateInputKeyword(HP, "gravity");
  doublereal gravity = HP.GetReal();

  // parse the ground station location
  ValidateInputKeyword(HP, "ground_weather_station_location");
  ground_station_point[0] = HP.GetReal();
  ground_station_point[1] = HP.GetReal();
  ground_station_point[2] = HP.GetReal();

  // parse the component counts
  ValidateInputKeyword(HP, "number_of_flaps_per_wing");
  integer n_flaps_per_wing = HP.GetInt();
  ValidateInputKeyword(HP, "number_of_pylons_per_wing");
  n_pylons_per_wing = HP.GetInt();

  // n_components includes all components except the rotors
  ValidateInputKeyword(HP, "number_of_kite_components");
  integer n_components = HP.GetInt();

  // parse the keypoints (aka reference points)
  ValidateInputKeyword(HP, "keypoints");
  integer numRefPtElem = 3 * (n_components + 2 * n_pylons_per_wing);
  doublereal reference_points[numRefPtElem];
  for (int i = 0; i < n_components; i++)
  {
    reference_points[3 * i] = HP.GetReal();
    reference_points[3 * i + 1] = HP.GetReal();
    reference_points[3 * i + 2] = HP.GetReal();
  }

  // parse the mip node
  ValidateInputKeyword(HP, "mip_node");
  mip_node.pNode = dynamic_cast<StructNode *>(pDM->ReadNode(HP, Node::STRUCTURAL));
  
  // parse the component nodes into arrays
  BuildComponentNodeArray(pDM, HP, "fuselage", nodes_fuselage);
  BuildComponentNodeArray(pDM, HP, "starboard_wing", nodes_starwing);
  BuildComponentNodeArray(pDM, HP, "port_wing", nodes_portwing);
  BuildComponentNodeArray(pDM, HP, "vertical_stabilizer", nodes_vstab);
  BuildComponentNodeArray(pDM, HP, "starboard_horizontal_stabilizer", nodes_starhstab);
  BuildComponentNodeArray(pDM, HP, "port_horizontal_stabilizer", nodes_porthstab);
  nodes_starpylons.resize(n_pylons_per_wing);
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    std::string component_name = "starboard_pylon_";
    component_name.append(SSTR(i + 1));
    BuildComponentNodeArray(pDM, HP, component_name.c_str(), nodes_starpylons[i]);
  }
  nodes_portpylons.resize(n_pylons_per_wing);
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    std::string component_name = "port_pylon_";
    component_name.append(SSTR(i + 1));
    BuildComponentNodeArray(pDM, HP, component_name.c_str(), nodes_portpylons[i]);
  }
  BuildComponentNodeArray(pDM, HP, "starboard_rotors", nodes_starrotors);
  BuildComponentNodeArray(pDM, HP, "port_rotors", nodes_portrotors);

  integer total_node_count = nodes_fuselage.size()
                           + nodes_starwing.size()
                           + nodes_portwing.size()
                           + nodes_vstab.size()
                           + nodes_starhstab.size()
                           + nodes_porthstab.size();
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    total_node_count += nodes_portpylons[i].size() + nodes_portpylons[i].size();
  }
  total_node_count += nodes_starrotors.size() + nodes_portrotors.size();

  nodes.reserve(total_node_count);
  nodes.insert(nodes.begin(), nodes_fuselage.begin(), nodes_fuselage.end());
  nodes.insert(nodes.end(), nodes_starwing.begin(), nodes_starwing.end());
  nodes.insert(nodes.end(), nodes_portwing.begin(), nodes_portwing.end());
  nodes.insert(nodes.end(), nodes_vstab.begin(), nodes_vstab.end());
  nodes.insert(nodes.end(), nodes_starhstab.begin(), nodes_starhstab.end());
  nodes.insert(nodes.end(), nodes_porthstab.begin(), nodes_porthstab.end());
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    nodes.insert(nodes.end(), nodes_starpylons[i].begin(), nodes_starpylons[i].end());
  }
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    nodes.insert(nodes.end(), nodes_portpylons[i].begin(), nodes_portpylons[i].end());
  }
  nodes.insert(nodes.end(), nodes_starrotors.begin(), nodes_starrotors.end());
  nodes.insert(nodes.end(), nodes_portrotors.begin(), nodes_portrotors.end());

  // number of nodes per kite component excluding rotors
  integer component_node_counts[n_components];
  component_node_counts[0] = nodes_fuselage.size();
  component_node_counts[1] = nodes_starwing.size();
  component_node_counts[2] = nodes_portwing.size();
  component_node_counts[3] = nodes_vstab.size();
  component_node_counts[4] = nodes_starhstab.size();
  component_node_counts[5] = nodes_porthstab.size();
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    component_node_counts[6 + i] = nodes_starpylons[i].size();
  }
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    component_node_counts[6 + n_pylons_per_wing + i] = nodes_portpylons[i].size();
  }

  // build the node arrays for kite components excluding rotors
  node_count_no_rotors = 0;
  for (int i = 0; i < 6 + 2 * n_pylons_per_wing; i++)
  {
    node_count_no_rotors += component_node_counts[i];
  }
  numNodePtElem = 3 * node_count_no_rotors;
  node_points = new doublereal[numNodePtElem];
  numDCMElem = 9 * numNodePtElem;
  node_dcms = new doublereal[numDCMElem];
  
  for (int i = 0; i < node_count_no_rotors; i++)
  {
    Vec3 xcurr = nodes[i].pNode->GetXCurr();
    node_points[3 * i] = xcurr[0];
    node_points[3 * i + 1] = xcurr[1];
    node_points[3 * i + 2] = xcurr[2];

    // these are dGet(row, col)
    Mat3x3 rnode = nodes[i].pNode->GetRCurr();
    node_dcms[9 * i] = rnode.dGet(1, 1);
    node_dcms[9 * i + 1] = rnode.dGet(1, 2);
    node_dcms[9 * i + 2] = rnode.dGet(1, 3);
    node_dcms[9 * i + 3] = rnode.dGet(2, 1);
    node_dcms[9 * i + 4] = rnode.dGet(2, 2);
    node_dcms[9 * i + 5] = rnode.dGet(2, 3);
    node_dcms[9 * i + 6] = rnode.dGet(3, 1);
    node_dcms[9 * i + 7] = rnode.dGet(3, 2);
    node_dcms[9 * i + 8] = rnode.dGet(3, 3);
  }

  // build the node arrays for rotors
  rotor_node_count = nodes_starrotors.size() + nodes_portrotors.size();
  numRtrPtsElem = 3 * rotor_node_count;
  rotor_points = new doublereal[numRtrPtsElem];
  for (int i = 0; i < rotor_node_count; i++)
  {
    Vec3 xcurr = nodes[i + node_count_no_rotors].pNode->GetXCurr();
    rotor_points[3 * i] = xcurr[0];
    rotor_points[3 * i + 1] = xcurr[1];
    rotor_points[3 * i + 2] = xcurr[2];
  }

  // The kite is aligned with the Global Coordinate system
  Mat3x3 mip_dcm = mip_node.pNode->GetRCurr();
  doublereal pFusODCM[9];
  pFusODCM[0] = mip_dcm.dGet(1, 1);
  pFusODCM[1] = mip_dcm.dGet(1, 2);
  pFusODCM[2] = mip_dcm.dGet(1, 3);
  pFusODCM[3] = mip_dcm.dGet(2, 1);
  pFusODCM[4] = mip_dcm.dGet(2, 2);
  pFusODCM[5] = mip_dcm.dGet(2, 3);
  pFusODCM[6] = mip_dcm.dGet(3, 1);
  pFusODCM[7] = mip_dcm.dGet(3, 2);
  pFusODCM[8] = mip_dcm.dGet(3, 3);

  // call KFAST_Init method
  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_Init(&dt,
             &n_flaps_per_wing,
             &n_pylons_per_wing,
             &n_components,
             component_node_counts,
             kitefast_module_flags,
             kiteaerodyn_filename,
             inflowwind_filename,
             moordyn_filename,
             controller_filename,
             output_file_root,
             &gravity,
             ground_station_point,
             pFusODCM,
             &numRtrPtsElem,
             rotor_points,
             &numRefPtElem,
             reference_points,
             &numNodePtElem,
             node_points,
             &numDCMElem,
             node_dcms,
             &error_status,
             error_message);
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  // call KFAST_AssRes to initialize the loads
  integer numNodeLoadsElem = 6 * node_count_no_rotors; // force and moment components for each node
  doublereal *nodeLoads = new doublereal[numNodeLoadsElem];
  integer numRtrLoadsElem = numRtrPtsElem * 2; // force and moment components for each rotor node
  doublereal *rotorLoads = new doublereal[numRtrLoadsElem];

  _AssRes(1, numNodeLoadsElem, nodeLoads, numRtrLoadsElem, rotorLoads);

  delete[] nodeLoads;
  delete[] rotorLoads;

  // call KFAST_Output to initialize its output file
  doublereal current_time = Time.dGet();
  KFAST_Output(&current_time, &error_status, error_message);
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  // for future reference
  // Get the orientation matrix of blade root with respect to hub reference frame
  // if ((iNode % NElems) == 0) {
  //   bladeR[elem/NElems] = HP.GetRotRel(rf);
  // }
  //ReferenceFrame RF(m_pNode);
  //if (HP.IsKeyWord("position")) {
  //    m_tilde_f = HP.GetPosRel(RF);
  //  }
  //if (HP.IsKeyWord("orientation")) {
  //  m_tilde_Rh = HP.GetRotRel(RF);
  //}
}

ModuleKiteFAST::~ModuleKiteFAST(void)
{
  printdebug("~ModuleKiteFAST");

  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_End(&error_status, error_message);
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }
}

void ModuleKiteFAST::SetValue(DataManager *pDM, VectorHandler &X, VectorHandler &XP, SimulationEntity::Hints *ph)
{
  printdebug("SetValue");
  NO_OP;
}

void ModuleKiteFAST::ValidateInputKeyword(MBDynParser &HP, const char *keyword)
{
  printdebug("ValidateInputKeyword");
  silent_cout(keyword << std::endl);
  if (!HP.IsKeyWord(keyword))
  {
    silent_cerr("Input Error: cannot read keyword " << keyword << std::endl);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }
}

void ModuleKiteFAST::BuildComponentNodeArray(DataManager *pDM, MBDynParser &HP,
                                             const char *keyword,
                                             std::vector<KiteFASTNode> &node_array)
{
  printdebug("BuildComponentNodeArray");
  printdebug(keyword);
  if (!HP.IsKeyWord(keyword))
  {
    silent_cerr("Runtime Error: cannot read keyword " << keyword << std::endl);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  int node_count = HP.GetInt();

  node_array.resize(node_count);

  for (int i = 0; i < node_count; i++)
  {
    node_array[i].pNode = dynamic_cast<StructNode *>(pDM->ReadNode(HP, Node::STRUCTURAL));
    if (!node_array[i].pNode->bComputeAccelerations())
    {
      node_array[i].pNode->ComputeAccelerations(true);
    }
  }
}

void ModuleKiteFAST::InitOutputFile(std::string output_file_name)
{
  printdebug("InitOutputFile");

  outputfile.open(output_file_name.c_str());
  if (!outputfile)
  {
    silent_cerr("Runtime Error: cannot open file at " << output_file_name << std::endl);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  outputfile << std::setw(16) << "Time s"
             << std::setw(16) << "Node ID"
             << std::setw(16) << "pos - x"
             << std::setw(13) << "pos - y"
             << std::setw(13) << "pos - z"
             << std::endl;
}

void ModuleKiteFAST::Output(OutputHandler &OH) const
{
  printdebug("Output");

  if (outputfile)
  {
    for (int i = 0; i < 3; i++)
    {
      outputfile << std::setw(8)
                 << std::scientific
                 << std::setw(16) << Time.dGet()
                 << std::setw(16) << nodes[i].pNode->GetLabel()
                 << std::setw(16) << nodes[i].pNode->GetXCurr()
                 << std::endl;
    }
  }

  doublereal current_time = Time.dGet();
  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_Output(&current_time, &error_status, error_message);
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }
}

int ModuleKiteFAST::iGetNumConnectedNodes(void) const
{
  printdebug("iGetNumConnectedNodes");
  return nodes.size();
}

void ModuleKiteFAST::WorkSpaceDim(integer *piNumRows, integer *piNumCols) const
{
  printdebug("WorkSpaceDim");
  *piNumRows = 6 * iGetNumConnectedNodes();
  *piNumCols = 1;
}

VariableSubMatrixHandler &ModuleKiteFAST::AssJac(VariableSubMatrixHandler &WorkMat, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr)
{
  printdebug("AssJac");
  WorkMat.SetNullMatrix();
  return WorkMat;
}

void ModuleKiteFAST::Update(const VectorHandler &XCurr, const VectorHandler &XPrimeCurr)
{
  printdebug("Update");
}

void ModuleKiteFAST::_AssRes(integer first_iteration,
                             integer numNodeLoadsElem,
                             doublereal *nodeLoads,
                             integer numRtrLoadsElem,
                             doublereal *rotorLoads)
{
  printdebug("_AssRes");
  
  doublereal t = Time.dGet();
  integer numRtSpdRtrElem = rotor_node_count;
  doublereal RtSpd_PyRtr[numRtSpdRtrElem]; // rotational speed for each rotor element (rad/s)
  for (int i = 0; i < rotor_node_count; i++)
  {
    Vec3 wcurr = nodes[i + node_count_no_rotors].pNode->GetWCurr();
    RtSpd_PyRtr[i] = wcurr[0];
  }

  Vec3 vec3_fusOprev = mip_node.pNode->GetXPrev();
  doublereal FusO_prev[3];
  FusO_prev[0] = vec3_fusOprev[0];
  FusO_prev[1] = vec3_fusOprev[1];
  FusO_prev[2] = vec3_fusOprev[2];

  Vec3 vec3_fusO = mip_node.pNode->GetXCurr();
  doublereal FusO[3];
  FusO[0] = vec3_fusO[0];
  FusO[1] = vec3_fusO[1];
  FusO[2] = vec3_fusO[2];

  Mat3x3 fus0_dcm_prev = mip_node.pNode->GetRPrev();
  doublereal FusODCM_prev[9];
  FusODCM_prev[0] = fus0_dcm_prev.dGet(1, 1);
  FusODCM_prev[1] = fus0_dcm_prev.dGet(1, 2);
  FusODCM_prev[2] = fus0_dcm_prev.dGet(1, 3);
  FusODCM_prev[3] = fus0_dcm_prev.dGet(2, 1);
  FusODCM_prev[4] = fus0_dcm_prev.dGet(2, 2);
  FusODCM_prev[5] = fus0_dcm_prev.dGet(2, 3);
  FusODCM_prev[6] = fus0_dcm_prev.dGet(3, 1);
  FusODCM_prev[7] = fus0_dcm_prev.dGet(3, 2);
  FusODCM_prev[8] = fus0_dcm_prev.dGet(3, 3);

  Vec3 vec3_FusOv_prev = mip_node.pNode->GetVPrev();
  doublereal FusOv_prev[3];
  FusOv_prev[0] = vec3_FusOv_prev[0];
  FusOv_prev[1] = vec3_FusOv_prev[1];
  FusOv_prev[2] = vec3_FusOv_prev[2];

  Vec3 vec3_FusOomegas_prev = mip_node.pNode->GetWPrev();
  doublereal FusOomegas_prev[3];
  FusOomegas_prev[0] = vec3_FusOomegas_prev[0];
  FusOomegas_prev[1] = vec3_FusOomegas_prev[1];
  FusOomegas_prev[2] = vec3_FusOomegas_prev[2];

  Vec3 vec3_FusOacc_prev = mip_node.pNode->GetXPPPrev();
  doublereal FusOacc_prev[3];
  FusOacc_prev[0] = vec3_FusOacc_prev[0];
  FusOacc_prev[1] = vec3_FusOacc_prev[1];
  FusOacc_prev[2] = vec3_FusOacc_prev[2];

  node_velocities = new doublereal[numNodePtElem];
  node_omegas = new doublereal[numNodePtElem];
  for (int i = 0; i < node_count_no_rotors; i++)
  {
    Vec3 xcurr = nodes[i].pNode->GetXCurr();
    node_points[3 * i] = xcurr[0];
    node_points[3 * i + 1] = xcurr[1];
    node_points[3 * i + 2] = xcurr[2];

    Vec3 vcurr = nodes[i].pNode->GetVCurr();
    node_velocities[3 * i] = vcurr[0];
    node_velocities[3 * i + 1] = vcurr[1];
    node_velocities[3 * i + 2] = vcurr[2];

    Vec3 wcurr = nodes[i].pNode->GetWCurr();
    node_omegas[3 * i] = wcurr[0];
    node_omegas[3 * i + 1] = wcurr[1];
    node_omegas[3 * i + 2] = wcurr[2];

    // these are dGet(row, col)
    Mat3x3 rnode = nodes[i].pNode->GetRCurr();
    node_dcms[9 * i] = rnode.dGet(1, 1);
    node_dcms[9 * i + 1] = rnode.dGet(1, 2);
    node_dcms[9 * i + 2] = rnode.dGet(1, 3);
    node_dcms[9 * i + 3] = rnode.dGet(2, 1);
    node_dcms[9 * i + 4] = rnode.dGet(2, 2);
    node_dcms[9 * i + 5] = rnode.dGet(2, 3);
    node_dcms[9 * i + 6] = rnode.dGet(3, 1);
    node_dcms[9 * i + 7] = rnode.dGet(3, 2);
    node_dcms[9 * i + 8] = rnode.dGet(3, 3);
  }

  rotor_velocities = new doublereal[numRtrPtsElem];
  rotor_dcms = new doublereal[3 * numRtrPtsElem];
  for (int i = 0; i < rotor_node_count; i++)
  {
    Vec3 xcurr = nodes[i + node_count_no_rotors].pNode->GetXCurr();
    rotor_points[3 * i] = xcurr[0];
    rotor_points[3 * i + 1] = xcurr[1];
    rotor_points[3 * i + 2] = xcurr[2];

    Vec3 vcurr = nodes[i + node_count_no_rotors].pNode->GetVCurr();
    rotor_velocities[3 * i] = vcurr[0];
    rotor_velocities[3 * i + 1] = vcurr[1];
    rotor_velocities[3 * i + 2] = vcurr[2];

    // these are dGet(row, col)
    Mat3x3 rnode = nodes[i + node_count_no_rotors].pNode->GetRCurr();
    rotor_dcms[9 * i] = rnode.dGet(1, 1);
    rotor_dcms[9 * i + 1] = rnode.dGet(1, 2);
    rotor_dcms[9 * i + 2] = rnode.dGet(1, 3);
    rotor_dcms[9 * i + 3] = rnode.dGet(2, 1);
    rotor_dcms[9 * i + 4] = rnode.dGet(2, 2);
    rotor_dcms[9 * i + 5] = rnode.dGet(2, 3);
    rotor_dcms[9 * i + 6] = rnode.dGet(3, 1);
    rotor_dcms[9 * i + 7] = rnode.dGet(3, 2);
    rotor_dcms[9 * i + 8] = rnode.dGet(3, 3);
  }

  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_AssRes(&t,
               &first_iteration,
               &numRtSpdRtrElem, // length of next array
               RtSpd_PyRtr,      // array with rotor speeds
               ground_station_point,
               FusO_prev,
               FusO,
               FusODCM_prev,
               FusOv_prev,
               FusOomegas_prev,
               FusOacc_prev,
               &numNodePtElem,
               node_points,
               &numNodePtElem,
               node_velocities,
               &numNodePtElem,
               node_omegas,
               &numDCMElem,
               node_dcms,
               &numRtrPtsElem,
               rotor_points,
               rotor_velocities,
               rotor_dcms,
               &numNodeLoadsElem,
               nodeLoads,
               &numRtrLoadsElem,
               rotorLoads,
               &error_status,
               error_message);
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  delete node_velocities;
  delete node_omegas;
  delete rotor_velocities;
  delete rotor_dcms;
}

SubVectorHandler &ModuleKiteFAST::AssRes(SubVectorHandler &WorkVec, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr)
{
  printdebug("AssRes");
  integer numNodeLoadsElem = 6 * node_count_no_rotors;  // force and moment components for each node
  doublereal *nodeLoads = new doublereal[numNodeLoadsElem];
  integer numRtrLoadsElem = 6 * rotor_node_count;  // force and moment components for each rotor node
  
  doublereal *rotorLoads = new doublereal[numRtrLoadsElem];

  _AssRes(0, numNodeLoadsElem, nodeLoads, numRtrLoadsElem, rotorLoads);

  integer iNumRows, iNumCols;
  WorkSpaceDim(&iNumRows, &iNumCols);
  WorkVec.ResizeReset(iNumRows);

  for (int i = 0; i < node_count_no_rotors; i++)
  {
    // set indices where force/moment need to be put
    integer first_index = nodes[i].pNode->iGetFirstMomentumIndex();
    for (int j = 1; j <= 6; j++)
    {
      WorkVec.PutRowIndex(6 * i + j, first_index + j);
    }

    Vec3 force = Vec3(nodeLoads[6 * i + 0], nodeLoads[6 * i + 1], nodeLoads[6 * i + 2]);
    Vec3 moment = Vec3(nodeLoads[6 * i + 3], nodeLoads[6 * i + 4], nodeLoads[6 * i + 5]);
    WorkVec.Add(6 * i + 1, force);
    WorkVec.Add(6 * i + 4, moment);
  }

  for (int i = 0; i < rotor_node_count; i++)
  {
    // set indices where force/moment need to be put
    integer first_index = nodes[node_count_no_rotors + i].pNode->iGetFirstMomentumIndex();
    for (int j = 1; j <= 6; j++)
    {
      WorkVec.PutRowIndex(6 * node_count_no_rotors + 6 * i + j, first_index + j);
    }

    Vec3 force = Vec3(rotorLoads[6 * i + 0], rotorLoads[6 * i + 1], rotorLoads[6 * i + 2]);
    Vec3 moment = Vec3(rotorLoads[6 * i + 3], rotorLoads[6 * i + 4], rotorLoads[6 * i + 5]);
    WorkVec.Add(6 * node_count_no_rotors + 6 * i + 1, force);
    WorkVec.Add(6 * node_count_no_rotors + 6 * i + 4, moment);
  }

  delete[] nodeLoads;
  delete[] rotorLoads;

  return WorkVec;
}

void ModuleKiteFAST::BeforePredict(VectorHandler &X, VectorHandler &XP, VectorHandler &XPrev, VectorHandler &XPPrev) const
{
  printdebug("BeforePredict");
}

void ModuleKiteFAST::AfterPredict(VectorHandler &X, VectorHandler &XP)
{
  printdebug("AfterPredict");

  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_AfterPredict(&error_status, error_message);
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }
}

void ModuleKiteFAST::AfterConvergence(const VectorHandler &X, const VectorHandler &XP)
{
  printdebug("AfterConvergence");
}

extern "C" int module_init(const char *module_name, void *pdm, void *php)
{
  UserDefinedElemRead *rf = new UDERead<ModuleKiteFAST>;
  if (!SetUDE("ModuleKiteFAST", rf))
  {
    delete rf;
    silent_cerr("module-kitefastmbd: module_init(" << module_name << ") failed" << std::endl);
    return -1;
  }
  return 0;
}

// helper functions while in development
void ModuleKiteFAST::printdebug(std::string debugstring) const
{
#ifdef DEBUGUDE
  silent_cout("****** " << debugstring << "\t" << Time.dGet() << std::endl);
#endif
}

void ModuleKiteFAST::PrintNodeLocations(KiteFASTNode node)
{
  Vec3 location = node.pNode->GetXCurr();
  Vec3 accel = node.pNode->GetXPPCurr();
  printf("x: %f\ty: %f\tz: %f\tx\": %f\ty\": %f\tz:\"%f\n", location[0], location[1], location[2], accel[0], accel[1], accel[2]);
}

// these are specific for mbdyn, not used by us or KiteFAST
unsigned int ModuleKiteFAST::iGetNumPrivData(void) const
{
  printdebug("iGetNumPrivData");
  return 0;
}

void ModuleKiteFAST::GetConnectedNodes(std::vector<const Node *> &connectedNodes) const
{
  printdebug("GetConnectedNodes");
  connectedNodes.resize(nodes.size());
  for (int n = 0; n < nodes.size(); n++)
  {
    connectedNodes[n] = nodes[n].pNode;
  }
}

std::ostream &ModuleKiteFAST::Restart(std::ostream &out) const
{
  printdebug("Restart");
  return out << "module-kitefastmbd not implemented" << std::endl;
}

unsigned int ModuleKiteFAST::iGetInitialNumDof(void) const
{
  printdebug("iGetInitialNumDof");
  return 0;
}

void ModuleKiteFAST::InitialWorkSpaceDim(integer *piNumRows, integer *piNumCols) const
{
  printdebug("InitialWorkSpaceDim");
  *piNumRows = 0;
  *piNumCols = 0;
}

VariableSubMatrixHandler &ModuleKiteFAST::InitialAssJac(VariableSubMatrixHandler &WorkMat, const VectorHandler &XCurr)
{
  printdebug("InitialAssJac");

  // should not be called, since initial workspace is empty
  ASSERT(0);
  WorkMat.SetNullMatrix();
  return WorkMat;
}

SubVectorHandler &ModuleKiteFAST::InitialAssRes(SubVectorHandler &WorkVec, const VectorHandler &XCurr)
{
  printdebug("InitialAssRes");

  // should not be called, since initial workspace is empty
  ASSERT(0);
  WorkVec.ResizeReset(0);
  return WorkVec;
}
