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

// #define DEBUGUDE

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

  // store the data manager on the class to use in AssRes
  data_manager = pDM;

  // setup the time drive
  Time.Set(new TimeDriveCaller(pDM->pGetDrvHdl()));
  printdebug("initial time ");

  // *** input parsing ***

  // parse the kitefast module flags
  // 0 = off, 1 = on
  ValidateInputKeyword(HP, "fast_submodule_flags");
  kitefast_module_flags[0] = HP.GetInt(); // kiteaerodyn
  kitefast_module_flags[1] = HP.GetInt(); // inflowwind
  kitefast_module_flags[2] = HP.GetInt(); // moordyn
  kitefast_module_flags[3] = HP.GetInt(); // controller

  // parse the kitefast module input files
  ValidateInputKeyword(HP, "fast_submodule_input_files");
  strcpy(kiteaerodyn_filename, HP.GetFileName());
  strcpy(inflowwind_filename, HP.GetFileName());
  strcpy(moordyn_filename, HP.GetFileName());
  strcpy(controller_filename, HP.GetFileName());

  // parse the output file settings and initialize the output file
  ValidateInputKeyword(HP, "output_file_root");
  strcpy(output_file_root, HP.GetFileName());
  std::string output_file_name = output_file_root;
  output_file_name.append("MBD.out");
  InitOutputFile(output_file_name);

  // parse the initial time
  ValidateInputKeyword(HP, "initial_time");
  initial_time = HP.GetReal();

  // parse the time step
  ValidateInputKeyword(HP, "time_step");
  time_step = HP.GetReal();

  // parse the gravity
  ValidateInputKeyword(HP, "gravity");
  gravity = HP.GetReal();

  // parse the ground station location
  ValidateInputKeyword(HP, "ground_weather_station_location");
  ground_station_point[0] = HP.GetReal();
  ground_station_point[1] = HP.GetReal();
  ground_station_point[2] = HP.GetReal();

  // parse the component counts
  ValidateInputKeyword(HP, "number_of_flaps_per_wing");
  n_flaps_per_wing = HP.GetInt();
  ValidateInputKeyword(HP, "number_of_pylons_per_wing");
  n_pylons_per_wing = HP.GetInt();

  // n_components includes all components except the rotors
  ValidateInputKeyword(HP, "number_of_kite_components");
  n_components = HP.GetInt();

  // parse the keypoints (aka reference points)
  ValidateInputKeyword(HP, "keypoints");
  reference_points = new doublereal[3 * n_components];
  for (int i = 0; i < n_components; i++)
  {
    reference_points[3 * i] = HP.GetReal();
    reference_points[3 * i + 1] = HP.GetReal();
    reference_points[3 * i + 2] = HP.GetReal();
  }

  // parse the mip node
  ValidateInputKeyword(HP, "mip_node");
  mip_node.pNode = dynamic_cast<StructNode *>(pDM->ReadNode(HP, Node::STRUCTURAL));
  
  // parse the component nodes and beams into arrays
  BuildComponentArrays(pDM, HP, "fuselage", nodes_fuselage, beams_fuselage);
  BuildComponentArrays(pDM, HP, "wing_starboard", nodes_starwing, beams_starwing);
  BuildComponentArrays(pDM, HP, "wing_port", nodes_portwing, beams_portwing);
  BuildComponentArrays(pDM, HP, "vertical_stabilizer", nodes_vstab, beams_vstab);
  BuildComponentArrays(pDM, HP, "horizontal_stabilizer_starboard", nodes_starhstab, beams_starhstab);
  BuildComponentArrays(pDM, HP, "horizontal_stabilizer_port", nodes_porthstab, beams_porthstab);

  nodes_starpylons.resize(n_pylons_per_wing);
  beams_starpylons.resize(n_pylons_per_wing);
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    std::string component_name = "pylon_starboard_";
    component_name.append(SSTR(i + 1));
    BuildComponentArrays(pDM, HP, component_name.c_str(), nodes_starpylons[i], beams_starpylons[i]);
  }
  nodes_portpylons.resize(n_pylons_per_wing);
  beams_portpylons.resize(n_pylons_per_wing);
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    std::string component_name = "pylon_port_";
    component_name.append(SSTR(i + 1));
    BuildComponentArrays(pDM, HP, component_name.c_str(), nodes_portpylons[i], beams_portpylons[i]);
  }
  BuildComponentArrays(pDM, HP, "starboard_rotors", nodes_starrotors, beams_throwaway);
  BuildComponentArrays(pDM, HP, "port_rotors", nodes_portrotors, beams_throwaway);

  // build a single node array
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

  // build a single beam array
  total_beam_count = beams_fuselage.size()
                   + beams_starwing.size()
                   + beams_portwing.size()
                   + beams_vstab.size()
                   + beams_starhstab.size()
                   + beams_porthstab.size();
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    total_beam_count += beams_portpylons[i].size() + beams_portpylons[i].size();
  }

  beams.reserve(total_beam_count);
  beams.insert(beams.begin(), beams_fuselage.begin(), beams_fuselage.end());
  beams.insert(beams.end(), beams_starwing.begin(), beams_starwing.end());
  beams.insert(beams.end(), beams_portwing.begin(), beams_portwing.end());
  beams.insert(beams.end(), beams_vstab.begin(), beams_vstab.end());
  beams.insert(beams.end(), beams_starhstab.begin(), beams_starhstab.end());
  beams.insert(beams.end(), beams_porthstab.begin(), beams_porthstab.end());
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    beams.insert(beams.end(), beams_starpylons[i].begin(), beams_starpylons[i].end());
  }
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    beams.insert(beams.end(), beams_portpylons[i].begin(), beams_portpylons[i].end());
  }

  // number of nodes per kite component excluding rotors
  component_node_counts = new integer[n_components];
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
  node_points = new doublereal[3 * node_count_no_rotors];
  node_dcms = new doublereal[9 * node_count_no_rotors];

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
  n_rotor_points = nodes_starrotors.size() + nodes_portrotors.size();
  rotor_points = new doublereal[3 * n_rotor_points];
  for (int i = 0; i < n_rotor_points; i++)
  {
    Vec3 xcurr = nodes[i + node_count_no_rotors].pNode->GetXCurr();
    rotor_points[3 * i] = xcurr[0];
    rotor_points[3 * i + 1] = xcurr[1];
    rotor_points[3 * i + 2] = xcurr[2];
  }

  // The kite is aligned with the Global Coordinate system
  Mat3x3 mip_dcm = mip_node.pNode->GetRCurr();
  fuselage_dcm[0] = mip_dcm.dGet(1, 1);
  fuselage_dcm[1] = mip_dcm.dGet(1, 2);
  fuselage_dcm[2] = mip_dcm.dGet(1, 3);
  fuselage_dcm[3] = mip_dcm.dGet(2, 1);
  fuselage_dcm[4] = mip_dcm.dGet(2, 2);
  fuselage_dcm[5] = mip_dcm.dGet(2, 3);
  fuselage_dcm[6] = mip_dcm.dGet(3, 1);
  fuselage_dcm[7] = mip_dcm.dGet(3, 2);
  fuselage_dcm[8] = mip_dcm.dGet(3, 3);

  // call KFAST_Init method
  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_Init(&time_step,
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
             &print_summary_file,
             &gravity,
             ground_station_point,
             fuselage_dcm,
             &n_rotor_points,
             rotor_points,
             rotor_masses,
             rotor_rotational_inertias,
             rotor_translational_inertias,
             rotor_cm_offsets,
             reference_points,
             &node_count_no_rotors,
             node_points,
             node_dcms,
             &n_fuselage_outputs,
             fuselage_output_nodes.data(),
             &n_starboard_wing_outputs,
             starboard_wing_output_nodes.data(),
             &n_port_wing_outputs,
             port_wing_output_nodes.data(),
             &n_vertical_stabilizer_outputs,
             vertical_stabilizer_output_nodes.data(),
             &n_starboard_horizontal_stabilizer_outputs,
             starboard_horizontal_stabilizer_output_nodes.data(),
             &n_port_horizontal_stabilizer_outputs,
             port_horizontal_stabilizer_output_nodes.data(),
             &n_pylon_outputs,
             pylon_output_nodes.data(),
             &n_output_channels,
             output_channels_array,
             &error_status,
             error_message);
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }
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

void ModuleKiteFAST::BuildComponentArrays(DataManager *pDM, MBDynParser &HP,
                                          const char *keyword,
                                          std::vector<KiteFASTNode> &node_array,
                                          std::vector<KiteFASTBeam> &beam_array)
{
  printdebug("BuildComponentArrays");
  printdebug(keyword);
  if (!HP.IsKeyWord(keyword))
  {
    silent_cerr("Runtime Error: cannot read keyword " << keyword << std::endl);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  BuildComponentNodeArray(pDM, HP, node_array);

  // if keyword contains "rotor", dont read beams
  std::string kwd(keyword);
  if (kwd.find("rotor") != std::string::npos) return;
  
  BuildComponentBeamArray(pDM, HP, beam_array);
}

void ModuleKiteFAST::BuildComponentNodeArray(DataManager *pDM,
                                             MBDynParser &HP,
                                             std::vector<KiteFASTNode> &node_array)
{
  printdebug("BuildComponentNodeArray");

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

void ModuleKiteFAST::BuildComponentBeamArray(DataManager *pDM,
                                             MBDynParser &HP,
                                             std::vector<KiteFASTBeam> &beam_array)
{
  printdebug("BuildComponentBeamArray");

  int beam_count = HP.GetInt();
  beam_array.resize(beam_count);
  for (int i = 0; i < beam_count; i++)
  {
    beam_array[i].pBeam = dynamic_cast<Beam *>(pDM->ReadElem(HP, Beam::BEAM));
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

  for (int i = 0; i < total_beam_count; i++)
  {
    printf("beam #%d ", i);
    integer index = beams[i].pBeam->iGetPrivDataIdx("pI.Fx");
    doublereal value = beams[i].pBeam->dGetPrivData(index);
    printf("pI.Fx = %f ", value);
    index = beams[i].pBeam->iGetPrivDataIdx("pII.Fx");
    value = beams[i].pBeam->dGetPrivData(index);
    printf("pII.Fx = %f\n", value);
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

void ModuleKiteFAST::_AssRes(doublereal *node_loads, doublereal *rotor_loads)
{
  printdebug("_AssRes");
  
  // exit if the input time step and mbdyn's time step do not match
  if (time_step != data_manager->pGetDrvHdl()->dGetTimeStep())
  {
    silent_cout("The given KiteFAST time step does match the mbdyn time step." << std::endl);
    silent_cout("KiteFAST: " << time_step << std::endl);
    silent_cout("MBDyn: " << data_manager->pGetDrvHdl()->dGetTimeStep() << std::endl);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  doublereal t = Time.dGet();
  first_iteration = 0;  // 0 no - 1 yes
  if (t == initial_time) {
    first_iteration = 1;
  }

  // fuselage quantities refer to the MIP node
  Vec3 vec3_fusOprev = mip_node.pNode->GetXPrev();
  fuselage_position_prev[0] = vec3_fusOprev[0];
  fuselage_position_prev[1] = vec3_fusOprev[1];
  fuselage_position_prev[2] = vec3_fusOprev[2];

  Vec3 vec3_fusO = mip_node.pNode->GetXCurr();  
  fuselage_position[0] = vec3_fusO[0];
  fuselage_position[1] = vec3_fusO[1];
  fuselage_position[2] = vec3_fusO[2];

  Mat3x3 fus0_dcm_prev = mip_node.pNode->GetRPrev();
  fuselage_dcm_prev[0] = fus0_dcm_prev.dGet(1, 1);
  fuselage_dcm_prev[1] = fus0_dcm_prev.dGet(1, 2);
  fuselage_dcm_prev[2] = fus0_dcm_prev.dGet(1, 3);
  fuselage_dcm_prev[3] = fus0_dcm_prev.dGet(2, 1);
  fuselage_dcm_prev[4] = fus0_dcm_prev.dGet(2, 2);
  fuselage_dcm_prev[5] = fus0_dcm_prev.dGet(2, 3);
  fuselage_dcm_prev[6] = fus0_dcm_prev.dGet(3, 1);
  fuselage_dcm_prev[7] = fus0_dcm_prev.dGet(3, 2);
  fuselage_dcm_prev[8] = fus0_dcm_prev.dGet(3, 3);

  Mat3x3 fus0_dcm = mip_node.pNode->GetRCurr();
  fuselage_dcm[0] = fus0_dcm.dGet(1, 1);
  fuselage_dcm[1] = fus0_dcm.dGet(1, 2);
  fuselage_dcm[2] = fus0_dcm.dGet(1, 3);
  fuselage_dcm[3] = fus0_dcm.dGet(2, 1);
  fuselage_dcm[4] = fus0_dcm.dGet(2, 2);
  fuselage_dcm[5] = fus0_dcm.dGet(2, 3);
  fuselage_dcm[6] = fus0_dcm.dGet(3, 1);
  fuselage_dcm[7] = fus0_dcm.dGet(3, 2);
  fuselage_dcm[8] = fus0_dcm.dGet(3, 3);

  Vec3 vec3_FusOv_prev = mip_node.pNode->GetVPrev();
  fuselage_vels_prev[0] = vec3_FusOv_prev[0];
  fuselage_vels_prev[1] = vec3_FusOv_prev[1];
  fuselage_vels_prev[2] = vec3_FusOv_prev[2];

  Vec3 vec3_FusOv = mip_node.pNode->GetVCurr();
  fuselage_vels[0] = vec3_FusOv[0];
  fuselage_vels[1] = vec3_FusOv[1];
  fuselage_vels[2] = vec3_FusOv[2];

  Vec3 vec3_FusOomegas_prev = mip_node.pNode->GetWPrev();
  fuselage_omegas_prev[0] = vec3_FusOomegas_prev[0];
  fuselage_omegas_prev[1] = vec3_FusOomegas_prev[1];
  fuselage_omegas_prev[2] = vec3_FusOomegas_prev[2];

  Vec3 vec3_FusOomegas = mip_node.pNode->GetWCurr();
  fuselage_omegas[0] = vec3_FusOomegas[0];
  fuselage_omegas[1] = vec3_FusOomegas[1];
  fuselage_omegas[2] = vec3_FusOomegas[2];

  Vec3 vec3_FusOacc_prev = mip_node.pNode->GetXPPPrev();
  fuselage_accs_prev[0] = vec3_FusOacc_prev[0];
  fuselage_accs_prev[1] = vec3_FusOacc_prev[1];
  fuselage_accs_prev[2] = vec3_FusOacc_prev[2];

  Vec3 vec3_FusOacc = mip_node.pNode->GetXPPCurr();
  fuselage_accs[0] = vec3_FusOacc[0];
  fuselage_accs[1] = vec3_FusOacc[1];
  fuselage_accs[2] = vec3_FusOacc[2];

  // TODO: what are alphas??
  fuselage_alphas[0] = 0;
  fuselage_alphas[1] = 0;
  fuselage_alphas[2] = 0;

  node_vels = new doublereal[3 * node_count_no_rotors];
  node_omegas = new doublereal[3 * node_count_no_rotors];
  node_accs = new doublereal[3 * node_count_no_rotors];
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

    Vec3 vcurr = nodes[i].pNode->GetVCurr();
    node_vels[3 * i] = vcurr[0];
    node_vels[3 * i + 1] = vcurr[1];
    node_vels[3 * i + 2] = vcurr[2];

    Vec3 wcurr = nodes[i].pNode->GetWCurr();
    node_omegas[3 * i] = wcurr[0];
    node_omegas[3 * i + 1] = wcurr[1];
    node_omegas[3 * i + 2] = wcurr[2];

    Vec3 acc_curr = nodes[i].pNode->GetXPPCurr();
    node_accs[3 * i] = acc_curr[0];
    node_accs[3 * i + 1] = acc_curr[1];
    node_accs[3 * i + 2] = acc_curr[2];
  }

  rotor_dcms = new doublereal[9 * n_rotor_points];
  rotor_vels = new doublereal[3 * n_rotor_points];
  rotor_omegas = new doublereal[3 * n_rotor_points];
  rotor_accs = new doublereal[3 * n_rotor_points];
  for (int i = 0; i < n_rotor_points; i++)
  {
    Vec3 xcurr = nodes[i + node_count_no_rotors].pNode->GetXCurr();
    rotor_points[3 * i] = xcurr[0];
    rotor_points[3 * i + 1] = xcurr[1];
    rotor_points[3 * i + 2] = xcurr[2];

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

    Vec3 vcurr = nodes[i + node_count_no_rotors].pNode->GetVCurr();
    rotor_vels[3 * i] = vcurr[0];
    rotor_vels[3 * i + 1] = vcurr[1];
    rotor_vels[3 * i + 2] = vcurr[2];

    Vec3 wcurr = nodes[i + node_count_no_rotors].pNode->GetWCurr();
    rotor_omegas[3 * i] = wcurr[0];
    rotor_omegas[3 * i + 1] = wcurr[1];
    rotor_omegas[3 * i + 2] = wcurr[2];

    Vec3 acc_curr = nodes[i + node_count_no_rotors].pNode->GetXPPCurr();
    rotor_accs[3 * i] = acc_curr[0];
    rotor_accs[3 * i + 1] = acc_curr[1];
    rotor_accs[3 * i + 2] = acc_curr[2];
  }

  // TODO: what are alphas??
  rotor_alphas[0] = 0;
  rotor_alphas[1] = 0;
  rotor_alphas[2] = 0;

  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_AssRes(&t,
               &first_iteration,
               ground_station_point,
               fuselage_position_prev,
               fuselage_position,
               fuselage_dcm_prev,
               fuselage_dcm,
               fuselage_vels_prev,
               fuselage_vels,
               fuselage_omegas_prev,
               fuselage_omegas,
               fuselage_accs_prev,
               fuselage_accs,
               fuselage_alphas,
               &node_count_no_rotors,
               node_points,
               node_dcms,
               node_vels,
               node_omegas,
               node_accs,
               &n_rotor_points,
               rotor_points,
               rotor_dcms,
               rotor_vels,
               rotor_omegas,
               rotor_accs,
               rotor_alphas,
               node_loads,
               rotor_loads,
               &error_status,
               error_message);
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  delete[] node_vels;
  delete[] node_omegas;
  delete[] rotor_dcms;
  delete[] rotor_vels;
  delete[] rotor_omegas;
  delete[] rotor_accs;
}

SubVectorHandler &ModuleKiteFAST::AssRes(SubVectorHandler &WorkVec, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr)
{
  printdebug("AssRes");

  // get the loads from KFAST_AssRes and apply to the mbdyn model
  integer n_node_loads = 6 * node_count_no_rotors;  // force and moment components for each node
  node_loads = new doublereal[n_node_loads];

  integer n_rotor_loads = 6 * n_rotor_points;  // force and moment components for each rotor node
  rotor_loads = new doublereal[n_rotor_loads];

  _AssRes(node_loads, rotor_loads);

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

    Vec3 force = Vec3(node_loads[6 * i + 0], node_loads[6 * i + 1], node_loads[6 * i + 2]);
    Vec3 moment = Vec3(node_loads[6 * i + 3], node_loads[6 * i + 4], node_loads[6 * i + 5]);
    WorkVec.Add(6 * i + 1, force);
    WorkVec.Add(6 * i + 4, moment);
  }

  for (int i = 0; i < n_rotor_points; i++)
  {
    // set indices where force/moment need to be put
    integer first_index = nodes[node_count_no_rotors + i].pNode->iGetFirstMomentumIndex();
    for (int j = 1; j <= 6; j++)
    {
      WorkVec.PutRowIndex(6 * node_count_no_rotors + 6 * i + j, first_index + j);
    }

    Vec3 force = Vec3(rotor_loads[6 * i + 0], rotor_loads[6 * i + 1], rotor_loads[6 * i + 2]);
    Vec3 moment = Vec3(rotor_loads[6 * i + 3], rotor_loads[6 * i + 4], rotor_loads[6 * i + 5]);
    WorkVec.Add(6 * node_count_no_rotors + 6 * i + 1, force);
    WorkVec.Add(6 * node_count_no_rotors + 6 * i + 4, moment);
  }

  delete[] node_loads;
  delete[] rotor_loads;

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
  silent_cout("****** " << debugstring << "\t (time: " << Time.dGet() << ")" << std::endl);
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
