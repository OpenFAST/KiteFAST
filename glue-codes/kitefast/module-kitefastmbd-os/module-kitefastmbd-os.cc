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

#include "module-kitefastmbd-os.h"

// #define DEBUGUDE

ModuleKiteFASTOS::ModuleKiteFASTOS(unsigned uLabel, const DofOwner *pDO, DataManager *pDM, MBDynParser &HP) : Elem(uLabel, flag(0)), UserDefinedElem(uLabel, pDO)
{
  if (!HP.IsArg())
  {
    silent_cout("No Arguments provided for ModuleKiteFASTOS for UDE IDE=" << uLabel << std::endl);
    throw NoErr(MBDYN_EXCEPT_ARGS);
  }

  if (HP.IsKeyWord("help"))
  {
    silent_cout(std::endl
                << "Module: KiteFastMBD" << std::endl
                << "Author: Rafael Mudafort <rafael.mudafort@nrel.gov>" << std::endl
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

  // set this variable for use in AssRes
  first_iteration = 1;

  // parse the simulation type
  ValidateInputKeyword(HP, "simulation_type");
  integer simulation_type = HP.GetReal();

  // parse the kitefast module flags
  // 0 = off, 1 = on
  ValidateInputKeyword(HP, "fast_submodule_flags");
  integer kitefast_module_flags[6];
  kitefast_module_flags[0] = HP.GetInt(); // kiteaerodyn
  kitefast_module_flags[1] = HP.GetInt(); // inflowwind
  kitefast_module_flags[2] = HP.GetInt(); // moordyn tether
  kitefast_module_flags[3] = HP.GetInt(); // controller
  kitefast_module_flags[4] = HP.GetInt(); // hydrodyn
  kitefast_module_flags[5] = HP.GetInt(); // moordyn mooring

  // parse the kitefast module input files
  char kiteaerodyn_filename[INTERFACE_STRING_LENGTH];
  char inflowwind_filename[INTERFACE_STRING_LENGTH];
  char moordyn_tether_filename[INTERFACE_STRING_LENGTH];
  char controller_filename[INTERFACE_STRING_LENGTH];
  char hydrodyn_filename[INTERFACE_STRING_LENGTH];
  char moordyn_mooring_filename[INTERFACE_STRING_LENGTH];
  char output_file_root[INTERFACE_STRING_LENGTH];
  ValidateInputKeyword(HP, "fast_submodule_input_files");
  strcpy(kiteaerodyn_filename, HP.GetFileName());
  strcpy(inflowwind_filename, HP.GetFileName());
  strcpy(moordyn_tether_filename, HP.GetFileName());
  strcpy(controller_filename, HP.GetFileName());
  strcpy(hydrodyn_filename, HP.GetFileName());
  strcpy(moordyn_mooring_filename, HP.GetFileName());

  // parse the output file settings and initialize the output file
  ValidateInputKeyword(HP, "output_file_root");
  strcpy(output_file_root, HP.GetFileName());

  // parse the flag to print the kitefast summary file
  // 0 = off, 1 = on
  ValidateInputKeyword(HP, "print_kitefast_summary_file");
  integer print_summary_file = HP.GetInt();

  // parse the time step
  ValidateInputKeyword(HP, "time_step");
  time_step = HP.GetReal();

  // parse the max time
  ValidateInputKeyword(HP, "time_max");
  doublereal time_max = HP.GetReal();

  // parse the gravity
  ValidateInputKeyword(HP, "gravity");
  doublereal gravity = HP.GetReal();

  // parse the KiteAeroDyn Outputs interpolation order.  0 = hold KAD outputs between KAD calls,
  //   1 = Linearly interpolate outputs, 2 = 2nd order interpolation of outputs
  ValidateInputKeyword(HP, "kiteaerodyn_interpolation_order");
  integer KAD_interpolation_order = HP.GetInt();

  // parse the wind reference station location
  ValidateInputKeyword(HP, "wind_reference_station_node");
  wind_reference_station_node.pNode = dynamic_cast<StructNode *>(pDM->ReadNode(HP, Node::STRUCTURAL));
  Vec3 wind_reference_xcurr = wind_reference_station_node.pNode->GetXCurr();
  doublereal wind_reference_station_position[3];
  wind_reference_station_position[0] = wind_reference_xcurr[0];
  wind_reference_station_position[1] = wind_reference_xcurr[1];
  wind_reference_station_position[2] = wind_reference_xcurr[2];

  // parse the ground station location
  ValidateInputKeyword(HP, "ground_station_node");
  doublereal ground_station_reference_point[3];
  ground_station_reference_point[0] = HP.GetReal();
  ground_station_reference_point[1] = HP.GetReal();
  ground_station_reference_point[2] = HP.GetReal();

  // parse the component counts
  ValidateInputKeyword(HP, "number_of_flaps_per_wing");
  integer n_flaps_per_wing = HP.GetInt();
  ValidateInputKeyword(HP, "number_of_pylons_per_wing");
  integer n_pylons_per_wing = HP.GetInt();

  // n_components includes all components except the rotors
  ValidateInputKeyword(HP, "number_of_kite_components");
  integer n_components = HP.GetInt();

  // parse the keypoints (aka reference points)
  ValidateInputKeyword(HP, "keypoints");
  doublereal *reference_points = new doublereal[3 * n_components];
  doublereal *platform_reference_point = new doublereal[3];
  for (int i = 0; i < n_components; i++)
  {
    reference_points[3 * i] = HP.GetReal();
    reference_points[3 * i + 1] = HP.GetReal();
    reference_points[3 * i + 2] = HP.GetReal();
  }
  platform_reference_point[0] = HP.GetReal();
  platform_reference_point[1] = HP.GetReal();
  platform_reference_point[2] = HP.GetReal();

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
    component_name.append(std::to_string(i + 1));
    BuildComponentArrays(pDM, HP, component_name.c_str(), nodes_starpylons[i], beams_starpylons[i]);
  }
  nodes_portpylons.resize(n_pylons_per_wing);
  beams_portpylons.resize(n_pylons_per_wing);
  for (int i = 0; i < n_pylons_per_wing; i++)
  {
    std::string component_name = "pylon_port_";
    component_name.append(std::to_string(i + 1));
    BuildComponentArrays(pDM, HP, component_name.c_str(), nodes_portpylons[i], beams_portpylons[i]);
  }
  BuildComponentArrays(pDM, HP, "starboard_rotors", nodes_starrotors, beams_throwaway);
  BuildComponentArrays(pDM, HP, "port_rotors", nodes_portrotors, beams_throwaway);

  BuildComponentArrays(pDM, HP, "platform", nodes_platform, beams_throwaway);
  platform_node = nodes_platform[0];

  // build a single node array
  integer total_node_count = nodes_fuselage.size() + nodes_starwing.size() + nodes_portwing.size() + nodes_vstab.size() + nodes_starhstab.size() + nodes_porthstab.size();
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
  total_beam_count = beams_fuselage.size() + beams_starwing.size() + beams_portwing.size() + beams_vstab.size() + beams_starhstab.size() + beams_porthstab.size();
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
  integer *component_node_counts = new integer[n_components];
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

  // build the node points and properties arrays for kite components excluding rotors
  node_count_no_rotors = 0;
  for (int i = 0; i < 6 + 2 * n_pylons_per_wing; i++)
  {
    node_count_no_rotors += component_node_counts[i];
  }
  doublereal *node_points = new doublereal[3 * node_count_no_rotors];
  doublereal *node_dcms = new doublereal[9 * node_count_no_rotors];

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
  doublereal *platform_mip_dcm = new doublereal[9];
  Mat3x3 platform_rnode = platform_node.pNode->GetRCurr();
  platform_mip_dcm[0] = platform_rnode.dGet(1, 1);
  platform_mip_dcm[1] = platform_rnode.dGet(1, 2);
  platform_mip_dcm[2] = platform_rnode.dGet(1, 3);
  platform_mip_dcm[3] = platform_rnode.dGet(2, 1);
  platform_mip_dcm[4] = platform_rnode.dGet(2, 2);
  platform_mip_dcm[5] = platform_rnode.dGet(2, 3);
  platform_mip_dcm[6] = platform_rnode.dGet(3, 1);
  platform_mip_dcm[7] = platform_rnode.dGet(3, 2);
  platform_mip_dcm[8] = platform_rnode.dGet(3, 3);

  // The kite is aligned with the Global Coordinate system
  Mat3x3 mip_dcm_matrix = mip_node.pNode->GetRCurr();
  doublereal mip_dcm[9];
  mip_dcm[0] = mip_dcm_matrix.dGet(1, 1);
  mip_dcm[1] = mip_dcm_matrix.dGet(1, 2);
  mip_dcm[2] = mip_dcm_matrix.dGet(1, 3);
  mip_dcm[3] = mip_dcm_matrix.dGet(2, 1);
  mip_dcm[4] = mip_dcm_matrix.dGet(2, 2);
  mip_dcm[5] = mip_dcm_matrix.dGet(2, 3);
  mip_dcm[6] = mip_dcm_matrix.dGet(3, 1);
  mip_dcm[7] = mip_dcm_matrix.dGet(3, 2);
  mip_dcm[8] = mip_dcm_matrix.dGet(3, 3);

  // get the rotor properties (points, mass, inertia, and cm offset arrays)
  n_rotor_points = 4 * n_pylons_per_wing;
  doublereal *rotor_points = new doublereal[3 * n_rotor_points];
  doublereal *rotor_masses = new doublereal[n_rotor_points];
  doublereal *rotor_rotational_inertias = new doublereal[n_rotor_points];
  doublereal *rotor_translational_inertias = new doublereal[n_rotor_points];
  doublereal *rotor_cm_offsets = new doublereal[3 * n_rotor_points];

  for (int i = 0; i < n_rotor_points; i++)
  {
    Vec3 xcurr = nodes[i + node_count_no_rotors].pNode->GetXCurr();
    rotor_points[3 * i] = xcurr[0];
    rotor_points[3 * i + 1] = xcurr[1];
    rotor_points[3 * i + 2] = xcurr[2];
  }

  ValidateInputKeyword(HP, "starboard_rotor_properties");
  for (int i = 0; i < n_rotor_points / 2; i++)
  {
    rotor_masses[i] = HP.GetReal();
    rotor_rotational_inertias[i] = HP.GetReal();
    rotor_translational_inertias[i] = HP.GetReal();
    rotor_cm_offsets[3 * i + 0] = HP.GetReal();
    rotor_cm_offsets[3 * i + 1] = HP.GetReal();
    rotor_cm_offsets[3 * i + 2] = HP.GetReal();
  }

  ValidateInputKeyword(HP, "port_rotor_properties");
  for (int i = n_rotor_points / 2; i < n_rotor_points; i++)
  {
    rotor_masses[i] = HP.GetReal();
    rotor_rotational_inertias[i] = HP.GetReal();
    rotor_translational_inertias[i] = HP.GetReal();
    rotor_cm_offsets[3 * i + 0] = HP.GetReal();
    rotor_cm_offsets[3 * i + 1] = HP.GetReal();
    rotor_cm_offsets[3 * i + 2] = HP.GetReal();
  }

  // parse the output information
  integer n_fuselage_outputs, n_starboard_wing_outputs, n_port_wing_outputs;
  integer n_vertical_stabilizer_outputs;
  integer n_starboard_horizontal_stabilizer_outputs, n_port_horizontal_stabilizer_outputs;
  integer n_pylon_outputs;
  std::vector<integer> fuselage_output_nodes, starboard_wing_output_nodes, port_wing_output_nodes;
  std::vector<integer> vertical_stabilizer_output_nodes;
  std::vector<integer> starboard_horizontal_stabilizer_output_nodes, port_horizontal_stabilizer_output_nodes;
  std::vector<integer> pylon_output_nodes;

  integer n_output_channels;
  std::vector<char *> output_channel_array;
  integer *output_channel_lengths;

  BuildComponentOutputArray(HP, "fuselage_outputs", n_fuselage_outputs, fuselage_output_nodes);
  BuildComponentOutputArray(HP, "wing_starboard_outputs", n_starboard_wing_outputs, starboard_wing_output_nodes);
  BuildComponentOutputArray(HP, "wing_port_outputs", n_port_wing_outputs, port_wing_output_nodes);
  BuildComponentOutputArray(HP, "vertical_stabilizer_outputs", n_vertical_stabilizer_outputs, vertical_stabilizer_output_nodes);
  BuildComponentOutputArray(HP, "horizontal_stabilizer_starboard_outputs", n_starboard_horizontal_stabilizer_outputs, starboard_horizontal_stabilizer_output_nodes);
  BuildComponentOutputArray(HP, "horizontal_stabilizer_port_outputs", n_port_horizontal_stabilizer_outputs, port_horizontal_stabilizer_output_nodes);
  BuildComponentOutputArray(HP, "pylon_outputs", n_pylon_outputs, pylon_output_nodes);
  ValidateInputKeyword(HP, "output_channels");

  n_output_channels = HP.GetInt();

  // put the output channel names into a vector<string> and convert to vector<char *>
  std::vector<std::string> output_channel_vector;
  output_channel_vector.resize(n_output_channels);
  output_channel_array.reserve(output_channel_vector.size());
  output_channel_lengths = new integer[n_output_channels];
  for (int i = 0; i < n_output_channels; i++)
  {
    // get the string from the mbdyn input file
    std::string channel_string = HP.GetString();
    output_channel_vector[i] = channel_string;

    // cast the string to char *
    output_channel_array[i] = const_cast<char *>(output_channel_vector[i].c_str());

    // store the length of this channel
    output_channel_lengths[i] = channel_string.length();
  }

  // call KFAST_Init method
  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_OS_Init(&simulation_type,
             &time_step,
             &time_max,
             &n_flaps_per_wing,
             &n_pylons_per_wing,
             &n_components,
             component_node_counts,
             kitefast_module_flags,
             kiteaerodyn_filename,
             inflowwind_filename,
             moordyn_tether_filename,
             controller_filename,
             hydrodyn_filename,
             moordyn_mooring_filename,
             output_file_root,
             &print_summary_file,
             &gravity,
             &KAD_interpolation_order,
             mip_dcm,
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
             platform_reference_point,
             platform_mip_dcm,
             ground_station_reference_point,
             &n_output_channels,
             output_channel_array.data(),
             output_channel_lengths,
             &error_status,
             error_message);
  printdebug("KFAST_Init error");
  printdebug("    status: " + std::to_string(error_status) + ";");
  printdebug("    message: " + std::string(error_message) + ";");
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  delete[] reference_points;
  delete[] platform_reference_point;
  delete[] component_node_counts;
  delete[] node_points;
  delete[] node_dcms;
  delete[] platform_mip_dcm;
  delete[] rotor_points;
  delete[] rotor_masses;
  delete[] rotor_rotational_inertias;
  delete[] rotor_translational_inertias;
  delete[] rotor_cm_offsets;
  delete[] output_channel_lengths;
}

ModuleKiteFASTOS::~ModuleKiteFASTOS(void)
{
  printdebug("~ModuleKiteFASTOS");

  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_OS_End(&error_status, error_message);
  printdebug("KFAST_End error");
  printdebug("    status: " + std::to_string(error_status) + ";");
  printdebug("    message: " + std::string(error_message) + ";");
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }
}

void ModuleKiteFASTOS::SetValue(DataManager *pDM, VectorHandler &X, VectorHandler &XP, SimulationEntity::Hints *ph)
{
  printdebug("SetValue");
  NO_OP;
}

void ModuleKiteFASTOS::ValidateInputKeyword(MBDynParser &HP, const char *keyword)
{
  printdebug("ValidateInputKeyword - " + std::string(keyword));

  if (!HP.IsKeyWord(keyword))
  {
    silent_cerr("Input Error: cannot read keyword " << keyword << std::endl);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }
}

void ModuleKiteFASTOS::BuildComponentArrays(DataManager *pDM, MBDynParser &HP,
                                          const char *keyword,
                                          std::vector<KiteFASTNode> &node_array,
                                          std::vector<KiteFASTBeam> &beam_array)
{
  printdebug("BuildComponentArrays - " + std::string(keyword));

  if (!HP.IsKeyWord(keyword))
  {
    silent_cerr("Runtime Error: cannot read keyword " << keyword << std::endl);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  BuildComponentNodeArray(pDM, HP, node_array);

  // if keyword contains "rotor" or "platform", dont read beams
  std::string kwd(keyword);
  if (kwd.find("rotor") != std::string::npos || kwd.find("platform") != std::string::npos)
    return;

  BuildComponentBeamArray(pDM, HP, beam_array);
}

void ModuleKiteFASTOS::BuildComponentNodeArray(DataManager *pDM,
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

void ModuleKiteFASTOS::BuildComponentBeamArray(DataManager *pDM,
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

void ModuleKiteFASTOS::BuildComponentOutputArray(MBDynParser &HP,
                                               const char *keyword,
                                               integer &n_outputs,
                                               std::vector<int> &output_nodes)
{
  printdebug("BuildComponentOutputArray");
  printdebug(keyword);

  ValidateInputKeyword(HP, keyword);
  n_outputs = HP.GetInt();
  output_nodes.resize(n_outputs);
  for (int i = 0; i < n_outputs; i++)
  {
    output_nodes[i] = HP.GetInt();
  }
}

doublereal ModuleKiteFASTOS::GetPrivateData(KiteFASTBeam beam, const char *private_data) const
{
  printdebug("_GetPrivateData");

  integer index = beam.pBeam->iGetPrivDataIdx(private_data);
  doublereal value = beam.pBeam->dGetPrivData(index);
  return value;
}

void ModuleKiteFASTOS::Output(OutputHandler &OH) const
{
  printdebug("Output");

  doublereal current_time = Time.dGet();
  integer n_gauss_load_points = 2 * total_beam_count;
  doublereal *gauss_point_loads = new doublereal[6 * 2 * total_beam_count]; // 6 components for 2 gauss points per beam
  for (int i = 0; i < total_beam_count; i++)
  {
    integer j = 12 * i;
    gauss_point_loads[j + 0] = GetPrivateData(beams[i], "pI.Fx");
    gauss_point_loads[j + 1] = GetPrivateData(beams[i], "pI.Fy");
    gauss_point_loads[j + 2] = GetPrivateData(beams[i], "pI.Fz");
    gauss_point_loads[j + 3] = GetPrivateData(beams[i], "pI.Mx");
    gauss_point_loads[j + 4] = GetPrivateData(beams[i], "pI.My");
    gauss_point_loads[j + 5] = GetPrivateData(beams[i], "pI.Mz");
    gauss_point_loads[j + 6] = GetPrivateData(beams[i], "pII.Fx");
    gauss_point_loads[j + 7] = GetPrivateData(beams[i], "pII.Fy");
    gauss_point_loads[j + 8] = GetPrivateData(beams[i], "pII.Fz");
    gauss_point_loads[j + 9] = GetPrivateData(beams[i], "pII.Mx");
    gauss_point_loads[j + 10] = GetPrivateData(beams[i], "pII.My");
    gauss_point_loads[j + 11] = GetPrivateData(beams[i], "pII.Mz");
  }

  integer error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_OS_Output(&current_time, &n_gauss_load_points, gauss_point_loads, &error_status, error_message);
  printdebug("KFAST_Output error");
  printdebug("    status: " + std::to_string(error_status) + ";");
  printdebug("    message: " + std::string(error_message) + ";");
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  delete[] gauss_point_loads;
}

int ModuleKiteFASTOS::iGetNumConnectedNodes(void) const
{
  printdebug("iGetNumConnectedNodes");
  return nodes.size();
}

void ModuleKiteFASTOS::WorkSpaceDim(integer *piNumRows, integer *piNumCols) const
{
  printdebug("WorkSpaceDim");
  *piNumRows = 6 * iGetNumConnectedNodes();
  *piNumCols = 1;
}

VariableSubMatrixHandler &ModuleKiteFASTOS::AssJac(VariableSubMatrixHandler &WorkMat, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr)
{
  printdebug("AssJac");
  WorkMat.SetNullMatrix();
  return WorkMat;
}

void ModuleKiteFASTOS::Update(const VectorHandler &XCurr, const VectorHandler &XPrimeCurr)
{
  printdebug("Update");
}

void ModuleKiteFASTOS::_AssRes(doublereal *node_loads, doublereal *rotor_loads)
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

  doublereal mip_position[3];
  Vec3 vec3_mip_pos = mip_node.pNode->GetXCurr();
  mip_position[0] = vec3_mip_pos[0];
  mip_position[1] = vec3_mip_pos[1];
  mip_position[2] = vec3_mip_pos[2];

  doublereal mip_dcm[9];
  Mat3x3 mat3_mip_dcm = mip_node.pNode->GetRCurr();
  mip_dcm[0] = mat3_mip_dcm.dGet(1, 1);
  mip_dcm[1] = mat3_mip_dcm.dGet(1, 2);
  mip_dcm[2] = mat3_mip_dcm.dGet(1, 3);
  mip_dcm[3] = mat3_mip_dcm.dGet(2, 1);
  mip_dcm[4] = mat3_mip_dcm.dGet(2, 2);
  mip_dcm[5] = mat3_mip_dcm.dGet(2, 3);
  mip_dcm[6] = mat3_mip_dcm.dGet(3, 1);
  mip_dcm[7] = mat3_mip_dcm.dGet(3, 2);
  mip_dcm[8] = mat3_mip_dcm.dGet(3, 3);

  doublereal mip_vels[3];
  Vec3 vec3_mip_vels = mip_node.pNode->GetVCurr();
  mip_vels[0] = vec3_mip_vels[0];
  mip_vels[1] = vec3_mip_vels[1];
  mip_vels[2] = vec3_mip_vels[2];

  doublereal mip_omegas[3];
  Vec3 vec3_mip_omegas = mip_node.pNode->GetWCurr();
  mip_omegas[0] = vec3_mip_omegas[0];
  mip_omegas[1] = vec3_mip_omegas[1];
  mip_omegas[2] = vec3_mip_omegas[2];

  doublereal mip_accs[3];
  Vec3 vec3_mip_acc = mip_node.pNode->GetXPPCurr();
  mip_accs[0] = vec3_mip_acc[0];
  mip_accs[1] = vec3_mip_acc[1];
  mip_accs[2] = vec3_mip_acc[2];

  doublereal mip_alphas[3];
  Vec3 vec3_mip_alphas = mip_node.pNode->GetWPCurr();
  mip_alphas[0] = vec3_mip_alphas[0];
  mip_alphas[1] = vec3_mip_alphas[1];
  mip_alphas[2] = vec3_mip_alphas[2];

  doublereal *node_points = new doublereal[3 * node_count_no_rotors];
  doublereal *node_dcms = new doublereal[9 * node_count_no_rotors];
  doublereal *node_vels = new doublereal[3 * node_count_no_rotors];
  doublereal *node_omegas = new doublereal[3 * node_count_no_rotors];
  doublereal *node_accs = new doublereal[3 * node_count_no_rotors];
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

  doublereal *rotor_points = new doublereal[3 * n_rotor_points];
  doublereal *rotor_dcms = new doublereal[9 * n_rotor_points];
  doublereal *rotor_vels = new doublereal[3 * n_rotor_points];
  doublereal *rotor_omegas = new doublereal[3 * n_rotor_points];
  doublereal *rotor_accs = new doublereal[3 * n_rotor_points];
  doublereal *rotor_alphas = new doublereal[3 * n_rotor_points];
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

    Vec3 angacc_curr = nodes[i + node_count_no_rotors].pNode->GetWPCurr();
    rotor_alphas[3 * i] = angacc_curr[0];
    rotor_alphas[3 * i + 1] = angacc_curr[1];
    rotor_alphas[3 * i + 2] = angacc_curr[2];
  }

  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_AssRes(&t,
               &first_iteration,
               ground_station_point,
               mip_position,
               mip_dcm,
               mip_vels,
               mip_omegas,
               mip_accs,
               mip_alphas,
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

  printdebug("KFAST_AssRes error");
  printdebug("    status: " + std::to_string(error_status) + ";");
  printdebug("    message: " + std::string(error_message) + ";");
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  delete[] node_points;
  delete[] node_dcms;
  delete[] node_vels;
  delete[] node_omegas;
  delete[] node_accs;
  delete[] rotor_points;
  delete[] rotor_dcms;
  delete[] rotor_vels;
  delete[] rotor_omegas;
  delete[] rotor_accs;
  delete[] rotor_alphas;
}

SubVectorHandler &ModuleKiteFASTOS::AssRes(SubVectorHandler &WorkVec, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr)
{
  printdebug("AssRes");

  // get the loads from KFAST_AssRes and apply to the mbdyn model
  integer n_node_loads = 6 * node_count_no_rotors; // force and moment components for each node
  doublereal *node_loads = new doublereal[n_node_loads];

  integer n_rotor_loads = 6 * n_rotor_points; // force and moment components for each rotor node
  doublereal *rotor_loads = new doublereal[n_rotor_loads];

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

void ModuleKiteFASTOS::BeforePredict(VectorHandler &X, VectorHandler &XP, VectorHandler &XPrev, VectorHandler &XPPrev) const
{
  printdebug("BeforePredict");
}

void ModuleKiteFASTOS::AfterPredict(VectorHandler &X, VectorHandler &XP)
{
  printdebug("AfterPredict");

  // After the first time step is complete, set this to NO - 0
  first_iteration = 0;
  doublereal t = Time.dGet();
  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_OS_AfterPredict(&t, &error_status, error_message);
  printdebug("KFAST_AfterPredict error");
  printdebug("    status: " + std::to_string(error_status) + ";");
  printdebug("    message: " + std::string(error_message) + ";");
  if (error_status >= AbortErrLev)
  {
    printf("error status %d: %s\n", error_status, error_message);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }
}

void ModuleKiteFASTOS::AfterConvergence(const VectorHandler &X, const VectorHandler &XP)
{
  printdebug("AfterConvergence");
}

extern "C" int module_init(const char *module_name, void *pdm, void *php)
{
  UserDefinedElemRead *rf = new UDERead<ModuleKiteFASTOS>;
  if (!SetUDE("ModuleKiteFASTOS", rf))
  {
    delete rf;
    silent_cerr("module-kitefastmbd-os: module_init(" << module_name << ") failed" << std::endl);
    return -1;
  }
  return 0;
}

// helper functions while in development
void ModuleKiteFASTOS::printdebug(std::string debugstring) const
{
#ifdef DEBUGUDE
  silent_cout("****** " << debugstring << "\t (time: " << Time.dGet() << ")" << std::endl);
#endif
}

void ModuleKiteFASTOS::PrintNodeLocations(KiteFASTNode node)
{
  Vec3 location = node.pNode->GetXCurr();
  Vec3 accel = node.pNode->GetXPPCurr();
  printf("x: %f\ty: %f\tz: %f\tx\": %f\ty\": %f\tz:\"%f\n", location[0], location[1], location[2], accel[0], accel[1], accel[2]);
}

void ModuleKiteFASTOS::InitOutputFile(std::string output_file_name)
{
  printdebug("InitOutputFile");

  if (output_file_name.find("MBD") != std::string::npos)
  {
    output_file_name.append("MBD.out");
  }

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

// these are specific for mbdyn, not used by us or KiteFAST
unsigned int ModuleKiteFASTOS::iGetNumPrivData(void) const
{
  printdebug("iGetNumPrivData");
  return 0;
}

void ModuleKiteFASTOS::GetConnectedNodes(std::vector<const Node *> &connectedNodes) const
{
  printdebug("GetConnectedNodes");
  connectedNodes.resize(nodes.size());
  for (int n = 0; n < nodes.size(); n++)
  {
    connectedNodes[n] = nodes[n].pNode;
  }
}

std::ostream &ModuleKiteFASTOS::Restart(std::ostream &out) const
{
  printdebug("Restart");
  return out << "module-kitefastmbd not implemented" << std::endl;
}

unsigned int ModuleKiteFASTOS::iGetInitialNumDof(void) const
{
  printdebug("iGetInitialNumDof");
  return 0;
}

void ModuleKiteFASTOS::InitialWorkSpaceDim(integer *piNumRows, integer *piNumCols) const
{
  printdebug("InitialWorkSpaceDim");
  *piNumRows = 0;
  *piNumCols = 0;
}

VariableSubMatrixHandler &ModuleKiteFASTOS::InitialAssJac(VariableSubMatrixHandler &WorkMat, const VectorHandler &XCurr)
{
  printdebug("InitialAssJac");

  // should not be called, since initial workspace is empty
  ASSERT(0);
  WorkMat.SetNullMatrix();
  return WorkMat;
}

SubVectorHandler &ModuleKiteFASTOS::InitialAssRes(SubVectorHandler &WorkVec, const VectorHandler &XCurr)
{
  printdebug("InitialAssRes");

  // should not be called, since initial workspace is empty
  ASSERT(0);
  WorkVec.ResizeReset(0);
  return WorkVec;
}
