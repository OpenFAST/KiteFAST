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
  kitefast_module_flags = (int *)malloc(4 * sizeof(int));
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
  output_file_name = strcat(output_file_root, "MBD.out");
  InitOutputFile(output_file_name);

  // parse the time step
  ValidateInputKeyword(HP, "time_step");
  dt = HP.GetReal();

  // parse the gravity
  ValidateInputKeyword(HP, "gravity");
  gravity = HP.GetReal();

  // parse the ground station location
  ValidateInputKeyword(HP, "ground_weather_station_location");
  ground_station_point = (doublereal *)malloc(3 * sizeof(doublereal));
  ground_station_point[0] = HP.GetReal();
  ground_station_point[1] = HP.GetReal();
  ground_station_point[2] = HP.GetReal();

  // parse the component counts
  ValidateInputKeyword(HP, "number_of_flaps_per_wing");
  n_flaps_per_wing = HP.GetInt();
  ValidateInputKeyword(HP, "number_of_pylons_per_wing");
  n_pylons_per_wing = HP.GetInt();
  ValidateInputKeyword(HP, "number_of_kite_components");
  n_components = HP.GetInt();

  // ***
  // n_components includes all components except the rotors
  // ***

  // parse the keypoints (aka reference points)
  ValidateInputKeyword(HP, "keypoints");
  numRefPtElem = 3 * (n_components + 2 * 2 * n_pylons_per_wing);
  reference_points = (doublereal *)malloc(numRefPtElem * sizeof(doublereal));

  // 2 * 2 * n_pylons_per_wing = 2 wings * 2 rotors per pylon * # of pylons = total rotor count
  for (int i = 0; i < n_components + 2 * 2 * n_pylons_per_wing; i++)
  {
    reference_points[3 * i] = HP.GetReal();
    reference_points[3 * i + 1] = HP.GetReal();
    reference_points[3 * i + 2] = HP.GetReal();
  }

  // parse the component nodes into arrays
  component_reference_node_indeces = (int *)malloc(n_components * sizeof(int));
  BuildComponentNodeArray(pDM, HP, "fuselage", nodes_fuselage, component_reference_node_indeces[0]);
  BuildComponentNodeArray(pDM, HP, "starboard_wing", nodes_starwing, component_reference_node_indeces[1]);
  BuildComponentNodeArray(pDM, HP, "port_wing", nodes_portwing, component_reference_node_indeces[2]);
  BuildComponentNodeArray(pDM, HP, "vstab", nodes_vstab, component_reference_node_indeces[3]);
  BuildComponentNodeArray(pDM, HP, "starboard_hstab", nodes_starhstab, component_reference_node_indeces[4]);
  BuildComponentNodeArray(pDM, HP, "port_hstab", nodes_porthstab, component_reference_node_indeces[5]);
  BuildComponentNodeArray(pDM, HP, "starboard_pylon1", nodes_starpylon1, component_reference_node_indeces[6]);
  // BuildComponentNodeArray(pDM, HP, "starboard_pylon2", nodes_starpylon2, component_reference_node_indeces[7]);
  BuildComponentNodeArray(pDM, HP, "port_pylon1", nodes_portpylon1, component_reference_node_indeces[7]);
  // BuildComponentNodeArray(pDM, HP, "port_pylon2", nodes_portpylon2, component_reference_node_indeces[9]);
  BuildComponentNodeArray(pDM, HP, "starboard_rotors", nodes_starrotors, component_reference_node_indeces[8]);
  BuildComponentNodeArray(pDM, HP, "port_rotors", nodes_portrotors, component_reference_node_indeces[9]);
  // BuildComponentNodeArray(pDM, HP, "bridle", nodes_bridle, component_reference_node_indeces[12]);

  node_count = nodes_fuselage.size()
             + nodes_portwing.size()
             + nodes_starwing.size()
             + nodes_vstab.size()
             + nodes_porthstab.size()
             + nodes_starhstab.size()
             + nodes_portpylon1.size()
            //  + nodes_portpylon2.size()
             + nodes_starpylon1.size()
            //  + nodes_starpylon2.size()
             + nodes_portrotors.size()
             + nodes_starrotors.size();
            //  + nodes_bridle.size();

  nodes.reserve(node_count);
  nodes.insert(nodes.begin(), nodes_fuselage.begin(), nodes_fuselage.end());
  nodes.insert(nodes.end(), nodes_starwing.begin(), nodes_starwing.end());
  nodes.insert(nodes.end(), nodes_portwing.begin(), nodes_portwing.end());
  nodes.insert(nodes.end(), nodes_vstab.begin(), nodes_vstab.end());
  nodes.insert(nodes.end(), nodes_starhstab.begin(), nodes_starhstab.end());
  nodes.insert(nodes.end(), nodes_porthstab.begin(), nodes_porthstab.end());
  nodes.insert(nodes.end(), nodes_starpylon1.begin(), nodes_starpylon1.end());
  // nodes.insert(nodes.end(), nodes_starpylon2.begin(), nodes_starpylon2.end());
  nodes.insert(nodes.end(), nodes_portpylon1.begin(), nodes_portpylon1.end());
  // nodes.insert(nodes.end(), nodes_portpylon2.begin(), nodes_portpylon2.end());
  nodes.insert(nodes.end(), nodes_starrotors.begin(), nodes_starrotors.end());
  nodes.insert(nodes.end(), nodes_portrotors.begin(), nodes_portrotors.end());
  // nodes.insert(nodes.end(), nodes_bridle.begin(), nodes_bridle.end());

  // number of nodes per kite component excluding rotors
  component_node_counts = (int *)malloc(n_components * sizeof(int));
  component_node_counts[0] = nodes_fuselage.size();   // fuselage nodes
  component_node_counts[1] = nodes_starwing.size();   // starboard wing nodes
  component_node_counts[2] = nodes_portwing.size();   // port wing nodes
  component_node_counts[3] = nodes_vstab.size();      // vertical stabilizer nodes
  component_node_counts[4] = nodes_starhstab.size();  // starboard horizontal stabilizer nodes
  component_node_counts[5] = nodes_porthstab.size();  // port horizontal stabilizer nodes
  component_node_counts[6] = nodes_starpylon1.size(); // starboard inboard pylon nodes
  // component_node_counts[7] = nodes_starpylon2.size(); // starboard outboard pylon nodes
  component_node_counts[7] = nodes_portpylon1.size(); // port inboard pylon nodes
  // component_node_counts[9] = nodes_portpylon2.size(); // port outboard pylon nodes

  // build the node arrays for kite components excluding rotors
  int component_node_count = nodes.size() - nodes_starrotors.size() - nodes_portrotors.size();
  numNodePtElem = 3 * component_node_count;
  doublereal node_points[numNodePtElem];
  // node_points = (doublereal *)malloc(numNodePtElem * sizeof(doublereal));
  numDCMElem = 9 * component_node_count;
  doublereal node_dcms[numDCMElem];
  // node_dcms = (doublereal *)malloc(numDCMElem * sizeof(doublereal));

  for (int i = 0; i < component_node_count; i++)
  {
    Vec3 xcurr = nodes[i].pNode->GetXCurr();
    node_points[3 * i] = xcurr[0];
    node_points[3 * i + 1] = xcurr[1];
    node_points[3 * i + 2] = xcurr[2];

    // these are dGet(row, col)
    Mat3x3 rnode = nodes[i].pNode->GetRCurr();
    node_dcms[9 * i] = rnode.dGet(0, 0);
    node_dcms[9 * i + 1] = rnode.dGet(0, 1);
    node_dcms[9 * i + 2] = rnode.dGet(0, 2);
    node_dcms[9 * i + 3] = rnode.dGet(1, 0);
    node_dcms[9 * i + 4] = rnode.dGet(1, 1);
    node_dcms[9 * i + 5] = rnode.dGet(1, 2);
    node_dcms[9 * i + 6] = rnode.dGet(2, 0);
    node_dcms[9 * i + 7] = rnode.dGet(2, 1);
    node_dcms[9 * i + 8] = rnode.dGet(2, 2);
  }

  // build the node arrays for rotors
  int rotors_node_count = nodes_starrotors.size() + nodes_portrotors.size();
  numRtrPtsElem = 3 * rotors_node_count;
  rotor_points = (doublereal *)malloc(numRtrPtsElem * sizeof(doublereal));
  for (int i = component_node_count; i < rotors_node_count; i++)
  {
    Vec3 xcurr = nodes[i].pNode->GetXCurr();
    rotor_points[3 * i] = xcurr[0];
    rotor_points[3 * i + 1] = xcurr[1];
    rotor_points[3 * i + 2] = xcurr[2];
  }

  int numRtSpdRtrElem;
  double *pRtSpd_PyRtr;
  double *pFusODCM;
  double *pFusO;
  double *pFusOv;
  double *pFusOomegas;
  double *pFusOacc;
  double *pNodeVels;
  double *pNodeOmegas;

  // Test the FusODCM as a 1D array instead of a 2D array
  // The kite is aligned with the Global Coordinate system
  int mip_index = component_reference_node_indeces[0];
  KiteFASTNode mipnode = nodes_fuselage[mip_index];
  Mat3x3 mip_dcm = mipnode.pNode->GetRCurr();
  printf("%f %f %f\n", mip_dcm.dGet(0, 0), mip_dcm.dGet(0, 1), mip_dcm.dGet(0, 2));
  printf("%f %f %f\n", mip_dcm.dGet(1, 0), mip_dcm.dGet(1, 1), mip_dcm.dGet(1, 2));
  printf("%f %f %f\n", mip_dcm.dGet(2, 0), mip_dcm.dGet(2, 1), mip_dcm.dGet(2, 2));

  pFusODCM = (doublereal *)malloc(9 * sizeof(doublereal));
  pFusODCM[0] = doublereal(-1.0);
  pFusODCM[1] = doublereal(0.0);
  pFusODCM[2] = doublereal(0.0);
  pFusODCM[3] = doublereal(0.0);
  pFusODCM[4] = doublereal(1.0);
  pFusODCM[5] = doublereal(0.0);
  pFusODCM[6] = doublereal(0.0);
  pFusODCM[7] = doublereal(0.0);
  pFusODCM[8] = doublereal(-1.0);

  // int index = 6;
  // int indexm1 = index-1;
  // int begin = 0;
  // for (int h = 0; h <= indexm1; h++) {
  //   begin += component_node_counts[h];
  // }
  // for (int h = begin; h < begin + component_node_counts[index]; h++)
  // {
  //   printf("%-8.5f  %-8.5f  %-8.5f  0.000     1.000     1\n", -1 * node_points[3 * h + 0], node_points[3 * h + 1], node_points[8] - node_points[3 * h + 2]);
  // }

  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  KFAST_Init(&dt, &n_flaps_per_wing, &n_pylons_per_wing, &n_components, component_node_counts,
             kitefast_module_flags, kiteaerodyn_filename, inflowwind_filename, moordyn_filename, controller_filename,
             output_file_root, &gravity, ground_station_point, pFusODCM, &numRtrPtsElem, rotor_points, &numRefPtElem,
             reference_points, &numNodePtElem, node_points, &numDCMElem, node_dcms, &error_status, error_message);
  silent_cout(error_status << error_message << std::endl);
  if (error_status >= AbortErrLev)
  {
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  // for future reference
  // Get the orientation matrix of blade root with respect to hub reference frame
  // if ((iNode % NElems) == 0) {
  //   bladeR[elem/NElems] = HP.GetRotRel(rf);
  // }
  // m_pNode = pDM->ReadNode<const StructNode, Node::STRUCTURAL>(HP);
  //if (!m_pNode->bComputeAccelerations()) {
  // const_cast<StructNode *>(m_pNode)->ComputeAccelerations(true);
  //}
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
                                             std::vector<KiteFASTNode> &node_array,
                                             int &ref_node_index)
{
  printdebug("BuildComponentNodeArray");
  if (!HP.IsKeyWord(keyword))
  {
    silent_cerr("Runtime Error: cannot read keyword " << keyword << std::endl);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }

  int node_count = HP.GetInt();
  ref_node_index = HP.GetInt();

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
  // outputfile << std::setw(16) << "Node ID"
  //            << std::setw(16) << "Time s"
  //            << std::setw(16) << "prev - x"
  //            << std::setw(13) << "prev - y"
  //            << std::setw(13) << "prev - z"
  //            << std::setw(16) << "prev - R11"
  //            << std::setw(13) << "prev - R12"
  //            << std::setw(13) << "prev - R13"
  //            << std::setw(13) << "prev - R21"
  //            << std::setw(13) << "prev - R22"
  //            << std::setw(13) << "prev - R23"
  //            << std::setw(13) << "prev - R31"
  //            << std::setw(13) << "prev - R32"
  //            << std::setw(13) << "prev - R33"
  //            << std::endl;
  // outputfile << std::setw(8)
  //            << std::scientific
  //            << std::setw(16) << Time.dGet()
  //            << std::setw(16) << nodes[0].pNode->GetLabel()
  //            << std::setw(16) << nodes[0].pNode->GetXPrev()
  //            << std::setw(16) << nodes[0].pNode->GetRPrev()
  //            << std::endl;
  // outputfile << std::setw(16) << "Node ID"
  //            << std::setw(16) << "Time s"
  //            << std::setw(16) << "curr - x"
  //            << std::setw(13) << "curr - y"
  //            << std::setw(13) << "curr - z"
  //            << std::setw(16) << "curr - R11"
  //            << std::setw(13) << "curr - R12"
  //            << std::setw(13) << "curr - R13"
  //            << std::setw(13) << "curr - R21"
  //            << std::setw(13) << "curr - R22"
  //            << std::setw(13) << "curr - R23"
  //            << std::setw(13) << "curr - R31"
  //            << std::setw(13) << "curr - R32"
  //            << std::setw(13) << "curr - R33"
  //            << std::endl;
  // outputfile << std::setw(8)
  //            << std::scientific
  //            << std::setw(16) << Time.dGet()
  //            << std::setw(16) << nodes[0].pNode->GetLabel()
  //            << std::setw(16) << nodes[0].pNode->GetXCurr()
  //            << std::setw(16) << nodes[0].pNode->GetRCurr()
  //            << std::endl;

  // outputfile << std::setw(16) << "Time s"
  //            << std::setw(16) << "Node ID"
  //            << std::setw(16) << "pos - x"
  //            << std::setw(13) << "pos - y"
  //            << std::setw(13) << "pos - z"
  //            << std::setw(16) << "vel - x"
  //            << std::setw(13) << "vel - y"
  //            << std::setw(13) << "vel - z"
  //            << std::setw(16) << "acc - x"
  //            << std::setw(13) << "acc - y"
  //            << std::setw(13) << "acc - z"
  //            << std::endl;
}

void ModuleKiteFAST::Output(OutputHandler &OH) const
{
  printdebug("Output");
  // if (outputfile)
  //   outputfile << std::setw(8)
  //              << std::scientific
  //              << std::setw(16) << Time.dGet()
  //              << std::setw(16) << nodes[0].pNode->GetLabel()
  //              << std::setw(16) << nodes[0].pNode->GetXCurr()
  //              << std::setw(16) << nodes[0].pNode->GetVCurr()
  //              << std::setw(16) << nodes[0].pNode->GetXPPCurr()
  //             //  << std::setw(16) << nodes[0].pNode->GetXPPPrev()
  //              << std::endl;
  // if (outputfile)
  //   outputfile << std::setw(16) << "Time s"
  //               << std::setw(16) << "Node ID"
  //               << std::setw(16) << "prev - x"
  //               << std::setw(13) << "prev - y"
  //               << std::setw(13) << "prev - z"
  //               << std::setw(16) << "prev - R11"
  //               << std::setw(13) << "prev - R12"
  //               << std::setw(13) << "prev - R13"
  //               << std::setw(13) << "prev - R21"
  //               << std::setw(13) << "prev - R22"
  //               << std::setw(13) << "prev - R23"
  //               << std::setw(13) << "prev - R31"
  //               << std::setw(13) << "prev - R32"
  //               << std::setw(13) << "prev - R33"
  //               << std::endl;
  // outputfile << std::setw(8)
  //            << std::scientific
  //            << std::setw(16) << Time.dGet()
  //            << std::setw(16) << nodes[2].pNode->GetLabel()
  //            << std::setw(16) << nodes[2].pNode->GetXPrev()
  //            << std::setw(16) << nodes[2].pNode->GetRPrev()
  //            << std::endl;
  // outputfile << std::setw(16) << "Time s"
  //            << std::setw(16) << "Node ID"
  //            << std::setw(16) << "curr - x"
  //            << std::setw(13) << "curr - y"
  //            << std::setw(13) << "curr - z"
  //            << std::setw(16) << "curr - R11"
  //            << std::setw(13) << "curr - R12"
  //            << std::setw(13) << "curr - R13"
  //            << std::setw(13) << "curr - R21"
  //            << std::setw(13) << "curr - R22"
  //            << std::setw(13) << "curr - R23"
  //            << std::setw(13) << "curr - R31"
  //            << std::setw(13) << "curr - R32"
  //            << std::setw(13) << "curr - R33"
  //            << std::endl;
  // outputfile << std::setw(8)
  //            << std::scientific
  //            << std::setw(16) << Time.dGet()
  //            << std::setw(16) << nodes[2].pNode->GetLabel()
  //            << std::setw(16) << nodes[2].pNode->GetXCurr()
  //            << std::setw(16) << nodes[2].pNode->GetRCurr()
  //            << std::endl
  //            << std::endl;

  int error_status;
  char error_message[INTERFACE_STRING_LENGTH];
  doublereal current_time = Time.dGet();
  KFAST_Output(&current_time, &error_status, error_message);
  if (error_status >= AbortErrLev)
  {
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

SubVectorHandler &ModuleKiteFAST::AssRes(SubVectorHandler &WorkVec, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr)
{
  printdebug("AssRes");
  ASSERT(0);
  WorkVec.ResizeReset(0);
  return WorkVec;

  // integer iNumRows, iNumCols;
  // WorkSpaceDim(&iNumRows, &iNumCols);
  // WorkVec.ResizeReset(iNumRows);

  // for (int elem = 0; elem < nodes.size(); elem++)
  // {
  //   /*
  //    * set indices where force/moment need to be put
  //    */
  //   integer iFirstIndex = nodes[elem].pNode->iGetFirstMomentumIndex();
  //   for (int i = 1; i <= 6; i++)
  //   {
  //     WorkVec.PutRowIndex(6 * elem + i, iFirstIndex + i);
  //   }
  // }

  // integer iFirstIndex = nodes[i].pNode->iGetFirstMomentumIndex();
  // for (int i = 1; i <= 6; i++) {
  //   WorkVec.PutRowIndex( i, iFirstIndex + i);
  // }
  // WorkVec.Add(1, Vec3(0.0, 0.0, -1000*sin(10*dCurTime)));
  // WorkVec.Add(4, Vec3(0.0, 0.0, 0.0));
  // WorkVec.Add(320, Vec3(0.0, 0.0, 1000.));
  
  // return WorkVec;
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
