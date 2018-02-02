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

// #define DEBUG

KiteADelem::KiteADelem(unsigned uLabel, const DofOwner *pDO, DataManager *pDM, MBDynParser &HP) : Elem(uLabel, flag(0)), UserDefinedElem(uLabel, pDO)
{
  if (HP.IsKeyWord("help"))
  {
    silent_cout(std::endl
                << "Module:   KiteFastMBD" << std::endl
                << "Author:   Rick Damiani <rick.damiani@nrel.gov>  et al." << std::endl
                << "Organization:  NREL/NWTC" << std::endl
                << "    http://www.nrel.gov/" << std::endl
                << "" << std::endl
                << "Syntax:" << std::endl
                << "  user defined : <label> , KiteADelem ," << std::endl
                << "    nElmNodes, <numer of associated nodes>," << std::endl
                << "    <node IDs spearated by "
                   ","
                   ">,"
                << std::endl
                << "    [position , (Vec3)<offset> ]" << std::endl
                << "    [ , orientation , (OrientationMatrix)<orientation> ]" << std::endl
                << "" << std::endl
                << "All rights reserved" << std::endl
                << std::endl);

    if (!HP.IsArg())
    {
      silent_cout("No Arguments provided for KiteADelem for UDE IDE=" << uLabel << std::endl);
      throw NoErr(MBDYN_EXCEPT_ARGS);
    }
  }

  // setup the time drive
  Time.Set(new TimeDriveCaller(pDM->pGetDrvHdl()));
  printf("initial time: %f\n", Time.dGet());

  printf("************\n");
  int in = 10;
  int out = 999;
  (void)__FC_DECL__(test_testme)(&in, &out);

  // test data transfer
  int array_size = 10;
  double location_array[array_size];
  for (int i = 0; i < array_size; i++)
  {
    location_array[i] = (float)i;
  }
  (void)__FC_DECL__(test_array_transfer)(location_array, &array_size);

  printf("************\n");

  // build the node arrays
  BuildComponentNodeArray(pDM, HP, "fuselage", nodes_fuselage);
  BuildComponentNodeArray(pDM, HP, "port_wing", nodes_portwing);
  BuildComponentNodeArray(pDM, HP, "starboard_wing", nodes_starwing);
  BuildComponentNodeArray(pDM, HP, "vstab", nodes_vstab);
  BuildComponentNodeArray(pDM, HP, "port_hstab", nodes_porthstab);
  BuildComponentNodeArray(pDM, HP, "starboard_hstab", nodes_starhstab);
  BuildComponentNodeArray(pDM, HP, "port_pylon1", nodes_portpylon1);
  BuildComponentNodeArray(pDM, HP, "port_pylon2", nodes_portpylon2);
  BuildComponentNodeArray(pDM, HP, "starboard_pylon1", nodes_starpylon1);
  BuildComponentNodeArray(pDM, HP, "starboard_pylon2", nodes_starpylon2);
  BuildComponentNodeArray(pDM, HP, "port_rotors", nodes_portrotors);
  BuildComponentNodeArray(pDM, HP, "starboard_rotors", nodes_starrotors);
  BuildComponentNodeArray(pDM, HP, "bridle", nodes_bridle);

  node_count = nodes_fuselage.size() + nodes_portwing.size() + nodes_starwing.size() + nodes_vstab.size() + nodes_porthstab.size() + nodes_starhstab.size() + nodes_portpylon1.size() + nodes_portpylon2.size() + nodes_starpylon1.size() + nodes_starpylon2.size() + nodes_portrotors.size() + nodes_starrotors.size() + nodes_bridle.size();

  nodes.reserve(node_count);
  nodes.insert(nodes.begin(), nodes_fuselage.begin(), nodes_fuselage.end());
  nodes.insert(nodes.end(), nodes_portwing.begin(), nodes_portwing.end());
  nodes.insert(nodes.end(), nodes_starwing.begin(), nodes_starwing.end());
  nodes.insert(nodes.end(), nodes_vstab.begin(), nodes_vstab.end());
  nodes.insert(nodes.end(), nodes_porthstab.begin(), nodes_porthstab.end());
  nodes.insert(nodes.end(), nodes_starhstab.begin(), nodes_starhstab.end());
  nodes.insert(nodes.end(), nodes_portpylon1.begin(), nodes_portpylon1.end());
  nodes.insert(nodes.end(), nodes_portpylon2.begin(), nodes_portpylon2.end());
  nodes.insert(nodes.end(), nodes_starpylon1.begin(), nodes_starpylon1.end());
  nodes.insert(nodes.end(), nodes_starpylon2.begin(), nodes_starpylon2.end());
  nodes.insert(nodes.end(), nodes_portrotors.begin(), nodes_portrotors.end());
  nodes.insert(nodes.end(), nodes_starrotors.begin(), nodes_starrotors.end());
  nodes.insert(nodes.end(), nodes_bridle.begin(), nodes_bridle.end());

  // configure the output file
  if (!HP.IsKeyWord("output_file_name"))
  {
    silent_cerr("Input Error: cannot read keyword output_file_name" << std::endl);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }
  output_file_name = HP.GetFileName();
  if (output_file_name.length() == 0)
  {
    silent_cerr("Input Error: cannot read output_file_name" << std::endl);
    throw ErrGeneric(MBDYN_EXCEPT_ARGS);
  }
  InitOutputFile(output_file_name);

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

KiteADelem::~KiteADelem(void)
{
  printdebug("~KiteADelem");
  NO_OP;
}

void KiteADelem::SetValue(DataManager *pDM, VectorHandler &X, VectorHandler &XP, SimulationEntity::Hints *ph)
{
  printdebug("SetValue");
  NO_OP;
}

void KiteADelem::BuildComponentNodeArray(DataManager *pDM, MBDynParser &HP, const char *keyword, std::vector<AeroNode> &node_array)
{
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

void KiteADelem::InitOutputFile(std::string output_file_name)
{
  printdebug("InitOutputFile");
  outputfile.open(output_file_name);
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

void KiteADelem::Output(OutputHandler &OH) const
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
    outputfile << std::setw(16) << "Time s"
             << std::setw(16) << "Node ID"
             << std::setw(16) << "prev - x"
             << std::setw(13) << "prev - y"
             << std::setw(13) << "prev - z"
             << std::setw(16) << "prev - R11"
             << std::setw(13) << "prev - R12"
             << std::setw(13) << "prev - R13"
             << std::setw(13) << "prev - R21"
             << std::setw(13) << "prev - R22"
             << std::setw(13) << "prev - R23"
             << std::setw(13) << "prev - R31"
             << std::setw(13) << "prev - R32"
             << std::setw(13) << "prev - R33"
             << std::endl;
  outputfile << std::setw(8)
             << std::scientific
             << std::setw(16) << Time.dGet()
             << std::setw(16) << nodes[2].pNode->GetLabel()
             << std::setw(16) << nodes[2].pNode->GetXPrev()
             << std::setw(16) << nodes[2].pNode->GetRPrev()
             << std::endl;
  outputfile << std::setw(16) << "Time s"
             << std::setw(16) << "Node ID"
             << std::setw(16) << "curr - x"
             << std::setw(13) << "curr - y"
             << std::setw(13) << "curr - z"
             << std::setw(16) << "curr - R11"
             << std::setw(13) << "curr - R12"
             << std::setw(13) << "curr - R13"
             << std::setw(13) << "curr - R21"
             << std::setw(13) << "curr - R22"
             << std::setw(13) << "curr - R23"
             << std::setw(13) << "curr - R31"
             << std::setw(13) << "curr - R32"
             << std::setw(13) << "curr - R33"
             << std::endl;
  outputfile << std::setw(8)
             << std::scientific
             << std::setw(16) << Time.dGet()
             << std::setw(16) << nodes[2].pNode->GetLabel()
             << std::setw(16) << nodes[2].pNode->GetXCurr()
             << std::setw(16) << nodes[2].pNode->GetRCurr()
             << std::endl
             << std::endl;
}

int KiteADelem::iGetNumConnectedNodes(void) const
{
  // printdebug("iGetNumConnectedNodes");
  return nodes.size();
}

void KiteADelem::WorkSpaceDim(integer *piNumRows, integer *piNumCols) const
{
  // printdebug("WorkSpaceDim");
  *piNumRows = 6 * iGetNumConnectedNodes();
  *piNumCols = 1;
}

VariableSubMatrixHandler &KiteADelem::AssJac(VariableSubMatrixHandler &WorkMat, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr)
{
  printdebug("AssJac");
  WorkMat.SetNullMatrix();
  return WorkMat;
}

void KiteADelem::Update(const VectorHandler &XCurr, const VectorHandler &XPrimeCurr)
{
  printdebug("Update");
}

SubVectorHandler &KiteADelem::AssRes(SubVectorHandler &WorkVec, doublereal dCoef, const VectorHandler &XCurr, const VectorHandler &XPrimeCurr)
{
  printdebug("AssRes");
  integer iNumRows, iNumCols;
  WorkSpaceDim(&iNumRows, &iNumCols);
  WorkVec.ResizeReset(iNumRows);

  for (int elem = 0; elem < nodes.size(); elem++)
  {
    /*
     * set indices where force/moment need to be put
     */
    integer iFirstIndex = nodes[elem].pNode->iGetFirstMomentumIndex();
    for (int i = 1; i <= 6; i++)
    {
      WorkVec.PutRowIndex(6 * elem + i, iFirstIndex + i);
    }
  }

  // integer iFirstIndex = nodes[i].pNode->iGetFirstMomentumIndex();
  // for (int i = 1; i <= 6; i++) {
  //   WorkVec.PutRowIndex( i, iFirstIndex + i);
  // }
  // WorkVec.Add(1, Vec3(0.0, 0.0, -1000*sin(10*dCurTime)));
  // WorkVec.Add(4, Vec3(0.0, 0.0, 0.0));
  WorkVec.Add(320, Vec3(0.0, 0.0, 1000.));

  return WorkVec;
}

void KiteADelem::BeforePredict(VectorHandler &X, VectorHandler &XP, VectorHandler &XPrev, VectorHandler &XPPrev) const
{
  printdebug("BeforePredict");
}

void KiteADelem::AfterPredict(VectorHandler &X, VectorHandler &XP)
{
  printdebug("AfterPredict");
}

void KiteADelem::AfterConvergence(const VectorHandler &X, const VectorHandler &XP)
{
  printdebug("AfterConvergence");
}

extern "C" int module_init(const char *module_name, void *pdm, void *php)
{
  UserDefinedElemRead *rf = new UDERead<KiteADelem>;
  if (!SetUDE("KiteADelem", rf))
  {
    delete rf;
    silent_cerr("module-kitefastmbd: module_init(" << module_name << ") failed" << std::endl);
    return -1;
  }
  return 0;
}

// helper functions while in development
void KiteADelem::printdebug(std::string debugstring) const
{
#ifdef DEBUG
  silent_cout("****** " << debugstring << "\t" << Time.dGet() << std::endl);
#endif
}

void KiteADelem::PrintNodeLocations(AeroNode node)
{
  Vec3 location = node.pNode->GetXCurr();
  Vec3 accel = node.pNode->GetXPPCurr();
  printf("x: %f\ty: %f\tz: %f\tx\": %f\ty\": %f\tz:\"%f\n", location[0], location[1], location[2], accel[0], accel[1], accel[2]);
}

// these are specific for mbdyn, not used by us or KiteFAST
unsigned int KiteADelem::iGetNumPrivData(void) const
{
  printdebug("iGetNumPrivData");
  return 0;
}

void KiteADelem::GetConnectedNodes(std::vector<const Node *> &connectedNodes) const
{
  printdebug("GetConnectedNodes");
  connectedNodes.resize(nodes.size());
  for (int n = 0; n < nodes.size(); n++)
  {
    connectedNodes[n] = nodes[n].pNode;
  }
}

std::ostream &KiteADelem::Restart(std::ostream &out) const
{
  printdebug("Restart");
  return out << "module-kitefastmbd not implemented" << std::endl;
}

unsigned int KiteADelem::iGetInitialNumDof(void) const
{
  printdebug("iGetInitialNumDof");
  return 0;
}

void KiteADelem::InitialWorkSpaceDim(integer *piNumRows, integer *piNumCols) const
{
  printdebug("InitialWorkSpaceDim");
  *piNumRows = 0;
  *piNumCols = 0;
}

VariableSubMatrixHandler &KiteADelem::InitialAssJac(VariableSubMatrixHandler &WorkMat, const VectorHandler &XCurr)
{
  printdebug("InitialAssJac");

  // should not be called, since initial workspace is empty
  ASSERT(0);
  WorkMat.SetNullMatrix();
  return WorkMat;
}

SubVectorHandler &KiteADelem::InitialAssRes(SubVectorHandler &WorkVec, const VectorHandler &XCurr)
{
  printdebug("InitialAssRes");

  // should not be called, since initial workspace is empty
  ASSERT(0);
  WorkVec.ResizeReset(0);
  return WorkVec;
}
