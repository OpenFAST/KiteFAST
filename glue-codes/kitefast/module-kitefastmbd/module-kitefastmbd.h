/* $Header: /var/cvs/mbdyn/mbdyn/mbdyn-1.0/modules/module-aerodyn/NREL_AeroDyn.h,v 1.8 2017/01/12 14:47:15 masarati Exp $ */
/*
 * Copyright (C) 2017-
 *
 * Rick Damiani <rick.damiani@nrel.gov>
 *
 * NWTC/NREL
 * Golden, CO
 *
 * Changing this copyright notice is forbidden.
 *
 * This header file is free software; you can redistribute it at will,
 * under the same license conditions of the AeroDyn package.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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

// test data transfers from c++ to fortran
extern int __FC_DECL__(test_array_transfer)(double[], int *);

extern int __FC_DECL__(test_testme)(int *in, int *out);

/*
 * This subroutine is to pass the current simulation time
 * from MBDyn to AeroDyn!
 * c_time: current time
 */
extern int __FC_DECL__(mbdyn_sim_time)(doublereal *c_time);

/*
 * This subroutine is to pass the current simulation time step
 * from MBDyn to AeroDyn!
 * dt: time step
 */
extern int __FC_DECL__(mbdyn_time_step)(f_real *dt);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* KiteFAST_MBD_H */

class KiteADelem : virtual public Elem, public UserDefinedElem
{
private:
  struct AeroNode
  {
    StructNode *pNode;
  };

  int node_count; // number of nodes connected to this element
  std::vector<AeroNode> nodes;
  std::vector<AeroNode> nodes_fuselage;
  std::vector<AeroNode> nodes_portwing;
  std::vector<AeroNode> nodes_starwing;
  std::vector<AeroNode> nodes_vstab;
  std::vector<AeroNode> nodes_porthstab;
  std::vector<AeroNode> nodes_starhstab;
  std::vector<AeroNode> nodes_portpylon1;
  std::vector<AeroNode> nodes_portpylon2;
  std::vector<AeroNode> nodes_starpylon1;
  std::vector<AeroNode> nodes_starpylon2;
  std::vector<AeroNode> nodes_portrotors;
  std::vector<AeroNode> nodes_starrotors;
  std::vector<AeroNode> nodes_bridle;

  std::string output_file_name;
  mutable std::ofstream outputfile;

  bool bFirst;
  DriveOwner Time;

public:
  KiteADelem(unsigned uLabel, const DofOwner *pDO, DataManager *pDM, MBDynParser &HP);
  virtual ~KiteADelem(void);
  void SetValue(DataManager *pDM, VectorHandler &X, VectorHandler &XP, SimulationEntity::Hints *ph);
  void BuildComponentNodeArray(DataManager *pDM, MBDynParser &HP, const char *keyword, std::vector<AeroNode> &node_array);
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
  void PrintNodeLocations(AeroNode node);

  // these are specific for mbdyn, not used by us or KiteFAST
  unsigned int iGetNumPrivData(void) const;
  void GetConnectedNodes(std::vector<const Node *> &connectedNodes) const;
  std::ostream &Restart(std::ostream &out) const;
  virtual unsigned int iGetInitialNumDof(void) const;
  virtual void InitialWorkSpaceDim(integer *piNumRows, integer *piNumCols) const;
  VariableSubMatrixHandler &InitialAssJac(VariableSubMatrixHandler &WorkMat, const VectorHandler &XCurr);
  SubVectorHandler &InitialAssRes(SubVectorHandler &WorkVec, const VectorHandler &XCurr);
};
