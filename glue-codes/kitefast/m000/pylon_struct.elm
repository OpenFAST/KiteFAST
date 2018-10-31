#Pylon Beam Definition

set: integer rpy1beam =13000;

set: integer lpy1beam =17000;


# Cross Section Stiffness Matrix
set:  EA  = 1.0e+8;
set:  GAy = 1.0e+8;
set:  GAz = 1.0e+8;
set:  GJx  = 1.0e+6;
set:  EJyy = 2.0e+6;
set:  EJzz = 4.0e+8;
set:  yeamsc = 0.00;   #distance along local y of EA_center from shear center
set:  zeamsc = 0.00;  #distance along local z of EA_center from shear center

# mass per length
set:  m = 7.;
set:  L_beam = 1.;

# inertial moment per length
set:  Ixx  = 1;
set:  Iyy  = 0.;
set:  Izz  = 0.;

# *********  left pylon 1 **********

set:  curr_beam  = lpy1beam;
set:  beam_ref1   = lpy1_RF;
set:  beam_ref2   = lpy1_RF;
set:  twist1 = 0.*deg2rad;
set:  twist2 = 0.*deg2rad;
set:  beam_node1 = lpy1root_node;
set:  beam_node2 = lpy1root_node+1;
set:  beam_node3 = lpy1root_node+2;
include: "generic_beamel.elm";

# *********  right pylon 1 **********

set:  curr_beam  = rpy1beam;
set:  beam_ref1   = rpy1_RF;
set:  beam_ref2   = rpy1_RF;
set:  twist1 = 0.*deg2rad;
set:  twist2 = 0.*deg2rad;
set:  beam_node1 = rpy1root_node;
set:  beam_node2 = rpy1root_node+1;
set:  beam_node3 = rpy1root_node+2;
include: "generic_beamel.elm";
