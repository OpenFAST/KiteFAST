#Horizontal Stabilizer Beam Definition

set: integer lhsbeam =9000;
set: integer rhsbeam =11000;


# Cross Section Stiffness Matrix
set:  EA  = 1.0e+8;
set:  GAy = 1.0e+8;
set:  GAz = 1.0e+8;
set:  GJx  = 1.0e+6;
set:  EJyy = 2.0e+6;
set:  EJzz = 4.0e+8;
set:  yeamsc = 0.0;   #distance along local y of EA_center from shear center
set:  zeamsc = 0.0;  #distance along local z of EA_center from shear center

# mass per length
set:  m = 10.;
set:  L_beam = 1.;

# inertial moment per length
set:  Ixx  = 1;
set:  Iyy  = 0.;
set:  Izz  = 0.;

# *********  left HS **********

set:  curr_beam   = lhsbeam;
set:  beam_ref1   = lhsroot_RF;
set:  beam_ref2   = lhsroot_RF;
set:  twist1 = 0.;
set:  twist2 = 0.;
set:  beam_node1 = lhsroot_node;
set:  beam_node2 = lhsroot_node +1;
set:  beam_node3 = lhsroot_node +2;
include: "generic_beamel.elm";

# *********  right HS **********

set:  curr_beam   = rhsbeam;
set:  beam_ref1   = rhsroot_RF;
set:  beam_ref2   = rhsroot_RF;
set:  twist1 = 0.;
set:  twist2 = 0.;
set:  beam_node1 = rhsroot_node;
set:  beam_node2 = rhsroot_node +1;
set:  beam_node3 = rhsroot_node +2;
include: "generic_beamel.elm";
