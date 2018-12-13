#Wing Beam Definition

set: integer rwbeam =3000;
set: integer lwbeam =5000;


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

# *********  left wing **********

set:  curr_beam   = lwbeam;
set:  beam_ref1   = lwroot_RF;
set:  beam_ref2   = lwroot_RF;
set:  twist1 = 0.*deg2rad;
set:  twist2 = 0.*deg2rad;
set:  beam_node1 = lwroot_node;
set:  beam_node2 = lwroot_node+1;
set:  beam_node3 = lwroot_node+2;
include: "generic_beamel.elm";

# *********  right wing **********
set:  curr_beam   = rwbeam;
set:  beam_ref1   = rwroot_RF;
set:  beam_ref2   = rwroot_RF;
set:  twist1 = 0.*deg2rad;
set:  twist2 = 0.*deg2rad;
set:  beam_node1 = rwroot_node;
set:  beam_node2 = rwroot_node+1;
set:  beam_node3 = rwroot_node+2;
include: "generic_beamel.elm";
