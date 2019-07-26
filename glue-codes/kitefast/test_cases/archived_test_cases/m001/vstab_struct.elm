#Vertical Stabilizer Beam Definition


set: integer vsbeam =7000;

# Cross Section Stiffness Matrix
set:  EA  = 1.0e+8;
set:  GAy = 1.0e+8;
set:  GAz = 1.0e+8;
set:  GJx  = 1.0e+6;
set:  EJyy = 2.0e+6;
set:  EJzz = 4.0e+8;
set:  yeamsc = 0.05;   #distance along local y of EA_center from shear center
set:  zeamsc = 0.07;  #distance along local z of EA_center from shear center

# mass per length
set:  m = 7.;
set:  L_beam = 1.;

# inertial moment per length
set:  Ixx  = 1;
set:  Iyy  = 0.;
set:  Izz  = 0.;


# *********  VS **********

set: curr_beam  = vsbeam;
set:  beam_ref1   = vsroot_RF;
set:  beam_ref2   = vsroot_RF;
set:  twist1 = 0.;
set:  twist2 = 0.;
set:  beam_node1 = vsroot_node;
set:  beam_node2 = vsroot_node +1;
set:  beam_node3 = vsroot_node +2;
include: "generic_beamel.elm";
