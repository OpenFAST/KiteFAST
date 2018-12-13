#Fuselage Beam Definition

set: integer fusbeam  =1000;

# Cross Section Stiffness Matrix
set: real EA  = 1.0e+8;
set: real GAy = 1.0e+8;
set: real GAz = 1.0e+8;
set: real GJx  = 1.0e+6;
set: real EJyy = 2.0e+6;
set: real EJzz = 4.0e+8;
set: real yeamsc = 0.0;   #distance along local y of EA_center from shear center
set: real zeamsc = 0.0;  #distance along local z of EA_center from shear center


# mass per length
set: real m = 10.;
set: real L_beam = 1.;

# inertial moment per length
set: real Ixx  = 1;
set: real Iyy  = 0.;
set: real Izz  = 0.;


# *********  fuselage ************

set: integer curr_beam  = fusbeam;
set: integer beam_ref1   = MIP_RF;
set: integer beam_ref2   = MIP_RF;
set: real twist1 = 0.;
set: real twist2 = 0.;
set: integer beam_node1 = MIP_node-1;
set: integer beam_node2 = MIP_node;
set: integer beam_node3 = MIP_node+1;
include: "generic_beamel.elm";

#Add extra mass at the nose MIP_node-1
set: real extramass = 15.853 ;#kg to move the CG forward
body: 105 , beam_node1,
        extramass,
        reference,MIP_RF,7.231,0.,0.,
        diag, 0.,0.,0.;
