# Rotors Body Assignments

# right wing rotors
#remark: "rrot1_node", rrot1_node;

set: integer rotnodeID = rrot1_node;
#remark: "rotnodeID", rotnodeID;
set: real rot_m = 8.;
set: real rot_Ixx = 10.;
set: real rot_Iyy = 10.;
set: real rot_Izz = 1000.;
include: "rotor_body.elm";

set:  rotnodeID = rrot2_node;
set:  rot_m = 8.;
set:  rot_Ixx = 10.;
set:  rot_Iyy = 10.;
set:  rot_Izz = 20.;
include: "rotor_body.elm";

# left wing rotors
set:  rotnodeID = lrot1_node;
set:  rot_m = 8.;
set:  rot_Ixx = 10.;
set:  rot_Iyy = 10.;
set:  rot_Izz = 20.;
include: "rotor_body.elm";

set:  rotnodeID = lrot2_node;
set:  rot_m = 8.;
set:  rot_Ixx = 10.;
set:  rot_Iyy = 10.;
set:  rot_Izz = 20.;
include: "rotor_body.elm";
