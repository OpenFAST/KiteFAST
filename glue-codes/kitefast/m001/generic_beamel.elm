#Generic beam element properties for the fuselage beams

set:   l1 = (0.5-1/(2*(3^0.5)))*L_beam;
set:   l2 = 1/(3^0.5)*L_beam;
set:   l3 = l1;

# nodal partition
set:  m1  = m*l1;   # nodal mass of the three part of the beam
set:  m2  = m*l2;
set:  m3  = m*l3;

set:  Jx1 = Ixx*l1;
set:  Jy1 = Iyy*l1 + (1/12)*l1^2*m1;
set:  Jz1 = Izz*l1 + (1/12)*l1^2*m1;
set:  Jx2 = Ixx*l2;
set:  Jy2 = Iyy*l2 + (1/12)*l2^2*m2;
set:  Jz2 = Izz*l2 + (1/12)*l2^2*m2;
set:  Jx3 = Ixx*l3;
set:  Jy3 = Iyy*l3 + (1/12)*l3^2*m3;
set:  Jz3 = Izz*l3 + (1/12)*l3^2*m3;


# inertial properties
body: curr_beam , beam_node1,
        m1,
        reference,node,null,
        diag, Jx1, Jy1, Jz1;
body: curr_beam+1, beam_node2,
        m2,
        reference,node,null,
        diag, Jx2, Jy2, Jz2;
body: curr_beam+2, beam_node3,
        m3,
        reference,node,null,
        diag, Jx3, Jy3, Jz3;

# _________________elastic properties_________________
beam3:  curr_beam,
        beam_node1, reference, node, null,
        beam_node2, reference, node, null,
        beam_node3, reference, node, null,
        reference, beam_ref1,  1, 1.,          0.,          0.,
                               2, 0., cos(twist1), sin(twist1),
        linear elastic generic, sym,
             EA,   0.,   0.,   0.,   zeamsc*EA,        -yeamsc*EA,
                  GAy,   0.,   0.,          0.,                0.,
                        GAz,   0.,          0.,                0.,
                              GJx,          0.,                0.,
                              EJyy+zeamsc^2*EA, -zeamsc*yeamsc*EA,
                                                 EJzz+yeamsc^2*EA,
        reference, beam_ref2,  1, 1.,          0.,          0.,
                               2, 0., cos(twist2), sin(twist2),
        same; # the keyword same must be replaced by the linear elastic...... and the new 6x6 for station 2 of the beam
