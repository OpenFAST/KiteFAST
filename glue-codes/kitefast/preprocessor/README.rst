MBDyn Preprocessor Documentation
================================

This is the initial version of the preprocessor for MBDyn and KiteFAST. The
purpose of this tool is to abstract the complexity of the MBDyn input file
into more human readable format.

General Guidelines
------------------
- Quantities are in SI units (kg, m, s); angles are in degrees and angular
  velocities are in rad/s
- Reference lines should be monotonically increasing or decreasing in their
  primary direction; overlapping points will cause an error
- The kite coordinate system is

 - x pointed forward (in the primary direction of flight)
 - y pointed starboard (right) (when looking in the primary direction of
   flight)
 - z pointed down (following the right-hand rule)

- The global coordinate system relative to the kite's coordinate system at
  level flight when traveling in the nominally upwind direction is rotated 180
  degrees about y:

 - global x = -1 * kite x
 - global y =      kite y
 - global z = -1 * kite z

YAML syntax
-----------
"YAML (YAML Ain't Markup Language) is a human-readable data serialization
language." It is a superset of JSON which uses indentation and a few key
symbols as control structure.
See the `YAML documentation <https://pyyaml.org/wiki/PyYAMLDocumentation>`_
for detailed information. Here are some general rules regarding yaml and other
rules specific to the preprocessor input file:

- Indentation is meaningful (just like Python syntax)
- Comments can be placed anywhere with a "#"
- Scalar values can be given in-line with their key or indented on the
  following line; these two are equivalent

::

    title: M600 PreProcessor Input File

    title:
        M600 PreProcessor Input File

- Vector quantities are comma separated; for example

::

    gravity:
        [0.0, 0.0, -9.81]

- List elements are denoted by a leading "-" such as

::

    element_end_nodes:
        #  z       twist    attached component   point mass
        #  m       deg                           kg
        - [-5.000, 0.0,     none,                0]
        - [-3.000, 0.0,     none,                0]
        - [ 0.000, 0.0,     wing,                0]
        - [ 2.000, 0.0,     none,                0]

- Null values in the "attached component" columns should be denoted by "none"

Kite Geometry Assumtions
------------------------
- Vertical stabilizer is attached to the aft-most node of the fuselage
- Nacelles are attached to the endpoints of the pylons
- Wings have an equal number of pylons

Key Points
----------
The ``keypoints`` table locates each component's origin point. These points
are defined in the kite coordinate system.

The only keypoint which is not allowed to be specified by the user is the
fuselage keypoint which is hard coded to the kite's origin at (0, 0, 0).

Finite Element Beams
--------------------
A three-node beam element is used in the MBDyn solver. These elements are
defined in the input file by their end nodes in the ``element_end_nodes`` table
and the third node is automatically added by the preprocessor at the midpoint.

Components
----------
Structural components are defined by three tables

- element_end_nodes
- stiffness_matrix
- mass_distribution

Each component has a primary axis

 - fuselage: x
 - wings: y
 - vertical stabilizer: z
 - horizontal stabilizer: y
 - horizontal stabilizer: y
 - pylons: z

element_end_nodes
~~~~~~~~~~~~~~~~~
This table defines the end nodes for each element in the finite element solver.
The nodes are located in space by a single-component offset in the direction of
the primary axis of the associated kite component relative to origin defined in
the ``keypoints`` table in the kite coordinate system. This table also
specifies the twist at each node, connects other kite components, and adds a
point mass. The twist applies only to the section stiffness not the
mass, center of mass, or intertia, and it should be given relative to the
positive direction of the component's primary axis.

Nodes have a local coordinate system which is initially aligned with the kite
system but rotated by the node's twist in the component's primary axis.

stiffness_matrix
~~~~~~~~~~~~~~~~
This table defines the diagonal and upper-triangular portion of a symmetric 6x6
cross-sectional stiffness matrix at each end node. Thus, the number of rows in
this table must match the number of rows in the ``element_end_nodes`` table.
These stiffness quantities are defined at the node in the node's coordinate
system.

It is important to note that MBDyn expects the stiffness properties at the
finite element's gaussian points which are located at +/- 1/sqrt(3) from the
element midpoint (nondimensionally). The nodal stiffness properties will be
interpolated by the preprocessor to these locations.

mass_distribution
~~~~~~~~~~~~~~~~~
This table defines the cross-sectional mass, center of mass, and inertia
distribution at each end node. Thus, the number of rows in this table must
match the number of rows in the ``element_end_nodes`` table. These mass and
inertia quantities are defined at the node in the node's coordinate system.
The given nodal mass distribution will be integrated and distributed as
lumped masses by the preprocessor.
