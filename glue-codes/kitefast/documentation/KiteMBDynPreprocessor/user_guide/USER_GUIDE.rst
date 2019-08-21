KiteMBDyn Preprocessor User Guide
=================================

This document is a user guide for the KiteMBDyn Preprocessor as it relates to
KiteFAST and MBDyn. The purpose of this tool is to abstract the complexity of
the MBDyn input file into more user friendly format.

A separate developer's guide is planned for future development.

The KiteMBDyn Preprocessor is a Python framework for constructing generic
``components`` and attaching them in a particular way to create a full
``model``. Each model also has an associated format for the main MBDyn
files which will describe it. The geometry is given in a standard YAML format
with straight components and a mass distribution. The Preprocessor interpolates
structural quantities to the Gauss locations for the structural solve and
distributes the integrated mass and inertias into a lumped mass and inertia.
See the KiteMBDyn Preprocessor Theory for more information.

Command Line interface
----------------------
The KiteMBDyn Preprocessor should be executed via a bash command line.
The command line interface can be explored by supplying the `-h` flag.

.. code-block:: bash

    $ python preprocess.py -h
        usage: preprocess.py [-h] -i INPUT_FILE [-o [OUTPUT_DIRECTORY]]
                            [-m [MODEL_TYPE]] [-info]

        Creates a set of MBDyn input files from a model definition.

        optional arguments:
        -h, --help            show this help message and exit
        -i INPUT_FILE, --input-file INPUT_FILE
                                Path to the preprocessor input file
        -o [OUTPUT_DIRECTORY], --output-directory [OUTPUT_DIRECTORY]
                                Path to the directory where the output should be
                                written
        -m [MODEL_TYPE], --model-type [MODEL_TYPE]
                                Available options: kite (default), beam
        -info, --show-component-info
                                Display mass, center of mass, and inertia for each
                                component

General Guidelines
------------------
- Quantities are in SI units (kg, m, s); angles are in degrees and angular
  velocities are in rad/s
- Reference lines should be monotonically increasing or decreasing in their
  primary direction; overlapping points will cause an error
- The kite coordinate system (body frame) is

 - x pointed forward (in the primary direction of flight)
 - y pointed starboard (right) (when looking in the primary direction of
   flight)
 - z pointed down (following the right-hand rule)

- The global coordinate system (inertial frame) relative to the kite's
  coordinate system at level flight when traveling in the nominally upwind
  direction is rotated 180 degrees about y:

 - global x = -1 * kite x
 - global y =      kite y
 - global z = -1 * kite z

YAML syntax
-----------
"YAML (YAML Ain't Markup Language) is a human-readable data serialization
language." It is a superset of JSON which uses indentation and a few key
symbols as control structure.
See the `YAML documentation <https://pyyaml.org/wiki/PyYAMLDocumentation>`_
for detailed information. Here are some general rules regarding YAML and other
rules specific to the preprocessor input file:

- Indentation is meaningful (just like Python syntax)
- Comments can be placed anywhere with a "#"
- Scalar values can be given in-line with their key or indented on the
  following line; these two are equivalent

.. code-block:: yaml

    title: M600 PreProcessor Input File

    title:
        M600 PreProcessor Input File

- Vector quantities are comma separated and wrapped in brackets; for example

.. code-block:: yaml

    gravity:
        [0.0, 0.0, -9.81]

- List elements are denoted by a leading "-" such as

.. code-block:: yaml

    element_end_nodes:
        #  z       twist    attached component   point mass
        #  m       deg                           kg
        - [-5.000, 0.0,     none,                0]
        - [-3.000, 0.0,     none,                0]
        - [ 0.000, 0.0,     wing,                0]
        - [ 2.000, 0.0,     none,                0]

- NOTE: Null values in the "attached component" columns should be denoted
  by "none"

Kite Geometry Assumptions and Limitations
-----------------------------------------
- Nacelles are attached to the endpoints of the pylons
- Wings have an equal number of pylons
- All components are geometrically straight; any curvature is modeled by center
  of mass offsets

Key Points
----------
The ``keypoints`` table locates each component's origin point. These points
are defined in the kite coordinate system.

The only keypoint which is not allowed to be specified by the user is the
fuselage keypoint which is hard coded to the kite's origin at (0, 0, 0).

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
A three-node beam element is used in the MBDyn solver. These elements are
defined in the input file by their end nodes in the ``element_end_nodes`` table
and the third node is automatically added by the preprocessor at the midpoint.

These nodes are located in space by a single-component offset in the direction
of the primary axis of the associated kite component relative to origin defined
in the ``keypoints`` table in the kite coordinate system. This table also
specifies the twist at each node, connects other kite components, and adds a
point mass. The twist applies only to the section stiffness not the
mass, center of mass, or inertia, and it should be given relative to the
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

Rotors
------
The ``rotor`` blocks are distinct from other components in that they models a
single body in space with an associates mass, center of mass offset,
translational inertia and rotational inertia.

.. code-block:: yaml

    rotor_assembly:
        starboard:
            1:
                upper:
                    rotor:
                        mass_properties:
                            # [mass, Cmx,  I_rot, I_trans]
                            # [  kg,   m, kg*m^2,  kg*m^2]
                            [7.700, 0.0, 1.610, 0.805]

                    nacelle:
                        mass_properties:
                            # [mass, Cmx, Cmy, Cmz,    Ixx,    Iyy,    Izz,    Ixy,    Ixz,    Iyz]
                            # [  kg,   m,   m,   m, kg*m^2, kg*m^2, kg*m^2, kg*m^2, kg*m^2, kg*m^2]
                            [55.150, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                lower:
                    rotor:
                        mass_properties:
                            # [mass, Cmx, I_rot, I_trans]
                            # [  kg,   m, kg*m^2,  kg*m^2]
                            [7.700, 0.0, 1.610, 0.805]

                    nacelle:
                        mass_properties:
                            # [mass, Cmx, Cmy, Cmz,    Ixx,    Iyy,    Izz,    Ixy,    Ixz,    Iyz]
                            # [  kg,   m,   m,   m, kg*m^2, kg*m^2, kg*m^2, kg*m^2, kg*m^2, kg*m^2]
                            [55.150, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

Platform
--------
Similar to the rotors, the ``platform`` block models a single body in space
with an associates mass, center of mass offset, translational inertia and
rotational inertia.

Though one point mass is modeled, two points must be given. In additional to
the node location, an IMU location is also supplied and passed to KiteFAST.

.. code-block:: yaml

    platform:
        mass_properties:
            # [mass, CM offset, translational intertia, rotational inertia]
            [7.700, 0.0, 1.610, 0.805]

        node_location:
            [0.0, 0.0, 0.0]

        imu_location:
            [1.0, 1.0, 1.0]

Simulation Controls
-------------------
Various simulation controls for the MBDyn and KiteFAST portions of the
simulation are given in the ``simulation_controls`` section. These are
generally passed directly to the appropriate portions of the software
and are not modified by the preprocessor except where indicated below.
Thus, various types of data can generally be interchanges. For example,
most fields allow for their values to be a number wrapped in quotes ("10")
or a numeric value (10). This is useful in fields where MBDyn allows various
forms of input like ``max_iterations``.

fast_submodules, fast_submodule_input_files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Settings for enabling the various modules of the system and specifying the
necessary file locations.

.. code-block:: yaml

    fast_submodules:
        kiteaerodyn: true
        inflowwind: true
        moordyn: true
        controller: true

    fast_submodule_input_files:
        kiteaerodyn_input: "../kiteaerodyn/simple_m600_model.inp"
        inflowwind_input: "../kiteinflow/kiteInflowWind.dat"
        moordyn_input: "../kitemooring/m600-MoorDyn.dat"
        controller_input: "../../../../build/modules-local/kitefast-controller/libkitefastcontroller_controller.so"

print_kitefast_summary_file, kitefast_output_file_root_name
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Settings for the KiteFAST output.

.. code-block:: yaml

    print_kitefast_summary_file:
        false

    kitefast_output_file_root_name:
        "KiteFast"

kiteaerodyn_interpolation_order
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Order of interpolation for KiteAeroDyn outputs.

- 0: Hold KAD outputs between KAD calls
- 1: Linearly interpolate outputs
- 2: 2nd order interpolation of outputs

.. code-block:: yaml

    kiteaerodyn_interpolation_order:
        0

time
~~~~
Time settings for the simulation. These are important to both MBDyn and
KiteFAST.

.. code-block:: yaml

    time:
        timestep:
            1e-3

        final:
            60

Solver settings
~~~~~~~~~~~~~~~
These solver settings are relevant only to MBDyn.

The ``tolerance`` is the threshold for convergence in the iterative residual
and ``max_iterations`` is the number of iterations to complete before
aborting the simulation if the tolerance is not reached. An additional option
is available for whereby specifying "N, at most" avoids the residual check and
completes N number of iterations. If the final residual is less than the first
residual, the time marching continues.

TODO: descrive ``derivatives``, ``linear_solver``

.. code-block:: yaml

    tolerance:
        10

    max_iterations:
        10

    derivatives:
        tolerance:
            10e0

        max_iteration:
            200

        coefficient:
            1e-4

    linear_solver:
        naive

Model Settings
~~~~~~~~~~~~~~
These are additional miscellaneous flags for further modeling configurations.

The ``rigid_model`` feature adds a joint between each node in the model such
that there are no elastics calculated. It must be either "true" or "false"

``debug`` adds additional MBDyn debugging information. This is primarily useful
for determining which portions of the model are causing convergence issues. It
must be either "true" or "false"

.. code-block:: yaml

    rigid_model:
        false

    debug:
        false

Reference Points
~~~~~~~~~~~~~~~~
Some reference points must be defined for the offshore simulation.

The ``wind_reference_station`` locates the anemometer location on the buoy,
the ``ground_weather_station`` locates the ground station. These points are
passed directly to KiteFAST and used to interface with the controller. Both
points are given in order of x-y-z components relative to the global origin.

.. code-block:: yaml

    wind_reference_station:
        location:
            [2.0, 2.0, 2.0]

    ground_weather_station:
        location:
            [-0.202, -1.627, 19.860]

Initial Conditions
~~~~~~~~~~~~~~~~~~
The location, orientation, and velocity of the model at initial time are given
here.

The ``location`` places the MIP of the kite in the global frame. Since the
fuselage keypoint is forced to (0,0,0), this point is passed to KiteFAST as the
fuselage key point in the back side. it is given in order of x-y-z components
relative to the global origin.

The ``orientation`` field lists the initial Euler angles of the kite in order
of roll-pitch-yaw. These angles are converted to a DCM in the Preprocessor
using the SciPy Spatial Transform library. This rotation is "intrinsic"
(https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_euler.html#scipy.spatial.transform.Rotation.from_euler)
meaning that each rotation occurs relative to the body frame. The roll occurs
first. Then, the kite is pitched relative to the orientation after the roll.
Finally, the yaw rotation happens relative to the intermediate orientation of
the kite after the pitch.

The ``velocities`` are given in both translational and rotational components.
Translational velocities are given in m/s in order of x-y-z components, and
rotational velocities are given in rad/s in order of roll-pitch-yaw components.

.. code-block:: yaml

    initial_conditions:
        location:
            [126.6398, -379.6509, 172.8097]

        orientation:
            [-135.2768,-26.8122,15.2612]

        velocity:
            translational:
                # [x, y, z]
                [53.9008, 3.3295, -29.7069]

            rotational:
                # [roll, pitch, yaw]
                [0.1474, -0.2842, -0.3232]


Outputs
-------
The general KiteFAST outputs, those that are not associated with a specific
module, are requested through the Preprocessor input file. The logic follows
the general OpenFAST structure for requesting outputs.

To request a node for output, list its index (these are indexed from 1) under
the given component. Then, construct the output channel string by combining the
physical quantity abbreviation with the index of the node in the list of
requested nodes under a component. For example, the block below enables
output for fuselage nodes 2 and 3. To output the x-component of the deflection
for these nodes, the corresponding strings are "Fus1TDx" and "Fus2TDx".
Additional channels are available for the entire kite. To turn off output for
a particular component, list "- 0" for the nodes.

See KiteFASTMBD_Plan.pdf for more information.

TODO: Describe outputs for the pylons and rotors

.. code-block:: yaml

    output:
        fuselage_nodes:
            - 4
            - 5

        wing_starboard_nodes:
            - 2
            - 3

        wing_port_nodes:
            - 0

        vertical_stabilizer_nodes:
            - 0

        horizontal_stabilizer_starboard_nodes:
            - 0

        horizontal_stabilizer_port_nodes:
            - 0

        pylon_nodes:
            - 1
            - 2

        output_channels:
            - "Fus1TDx"
            - "Fus1TDy"
            - "Fus1TDz"
            - "SWn1TDx"
            - "SWn1TDy"
            - "SWn1TDz"
            - "PP12FRc"
            - "KiteRoll"
            - "KitePitch"
            - "KiteYaw"
