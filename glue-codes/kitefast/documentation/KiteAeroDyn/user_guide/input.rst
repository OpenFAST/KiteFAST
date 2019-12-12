.. _kad_input:

Input Files
===========

The user configures the aerodynamic model parameters via a primary
KiteAeroDyn input file, as well as separate input files for airfoil and
rotor performance data. When used in standalone mode, an additional driver input
file is required. This driver file specifies initialization inputs
normally provided to KiteAeroDyn through KiteFAST, as well as the per-time-step
inputs to KiteAeroDyn.

As an example, the ``driver.dvr`` file is the main driver, the ``input.dat`` 
is the primary input file, the ``rotor.dat`` file contains the rotor
performance data, and the ``airfoil.dat`` file contains the airfoil
angle of attack, lift, drag, and moment coefficients. 
Example input files are included in :numref:`kad_appendix`.

No lines should be added or removed from the input files, except in
tables where the number of rows is specified, lines to specify requested output channel names, 
and comment lines in the airfoil data files.

Units
-----

KiteAeroDyn uses the SI system (kg, m, s, N), except for angles. Angles are assumed to be in
degrees unless otherwise specified.  

Additionally, the user-created KiteFAST controller determines the ``Ctrl`` 
(control signal, e.g. flap angle) units in the airfoil input file.

-------------------------

The driver input file is only needed for the standalone version of
KiteAeroDyn and contains inputs normally defined by the calling program (KiteFAST), and necessary to
control the aerodynamic simulation for uncoupled models. A sample
KiteAeroDyn driver input file is given in 
:numref:`kad_appendix`.

Set the ``Echo`` flag in this file to TRUE if you wish to have the
``KiteAeroDyn_Driver`` executable echo the contents of the driver input file (useful
for debugging errors in the driver file). The echo file has the naming
convention of *KAD_DvrFile.ech*, where ``KAD_DvrFile`` is
name of the KiteAeroDyn driver input file.  ``DTAero`` is the time interval (time step) for 
the aerodynamic calculations, in seconds.
``KAD_InFile`` is the filename of the primary KiteAeroDyn input file.
This name should be in quotations and can contain an absolute path or a
relative path.

The ENERGY KITE REFERENCE CONFIGURATION section defines the KiteAeroDyn-required 
geometry for the kite, see Figure 1. ``NumFlaps`` specifies the number
of control flaps per wing-side (starboard/port).  ``NumPylons`` specifies the number 
of pylons per wing side (starboard/port).  ``NumFlaps`` and ``NumPylons`` 
must each be greater than zero. Each kite component is defined relative to 
its reference point.  These reference points are specified in the kite 
coordinate system.  The table consists of three numbers (separated by spaces) on a single line to 
represent the x, y, and z coordinates of that component's reference point.
There must be a line for each pylon reference point (starting inboard and moving outboard) and a line for each rotor (top, bottom) on each pylon.  For example, if 
``NumPylons`` equals 2, then there will be 4 lines corresponding to the 
various pylon reference points (2 for the starboard wing, 2 for the port wing) and 8 lines corresponding to the top and bottom rotors on each pylon.  The Fuselage must have its reference 
point located at (0,0,0) and therefore does not show up in the table.

The I/O SETTINGS section controls the creation of the results file. If
``OutFileRoot`` is specified, the results file will have the filename
*OutFileRoot.out*.  If an empty string is provided for
``OutFileRoot``, then the driver file’s root name will be used
instead. If ``TabDel`` is ``TRUE``, a TAB character is used between
columns in the output file; if FALSE, fixed-width is used otherwise.
``OutFmt`` is any valid Fortran numeric format string, which is used
for text output, excluding the time channel. The resulting field should
be 10 characters, but KiteAeroDyn does not check ``OutFmt`` for validity.
If you want a sound generated on program exit, set ``Beep`` to true.

.. figure:: figs/kad_driver_geom.png
   :width: 60%
   :align: center

KiteAeroDyn Driver Kite Geometry
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The WIND AND ENERGY KITE TIME-HISTORY MOTION section allows you to execute a 
simulation for the given kite specified in the ENERGY KITE REFERENCE CONFIGURATION section and 
its corresponding ``KAD_InFile`` based on a set of inputs which drive the motions of the kite.

The local undisturbed wind speed at any elevation above the ground is determined using,

.. math::
   :label: windspeed

   U(Z) = \mathrm{HWindSpd} \times \left( \frac{Z}{\mathrm{RefHt}} \right)^\mathrm{PLexp}

where :math:`\mathrm{HWindSpd}` is the steady wind speed located at elevation :math:`\mathrm{RefHt}`, :math:`Z` is the
instantaneous elevation of an analysis node above the ground (or
above the MSL for offshore kites, and :math:`\mathrm{PLexp}` is the power-law shear exponent. 
This wind propagates in the direction of :math:`\mathrm{HWindDir}`  (in degrees), which follows meteorological convention (clockwise when looking downward), with 0 aligned with global X and positive rotation towards global -Y.

There will be one row in the subsequent table for each of
the time steps specified, ``NumTimes``, (plus two table header lines). The
information within each row of the table fully specifies the necessary inputs to KiteAeroDyn for a given time step. 
Each row contains the following columns (in the given order) [Note: the presence of some columns depends on the values of ``NumPylons`` and ``NumFlaps``]: 
``Time``: timestamp in seconds for the row of input data
``KitePxi``, ``KitePyi``, ``KitePzi``: X, Y, and Z location of the kite's fuselage reference (0,0,0) point in the inertial, global reference system, (m) 
``KiteRoll``, ``KitePitch``, ``KiteYaw``: orientation of the kite, specified via a 1-2-3 Euler sequence (degrees)
``KiteTVxi``, ``KiteTVyi``, ``KiteTVzi``: translational velocities of the kite's fuselage reference (0,0,0) point in the inertial, global reference system, (m/s) 
``KiteRVxi``,  ``KiteRVyi``,  ``KiteRVzi``: rotational velocities about the kite's coordinate system in the inertial, global reference system, (degrees/s) 
``SP1TRtSpd``, ``SP1BRtSpd``, ``SP2TRtSpd``, ``SP2BRtSpd``: Starboard wing rotor speeds starting inboard and moving outboard and alternating top and then bottom on each pylon [number of columns must match ``NumPylons`` times two] (radians/s)
``PP1TRtSpd``, ``PP1BRtSpd``, ``PP2TRtSpd``, ``PP2BRtSpd``: Port wing rotor speeds starting inboard and moving outboard and alternating top and then bottom on each pylon [number of columns must match ``NumPylons`` times two] (radians/s)
``SP1TPitch``, ``SP1BPitch``, ``SP2TPitch``, ``SP2BPitch``: Starboard wing collective blade pitch angles starting inboard and moving outboard and alternating top and then bottom on each pylon [number of columns must match ``NumPylons`` times two] (degrees)
``PP1TPitch``, ``PP1BPitch``, ``PP2TPitch``, ``PP2BPitch``: Port wing collective blade pitch angles starting inboard and moving outboard and alternating top and then bottom on each pylon [number of columns must match ``NumPylons`` times two] (degrees)
``SFlp1Ctrl``, ``SFlp2Ctrl``, ``SFlp3Ctrl``: Starboard wing flap control angle settings [number of columns must match ``NumFlaps``] (units must match airfoil tables ``Ctrl`` units)
``PFlp1Ctrl``, ``PFlp2Ctrl``, ``PFlp3Ctrl``: Port wing flap control angle settings [number of columns must match ``NumFlaps``] (units must match airfoil tables ``Ctrl`` units)
``Rudr1Ctrl``, ``Rudr2Ctrl``: Control angle settings for the vertical stabilizer surface (units must match airfoil tables ``Ctrl`` units)
``SElv1Ctrl`` ``SElv2Ctrl``: Control angle settings for the starboard horizontal stabilizer surface (units must match airfoil tables ``Ctrl`` units)
``PElv1Ctrl`` ``PElv2Ctrl``: Control angle settings for the port horizontal stabilizer surface (units must match airfoil tables ``Ctrl`` units)

The first row of the TIME-HISTORY MOTION table must have a time stamp of 0.0 seconds. 
The ``Time`` values must then be monotonically increasing for the remaining rows.  
The rows do not need to be spaced ``DTAero`` seconds apart, even though the time marching increments on ``DTAero``.
The driver code will linearly interpolate data in the motion table to generate appropriate inputs for the nth ``DTAero`` increment. 
The simulation will end at the timestamp which is an integer multiple of ``DTAero`` but is less than or equal to the last timestamp in the motion table.

KiteAeroDyn Primary Input File
------------------------------
 
The primary KiteAeroDyn input file defines modeling options, environmental
conditions (except freestream flow), airfoils, aerodynamic nodal
discretization and properties, rotor properties, as well as output file specifications.

The file is organized into several functional sections. Each section
corresponds to an aspect of the aerodynamics model. A sample KiteAeroDyn
primary input file is given in 
:numref:`kad_appendix`.

The input file begins with two lines of header information which is for
your use, but is not used by the software.

Simulation Control
~~~~~~~~~~~~~~~~~~

Set the ``Echo`` flag to TRUE if you wish to have KiteAeroDyn echo the
contents of the KiteAeroDyn primary input file (useful
for debugging errors). The echo file has the naming
convention of *BaseFilename.KAD.ech*. ``BaseFilename`` is the filename of the
primary KiteAeroDyn input file.

``DTAero`` sets the time step for the aerodynamic calculations. 
The keyword ``DEFAULT`` for ``DTAero`` may be used to indicate that KiteAeroDyn should employ the
time step prescribed by the driver code (KiteFAST or the standalone driver
program).

Set ``LiftMod`` to 1 if you want to disable wake/induction
Effects (resulting in a simple geometric angle of attack) or 2 to include these effects using the vortex-step model. When
``RotorMod`` is set to 0, no rotor power or loads are computed.
Setting ``RotorMod`` to 1 causes KiteAeroDyn to compute loads and power via a simple actuator disk model.

The ``UseCM`` option is currently unimplemented, and defaults to TRUE, which means the ``InColCM``, described in the Airfoil Information section, below, must be > 0. 


Environmental Conditions
~~~~~~~~~~~~~~~~~~~~~~~~

``AirDens`` specifies the fluid density and must be a value greater
than zero; a typical value is around 1.225 kg/m\ :sup:`3` for air.
``KinVisc`` specifies the kinematic viscosity of the air (used in the
Reynolds number calculation); a typical value is around 1.460E-5
m\ :sup:`2`/s for air. ``SpdSound`` is the speed of sound in air; a typical value is around 340.3 m/s.

Lifting Line Vortex-step Method Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The input parameters in this section are not used when ``LiftMod = 1``.

``VSMMod`` determines the propagation direction for the wakes. Set ``VSMMod`` to 1 
to use the local chord to determine the wake alignment. 
Set ``VSMMod`` to 2 to align the wakes with the kite-averaged free stream direction. 
``VSMToler`` sets the convergence threshold for the iterative
nonlinear Newton solve of the vortex solution. ``VSMToler`` represents the tolerance of the
residual equation, which has the units of circulation times velocity.
When the keyword ``DEFAULT`` is used in place of a numerical value,
``VSMToler`` will be set to 1E-4. ``VSMMaxIter`` determines the maximum
number of Netwon iterations in the solve. When the keyword ``DEFAULT`` is used in place of a numerical value,
``VSMMaxIter`` will be set to 40. If the residual value of
the solve is not less than or equal to ``VSMToler`` in
``VSMMaxIter`` iterations, KiteAeroDyn will exit the solver and return an error
message.  ``VSMPerturb`` sets the perturbation size for computing the Jacobian in the Newton iterations.   
When the keyword ``DEFAULT`` is used in place of a numerical value,
``VSMPerturb`` will be set to 0.05, which has the units of circulation times velocity. We recommend using these defaults.

.. _airfoil_information:

Airfoil Information
~~~~~~~~~~~~~~~~~~~

This section defines the airfoil data input file information. The
airfoil data input files themselves (one for each airfoil) include
tables containing coefficients of lift force, drag force, and optionally
pitching moment versus AoA, as well as UA model
parameters (currently unused by KiteAeroDyn), and are described in :numref:`airfoil_data_input_file`.

The ``AFTabMod`` setting determines the form of table lookup used in each airfoil data file.
*1* corresponds to a 1D lookup on angle of attack.  *2* provides a 2D lookup on angle of attack and Reynold's number.
*3* corresponds to a 2D lookup of angle of attack and a user-generated control value.
The next 4 lines in the AIRFOIL INFORMATION section relate to the
format of the tables of static airfoil coefficients within each of the
airfoil input files. ``InCol_Alfa``, ``InCol_Cl``,
``InCol_Cd``, and ``InCol_Cm are column
numbers in the tables containing the AoA, lift-force coefficient,
drag-force coefficient, and pitching-moment coefficient, respectively 
(normally these are 1, 2, 3, and 4, respectively).

Specify the number of airfoil data input files to be used using
``NumAFfiles``, followed by ``NumAFfiles`` lines of filenames. The
file names should be in quotations and can contain an absolute path or a
relative path e.g., “C:\\airfoils\\S809_CLN_298.dat” or
“airfoils\\S809_CLN_298.dat”. If you use relative paths, it is
relative to the location of the current working directory. The kite component sections
(described next) will reference these airfoil data using their line
identifier, where the first airfoil file is numbered 1 and the last
airfoil file is numbered ``NumAFfiles``.

Fuselage Properties
~~~~~~~~~~~~~~~~~~~

The fuselage is currently modeled as a drag-only body.  Specify the number of aerodynamic nodes with the
``NumFusNds`` parameter, followed by ``NumFusNds`` lines of nodal data (plus two table header lines).
``FusX``, ``FusY``, and ``FusZ`` determine the location of a node within the kite coordinate system,
and are relative to the fuselage reference point (0,0,0). These nodes can be ordered from either the front of the kite to the back (monotonically decreasing),
or from the back to the front (monotonically increasing).  Step changes in the fuselage geometry are specified with adjacent nodes having 
the same x-value but differing y or z values.  The airfoil at each node is assumed to be in the y-z plane, 
and-along with the nodal locations-the positive aerodynamic ``FusTwist`` is specified about positive x, 
and the chordlength (``FusChord``) and airfoil table ID (``FusAFID``) are specified. A zero-degree twist 
means positive y points toward the trailing edge and negative z points toward the suction side of the airfoil.


Starboard (Right) Wing Properties
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Specify the number of aerodynamic nodes with the
``NumSWnNds`` parameter, followed by ``NumSWnNds`` lines of nodal data (plus two table header lines).
``SWnX``, ``SWnY``, and ``SWnZ`` determine the location of a node within the kite coordinate system,
and are relative to the starboard wing's reference point (also given in the kite coordinate system). 
In the stand-alone KiteAeroDyn driver case, this point is specified
in the ENERGY KITE REFERENCE CONFIGURATION section of the driver input file.  In a KiteFAST-coupled simulation,
this point is defined in the preprocessor input file under the ``keypoints`` section and on the line labeled 
``wing/starboard``.  The locations of the aerodynamic nodes along the aerodynamic reference line aerodynamic center) 
with y monotonically increasing. The airfoil at each node is assumed to be rotated from the x-z plane based 
on the dihedral angle (``SWnDhdrl``) about negative x resulting in an inclined x-z’ plane (with y’ normal), 
and—along with the nodal locations—the positive aerodynamic twist (``SWnTwist``) is specified about positive y’, 
and the chordlength (``SWnChord``), airfoil table ID (``SWnAFID``), and flap ID (``SWnFlpID``) are specified. 
A zero-degree twist means negative x points toward the trailing edge and negative z’ points toward the 
suction side of the airfoil. Calculations for the lifting line vortex method take place at the 
midpoints between these nodes; instead of interpolating airfoil data, the airfoil and flap IDs 
at each midpoint is taken to be the airfoil and flap IDs of the corresponding node with lower y.


Port (Left) Wing Properties
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Specify the number of aerodynamic nodes with the
``NumPWnNds`` parameter, followed by ``NumPWnNds`` lines of nodal data (plus two table header lines).
``PWnX``, ``PWnY``, and ``PWnZ`` determine the location of a node within the kite coordinate system,
and are relative to the port wing's reference point (also given in the kite coordinate system). 
In the stand-alone KiteAeroDyn driver case, this point is specified
in the ENERGY KITE REFERENCE CONFIGURATION section of the driver input file.  In a KiteFAST-coupled simulation,
this point is defined in the preprocessor input file under the ``keypoints`` section and on the line labeled 
``wing/port``.  The locations of the aerodynamic nodes along the aerodynamic reference line (aerodynamic center) 
with y monotonically decreasing (negative values). The airfoil at each node is assumed to be rotated from the x-z plane based 
on the dihedral angle (``PWnDhdrl``) about positive x resulting in an inclined x-z’ plane (with y’ normal), 
and—along with the nodal locations—the positive aerodynamic twist (``PWnTwist``) is specified about positive y’, 
and the chordlength (``PWnChord``), airfoil table ID (``PWnAFID``), and flap ID (``PWnFlpID``) are specified. 
A zero-degree twist means negative x points toward the trailing edge and negative z’ points toward the 
suction side of the airfoil. Calculations for the lifting line vortex method take place at the 
midpoints between these nodes; instead of interpolating airfoil data, the airfoil and flap IDs 
at each midpoint is taken to be the airfoil and flap IDs of the corresponding node with higher (less negative) y.


Vertical Stabilizer Properties
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Specify the number of aerodynamic nodes with the
``NumVSNds`` parameter, followed by ``NumVSNds`` lines of nodal data (plus two table header lines).
``VSX``, ``VSY``, and ``VSZ`` determine the location of a node within the kite coordinate system,
and are relative to the vertical stabilizer's reference point (also given in the kite coordinate system). 
In the stand-alone KiteAeroDyn driver case, this point is specified
in the ENERGY KITE REFERENCE CONFIGURATION section of the driver input file.  In a KiteFAST-coupled simulation,
this point is defined in the preprocessor input file under the ``keypoints`` section and on the line labeled 
``stabilizer/vertical``.  The locations of the aerodynamic nodes (black nodes in figure above) along the 
aerodynamic reference line (aerodynamic center) are specified in the body-fixed (x,y,z) coordinate system relative 
to its origin, with z monotonically increasing (from possible negative to positive values). 
The airfoil at each node is assumed to be in the x-y plane, and—along with the nodal locations—the 
positive aerodynamic twist (``VSTwist``) is specified about positive z, 
and the chordlength (``VSChord``), airfoil table ID (``VSAFID``), 
and rudder ID (``VSRdrID``) are specified. A zero-degree twist means negative x points toward the trailing edge 
and positive y points toward the suction side of the airfoil. Calculations for the lifting line 
vortex method take place at the midpoints between these nodes; instead of interpolating airfoil data, 
the airfoil and rudder IDs at each midpoint is taken to be the airfoil and rudder IDs of the corresponding node with lower z.

Starboard (Right) Stabilizer Properties
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Specify the number of aerodynamic nodes with the
``NumSHSNds`` parameter, followed by ``NumSHSNds`` lines of nodal data (plus two table header lines).
``SHSX``, ``SHSY``, and ``SHSZ`` determine the location of a node within the kite coordinate system,
and are relative to the starboard horizontal stabilizer's reference point (also given in the kite coordinate system). 
In the stand-alone KiteAeroDyn driver case, this point is specified
in the ENERGY KITE REFERENCE CONFIGURATION section of the driver input file.  In a KiteFAST-coupled simulation,
this point is defined in the preprocessor input file under the ``keypoints`` section and on the line labeled 
``stabilizer/horizontal/starboard``.  The locations of the aerodynamic nodes (black nodes in figure above) 
along the aerodynamic reference line (aerodynamic center) are specified in the body-fixed (x,y,z) 
coordinate system relative to its origin, with y monotonically increasing. The airfoil at each node 
is assumed to be in the x-z plane, and—along with the nodal locations—the positive aerodynamic twist (``SHSTwist``) 
is specified about positive y, and the chordlength (``SHSChord``), airfoil table ID (``SHSAFID``), 
and elevator ID (``SHSElvID``) are specified. A zero-degree twist means negative x points toward the 
trailing edge and negative z points toward the suction side of the airfoil. Calculations for the 
lifting line vortex method take place at the midpoints between these nodes; instead of interpolating
airfoil data, the airfoil and elevator IDs at each midpoint is taken to be the airfoil and elevator IDs 
of the corresponding node with lower y.


Port (Left) Stabilizer Properties
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Specify the number of aerodynamic nodes with the
``NumPHSNds`` parameter, followed by ``NumPHSNds`` lines of nodal data (plus two table header lines).
``PHSX``, ``PHSY``, and ``PHSZ`` determine the location of a node within the kite coordinate system,
and are relative to the port horizontal stabilizer's reference point (also given in the kite coordinate system). 
In the stand-alone KiteAeroDyn driver case, this point is specified
in the ENERGY KITE REFERENCE CONFIGURATION section of the driver input file.  In a KiteFAST-coupled simulation,
this point is defined in the preprocessor input file under the ``keypoints`` section and on the line labeled 
``stabilizer/horizontal/port``.  The locations of the aerodynamic nodes (black nodes in figure above) 
along the aerodynamic reference line (aerodynamic center) are specified in the body-fixed (x,y,z) 
coordinate system relative to its origin, with y monotonically decreasing (negative values). The airfoil at each node 
is assumed to be in the x-z plane, and—along with the nodal locations—the positive aerodynamic twist (``PHSTwist``) 
is specified about positive y, and the chordlength (``PHSChord``), airfoil table ID (``PHSAFID``), 
and elevator ID (``PHSElvID``) are specified. A zero-degree twist means negative x points toward the 
trailing edge and negative z points toward the suction side of the airfoil. Calculations for the 
lifting line vortex method take place at the midpoints between these nodes; instead of interpolating
airfoil data, the airfoil and elevator IDs at each midpoint is taken to be the airfoil and elevator IDs 
of the corresponding node with higher (less negative) y.


Pylon Properties
~~~~~~~~~~~~~~~~

Specify the number of aerodynamic nodes, per pylon,  with the
``NumPylNds`` parameter, followed by ``NumPylNds`` times 2 (starboard, port) times ``NumPylons`` lines of nodal data (plus two table header lines).
``PylX``, ``PlyY``, and ``PlyZ`` determine the location of a node within the kite coordinate system,
and are relative to the given pylon's reference point (also given in the kite coordinate system).
In the stand-alone KiteAeroDyn driver case, this point is specified
in the ENERGY KITE REFERENCE CONFIGURATION section of the driver input file.  In a KiteFAST-coupled simulation,
this point is defined in the preprocessor input file under the ``keypoints`` section and on the line labeled 
``pylon/starboard/PyID`` and ``pylon/port/PyID``, where ``PyID`` varies from 1 to ``NumPylons``. The node list
must be structured such that all starboard pylon nodes appear first, starting with the inner-most pylon and 
ending with the outer-most pylon. 
Then the port pylon nodes are listed, again starting with the inner-most pylon and ending with the outer-most pylon.
The locations of the aerodynamic nodes (black nodes in figure above) along the aerodynamic reference line 
(aerodynamic center) are specified in the body-fixed (x,y,z) coordinate system relative to its origin, with z monotonically 
increasing (from possibly negative to positive values). The airfoil at each node is assumed to be in the x-y plane, 
and—along with the nodal locations—the positive aerodynamic twist (``PylTwist``) is specified about positive z, 
and the chordlength (``PylChord``) and airfoil table ID (``PylAFID``) are specified. A zero-degree twist means negative 
x points toward the trailing edge and positive y points toward the suction side of the airfoil. 
Calculations for the lifting line vortex method take place at the midpoints between these nodes; 
instead of interpolating airfoil data, the airfoil ID at each midpoint is taken to be the 
airfoil ID of the corresponding node with lower z.


Rotor Properties
~~~~~~~~~~~~~~~~
The rotor properties are defined by giving the rotor's radius (``RtrRad``) in meters, and the filename for the rotor's 
performance data (``RtrInFile``) as a quoted string.  This data is provided in table form with one line for each rotor.
The node list must be structured such that all starboard pylon rotors appear first, starting with the inner-most pylon and 
ending with the outer-most pylon. Then the port pylon rotors are listed, again starting with the inner-most pylon 
and ending with the outer-most pylon. Each pylon must contain a line for the top rotor followed by the bottom rotor.
The table will contain a total of four times ``NumPylons`` lines.

Output Options
~~~~~~~~~~~~~~

Specifying ``SumPrint`` to TRUE causes KiteAeroDyn to generate a summary
file with name ``OutFileRoot**.KAD.sum*. ``OutFileRoot`` is either
specified in the I/O SETTINGS section of the driver input file when
running KiteAeroDyn standalone, or by the KiteFAST program when running a
coupled simulation.  
If ``OutSwtch`` is set to 1, outputs related to the vortex step method (VSM) calculations are sent to a file 
with the name, ``OutFileRoot.VSM.out``, and the user-requested output channels specified in the ``OutList``, described below,
are sent to a file with the name, ``OutFileRoot.KAD.out``.  If ``OutSwtch`` is set to 2, and the user is running an 
KiteFAST-driven simulation, the user-requested KiteAeroDyn outputs are sent to a single file with the name 
``OutFileRoot.out`` which also contains outputs of other KiteFAST modules.  If ``OutSwtch`` is set to 3, both file outputs occur,
in the case of an KiteFAST-driven simulation.  The ``OutFmt`` parameter controls the formatting for the output data.  
KiteAeroDyn currently does not check the validity of these format strings.  They need to be valid Fortran format strings.  
An example valid format string is: ``"ES11.4"``.    
   

KiteAeroDyn can output aerodynamic and kinematic quantities at up to nine nodes for each kite component.
``NFusOuts`` specifies the number of fuselage nodes that output is requested for (0 to 9) and ``FusOutNd`` 
on the next line is a list ``NFusOuts`` long of node numbers between 1 and ``NumFusNds`` (corresponding to 
a row number in the fuselage node table, separated by any combination of commas, semicolons, spaces, and/or tabs. 
``NSWnOuts`` specifies the number of starboard wing nodes that output is requested for (0 to 9) and ``SWnOutNd`` 
on the next line is a list ``NSWnOuts`` long of node numbers between 1 and ``NumSWnNds`` (corresponding to 
a row number in the starboard wing node table, separated by any combination of commas, semicolons, spaces, and/or tabs. 
``NPWnOuts`` specifies the number of port wing nodes that output is requested for (0 to 9) and ``PWnOutNd`` 
on the next line is a list ``NPWnOuts`` long of node numbers between 1 and ``NumPWnNds`` (corresponding to 
a row number in the port wing node table, separated by any combination of commas, semicolons, spaces, and/or tabs. 
``NVSOuts`` specifies the number of vertical stabilizer nodes that output is requested for (0 to 9) and ``VSOutNd`` 
on the next line is a list ``NVSOuts`` long of node numbers between 1 and ``NumVSNds`` (corresponding to 
a row number in the vertical stabilizer node table, separated by any combination of commas, semicolons, spaces, and/or tabs. 
``NSHSOuts`` specifies the number of starboard horizontal stabilizer nodes that output is requested for (0 to 9) and ``SHSOutNd`` 
on the next line is a list ``NSHSOuts`` long of node numbers between 1 and ``NumSHSNds`` (corresponding to 
a row number in the starboard horizontal stabilizer node table, separated by any combination of commas, semicolons, spaces, and/or tabs. 
``NPHSOuts`` specifies the number of port horizontal stabilizer nodes that output is requested for (0 to 9) and ``PHSOutNd`` 
on the next line is a list ``NPHSOuts`` long of node numbers between 1 and ``NumPHSNds`` (corresponding to 
a row number in the port horizontal stabilizer node table, separated by any combination of commas, semicolons, spaces, and/or tabs. 
``NPylOuts`` specifies the number of fuselage nodes that output is requested for (0 to 9) and ``PylOutNd`` 
on the next line is a list ``NPylOuts`` long of node numbers between 1 and ``NumPylNds`` (corresponding to 
a row number in the fuselage node table, separated by any combination of commas, semicolons, spaces, and/or tabs. 
Outputs for a given pylon use the same output node numbers listed via ``NPylOuts``. 

The ``OutList`` section controls output quantities generated by
KiteAeroDyn. Enter one or more lines containing quoted strings that in turn
contain one or more output parameter names. Separate output parameter
names by any combination of commas, semicolons, spaces, and/or tabs. If
you prefix a parameter name with a minus sign, “-”, underscore, “_”, or
the characters “m” or “M”, KiteAeroDyn will multiply the value for that
channel by –1 before writing the data. The parameters are written in the
order they are listed in the input file. KiteAeroDyn allows you to use
multiple lines so that you can break your list into meaningful groups
and so the lines can be shorter. You may enter comments after the
closing quote on any of the lines. Entering a line with the string “END”
at the beginning of the line or at the beginning of a quoted string
found at the beginning of the line will cause KiteAeroDyn to quit scanning
for more lines of channel names. Node-related quantities
are generated for the requested nodes identified through the various
``***OutNds`` lists above. If KiteAeroDyn encounters an
unknown/invalid channel name, it warns the users and will mark the
units of the suspect channel as ``Invalid``. Please refer to the Appendix for a
complete list of possible output parameters.

.. _airfoil_data_input_file:

Airfoil Data Input File
~~~~~~~~~~~~~~~~~~~~~~~

The airfoil data input files themselves (one for each airfoil) include
tables containing coefficients of lift force, drag force, and pitching
moment versus AoA, as well as unsteady airfoil aerodynamic model parameters. In these files, any
line whose first non-blank character is an exclamation point (!) is
ignored (for inserting comment lines). The non-comment lines should
appear within the file in order, but comment lines may be intermixed as
desired for reading clarity. A sample airfoil data input file is given
:numref:`kad_appendix`.

``InterpOrd`` is the order the static airfoil data is interpolated
when KiteAeroDyn uses table look-up to find the lift-, drag-, and optional
pitching-moment, and minimum pressure coefficients as a function of AoA.
When ``InterpOrd`` is 1, linear interpolation is used; when
``InterpOrd`` is 3, the data will be interpolated with cubic splines;
if the keyword ``DEFAULT`` is entered in place of a numerical value,
``InterpOrd`` is set to 3.

``NonDimArea`` is the nondimensional airfoil area (normalized by the
local ``BlChord`` squared), but is currently unused by KiteAeroDyn.
``NumCoords`` is the number of points to define the exterior shape of
the airfoil, plus one point to define the aerodynamic center, and
determines the number of rows in the subsequent table; ``NumCoords``
must be exactly zero or greater than or equal to three. For each point,
the nondimensional *X* and *Y* coordinates are specified in the table,
``X_Coord`` and ``Y_Coord`` (normalized by the local
``BlChord``). The first point must always locate the aerodynamic
center (reference point for the airfoil lift and drag forces, likely not
on the surface of the airfoil); the remaining points should define the
exterior shape of the airfoil. The airfoil shape is currently unused by
KiteAeroDyn, but when KiteAeroDyn is coupled to MBDyn, the airfoil shape will be
used by MBDyn for blade surface visualization when enabled.

Specify the number of Reynolds number- or aerodynamic-control
setting-dependent tables of data for the given airfoil via the
``NumTabs`` setting. The remaining parameters in the
airfoil data input files are entered separately for each table.

``Re`` and ``UserProp`` are the Reynolds number (in millions) and
aerodynamic-control (or user property) setting for the included table.
These values are used only when the ``AFTabMod`` parameter in the 
primary KiteAeroDyn input file is set to use 2D interpolation based on 
``Re`` or ``UserProp``. If 1D interpolation (based only on angle of attack)
is used, only the first table in the file will be used.

Set ``InclUAdata`` to TRUE if you are including the 32 unsteady airfoil aerodynamics model
parameters.  Because unsteady airfoil aerodynamics are not accounted for in KiteAeroDyn, these parameters are not further discussed here.  See the OpenFAST AeroDyn module documentation for more information (AeroDyn is distinct from KiteAeroDyn).

``NumAlf`` is the number of distinct AoA entries and determines the
number of rows in the subsequent table of static airfoil coefficients;
``NumAlf`` must be greater than or equal to one (``NumAlf = 1``
implies constant coefficients, regardless of the AoA). 

KiteAeroDyn will interpolate on AoA using the data provided via linear interpolation or via cubic
splines, depending on the setting of input ``InterpOrd`` above. 
If ``AFTabMod`` is set to ``1``, only the first airfoil table in each file
will be used. If ``AFTabMod`` is set to ``2``, KiteAeroDyn will find the
airfoil table that bounds the computed Reynolds number, and linearly interpolate
between the tables, using the logarithm of the Reynolds numbers. If ``AFTabMod`` is set to ``3``, KiteAeroDyn will find the airfoil table that bounds the computed control setting, and linearly interpolate
between the tables, using the control settings.

For each AoA, you must set the AoA (in degrees), ``alpha``, the lift-force
coefficient, ``Coefs``\ (:,1), the drag-force coefficient,
``Coefs(:,2)``, and optionally the pitching-moment coefficient,
``Coefs(:,3)``, but the column order depends on the settings of
``InCol_Alfa``, ``InCol_Cl``, ``InCol_Cd``, and ``InCol_Cm`` in the AIRFOIL INFORMATION section of the KiteAeroDyn
primary input file. AoA must be entered in monotonically increasing
order—from lowest to highest AoA—and the first row should be for AoA =
–180 and the last should be for AoA = +180 (unless ``NumAlf = 1``, in
which case AoA is unused). 

.. _rotor_data_input_file:

Rotor Data Input File
~~~~~~~~~~~~~~~~~~~~~

The rotor data input file contains the rotor performance coefficient data as a 
function of rotor speed, inflow velocity, inflow skew angle, and collective-rotor blade pitch.  
Separate files are used for each unique rotor. A sample rotor data input file and the local
rotor coordinate system is given in :numref:`kad_appendix`. The actuator disk is defined with local x normal to the disk (pointed forward, in the primary direction of flight) and positive rotation (Omega) about positive local x. The Vrel vector is always in the local x-y plane, and unless Skew is 0˚ or 180˚, the Vrel vector has a component along negative local y. Local z follows the right-hand rule. (That is, the local coordinate system rotates with the Vrel vector.)

The input file begins with two lines of header information which is for
your use, but is not used by the software.

``NumOmega`` specifies the number of rotor rotational speeds, 
``NumVrel`` specifies the number of relative inflow velocities, 
``NumSkew`` specifies the number of skew angles, and 
``NumPitch`` - specifies the number of pitch angles. Therefore, the data table 
will contain ``NumOmega`` X ``NumVrel`` X ``NumSkew`` X ``NumPitch`` lines (plus
two table header lines). Each of these values must be >= 2. 
The rotor table data contains 11 columns (in the following order, from left to right):

-  ``Omega`` specifies the rotor rotational velocity (rad/s);

-  ``Vrel`` specifies the inflow wind speed (m/s);

-  ``Skew`` specifies the skew angle (angle between local x and VRel vector, 
   positive angle about positive local z, in degrees and should be between 0 and 180 degrees (inclusive));

-  ``Pitch`` specifies the collective-rotor blade pitch angle (in degrees);

-  ``C_Fx`` specifies the thrust (x) force coefficient for the given operating conditions, with normalization factor, :math: `\rho D^4 (omega/(2 \pi))^2` ;

-  ``C_Fy`` specifies the transverse (y) force coefficient for the given operating conditions, with normalization factor, :math: `\rho D^4 (omega/(2 \pi))^2` ;

-  ``C_Fz`` specifies the transverse (z) force coefficient for the given operating conditions, with normalization factor, :math: `\rho D^4 (omega/(2 \pi))^2` ;

-  ``C_Mx`` specifies the torque (x) coefficient for the given operating conditions, with normalization factor, :math: `\rho D^5 (omega/(2 \pi))^2` ;

-  ``C_My`` specifies the transverse (y) moment coefficient for the given operating conditions
, with normalization factor, :math: `\rho D^5 (omega/(2 \pi))^2` ;


-  ``C_Mz`` specifies the transverse (z) moment coefficient for the given operating conditions
, with normalization factor, :math: `\rho D^5 (omega/(2 \pi))^2` ;


-  ``C_P`` specifies the power coefficient for the given operating conditions
, with normalization factor, :math: `\rho D^5 (omega/(2 \pi))^3` ;


The table must be constructed such that the ``Omega`` dependent variable varies most frequently, 
followed by ``Vrel``, and so forth, through the ``Pitch`` dependent variable which varies the least frequently.
