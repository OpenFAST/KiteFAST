.. _kad_intro:

Introduction
============

KiteAeroDyn is a time-domain aerodynamics module that is coupled in the MBDyn multi-body dynamics tool to enable aero-elastic simulation of kite systems. 
KiteAeroDyn can also be driven as a standalone code to compute kite aerodynamic response uncoupled from MBDyn.   

KiteAeroDyn calculates aerodynamic loads on the various aerodynamic surfaces of the kite system.
Aerodynamic calculations within KiteAeroDyn are based on the principles of
actuator lines, where the three-dimensional (3D) flow around a body is
approximated by local two-dimensional (2D) flow at cross sections, and
the distributed pressure and shear stresses are approximated by lift
forces, drag forces, and pitching moments lumped at a node in a 2D cross
section. Analysis nodes are distributed along the length of each blade
and tower, the 2D forces and moment at each node are computed as
distributed loads per unit length, and the total 3D aerodynamic loads
are found by integrating the 2D distributed loads along the length. When
KiteAeroDyn is coupled to MBDyn, the blade and tower analysis node
discretization may be independent from the discretization of the nodes
in the structural modules. The actuator line approximations restrict the
validity of the model to slender structures and 3D behavior is either
neglected, captured through corrections inherent in the model (e.g.,
tip-loss, hub-loss, or skewed-wake corrections), or captured in the
input data (e.g., rotational augmentation corrections applied to airfoil
data).

KiteAeroDyn assumes the kite geometry consists of a fuselage, starboard and port 
wings, a vertical stabilizer, starboard and port stabilizers, and one or two-
pylons located on a wing surface. Each kite component (or surface) is assummed to
have a spanwise direction. The 2D cross-sections where the aerodynamic analysis take place 
may follow the out-of-plane curvature, but in-plane sweep is assumed to be accomplished
by shearing, rather than rotation of the 2D cross section.

When KiteAeroDyn is coupled to MBDyn, KiteAeroDyn receives the instantaneous
(possibly displaced/deflected) structural position, orientation, and
velocities of analysis nodes on the kite. As with
curvature and sweep, the 2D cross sections where the blade aerodynamic
analysis takes place will follow the out-of-plane deflection, but
in-plane deflection is assumed to be accomplished by shearing, rather
than rotation of the 2D cross section. KiteAeroDyn also receives the local
freestream (undisturbed) fluid velocities at the analysis nodes.
(Fluid and structural calculations take place outside of the KiteAeroDyn
module and are passed as inputs to KiteAeroDyn by the driver code.) The
fluid and structural motions are provided at each coupling time step and
then KiteAeroDyn computes the aerodynamic loads on the analysis nodes
and returns them back to MBDyn as part of the aero-elastic calculation.
In standalone mode, the inputs to KiteAeroDyn are prescribed by a simple
driver code, without aero-elastic coupling.



[Add brief summary of VSM method here]

The blade airfoil aerodynamics can be steady or unsteady, except in the
case that a cavitation check is requested for MHK, in which case only
steady aerodynamics are supported. In the steady model, the supplied
static airfoil data — including the lift force, drag force, and optional
pitching moment and minimum pressure coefficients versus angle of attack
(AoA) — are used directly to calculate nodal loads. The
`AirfoilPrep <https://nwtc.nrel.gov/AirFoilPrep>`__ preprocessor can be
used to generate the needed static airfoil data based on uncorrected 2D
data (based, e.g., on airfoil tests in a wind tunnel or
`XFoil <http://web.mit.edu/drela/Public/web/xfoil/>`__), including
features to blend data between different airfoils, apply 3D rotational
augmentation, and extrapolate to high AoA. The unsteady airfoil
aerodynamic (UA) models account for flow hysteresis, including unsteady
attached flow, trailing-edge flow separation, dynamic stall, and flow
reattachment. The UA models can be considered as 2D dynamic corrections
to the static airfoil response as a result of time-varying inflow
velocities and angles. Three semi-empirical UA models are available: the
original theoretical developments of Beddoes-Leishman (B-L), extensions
to the B-L developed by González, and extensions to the B-L model
developed by Minnema/Pierce. **While all of the UA models are documented
in this manual, the original B-L model is not yet functional. Testing
has shown that the González and Minnema/Pierce models produce reasonable
hysteresis of the normal force, tangential force, and pitching-moment
coefficients if the UA model parameters are set appropriately for a
given airfoil, Reynolds number, and/or Mach number. However, the
results will differ a bit from earlier versions of KiteAeroDyn, (which was
based on the Minnema/Pierce extensions to B-L) even if the default UA
model parameters are used, due to differences in the UA model logic
between the versions. We recommend that users run test cases with
uniform wind inflow and fixed yaw error (e.g., through the standalone
KiteAeroDyn driver) to examine the accuracy of the normal force, tangential
force, and pitching-moment coefficient hysteresis and to adjust the UA
model parameters appropriately.** The airfoil-, Reynolds-, and
Mach-dependent parameters of the UA models may be derived from the
static airfoil data. These UA models are valid for small to moderate AoA
under normal rotor operation; the steady model is more appropriate under
parked or idling conditions. The static airfoil data is always used in
the BEM iteration; when UA is enabled, it is applied after the BEM
iteration and after the skewed-wake correction. The UA models are not
set up to support linearization, so, UA must be disabled during
linearization analyses with KiteAeroDyn coupled to MBDyn. The interpolation
of airfoil data based on Reynolds number or aerodynamic-control setting
(e.g., flaps) is not yet available in KiteAeroDyn v15 and newer.

The influence of the tower on the fluid flow local to the blade is based
on a potential-flow and/or a tower-shadow model. The potential-flow
model uses the analytical potential-flow solution for flow around a
cylinder to model the tower dam effect on upwind rotors. In this model,
the freestream (undisturbed) flow at each blade node is disturbed based
on the location of the blade node relative to the tower and the tower
diameter, including lower velocities upstream and downstream of the
tower, higher velocities to the left and right of the tower, and
cross-stream flow. The Bak correction can optionally be included in the
potential-flow model, which augments the tower upstream disturbance and
improves the tower wake for downwind rotors based on the tower drag
coefficient. The tower shadow model can also be enabled to account for
the tower wake deficit on downwind rotors. This model includes an axial
flow deficit on the freestream fluid at each blade node dependent on the
location of the blade node relative to the tower and the tower diameter
and drag coefficient, based on the work of Powles. Both tower-influence
models are quasi-steady models, in that the disturbance is applied
directly to the freestream fluid at the blade nodes without dynamics,
and are applied within the BEM iteration.


The primary KiteAeroDyn input file defines modeling options, environmental
conditions (except freestream flow), airfoils, rotor properties, 
as well as output file specifications.
Airfoil data properties are read from dedicated inputs files (one for
each airfoil) and include coefficients of lift force, drag force, and
optional pitching moment and minimum pressure versus AoA, as well as UA
model parameters. Blade nodal discretization, geometry, twist, chord, and
airfoil identifier are likewise read from separate input files (one for
each blade).

:numref:`kad_input` describes the KiteAeroDyn input files. 
:numref:`kad_output` discusses the
output files generated by KiteAeroDyn; these include an echo file, summary
file, and the results file. 
:numref:`kad_modeling` provides modeling guidance when
using KiteAeroDyn. 
Example input files are included in :numref:`kad_input_files`. A summary of
available output channels are found :numref:`kad_output_channels`. 
