
KiteFAST Documentation
======================
Sections:
- Repository map
- Module documentation
- Test cases

The KiteFAST simulation software is made up of multiple packages
based on the modularization framework of OpenFAST:

- MBDyn: multibody dynamics software from Politecnico di Milano that
  drives the simulation and calculates the structural response in the system
- KiteMBDyn preprocessor: a utility for generating the kite-specific MBDyn
  input files from a single YAML file
- KiteFASTMBD: interface between MBDyn and the modules of KiteFAST
- KiteAeroDyn Driver: standalone driver for running KiteAeroDyn uncoupled
  from OpenFAST
- KiteAeroDyn: kite-specific aerodynamics module
- InflowWind: wind-inflow module
- MoorDyn: tether dynamics module
- Controller: kite controller module


Repository map
~~~~~~~~~~~~~~
The repository is called `sandbox` and contains all of the source code
and installation scripts required to compile and run kite simulations.

The primary branches for active development are `master` and `dev`. Generally,
`dev` contains the latest work and is merged into `master` after testing
is completed.

All paths in the documentation assume that the main directory is
called `sandbox`. In addition to other general OpenFAST related content,
this directory should have the following structure:

sandbox
├── glue-codes
│   └── kitefast
│       ├── documentation
│       │   ├── KiteAeroDyn
│       │   │   ├── KiteAeroDyn_Plan.pdf
│       │   │   ├── VSM_theory.pdf
│       │   │   └── user_guide
│       │   ├── KiteFASTMBD_Plan.pdf
│       │   ├── KiteMBDynPreprocessor
│       │   │   ├── MBDyn_Preprocessor_Theory.pdf
│       │   │   └── user_guide
│       │   ├── MBDyn
│       │   │   └── mbdyn-input-1.7.3.pdf
│       │   └── OUTLINE.rst
│       ├── kitefast_helpers.sh
│       ├── kitefast_install.sh
│       ├── kitefast_uninstall.sh
│       ├── kitefast_update.sh
│       ├── module-kitefastmbd
│       │   ├── Makefile.inc
│       │   ├── module-kitefastmbd.cc
│       │   └── module-kitefastmbd.h
│       ├── preprocessor
│       │   ├── docs
│       │   └── src
│       └── test_cases
│           ├── archived_test_cases
│           ├── kiteaerodyn
│           ├── kiteinflow
│           ├── kitemooring
│           ├── m600
│           ├── m600_eigenanalysis
│           ├── m600_prescribed_circle
│           ├── m600_prescribed_circle_STIFF
│           ├── m600_salf
│           ├── scripts
│           └── two_element_beam
└── modules
    ├── inflowwind
    │   └── src
    ├── kiteaerodyn
    │   └── src
    ├── kitefast-controller
    │   ├── analysis
    │   └── src
    ├── kitefast-library
    │   └── src
    ├── moordyn
    │   └── src
    ├── turbsim
    │   └── src
    └── vsm
        └── src

sandbox/glue-codes/kitefast
---------------------------
This directory contains the documentation, automated installation scripts,
MBDyn interface source code, preprocessor source code, and a collection of
test and example cases.

sandbox/modules
---------------
This directory contains the physics modules which make up OpenFAST. The
modules used by KiteFAST are InflowWind, KiteAeroDyn, kitefast-controller,
kitefast-library, VSM, and MoorDyn.


Module documentation
~~~~~~~~~~~~~~~~~~~~
The documentation is organized by the various physics modules that
make up KiteFAST, including MBDyn and the interfaces between. All
documentation is located at
`sandbox/sandbox/glue-codes/kitefast/documentation/`.

A high-level implementation plan of KiteFASTMBD, including the interfaces
to the physics modules of KiteFAST, the coupling with MBDyn, and the outputs
available to the user not otherwise set the module level is located at
`sandbox/sandbox/glue-codes/kitefast/documentation/KiteFASTMBD_Plan.pdf`.

The general OpenFAST documentation is available at
https://openfast.readthedocs.io/. The FAST Developer's Handbook is also
available at https://nwtc.nrel.gov/system/files/ProgrammingHandbook_Mod20130326.pdf.

KiteAeroDyn
-----------
The implementation plan for KiteAeroDyn, which includes a description of inputs
and outputs, is found at `KiteAeroDyn/KiteAeroDyn_Plan.pdf`. Additional theory
documentation for the VSM module is found at `KiteAeroDyn/VSM_Theory.pdf`. User
documentation is available at `KiteAeroDyn/user_guide`.

Contact Jason Jonkman <mailto:jason.jonkman@nrel.gov> and
Greg Hayman <mailto:hayman.gregory@gmail.com>.

KiteMBDyn Preprocessor
----------------------
The preprocessor theory documentation is available at
`KiteMBDynPreprocessor/MBDyn_Preprocessor_Theory.pdf`. This describes the
mechanisms by which the simplified model described in the YAML file is
converted to a full MBDyn model. User documentation is available at
`KiteMBDynPreprocessor/user_guide/USER_GUIDE.rst`.

Contact Rick Damiani <mailto:rick.damiani@nrel.gov> and Rafael Mudafort
<mailto:rafael.mudafort@nrel.gov>.

MBDyn
-----
https://www.mbdyn.org
This is a third-party software developed at University Politecnico di Milano.
In KiteFAST, MBDyn is used for the structural dynamics (other than the tether)
and as the driver to move the time-domain solution forward.

Its documentation can be found at `MBDyn/mbdyn-input-1.7.3.pdf` with
further documentation available at https://www.mbdyn.org/?Documentation.

Contact Pierangelo Masarati <mailto:pierangelo.masarati@polimi.it>.

InflowWind
----------
The general InflowWind documentation is available at
https://nwtc.nrel.gov/InflowWind/.

MoorDyn
-------
The general MoorDyn documentation is available at
https://nwtc.nrel.gov/MoorDyn/.


Test cases
~~~~~~~~~~
Various test cases were created throughout the development process with a
varying degree of complexity. All test cases are located at
`sandbox/sandbox/glue-codes/kitefast/test_cases`.

Unless otherwise noted, all cases share common input files which are located at

test_cases
├── kiteaerodyn
├── kiteinflow
└── kitemooring

The included tests cases are listed below starting with the simplex and ending
with the most complex. A short description of each follows.

test_cases
├── two_element_beam
├── m600_prescribed_circle
├── m600_prescribed_circle_STIFF
├── m600_salf
├── m600_eigenanalysis
└── m600

two_element_beam
----------------
This is a test case for the KiteMBDyn Preprocesser only. It consists of a few
simple "components" made of two or three beam-elements. This has primarily
been used to understand the manner in which the mass and inertias are
distributed and how MBDyn reacts.

This is a good case to understand the mechanics of the preprocessor and the
structure of the MBDyn input files, in general. It has elements and
nomenclature in common with a kite model, but it is a more generalized case.
While it does not run the KiteFASTMBD interface, it does generate files
which can be run with MBDyn alone.

m600_prescribed_circle
----------------------
This test case contains the geometry of a simplified m600 kite. By default,
KiteAeroDyn, InflowWind, and MoorDyn are enabled, but the controller is
disabled. The initial conditions are such that simulation begins with the kite
already in its crosswind loop.

This case specifies the position of the kite as a function of time. As such, it
is NOT free flying. This case is a good test for any modifications of a
component of the system as the aeroelastics are actually calculated but the
result is not fully coupled in the response.

The MBDyn input files are included directly in the repository and no
preprocessor input files is included. This case was created manually so that
the position could be prescribed to MBDyn. To modify this case, start with
`KiteMain.mbd`. The position as a function of time is described in
`position.csv`.

m600_prescribed_circle_STIFF
----------------------------
This test case is the same as `m600_prescribed_circle` with the addition of
joints from the end of each component that rigidly connect it to the closest
wing root node.

m600_salf
---------
This test case contains the geometry of a simplified m600 kite. By default,
KiteAeroDyn, InflowWind, and the controller are enabled, but MoorDyn
is disabled (meaning there is no tether). The initial conditions
are such that the kite begins its flight in a straight and level orientation
(SALF = straight and level flight).

The preprocessor input file is included and should be used to generate the
mbdyn case files.

m600_eigenanalysis
------------------
This test case contains the geometry of a simplified m600 kite. Rather than
simulating a time marching flight, this case performs an Eigenanalysis of the
structure in the absence of aerodynamics, the tether, and control actions
(all modules are disabled).

NOTE: Configuring MBDyn with `--enable-netcdf --with-lapack --enable-eig`
is required to use the eigen analysis features of this case.

The MBDyn input files are included directly in the repository and no
preprocessor input files is included. This case was created manually so that
the eigen analysis could be correcly configured. To modify this case, start
with `KiteMain.mbd`.

NOTE: dummy nodes cannot be included in the `KiteMain.mbd`.  These will cause
parsing issues when `BlenDyn` is used to visualize modes with `Blender`.

m600
----
**This is the main demonstrator for the entire simulation.**

This test case contains the geometry of a simplified m600 kite. By default, all
physics modules are enabled. The initial conditions are such that simulation
begins with the kite already in its crosswind loop.

The KiteMBDyn Preprocessor input file is included and should be used to
generate the MBDyn case files.
