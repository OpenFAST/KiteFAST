
KiteFAST-MBD Documentation
==========================
Sections:
- Repository map
- Theory documentation
- User documentation
- Example cases

The KiteFASTMBD simulation software is made up of multiple packages:

- MBDyn is the multibody dynamics software which drives the simulation and
  calculates the structural response in the system
- KiteFAST is the kite-specific physics modeling features in OpenFAST
- MBDyn preprocessor is a utility for generating the kite-specific MBDyn
  input files from a single YAML file

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
│   └── kitefast
│       ├── documentation
│       │   └── <text files and pdfs>
│       ├── kitefast_build.sh
│       ├── kitefast_remove.sh
│       ├── kitefast_update.sh
│       ├── module-kitefastmbd
│       │   └── <source code>
│       ├── preprocessor
│       │   └── <source code>
│       └── test_cases
│           └── <case directories>
├── module-local
│   ├── inflowwind
│   │   └── <source code>
│   ├── kiteaerodyn
│   │   └── <source code>
│   ├── kitefast-controller
│   │   └── <source code>
│   ├── kitefast-library
│   │   └── <source code>
│   └── vsm
│       └── <source code>
└── module-ext
    └── moordyn
        └── <source code>

sandbox/glue-codes/kitefast
---------------------------
This directory contains the documentation, automated installation scripts,
MBDyn interface source code, preprocessor source code, and a collection of
test and example cases.

sandbox/modules-local and sandbox/modules-ext
---------------------------------------------
This directory contains the physics modules which make up OpenFAST. The
KiteFAST-specific modules are inflowwind, kiteaerodyn, kitefast-controller,
kitefast-library, vsm, and moordyn.


Theory documentation
~~~~~~~~~~~~~~~~~~~~
The theory documentation is organized by the various physics modules which
make up KiteFAST, MBDyn, and the interface between. All theory documentation
is located at `sandbox/sandbox/glue-codes/kitefast/documentation/theory`.

A high-level implementation plan including details of the physics modules
added to OpenFAST, the coupling with MBDyn, and the outputs available to
the user is located at `theory/KiteFASTMBD_Plan.pdf`.

MBDyn
-----
https://www.mbdyn.org
This is a third-party software developed at University Politecnico di Milano.
Its documentation can be found at `documentation/mbdyn-input-1.7.3.pdf` with
further documentation available at https://www.mbdyn.org/?Documentation.

Contact Pierangelo Masarati <mailto:pierangelo.masarati@polimi.it>.

KiteFAST
--------
The general OpenFAST documentation is available at
https://openfast.readthedocs.io/.
Additional documentation for the VSM module is found at
`theory/VSM_Theory.pdf`. The implememtation plan for KiteAeroDyn which includes
a description of inputs and outputs is found at `theory/KiteAeroDyn_Plan.pdf`.

Contact Jason Jonkman <mailto:jason.jonkman@nrel.gov>.

MBDyn Preprocessor
------------------
The preprocessor theory documentation is available at
`theory/MBDyn_Preprocessor_Theory.pdf`. This describes the mechanisms by which
the simplified model described in the YAML file is converted to a full MBDyn
model.

Contact Rick Damiani <mailto:rick.damiani@nrel.gov>.


User documentation
~~~~~~~~~~~~~~~~~~
The user documentation provides guidance on inputs, outputs, and modeling
considerations of the entire system. All user documentation
is located at `sandbox/sandbox/glue-codes/kitefast/documentation/user_guides`.

The user documentation available are:
- general usage: user_guides/general_usage.rst
- MBDyn preprocesser: user_guides/mbdyn_preprocessor_usage.rst
- KiteAeroDyn: user_guides/kiteaerodyn_usage.rst


