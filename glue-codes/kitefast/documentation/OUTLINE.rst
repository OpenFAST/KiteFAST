
KiteFAST-MBD Documentation
==========================
Sections:
- Theory documentation
- Repository map
- Example/test cases

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

All paths in the documentation assume that the main directory is
called `sandbox`. In addition to other general OpenFAST related content,
this directory should have the following structure:

sandbox
├── glue-codes
│   └── kitefast
│       ├── documentation
│       │   └── <text files>
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
make up KiteFAST, MBDyn, and the interface between. An implementation
plan including details of the physics modules added to OpenFAST, the coupling
with MBDyn, and the outputs available to the user is located at
`sandbox/glue-codes/kitefast/documentation/KiteFASTMBD/KiteFASTMBD_Plan.pdf`.

MBDyn
-----
https://www.mbdyn.org
This is a third-party software developed at University Politecnico di Milano.
Its documentation can be found at
`sandbox/glue-codes/kitefast/documentation/mbdyn-input-1.7.3.pdf` with further
documentation available at https://www.mbdyn.org/?Documentation.

Contact Pierangelo Masarati <mailto:pierangelo.masarati@polimi.it>.

KiteFAST
--------
The general OpenFAST documentation is available at
https://openfast.readthedocs.io/. Additional documentation for the VSM module
is found at
`sandbox/glue-codes/kitefast/documentation/KiteAeroDyn/TheoryManual/VSM_Theory.pdf`.

Contact Jason Jonkman <mailto:jason.jonkman@nrel.gov>.

MBDyn Preprocessor
------------------
The preprocessor theory documentation is available at 
`sandbox/glue-codes/kitefast/documentation/MBDyn_Preprocessor/PrepRocessor_Theory.pdf`.

Contact Rick Damiani <mailto:rick.damiani@nrel.gov>.

