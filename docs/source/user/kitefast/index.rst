.. _kitefast:

KiteFAST User Guide
===================
The KiteFAST simulation software is made up of multiple packages
based on the `modularization framework <https://nwtc.nrel.gov/system/files/ProgrammingHandbook_Mod20130326.pdf>`_
of OpenFAST. A brief description and links to module-specific
documentation is provided below.

- :ref:`KiteMBDyn Preprocessor<kitembdynpreprocessor>`: a utility for
  generating the kite-specific MBDyn input files from a single YAML file
- :download:`KiteFASTMBD <KiteFASTMBD_Plan.pdf>`: interface between
  MBDyn and the modules of KiteFAST; high-level implementation plan,
  including the interfaces to the physics modules of KiteFAST, the
  coupling with MBDyn, and the outputs available to the user not
  otherwise set at the module level is available for download
- KiteAeroDyn Driver: standalone driver for running KiteAeroDyn uncoupled
  from OpenFAST
- :ref:`KiteAeroDyn<kiteaerodyn>`: kite-specific aerodynamics module
- `InflowWind <https://nwtc.nrel.gov/InflowWind/>`_: wind-inflow module
- `MoorDyn <https://nwtc.nrel.gov/MoorDyn/>`_: tether dynamics module
- Controller: kite controller module
- :ref:`mbdyn`: multibody dynamics software used for the
  structural dynamics (except tether), as the driver to move
  the time-domain solution forward, and the interface for forces and
  kinematics between KiteFAST and MBDyn

A series of test and example cases are included in the repository.
See :ref:`kitefast_examples` for more information.

Repository map
~~~~~~~~~~~~~~
The repository is called `sandbox` and contains all of the source code
and installation scripts required to compile and run kite simulations.

The primary branches for active development are `master` and `dev`. Generally,
`dev` contains the latest work and is merged into `master` after testing
is completed. See :ref:`github_workflow` for more information.

All paths in the documentation assume that the main directory is
called `sandbox`. In addition to other general OpenFAST related content,
this directory should have the following structure:

.. code-block:: bash

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
This directory contains documentation source, automated installation scripts,
MBDyn interface source code, preprocessor source code, and a collection of
test and example cases.

sandbox/modules
---------------
This directory contains the physics modules which make up OpenFAST. The
modules used by KiteFAST are InflowWind, KiteAeroDyn, kitefast-controller,
kitefast-library, VSM, and MoorDyn.

General workflow
~~~~~~~~~~~~~~~~
The high level workflow requires the steps listed below.

1. Create input files for the following software modules:

- KiteMBDyn Preprocessor
- KiteAeroDyn
- InflowWind (Optional)
- MoorDyn (Optional)

2. Generate the MBDyn input files with the KiteMBDyn Preprocessor
3. Run the simulation software
4. Postprocess results

Input files
-----------
Each component of KiteFAST requires a unique input file. The input files
are described in detail in the documentation corresponding to each module.

Generate MBDyn input files
--------------------------
The :ref:`kitembdynpreprocessor` is used to simplify the generation of the complex set
of MBDyn input files. The command syntax for this tool is given below.

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

For example, after the input file is completed, run the following command
to generate the MBDyn input file deck in the same directory as the preprocessor
input file.

.. code-block:: bash

    $ python preprocess.py -i model_file.yml

This will generate a file called ``KiteMain.mbd`` and a series of supporting
files which describe the model.

Run the simulation
------------------
The simulation is driven by the MBDyn executable. To run the simulation,
simply execute the following command.

.. code-block:: bash

    mbdyn KiteMain.mbd

MBDyn output will display in the terminal including the initial state
of the model and connectivity information for the nodes and beams.
Eventually, the output will stop but the program will continue to
execute. When it finishes, the terminal prompt will return and a
series of text files with extensions such as `.out`, `.ine`,
`.jnt`, `.log`, `.mov`, `.act`, `.frc`, `.sum`, `.bylog`,
`.ech` will exist in the case directory.

Postprocess results
-------------------

TODO


Troubleshooting
~~~~~~~~~~~~~~~

TODO: Guidance on things to tweak if issues
- timestep
- initial conditions
- solver settings
