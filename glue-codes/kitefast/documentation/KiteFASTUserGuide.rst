KiteFAST User Guide
===================
This document outlines the workflow for running the KiteFAST simulation
software. The high level workflow requires the steps listed below.

1. Create input files for the following software modules:

- KiteMBDyn Preprocessor
- KiteAeroDyn
- InflowWind (Optional)
- MoorDyn (Optional)

2. Generate the MBDyn input files with the KiteMBDyn Preprocessor
3. Run the simulation software
4. Postprocess results


1. Input files
~~~~~~~~~~~~~~
Each component of KiteFAST requires a unique input file. The input files
are described in detail in the corresponding documentation directory
for each module.

2. Generate MBDyn input files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The KiteMBDyn Preprocessor is used to simplify the generation of the complex set
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

3. Run the simulation
~~~~~~~~~~~~~~~~~~~~~
The simulation is driven by the MBDyn executable. To run the simulation,
simply execute the following command.

.. code-block:: bash

    mbdyn KiteMain.mbd

MBDyn output will display in the terminal including the initial state of the model
and connectivity information for the nodes and beams. Eventually, the output will
stop but the program will continue to execute. When it finishes, the terminal prompt
will return and a series of text files with extensions
such as `.out `, `.ine `, `.jnt `, `.log `, `.mov `, `.act `, `.frc `,
`.sum `, `.bylog `, `.ech` will exist in the case directory.

4. Postprocess results
~~~~~~~~~~~~~~~~~~~~~~

TODO


Troubleshooting
===============

TODO: Guidance on things to tweak if issues
- timestep
- initial conditions
- solver settings
