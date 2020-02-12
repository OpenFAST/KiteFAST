
.. _mbdyn:

MBDyn and User Defined Elements
===============================
MBDyn is a third-party multibody dynamics
software developed at Politecnico di Milano. In KiteFAST, MBDyn is used for the
structural dynamics (except tether) and as the driver to move
the time-domain solution forward. Its documentation can be downloaded
:download:`here <mbdyn-input-1.7.3.pdf>` with further documentation
available at https://www.mbdyn.org/?Documentation.

Contact `Pierangelo Masarati <mailto:pierangelo.masarati@polimi.it>`_ for any
questions specific to MBDyn.

.. _mbdyn_ude_dev_guide:

UDE: ModuleKiteFAST
~~~~~~~~~~~~~~~~~~~
MBDyn allows for custom modules, called User Defined Element (UDE), to interact
with its internal model and add user-defined loads during the calculation step.
The KiteFAST UDE is called `ModuleKiteFAST`. It consists of a C-header
file, `module-kitefastmbd.h`, and the corresponding C-source code,
`module-kitefastmbd.cc`.

When MBDyn is configured to load the UDE, it handles instantiating the
`ModuleKiteFAST` object during its own initialization routine, executing
the simulation steps, writing output, and finalizing data before returning.

The module source code file contains a constant for debugging at the top
that is commented by default.

.. code-block:: c

    // #define DEBUGUDE

To enable debugging output, simple uncomment this line and recompile.

All KiteFAST functions must supply an error status and message buffer. The
status is an integer and the message is a `char` array. These can be declared
and passed to the KiteFAST functions as demonstrated below.

.. code-block:: c

    doublereal t = Time.dGet();
    int error_status;
    char error_message[INTERFACE_STRING_LENGTH];
    KFAST_AfterPredict(&t, &error_status, error_message);
    printdebug("KFAST_AfterPredict error");
    printdebug("    status: " + std::to_string(error_status) + ";");
    printdebug("    message: " + std::string(error_message) + ";");
    if (error_status >= AbortErrLev)
    {
        printf("error status %d: %s\n", error_status, error_message);
        throw ErrGeneric(MBDYN_EXCEPT_ARGS);
    }

Initialization
--------------
The `ModuleKiteFAST` init function gathers model data from MBDyn
(which ultimately comes from the KiteMBDyn Preprocessor output) and
converts it into its own internal data structure. The module source
code contains many helper routines for parsing and manipulating data
in the initialization. At the end of initialization, the module calls
the KiteFAST initialization function `KFAST_Init`.

This is the first communication between MBDyn and KiteFAST.

Assemble Residual
-----------------
The `AssRes` function extracts the kinematics of the model from MBDyn and
passes them to KiteFAST through the `KFAST_AssRes` function. In return,
KiteFAST gives loads for every node. This function then populates the
`WorkVec`, an MBDyn construct to add loads to noads in the model.

This is the primary connection for passing motions and loads between
MBDyn and KiteFAST.

Output
------
A function called `Output` exists to provide a pipeline for MBDyn
outputs to be handled by KiteFAST in the typical OpenFAST-style
output file.

General development guidelines
------------------------------
All internal data structures in the UDE are stored as scalar values
or in a array using the C-STL `std::vector` class. The order of the
indices of the arrays `do` matter and are coupled with the indices
configured in the preprocessor. That is, beam nodes are sorted in
monotonically increasing order based on the primary axis of their
parent beam.

Keep in mind that some of the routines in the UDE are called several
times per time step. For any new memory allocated, a corresponding
delete-statement should exist. Otherwise, the memory footprint of
the module may grow quickly.

.. code-block:: c

    // allocate a new array
    doublereal *rotor_points = new doublereal[3 * n_rotor_points];

    // do something with the array
    // ...
    rotor_points[3 * i + 2] = xcurr[2];
    // ...

    // delete this array when finished with it
    delete[] rotor_points;
