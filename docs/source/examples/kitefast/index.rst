.. _kitefast_examples:

KiteFAST Example Cases
======================
Various test cases were created throughout the development process with a
varying degree of complexity. All test cases are located at
`sandbox/glue-codes/kitefast/test_cases`.

Unless otherwise noted, all cases share common input files which are located at

.. code-block:: bash

    test_cases
    ├── kiteaerodyn
    ├── kiteinflow
    └── kitemooring

The included tests cases are listed below starting with the simplex and ending
with the most complex. A short description of each follows.

.. code-block:: bash

    test_cases
    ├── two_element_beam
    ├── m600_prescribed_circle
    ├── m600_prescribed_circle_STIFF
    ├── m600_salf
    ├── m600_eigenanalysis
    └── m600

two_element_beam
~~~~~~~~~~~~~~~~
This is a test case for the KiteMBDyn Preprocesser only. It consists of a few
simple "components" made of two or three beam-elements. This has primarily
been used to understand the manner in which the mass and inertias are
distributed and how MBDyn reacts.

This is a good case to understand the mechanics of the preprocessor and the
structure of the MBDyn input files, in general. It has elements and
nomenclature in common with a kite model, but it is a more generalized case.
While it does not run the KiteFASTMBD interface, it does generate files
which can be run with MBDyn alone.

.. _m600_prescribed_circle:

m600_prescribed_circle
~~~~~~~~~~~~~~~~~~~~~~
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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
This test case is the same as :ref:`m600_prescribed_circle` with the addition of
joints from the end of each component that rigidly connect it to the closest
wing root node.

m600_salf
~~~~~~~~~
This test case contains the geometry of a simplified m600 kite. By default,
KiteAeroDyn, InflowWind, and the controller are enabled, but MoorDyn
is disabled (meaning there is no tether). The initial conditions
are such that the kite begins its flight in a straight and level orientation
(SALF = straight and level flight).

The preprocessor input file is included and should be used to generate the
mbdyn case files.

m600_eigenanalysis
~~~~~~~~~~~~~~~~~~
This test case contains the geometry of a simplified m600 kite. Rather than
simulating a time marching flight, this case performs an Eigenanalysis of the
structure in the absence of aerodynamics, the tether, and control actions
(all modules are disabled).

NOTE: Configuring MBDyn with `--enable-netcdf --with-lapack --enable-eig`
is required to use the eigen analysis features of this case.

The preprocessor input file is included with this case.  This includes the
directive to setup the eigen analysis.  To create the case, run the
preprocessor as per usual to create the MBDyn input file set. When the
eigen analysis is performed, the begining of the  KiteMain.out file will
contain the results of the eigen analysis with the real, imaginary, damping,
and frequency results for each of the modes.  This may include several
hundred modes. Visualization of the modes can be done with BlenDyn in
Blender.

NOTE: dummy nodes cannot be included in the `KiteMain.mbd`.  These will cause
parsing issues when `BlenDyn` is used to visualize modes with `Blender`.

m600
~~~~
**This is the main demonstrator for the entire simulation.**

This test case contains the geometry of a simplified m600 kite. By default, all
physics modules are enabled. The initial conditions are such that simulation
begins with the kite already in its crosswind loop.

The KiteMBDyn Preprocessor input file is included and should be used to
generate the MBDyn case files.
