.. _kad_output:

Output Files
============

KiteAeroDyn produces three types of output files: an echo file, a summary
file, and a time-series results file. The following sections detail the
purpose and contents of these files.

Echo Files
----------

If you set the ``Echo`` flag to ``TRUE`` in the KiteAeroDyn driver file or the
KiteAeroDyn primary input file, the contents of those files will be echoed
to a file with the naming conventions, *Basefile.ech* for the
where ``Basefile`` is either the name of the driver input file or the KiteAeroDyn input file. 
The echo files are
helpful for debugging your input files. The contents of an echo file
will be truncated if KiteAeroDyn encounters an error while parsing an input
file. The error usually corresponds to the line after the last
successfully echoed line.

Summary File
------------

KiteAeroDyn generates a summary file with the naming convention,
*OutFileRoot.KAD.sum* if the ``SumPrint`` parameter is set to ``TRUE``.
``OutFileRoot`` is either specified in the I/O SETTINGS section of the
driver input file when running KiteAeroDyn standalone, or by the KiteFAST
program when running a coupled simulation. This file summarizes key
information about your aerodynamics model, including which features have
been enabled and what outputs have been selected.

Results Files
-------------

In standalone mode, the KiteAeroDyn time-series results are written to text-based files with the naming convention
*OutFileRoot.KAD.out*, and *OutFileRoot.VSM.out*, where ``OutFileRoot`` is specified in the I/O
SETTINGS section of the driver input file. If KiteAeroDyn is coupled to KiteFAST, then KiteFAST
will generate a master results file that includes the KiteAeroDyn results if the ``OutSwtch`` parameter is set to 2 or 3.
and KiteAeroDyn will not write out its own results if the ``OutSwtch`` parameter is set to 1 or 3. The results are in table
format, where each column is a data channel (the first column always
being the simulation time), and each row corresponds to a simulation
output time step. The data channels are specified in the OUTPUTS section
of the KiteAeroDyn primary input file. The column format of the
KiteAeroDyn-generated files is specified using the ``OutFmt`` parameter of
the driver input file.
