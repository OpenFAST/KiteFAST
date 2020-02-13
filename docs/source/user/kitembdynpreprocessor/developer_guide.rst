
Preprocessor Developer Guide
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The KiteMBDyn Preprocessor (referred to as "preprocessor" going forward)
is a Python-based tool for abstracting the complexities of creating an MBDyn
for an energy kite. It contains various data structures to contain geometric
and physical information as well as calculate other derived quantities.
Further, the preprocessor contains a system for exporting the internal
data structures into text files in MBDyn format.

The general flow of data is

1. YAML input file parsed into Python dictionaries;
   one for simulation settings and another for geomtric and physical model data
2. The model dictionary is passed through a series of component classes
   that model the physical components of the kite; the component classes
   convert distributed quantities (mass, inertia) into lumped quanities and
   linearly interpolate values where needed
3. All component classes are stored in a high-level Kite class that coordinates
   joining components, as needed
4. The Kite class is given to an output handler that puts the required
   information into a template for the various MBDyn input files

Component classes
-----------------
Each component except rotors is a subclass of `BaseComponent`. This general
component class is truly the central data structure of the preprocessor.
It handles parsing the component dictionary and instantiating all subcomponents
like `Beam` and `StructuralNode`. Finally, export routines for MBDyn input
files are included.

The `BaseComponent` class should not be used directly, but rather
subclassed; for example:

.. code-blocK:: python

    class Fuselage(BaseComponent):
        def __init__(self, component_name, model_dict, mbdyn_ref_index):
            self.primary_axis = "x1"
            super().__init__(
                component_name,
                model_dict,
                self._interpolator,
                mbdyn_ref_index,
                self.primary_axis
            )

        def _interpolator(self, pos0, pos1, val0, val1, location_vec3):
            return (val0 * (pos1.x1 - location_vec3.x1) + val1 * (location_vec3.x1 - pos0.x1)) / (pos1.x1 - pos0.x1)

Here, a `Fuselage` class is defined which sets its primary axis to `x1`,
provides input information parsed through the `iohandler` module, and
establishes an `interpolator` function. All of these details are required
to properly use the `BaseComponent` super class.

The `primary_axis` attribute should specify the component's "spanwise" or
longest dimension. It is used to establish a proper orientation of reference,
sort, and perform transformations of physical quanitities. Further, the
`interpolator` function is required to define how new nodes are placed.

**NOTE**: The `BaseRotor` class is a special case component. It is very similar
to `BaseComponent`, but contains no `Beam` elements or any of the
corresponding functionality.

I/O handling
--------------
All low-level input and output handling is established in the `iohandler`
module.

Input
+++++
The `Input` class performs all of the parsing and repacking of the inputs.
It is a complex class, but its function is well defined and well implemented.
Unless asbolutely required or all functionality is well understood, it is
best to avoid restructuring this class.

Because all input is parsed as a series of nested dictionaries, this class
uses a custom function for traversing the nested dictionary recursively.
The keys for the nested dictionary follow the hierarchy established in
the input file, and value-dictionaries can be retrieved like this:

.. code-block:: python

    "starboard": _unpack_component_info(["stabilizer", "horizontal", "starboard"]),

Notice that the first key is the most generic and specificity is subsequently
introduced.

The objective of the input-parsing routines is to restructure the input in
a more flat hierarchy for rest of the preprocessor pipeline. The `model_dict`
ultimately contains a key-value pair for each component, as shown below.

.. code-block:: python

    model_dict["fuselage"] = _unpack_component_info(["fuselage"])
    model_dict["wing"] = {
        "number_of_flaps_per_wing": _deep_get(input_dict, ["wing", "number_of_flaps_per_wing"]),
        "starboard": _unpack_component_info(["wing", "starboard"]),
        "port": _unpack_component_info(["wing", "port"])
    }

The pylon and rotor dictionaries have more complexity, and the code itself
will serve as the best reference.

MBDyn file exporting
--------------------
The high-level MBDyn files have template-classes that accept the full model
and place data in the template strings. See the `main_set` and `main_mbd`
modules for details.



