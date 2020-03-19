#
# Copyright 2018 National Renewable Energy Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

from .iohandler import Output
from .mbdyn_types import Vec3

class MainMBD():

    def __init__(self,
                 simulation_controls,
                 mip_id,
                 keypoints,
                 joints,
                 fuselage,
                 starboard_wing, port_wing,
                 starboard_hstab, port_hstab,
                 vstab,
                 starboard_pylons, port_pylons,
                 starboard_rotors, port_rotors,
                 number_of_flaps_per_wing,
                 number_of_pylons_per_wing,
                 number_of_kite_components):

        self.initial_position = simulation_controls["initial_conditions"]["location"]
        constants = simulation_controls["constants"]
        self.gravity = Vec3(constants["gravity"])
        self.fast_submodules = simulation_controls["fast_submodules"]
        self.fast_submodule_input_files = simulation_controls["fast_submodule_input_files"]
        self.print_kitefast_summary_file = simulation_controls["print_kitefast_summary_file"]
        self.kitefast_output_file_root_name = simulation_controls["kitefast_output_file_root_name"]
        self.kiteaerodyn_interpolation_order = simulation_controls["kiteaerodyn_interpolation_order"]
        time = simulation_controls["time"]
        self.timestep = time["timestep"]
        self.final_time = time["final"]
        self.tolerance = simulation_controls["tolerance"]
        self.max_iterations = simulation_controls["max_iterations"]
        derivatives = simulation_controls["derivatives"]
        self.derivative_tolerance = derivatives["tolerance"]
        self.derivative_max_iteration = derivatives["max_iteration"]
        self.derivative_coefficient = derivatives["coefficient"]
        self.linear_solver = simulation_controls["linear_solver"]
        self.eigen_analysis = simulation_controls["eigen_analysis"]
        self.debug = simulation_controls["debug"]
        self.rigid_model = simulation_controls["rigid_model"]
        base_station = simulation_controls["base_station"]
        self.base_station_location = base_station["location"]
        output = simulation_controls["output"]
        self.mip_id = mip_id
        self.fuselage_output_nodes = output["fuselage_nodes"]
        self.wing_starboard_nodes = output["wing_starboard_nodes"]
        self.wing_port_outputs = output["wing_port_nodes"]
        self.vertical_stabilizer_outputs = output["vertical_stabilizer_nodes"]
        self.horizontal_stabilizer_starboard_outputs = output["horizontal_stabilizer_starboard_nodes"]
        self.horizontal_stabilizer_port_outputs = output["horizontal_stabilizer_port_nodes"]
        self.pylon_outputs = output["pylon_nodes"]
        self.output_channels = [] if output["output_channels"] is None else output["output_channels"]

        self.keypoints = keypoints

        self.joints = joints
        self.fuselage = fuselage
        self.starboard_wing = starboard_wing
        self.port_wing = port_wing
        self.starboard_hstab = starboard_hstab
        self.port_hstab = port_hstab
        self.vstab = vstab
        self.starboard_pylons = starboard_pylons  # this is [Pylon]
        self.port_pylons = port_pylons            # this is [Pylon]
        self.starboard_rotors = starboard_rotors  # this is [Rotor]
        self.port_rotors = port_rotors            # this is [Rotor]

        self.component_list = [self.fuselage,
                               self.starboard_wing,
                               self.port_wing,
                               self.vstab,
                               self.starboard_hstab,
                               self.port_hstab] \
                             + self.starboard_pylons \
                             + self.port_pylons

        self.number_of_flaps_per_wing = number_of_flaps_per_wing
        self.number_of_pylons_per_wing = number_of_pylons_per_wing
        self.number_of_kite_components = number_of_kite_components

        # number of nodes offset from the fuselage root
        # currently, its the first node in the fuselage
        self.mip_index = 0

    def export_mbd_file(self, output_directory):

        output = Output("{}/KiteMain.mbd".format(output_directory))

        output.write_line("# KiteMain.mbd")
        output.write_empty_line()
        output.write_empty_line()
        if not self.eigen_analysis:
            output.write_line("module load: \"libmodule-kitefastmbd\";")
            output.write_empty_line()
            output.write_empty_line()
        output.write_line("begin: data;")
        output.write_line("    problem: initial value;")
        output.write_line("end: data;")
        output.write_empty_line()
        output.write_empty_line()
        output.write_line("begin: initial value;")
        output.write_line("    initial time: 0.0;")
        output.write_line("    final time: {};".format(self.final_time))
        output.write_line("    time step: {};".format(self.timestep))
        output.write_line("    tolerance: {};".format(self.tolerance))
        output.write_line("    max iterations: {};".format(self.max_iterations))
        output.write_line("    derivatives tolerance: {};".format(self.derivative_tolerance))
        output.write_line("    derivatives max iterations: {};".format(self.derivative_max_iteration))
        output.write_line("    derivatives coefficient: {};".format(self.derivative_coefficient))
        output.write_line("    linear solver: {}, colamd, mt, 1, pivot factor, 1e-8;".format(self.linear_solver))
        if self.eigen_analysis:
            output.write_line("    eigenanalysis:")
            output.write_line("        1e-3,")
            output.write_line("        output matrices,")
            output.write_line("        output eigenvectors,")
            output.write_line("        output geometry,")
            output.write_line("        use lapack;")
        if self.debug:
            output.write_empty_line()
            output.write_line("    # for debugging")
            output.write_line("    output: iterations;")
            output.write_line("    output: residual;")
        output.write_line("end: initial value;")
        output.write_empty_line()
        output.write_empty_line()
        output.write_line("begin: control data;")
        output.write_line("    print: all;")
        output.write_line("    default orientation: orientation vector;")
        output.write_empty_line()
        output.write_line("    include: \"KiteMain.set\";")
        output.write_empty_line()
        output.write_line("    structural nodes:")
        for component in self.component_list:
            output.write_line("      + {}_node_count".format(component.component_name))
        output.write_line("      + {}  # 1 for each rotor".format(str(len(self.starboard_rotors) + len(self.port_rotors))))
        output.write_line("      + {}  # 1 for each nacelle".format(str(len(self.starboard_rotors) + len(self.port_rotors))))
        output.write_line("    ;")
        output.write_empty_line()
        output.write_line("    set: integer body_count =")
        for component in self.component_list:
            output.write_line(
                "      + {}_body_count".format(component.component_name))
        output.write_line("    ;")
        output.write_empty_line()
        output.write_line("    set: integer beam_count =")
        for component in self.component_list:
            output.write_line("      + {}_beam_count".format(component.component_name))
        output.write_line("    ;")
        output.write_empty_line()
        output.write_line("    rigid bodies:")
        output.write_line("      + body_count")
        output.write_line("      + {}  # 1 for each rotor".format(str(len(self.starboard_rotors) + len(self.port_rotors))))
        output.write_line("      + {}  # 1 for each nacelle".format(str(len(self.starboard_rotors) + len(self.port_rotors))))
        output.write_line("    ;")
        output.write_line("    beams: beam_count;")
        output.write_line("    joints: {};".format(str(len(self.joints))))
        output.write_line("    gravity;")
        if self.eigen_analysis:
            output.write_line("    loadable elements: 0;")
            output.write_line("    output results: netcdf;")
        else:
            output.write_line("    loadable elements: 1;")
        output.write_line("end: control data;")
        output.write_empty_line()
        output.write_empty_line()
        output.write_line("begin: nodes;")
        for component in self.component_list:
            output.write_line("    include: \"{}.nodes\";".format(component.component_name))
        for rotor in self.starboard_rotors + self.port_rotors:
            output.write_line("    include: \"{}.nodes\";".format(rotor.component_name))
        output.write_line("end: nodes;")
        output.write_empty_line()
        output.write_empty_line()
        output.write_line("begin: elements;")
        output.write_empty_line()
        output.write_line("    gravity: uniform,   0.000,   0.000,   1.000, const, {};".format(self.gravity.x3))
        output.write_empty_line()
        for joint in self.joints:
            output.write_line(str(joint))
            output.write_line("        ")
        for component in self.component_list:
            output.write_line("    include: \"{}.structural\";".format(component.component_name))
        for component in self.component_list:
            output.write_line("    include: \"{}.body\";".format(component.component_name))
        for component in self.starboard_rotors + self.port_rotors:
            output.write_line("    include: \"{}.elements\";".format(component.component_name))
        output.write_empty_line()
        output.write_line("    inertia: 1, body, all;")
        output.write_empty_line()
        if not self.eigen_analysis:
            output.write_line("    user defined: 1, ModuleKiteFAST,")
            output.write_line("        fast_submodule_flags,")
            output.write_line("            {},".format(1 if self.fast_submodules["kiteaerodyn"] is True else 0))
            output.write_line("            {},".format(1 if self.fast_submodules["inflowwind"] is True else 0))
            output.write_line("            {},".format(1 if self.fast_submodules["moordyn"] is True else 0))
            output.write_line("            {},".format(1 if self.fast_submodules["controller"] is True else 0))
            output.write_line("        fast_submodule_input_files,")
            output.write_line("            \"{}\",".format(self.fast_submodule_input_files["kiteaerodyn_input"]))
            output.write_line("            \"{}\",".format(self.fast_submodule_input_files["inflowwind_input"]))
            output.write_line("            \"{}\",".format(self.fast_submodule_input_files["moordyn_input"]))
            output.write_line("            \"{}\",".format(self.fast_submodule_input_files["controller_input"]))
            output.write_line("        output_file_root,")
            output.write_line("            \"{}\",".format(self.kitefast_output_file_root_name))
            output.write_line("        print_kitefast_summary_file,")
            output.write_line("            {},".format(1 if self.print_kitefast_summary_file is True else 0))
            output.write_line("        time_step,")
            output.write_line("            {},".format(self.timestep))
            output.write_line("        gravity,")
            output.write_line("            {},".format(-1 * self.gravity.x3))
            output.write_line("        kiteaerodyn_interpolation_order,")
            output.write_line("            {},".format(self.kiteaerodyn_interpolation_order))
            output.write_line("        base_station_location,")
            output.write_line("            {},".format(self.base_station_location))
            output.write_line("        number_of_flaps_per_wing,")
            output.write_line("            {},".format(self.number_of_flaps_per_wing))
            output.write_line("        number_of_pylons_per_wing,")
            output.write_line("            {},".format(self.number_of_pylons_per_wing))
            output.write_line("        number_of_kite_components,")
            output.write_line("            {},".format(self.number_of_kite_components))
            output.write_line("        keypoints,")
            output.write_line("         {},".format(self.initial_position))
            output.write_line("         {},".format(self.keypoints["wing/starboard"]))
            output.write_line("         {},".format(self.keypoints["wing/port"]))
            output.write_line("         {},".format(self.keypoints["stabilizer/vertical"]))
            output.write_line("         {},".format(self.keypoints["stabilizer/horizontal/starboard"]))
            output.write_line("         {},".format(self.keypoints["stabilizer/horizontal/port"]))
            for i, _ in enumerate(self.starboard_pylons):
                output.write_line("         {},".format(self.keypoints["pylon/starboard/{}".format(str(i + 1))]))
            for i, _ in enumerate(self.port_pylons):
                output.write_line("         {},".format(self.keypoints["pylon/port/{}".format(str(i + 1))]))
            output.write_line("        mip_node,")
            output.write_line("            {},".format(self.mip_id))
            for component in self.component_list:
                output.write_line("        {},".format(component.component_name))
                output.write_line("            {}_node_count,".format(component.component_name))
                for i in range(component.node_count):
                    output.write_line("            {}_root_node + {},".format(component.component_name, i))
                output.write_line("            {}_beam_count,".format(component.component_name))
                for i in range(component.beam_count):
                    output.write_line("            {}_beam + {},".format(component.component_name, 10 * i))
            output.write_line("        starboard_rotors,")
            output.write_line("            {},".format(str(len(self.starboard_rotors))))
            for component in self.starboard_rotors:
                output.write_line("            {}_root_node + 0,".format(component.component_name))
            output.write_line("        port_rotors,")
            output.write_line("            {},".format(str(len(self.port_rotors))))
            for i, component in enumerate(self.port_rotors):
                output.write_line("            {}_root_node + 0,".format(component.component_name))
            output.write_line("        starboard_rotor_properties,")
            for component in self.starboard_rotors:
                output.write_line("            {},".format(component.rotor_mass))
                output.write_line("            {},".format(component.rotor_rot_inertia))
                output.write_line("            {},".format(component.rotor_trans_inertia))
                output.write_line("            {},".format(component.rotor_cm_offset))
            output.write_line("        port_rotor_properties,")
            for component in self.starboard_rotors:
                output.write_line("            {},".format(component.rotor_mass))
                output.write_line("            {},".format(component.rotor_rot_inertia))
                output.write_line("            {},".format(component.rotor_trans_inertia))
                output.write_line("            {},".format(component.rotor_cm_offset))

            def _write_output_lines(nodes, header):
                output.write_line("        {},".format(header))

                # if the only node listed is 0, no output nodes are requested
                if nodes[0] is 0 and len(nodes) == 1:
                    output.write_line("            {},".format(0))
                else:
                    output.write_line("            {},".format(len(nodes)))
                    for node in nodes:
                        output.write_line("            {},".format(node))
        
            _write_output_lines(self.fuselage_output_nodes, "fuselage_outputs")
            _write_output_lines(self.wing_starboard_nodes, "wing_starboard_outputs")
            _write_output_lines(self.wing_port_outputs, "wing_port_outputs")
            _write_output_lines(self.vertical_stabilizer_outputs, "vertical_stabilizer_outputs")
            _write_output_lines(self.horizontal_stabilizer_starboard_outputs, "horizontal_stabilizer_starboard_outputs")
            _write_output_lines(self.horizontal_stabilizer_port_outputs, "horizontal_stabilizer_port_outputs")
            _write_output_lines(self.pylon_outputs, "pylon_outputs")
            output.write_line("        output_channels,")
            if len(self.output_channels) == 0:
                output.write_line("            {};".format(len(self.output_channels)))
            else:
                output.write_line("            {},".format(len(self.output_channels)))
                for i, channel in enumerate(self.output_channels):
                    output_string = "            {}".format(channel)
                    if i == len(self.output_channels)-1 :
                        output_string += ";"
                    else:
                        output_string += ","
                    output.write_line(output_string)
        output.write_line("end: elements;")
        output.write_empty_line()

        output.end()
