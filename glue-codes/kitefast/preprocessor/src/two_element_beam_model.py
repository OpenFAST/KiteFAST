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

from .base_model import BaseModel
from .components import Fuselage
from .iohandler import Output
from .mbdyn_types import Vec3
from .mbdyn_types import OrientationMatrix
from .mbdyn_types import ReferenceFrame


class TwoElementBeamModel(BaseModel):
    def __init__(self, input_simulation_dict, input_model_dict):
        super().__init__()

        # verify required components exist
        self.required_components = [
            ["fuselage"]
        ]
        self._verify_component_list(input_model_dict)

        # do any additional data preprocessing of the inputs
        self.model_dict = self._preprocess_model_dict(input_model_dict)
        self.simulation_dict = self._preprocess_simulation_dict(input_simulation_dict)

        # build the primary reference frame
        initial_conditions = self.simulation_dict["initial_conditions"]
        self.mip_reference_frame = ReferenceFrame(
            name="mip_rf",
            reference="global",
            absolute_position=initial_conditions["location"],
            absolute_orientation_matrix=OrientationMatrix(initial_conditions["orientation"]),
            absolute_velocity=initial_conditions["velocity"]["translational"],
            absolute_angular_velocity=initial_conditions["velocity"]["rotational"]
        )

        # model setup
        self.components = self._build_components(self.model_dict)
        self.fuselage = self.components[0]

        # simulation setup
        self.main_mbd = MainMBD(
            self.simulation_dict,
            self.model_dict["keypoints"],
            self.fuselage
        )
        self.main_set = MainSet(
            self.mip_reference_frame,
            self.fuselage
        )

    def _build_components(self, model_dict):

        def _build_component(component_type, path, mbdyn_ref_index):
            name = "_".join([str(p) for p in path])
            return component_type(
                name,
                self._deep_get(model_dict, path),
                mbdyn_ref_index)

        fuselage = _build_component(Fuselage, ["fuselage"], 1000)

        return fuselage,

    def export(self, output_directory):
        self.main_set.export_set_file(output_directory)
        self.main_mbd.export_mbd_file(output_directory)
        self.fuselage.export_all(output_directory)


class MainSet():
    def __init__(self, mip_reference_frame, fuselage):
        self.mip_reference_frame = mip_reference_frame
        self.fuselage = fuselage

    def export_set_file(self, output_directory):

        output = Output("{}/KiteMain.set".format(output_directory))

        output.write_line("# This file sets the reference id's and counters")
        output.write_empty_line()
        output.write_line("# *** Reference ***")
        output.write_empty_line()
        output.write_line("set: integer mip_rf = 10;")
        output.write_line("set: integer mip_node = 10; # most important point id, reference for all")
        output.write_line("{}".format(self.mip_reference_frame))
        output.write_empty_line()
        output.write_line("# *** Fuselage ***")
        output.write_line("set: integer fuselage_root_node = mip_node;")
        output.write_line("set: integer {}_node_count = {};".format(self.fuselage.component_name, self.fuselage.node_count))
        output.write_line("set: integer {}_body_count = {};".format(self.fuselage.component_name, self.fuselage.body_count))
        output.write_line("set: integer {}_beam_count = {};".format(self.fuselage.component_name, self.fuselage.beam_count))
        output.write_empty_line()
        output.write_line("# Initialize variables for reuse later")
        output.write_line("set: integer current_beam = 0;")
        output.write_line("set: integer beam_ref1 = 0;")
        output.write_line("set: integer beam_ref2 = 0;")
        output.write_line("set: real twist1 = 0.0;")
        output.write_line("set: real twist2 = 0.0;")
        output.write_line("set: integer beam_node1 = 0;")
        output.write_line("set: integer beam_node2 = 0;")
        output.write_line("set: integer beam_node3 = 0;")
        output.write_line("set: real L_beam = 0.0;")
        output.write_line("set: real l1 = 0.0;")
        output.write_line("set: real l2 = 0.0;")
        output.write_line("set: real l3 = 0.0;")
        output.write_line("set: real k1_11 = 0.0;")
        output.write_line("set: real k1_12 = 0.0;")
        output.write_line("set: real k1_13 = 0.0;")
        output.write_line("set: real k1_14 = 0.0;")
        output.write_line("set: real k1_15 = 0.0;")
        output.write_line("set: real k1_16 = 0.0;")
        output.write_line("set: real k1_22 = 0.0;")
        output.write_line("set: real k1_23 = 0.0;")
        output.write_line("set: real k1_24 = 0.0;")
        output.write_line("set: real k1_25 = 0.0;")
        output.write_line("set: real k1_26 = 0.0;")
        output.write_line("set: real k1_33 = 0.0;")
        output.write_line("set: real k1_34 = 0.0;")
        output.write_line("set: real k1_35 = 0.0;")
        output.write_line("set: real k1_36 = 0.0;")
        output.write_line("set: real k1_44 = 0.0;")
        output.write_line("set: real k1_45 = 0.0;")
        output.write_line("set: real k1_46 = 0.0;")
        output.write_line("set: real k1_55 = 0.0;")
        output.write_line("set: real k1_56 = 0.0;")
        output.write_line("set: real k1_66 = 0.0;")
        output.write_line("set: real k2_11 = 0.0;")
        output.write_line("set: real k2_12 = 0.0;")
        output.write_line("set: real k2_13 = 0.0;")
        output.write_line("set: real k2_14 = 0.0;")
        output.write_line("set: real k2_15 = 0.0;")
        output.write_line("set: real k2_16 = 0.0;")
        output.write_line("set: real k2_22 = 0.0;")
        output.write_line("set: real k2_23 = 0.0;")
        output.write_line("set: real k2_24 = 0.0;")
        output.write_line("set: real k2_25 = 0.0;")
        output.write_line("set: real k2_26 = 0.0;")
        output.write_line("set: real k2_33 = 0.0;")
        output.write_line("set: real k2_34 = 0.0;")
        output.write_line("set: real k2_35 = 0.0;")
        output.write_line("set: real k2_36 = 0.0;")
        output.write_line("set: real k2_44 = 0.0;")
        output.write_line("set: real k2_45 = 0.0;")
        output.write_line("set: real k2_46 = 0.0;")
        output.write_line("set: real k2_55 = 0.0;")
        output.write_line("set: real k2_56 = 0.0;")
        output.write_line("set: real k2_66 = 0.0;")
        output.write_line("set: real m = 0.0;")
        output.write_line("set: real Ixx1 = 0.0;")
        output.write_line("set: real Ixx2 = 0.0;")
        output.write_line("set: real Ixx3 = 0.0;")
        output.write_line("set: real Iyy1 = 0.0;")
        output.write_line("set: real Iyy2 = 0.0;")
        output.write_line("set: real Iyy3 = 0.0;")
        output.write_line("set: real Izz1 = 0.0;")
        output.write_line("set: real Izz2 = 0.0;")
        output.write_line("set: real Izz3 = 0.0;")
        output.write_line("set: real m1 = 0.0;")
        output.write_line("set: real m2 = 0.0;")
        output.write_line("set: real m3 = 0.0;")
        output.write_line("set: real cm_offx1 = 0.0;")
        output.write_line("set: real cm_offy1 = 0.0;")
        output.write_line("set: real cm_offz1 = 0.0;")
        output.write_line("set: real cm_offx2 = 0.0;")
        output.write_line("set: real cm_offy2 = 0.0;")
        output.write_line("set: real cm_offz2 = 0.0;")
        output.write_line("set: real cm_offx3 = 0.0;")
        output.write_line("set: real cm_offy3 = 0.0;")
        output.write_line("set: real cm_offz3 = 0.0;")
        output.write_line("set: integer rotor_node_id = 0;")
        output.write_line("set: real mass = 0.0;")
        output.write_line("set: real Ixx = 0.0;")
        output.write_line("set: real Iyy = 0.0;")
        output.write_line("set: real Izz = 0.0;")

        output.end()


class MainMBD():

    def __init__(self, simulation_controls, keypoints, fuselage):

        self.initial_position = simulation_controls["initial_conditions"]["location"]
        constants = simulation_controls["constants"]
        self.gravity = Vec3(constants["gravity"])
        self.fast_submodules = simulation_controls["fast_submodules"]
        self.fast_submodule_input_files = simulation_controls["fast_submodule_input_files"]
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
        self.debug = simulation_controls["debug"]
        self.rigid_model = simulation_controls["rigid_model"]
        base_station = simulation_controls["base_station"]
        self.base_station_location = base_station["location"]

        self.keypoints = keypoints
        self.fuselage = fuselage
        self.component_list = [self.fuselage]

    def export_mbd_file(self, output_directory):

        output = Output("{}/KiteMain.mbd".format(output_directory))

        output.write_line("# KiteWing.mbd")
        output.write_empty_line()
        output.write_empty_line()
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
        if self.debug:
            output.write_empty_line()
            output.write_line("    # for debugging")
            output.write_line("    output: iterations;")
            output.write_line("    output: residual;")
        output.write_line("end: initial value;")
        output.write_empty_line()
        output.write_empty_line()
        output.write_line("begin: control data;")
        # output.write_line("    print: all;")
        # output.write_line("    default output: none, structural nodes;")
        output.write_line("    default orientation: orientation vector;")
        output.write_empty_line()
        output.write_line("    include: \"KiteMain.set\";")
        output.write_empty_line()
        output.write_line("    structural nodes:")
        # output.write_line("      + 1  # MIP node")
        for component in self.component_list:
            output.write_line("      + {}_node_count".format(component.component_name))
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
        output.write_line("    ;")
        output.write_line("    beams: beam_count;")
        output.write_line("    gravity;")
        output.write_line("    joints: 1;")
        output.write_line("end: control data;")
        output.write_empty_line()
        output.write_empty_line()
        output.write_line("begin: nodes;")
        for component in self.component_list:
            output.write_line("    include: \"{}.nodes\";".format(component.component_name))
        output.write_line("end: nodes;")
        output.write_empty_line()
        output.write_empty_line()
        output.write_line("begin: elements;")
        output.write_empty_line()
        output.write_line("    gravity: uniform,   0.000,   0.000,   1.000, const, {};".format(self.gravity.x3))
        output.write_line("    joint: 0, clamp, 10, position, node, orientation, node;")
        output.write_empty_line()
        for component in self.component_list:
            output.write_line("    include: \"{}.structural\";".format(component.component_name))
        for component in self.component_list:
            output.write_line("    include: \"{}.body\";".format(component.component_name))
        output.write_empty_line()
        output.write_line("    inertia: 1, body, all;")
        output.write_empty_line()
        output.write_line("end: elements;")
        output.write_empty_line()

        output.end()
