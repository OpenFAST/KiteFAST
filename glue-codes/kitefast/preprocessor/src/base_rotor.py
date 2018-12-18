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
from .mbdyn_types import StructuralNode
from .mbdyn_types import Beam3
from .mbdyn_types import ReferenceFrame
import numpy as np


class BaseRotor():
    def __init__(self, component_name, model_dict, mbdyn_ref_index):

        # inputs
        self.component_name = component_name
        self.mbdyn_ref_index = mbdyn_ref_index  # starting index for nodes, beams

        # unpack the component dictionary
        self.mip = model_dict["keypoint"]
        self.coordinate_list = [self.mip]
        self.initial_rpm = model_dict["rotor"]["initial_rpm"]
        self.rotor_mass = model_dict["rotor"]["mass"]
        self.rotor_cm_offset = model_dict["rotor"]["cm_offset"]
        self.rotor_inertia = model_dict["rotor"]["inertia"]
        self.nacelle_mass = model_dict["nacelle"]["mass"]
        self.nacelle_cm_offset = model_dict["nacelle"]["cm_offset"]
        self.nacelle_inertia = model_dict["nacelle"]["inertia"]

        self.total_mass = self.rotor_mass + self.nacelle_mass
        self.total_cm_offset = [r + n for r, n in zip(self.rotor_cm_offset, self.nacelle_cm_offset)]
        self.center_of_mass = Vec3(self.total_cm_offset[0], self.total_cm_offset[1], self.total_cm_offset[2])
        self.total_inertia = [r + n for r, n in zip(self.rotor_inertia, self.nacelle_inertia)]

        # preprocess nodes
        self.reference_frame = ReferenceFrame(
            name=self.component_name + "_reference_frame",
            reference="mip_rf",
            absolute_position=self.coordinate_list[0],
            absolute_orientation_matrix="eye"
        )

        self.nodes = np.array([
            StructuralNode(
                parent_component=self.component_name,
                root_offset_index=0,
                reference_frame=self.reference_frame.name,
                position=Vec3(0.0, 0.0, 0.0),
                angular_velocity=Vec3(self.initial_rpm, 0.0, 0.0)
            )
        ])
        self.node_count = self.nodes.shape[0]

    ### export routines ###

    def export_all(self, output_directory):
        self.export_node_file(output_directory)
        self.export_element_file(output_directory)

    def export_node_file(self, output_directory):
        output = Output(
            "{}/{}.nodes".format(output_directory, self.component_name))

        output.write_line("# Structural nodes for the {}".format(self.component_name))
        output.write_empty_line()
        output.write_line("# *** local frame ***")
        output.write_line(str(self.reference_frame))
        output.write_empty_line()
        output.write_line("# *** nodes ***")
        output.write_line(str(self.nodes[0]))
        output.write_empty_line()
        output.end()

    def export_element_file(self, output_directory):
        output = Output("{}/{}.elements".format(output_directory, self.component_name))

        output.write_line("set: rotor_node_id = {};".format(self.mbdyn_ref_index))
        output.write_line("set: mass = {};".format(self.total_mass))
        output.write_line("set: cm_offx = {};".format(self.total_cm_offset[0]))
        output.write_line("set: cm_offy = {};".format(self.total_cm_offset[1]))
        output.write_line("set: cm_offz = {};".format(self.total_cm_offset[2]))
        output.write_line("set: Ixx = {};".format(self.total_inertia[0]))
        output.write_line("set: Iyy = {};".format(self.total_inertia[1]))
        output.write_line("set: Izz = {};".format(self.total_inertia[2]))
        output.write_line("body: rotor_node_id, rotor_node_id,")
        output.write_line("    mass,")
        output.write_line("    reference, node, cm_offx, cm_offy, cm_offz,")
        output.write_line("    diag, Ixx, Iyy, Izz;")

        output.end()
