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
from .mbdyn_types import Body
import numpy as np


class BaseRotor():
    def __init__(self, component_name, model_dict, mbdyn_ref_index):

        # inputs
        self.component_name = component_name
        self.mbdyn_ref_index = mbdyn_ref_index  # starting index for nodes, beams

        # unpack the component dictionary
        self.mip = model_dict["keypoint"]
        self.coordinate_list = [self.mip]

        # rotor
        self.rotor_mass = model_dict["rotor"]["mass"]
        self.rotor_cm_offset = Vec3(model_dict["rotor"]["cm_offset"], 0.0, 0.0)     # the rotors can only shift in the x-direction
        self.rotor_rot_inertia = model_dict["rotor"]["rotational_inertia"]
        self.rotor_trans_inertia = model_dict["rotor"]["translational_inertia"]

        # nacelle
        self.nacelle_mass = model_dict["nacelle"]["mass"]
        _cm_offset = model_dict["nacelle"]["cm_offset"]
        self.nacelle_cm_offset = Vec3(_cm_offset[0], _cm_offset[1], _cm_offset[2])
        self.nacelle_inertia = model_dict["nacelle"]["inertia"]

        self._preprocess()
        self._postprocess()

    def _preprocess(self):
        self.nodes = np.array([

            # rotor
            StructuralNode(
                parent_component=self.component_name,
                root_offset_index=0,
                position=self.coordinate_list[0]
            ),

            # nacelle
            StructuralNode(
                parent_component=self.component_name,
                root_offset_index=1,
                position=self.coordinate_list[0]
            )
        ])
        self.node_count = self.nodes.shape[0]

        self.bodies = np.array([

            # rotor
            # this body should have no mass or inertia as these loads are computed in kitefast
            Body(
                identifier=self.nodes[0].id,
                node=self.nodes[0],
                mass=0.0,  # self.rotor_mass ... this is always 0 because the mass is handled in kitefast
                added_mass=0.0,
                cm_offset=self.rotor_cm_offset
            ),

            # nacelle
            Body(
                identifier=self.nodes[1].id,
                node=self.nodes[1],
                mass=self.nacelle_mass,
                added_mass=0.0,
                cm_offset=self.nacelle_cm_offset,
                Ixx=self.nacelle_inertia[0],
                Iyy=self.nacelle_inertia[1],
                Izz=self.nacelle_inertia[2]
            )
        ])

    def _postprocess(self):
        # NOTE: this will not match mbdyn because the rotor mass is not included in the mbdyn model
        self.component_mass = self.rotor_mass + self.nacelle_mass
        self.added_mass = 0
        self.total_mass = self.component_mass + self.added_mass

        # calculate the total center of mass
        cg = Vec3(
            self.rotor_mass * self.nodes[0].position.x1 + self.nacelle_mass * self.nodes[1].position.x1,
            self.rotor_mass * self.nodes[0].position.x2 + self.nacelle_mass * self.nodes[1].position.x2,
            self.rotor_mass * self.nodes[0].position.x3 + self.nacelle_mass * self.nodes[1].position.x3
        )
        cg /= self.total_mass
        self.center_of_gravity = cg - self.mip

    ### export routines ###

    def export_all(self, output_directory):
        self.export_node_file(output_directory)
        self.export_element_file(output_directory)

    def export_node_file(self, output_directory):
        output = Output(
            "{}/{}.nodes".format(output_directory, self.component_name))

        output.write_line("# Structural nodes for the {}".format(self.component_name))
        output.write_empty_line()
        output.write_line("# *** nodes ***")
        for n in self.nodes:
            output.write_line(str(n))
            output.write_empty_line()
        output.end()

    def export_element_file(self, output_directory):
        output = Output("{}/{}.elements".format(output_directory, self.component_name))
        output.write_line("# *** bodies ***")
        for b in self.bodies:
            output.write_line(str(b))
            output.write_empty_line()
        output.end()
