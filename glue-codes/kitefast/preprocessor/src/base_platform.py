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


class BasePlatform():
    def __init__(self, component_name, model_dict, mbdyn_ref_index):

        # inputs
        self.component_name = component_name
        self.mbdyn_ref_index = mbdyn_ref_index  # starting index for nodes, beams

        # unpack the component dictionary
        self.node_location = model_dict["node_location"]
        self.imu_location = model_dict["imu_location"]
        self.wind_reference_station_location = model_dict["wind_reference_station_location"]
        self.ground_station_location = model_dict["ground_station_location"]

        self._preprocess()
        self._postprocess()

    def _preprocess(self):
        self.nodes = np.array([

            # platform
            StructuralNode(
                parent_component=self.component_name,
                root_offset_index=0,
                reference_frame="global",
                position=self.node_location
            ),

            # imu
            StructuralNode(
                parent_component=self.component_name,
                root_offset_index=1,
                reference_frame="global",
                position=self.imu_location
            ),

            # wind reference station
            StructuralNode(
                parent_component=self.component_name,
                root_offset_index=2,
                reference_frame="global",
                position=self.wind_reference_station_location
            ),

            # ground station
            StructuralNode(
                parent_component=self.component_name,
                root_offset_index=3,
                reference_frame="global",
                position=self.ground_station_location
            )
        ])
        self.node_count = self.nodes.shape[0]

    def _postprocess(self):
        # NOTE: this will not match mbdyn because the rotor mass is not included in the mbdyn model
        self.component_mass = 0
        self.added_mass = 0
        self.total_mass = self.component_mass + self.added_mass

        # calculate the total center of mass
        cg = Vec3(0.0, 0.0, 0.0)
        self.center_of_gravity = cg - self.node_location

    ### export routines ###

    def export_all(self, output_directory):
        self.export_node_file(output_directory)

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
