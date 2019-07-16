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


class MainSet():
    def __init__(self,
                 simulation_type,
                 mip_reference_frame,
                 components):

        self.simulation_type = simulation_type

        self.mip_reference_frame = mip_reference_frame

        if self.simulation_type < 3:
            self.fuselage = components[0]
            self.starboard_wing = components[1]
            self.port_wing = components[2]
            self.vstab = components[3]
            self.starboard_hstab = components[4]
            self.port_hstab = components[5]
            self.starboard_pylons = components[6]  # this is [Pylon]
            self.port_pylons = components[7]       # this is [Pylon]
            self.starboard_rotors = components[8]  # this is [Rotor]
            self.port_rotors = components[9]       # this is [Rotor]

        if self.simulation_type > 1:
            self.platform = components[-1]

    def export_set_file(self, output_directory):

        output = Output("{}/KiteMain.set".format(output_directory))

        output.write_line("# This file sets the reference id's and counters")
        output.write_empty_line()
        output.write_line("# *** Reference ***")
        output.write_empty_line()
        output.write_line("set: integer mip_rf = 10;")
        output.write_line("{}".format(self.mip_reference_frame))
        output.write_empty_line()
        if self.simulation_type < 3:
            output.write_line("# *** Fuselage ***")
            output.write_line("set: integer {}_root_node = {};".format(self.fuselage.component_name, self.fuselage.mbdyn_ref_index))
            output.write_line("set: integer {}_node_count = {};".format(self.fuselage.component_name, self.fuselage.node_count))
            output.write_line("set: integer {}_body_count = {};".format(self.fuselage.component_name, self.fuselage.body_count))
            output.write_line("set: integer {}_beam_count = {};".format(self.fuselage.component_name, self.fuselage.beam_count))
            output.write_empty_line()
            output.write_line("# *** Wing ***")
            output.write_line("set: integer {}_root_node = {};".format(self.starboard_wing.component_name, self.starboard_wing.mbdyn_ref_index))
            output.write_line("set: integer {}_node_count = {};".format(self.starboard_wing.component_name, self.starboard_wing.node_count))
            output.write_line("set: integer {}_body_count = {};".format(self.starboard_wing.component_name, self.starboard_wing.body_count))
            output.write_line("set: integer {}_beam_count = {};".format(self.starboard_wing.component_name, self.starboard_wing.beam_count))
            output.write_empty_line()
            output.write_line("set: integer {}_root_node = {};".format(self.port_wing.component_name, self.port_wing.mbdyn_ref_index))
            output.write_line("set: integer {}_node_count = {};".format(self.port_wing.component_name, self.port_wing.node_count))
            output.write_line("set: integer {}_body_count = {};".format(self.port_wing.component_name, self.port_wing.body_count))
            output.write_line("set: integer {}_beam_count = {};".format(self.port_wing.component_name, self.port_wing.beam_count))
            output.write_empty_line()
            output.write_line("# *** Horizontal Stabilizer ***")
            output.write_line("set: integer {}_root_node = {};".format(self.starboard_hstab.component_name, self.starboard_hstab.mbdyn_ref_index))
            output.write_line("set: integer {}_node_count = {};".format(self.starboard_hstab.component_name, self.starboard_hstab.node_count))
            output.write_line("set: integer {}_body_count = {};".format(self.starboard_hstab.component_name, self.starboard_hstab.body_count))
            output.write_line("set: integer {}_beam_count = {};".format(self.starboard_hstab.component_name, self.starboard_hstab.beam_count))
            output.write_empty_line()
            output.write_line("set: integer {}_root_node = {};".format(self.port_hstab.component_name, self.port_hstab.mbdyn_ref_index))
            output.write_line("set: integer {}_node_count = {};".format(self.port_hstab.component_name, self.port_hstab.node_count))
            output.write_line("set: integer {}_body_count = {};".format(self.port_hstab.component_name, self.port_hstab.body_count))
            output.write_line("set: integer {}_beam_count = {};".format(self.port_hstab.component_name, self.port_hstab.beam_count))
            output.write_empty_line()
            output.write_line("# *** Vertical Stabilizer ***")
            output.write_line("set: integer {}_root_node = {};".format(self.vstab.component_name, self.vstab.mbdyn_ref_index))
            output.write_line("set: integer {}_node_count = {};".format(self.vstab.component_name, self.vstab.node_count))
            output.write_line("set: integer {}_body_count = {};".format(self.vstab.component_name, self.vstab.body_count))
            output.write_line("set: integer {}_beam_count = {};".format(self.vstab.component_name, self.vstab.beam_count))
            output.write_empty_line()
            output.write_line("# *** Pylons ***")
            for pylon in self.starboard_pylons:
                output.write_line("set: integer {}_root_node = {};".format(pylon.component_name, pylon.mbdyn_ref_index))
                output.write_line("set: integer {}_node_count = {};".format(pylon.component_name, pylon.node_count))
                output.write_line("set: integer {}_body_count = {};".format(pylon.component_name, pylon.body_count))
                output.write_line("set: integer {}_beam_count = {};".format(pylon.component_name, pylon.beam_count))
            for pylon in self.port_pylons:
                output.write_line("set: integer {}_root_node = {};".format(pylon.component_name, pylon.mbdyn_ref_index))
                output.write_line("set: integer {}_node_count = {};".format(pylon.component_name, pylon.node_count))
                output.write_line("set: integer {}_body_count = {};".format(pylon.component_name, pylon.body_count))
                output.write_line("set: integer {}_beam_count = {};".format(pylon.component_name, pylon.beam_count))
            output.write_empty_line()
            output.write_line("# *** Rotors ***")
            for component in self.starboard_rotors + self.port_rotors:
                output.write_line("set: integer {}_reference_frame = {};".format(component.component_name, component.mbdyn_ref_index))
                output.write_line("set: integer {}_root_node = {};".format(component.component_name, component.mbdyn_ref_index))
                output.write_line("set: integer {}_node_count = {};".format(component.component_name, component.node_count))
        if self.simulation_type > 1:
            output.write_line("# *** Platform ***")
            output.write_line("set: integer {}_root_node = {};".format(self.platform.component_name, self.platform.mbdyn_ref_index))
            output.write_empty_line()
        output.write_line("# Initialize variables for reuse later")
        output.write_line("set: integer current_beam = 0;")
        output.write_line("set: real twist1 = 0.0;")
        output.write_line("set: real twist2 = 0.0;")
        output.write_line("set: integer beam_node1 = 0;")
        output.write_line("set: integer beam_node2 = 0;")
        output.write_line("set: integer beam_node3 = 0;")
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
        output.end()
