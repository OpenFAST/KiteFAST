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
                 mip_reference_frame,
                 fuselage,
                 starboard_wing, port_wing,
                 starboard_horizontal_stabilizer, port_horizontal_stabilizer,
                 vertical_stabilizer,
                 starboard_pylons, port_pylons,
                 starboard_rotors, port_rotors
                 ):
        self.mip_reference_frame = mip_reference_frame
        self.fuselage = fuselage
        self.starboard_wing = starboard_wing
        self.port_wing = port_wing
        self.starboard_horizontal_stabilizer = starboard_horizontal_stabilizer
        self.port_horizontal_stabilizer = port_horizontal_stabilizer
        self.vertical_stabilizer = vertical_stabilizer
        self.starboard_pylons = starboard_pylons
        self.port_pylons = port_pylons
        self.starboard_rotors = starboard_rotors
        self.port_rotors = port_rotors

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
        output.write_line("set: integer {}_beam_count = {};".format(self.fuselage.component_name, self.fuselage.beam_count))
        output.write_empty_line()
        output.write_line("# *** Wing ***")
        output.write_line("set: integer {}_root_node = {};".format(self.starboard_wing.component_name, self.starboard_wing.mbdyn_ref_index))
        output.write_line("set: integer {}_node_count = {};".format(self.starboard_wing.component_name, self.starboard_wing.node_count))
        output.write_line("set: integer {}_beam_count = {};".format(self.starboard_wing.component_name, self.starboard_wing.beam_count))
        output.write_empty_line()
        output.write_line("set: integer {}_root_node = {};".format(self.port_wing.component_name, self.port_wing.mbdyn_ref_index))
        output.write_line("set: integer {}_node_count = {};".format(self.port_wing.component_name, self.port_wing.node_count))
        output.write_line("set: integer {}_beam_count = {};".format(self.port_wing.component_name, self.port_wing.beam_count))
        output.write_empty_line()
        output.write_line("# *** Horizontal Stabilizer ***")
        output.write_line("set: integer {}_root_node = {};".format(self.starboard_horizontal_stabilizer.component_name, self.starboard_horizontal_stabilizer.mbdyn_ref_index))
        output.write_line("set: integer {}_node_count = {};".format(self.starboard_horizontal_stabilizer.component_name, self.starboard_horizontal_stabilizer.node_count))
        output.write_line("set: integer {}_beam_count = {};".format(self.starboard_horizontal_stabilizer.component_name, self.starboard_horizontal_stabilizer.beam_count))
        output.write_empty_line()
        output.write_line("set: integer {}_root_node = {};".format(self.port_horizontal_stabilizer.component_name, self.port_horizontal_stabilizer.mbdyn_ref_index))
        output.write_line("set: integer {}_node_count = {};".format(self.port_horizontal_stabilizer.component_name, self.port_horizontal_stabilizer.node_count))
        output.write_line("set: integer {}_beam_count = {};".format(self.port_horizontal_stabilizer.component_name, self.port_horizontal_stabilizer.beam_count))
        output.write_empty_line()
        output.write_line("# *** Vertical Stabilizer ***")
        output.write_line("set: integer {}_root_node = {};".format(self.vertical_stabilizer.component_name, self.vertical_stabilizer.mbdyn_ref_index))
        output.write_line("set: integer {}_node_count = {};".format(self.vertical_stabilizer.component_name, self.vertical_stabilizer.node_count))
        output.write_line("set: integer {}_beam_count = {};".format(self.vertical_stabilizer.component_name, self.vertical_stabilizer.beam_count))
        output.write_empty_line()
        output.write_line("# *** Pylons ***")
        for pylon in self.starboard_pylons:
            output.write_line("set: integer {}_root_node = {};".format(pylon.component_name, pylon.mbdyn_ref_index))
            output.write_line("set: integer {}_node_count = {};".format(pylon.component_name, pylon.node_count))
            output.write_line("set: integer {}_beam_count = {};".format(pylon.component_name, pylon.beam_count))
        for pylon in self.port_pylons:
            output.write_line("set: integer {}_root_node = {};".format(pylon.component_name, pylon.mbdyn_ref_index))
            output.write_line("set: integer {}_node_count = {};".format(pylon.component_name, pylon.node_count))
            output.write_line("set: integer {}_beam_count = {};".format(pylon.component_name, pylon.beam_count))
        output.write_empty_line()
        output.write_line("# *** Rotors ***")
        for component in self.starboard_rotors + self.port_rotors:
            output.write_line("set: integer {}_reference_frame = {};".format(component.component_name, component.mbdyn_ref_index))
            output.write_line("set: integer {}_root_node = {};".format(component.component_name, component.mbdyn_ref_index))
            output.write_line("set: integer {}_node_count = {};".format(component.component_name, component.node_count))
        output.write_empty_line()
        output.write_line("# Initialize variables for reuse later")
        output.write_line("set: integer curr_beam = 0;")
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
        output.write_line("set: real cm_offx = 0.0;")
        output.write_line("set: real cm_offy = 0.0;")
        output.write_line("set: real cm_offz = 0.0;")
        output.write_line("set: real Ixx = 0.0;")
        output.write_line("set: real Iyy = 0.0;")
        output.write_line("set: real Izz = 0.0;")

        output.end()
