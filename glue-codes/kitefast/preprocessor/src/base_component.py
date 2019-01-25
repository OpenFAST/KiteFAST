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
import numpy as np


class BaseComponent():
    def __init__(self, component_name, model_dict, interpolator, mbdyn_ref_index, primary_axis):

        # inputs
        self.component_name = component_name
        self.mbdyn_ref_index = mbdyn_ref_index  # starting index for nodes, beams
        self.primary_axis = primary_axis

        # unpack the component dictionary
        self.mip = model_dict["keypoint"]

        self.coordinate_list = [
            node + self.mip for node in model_dict["element_end_nodes"]]

        self.twist = model_dict["twist"]
        self.component = model_dict["component"]
        self.point_mass = model_dict["point_mass"]
        self.stiffness_matrix = model_dict["stiffness_matrix"]
        self.mass_distribution = model_dict["mass_distribution"]
        self.cm_offset = model_dict["cm_offset"]
        self.inertias = model_dict["inertias"]

        self.interpolator = interpolator

        # sort coordinates and associated quantities in ascending order
        zipped = list(zip(self.coordinate_list,
                          self.twist,
                          self.component,
                          self.point_mass,
                          self.stiffness_matrix,
                          self.mass_distribution,
                          self.cm_offset,
                          self.inertias
                          )
                      )
        if self.primary_axis == "x1":
            zipped.sort(key=lambda element: element[0].x1)
        elif self.primary_axis == "x2":
            zipped.sort(key=lambda element: element[0].x2)
        elif self.primary_axis == "x3":
            zipped.sort(key=lambda element: element[0].x3)
        unzipped = list(zip(*zipped))

        # self.coordinate_list,
        # self.twist,
        # self.component,
        # self.point_mass,
        # self.stiffness_matrix,
        # self.mass_distribution,
        # self.cm_offset,
        # self.inertias = unzipped

        self.coordinate_list = unzipped[0]
        self.twist = unzipped[1]
        self.component = unzipped[2]
        self.point_mass = unzipped[3]
        self.stiffness_matrix = unzipped[4]
        self.mass_distribution = unzipped[5]
        self.cm_offset = unzipped[6]
        self.inertias = unzipped[7]

        self._preprocess()
        self._postprocess()

    def _preprocess(self):
        self.nodes, self.node_count = self._preprocess_nodes()
        self.beams, self.beam_count = self._preprocess_beams()

    def _postprocess(self):
        # sum the point masses to get total mass
        self.total_mass = sum([beam.m1 + beam.m2 + beam.m3 for beam in self.beams])

        # place the point masses on the nodes and give the added nodes 0 point mass
        self.nodal_point_masses = [self.point_mass[0]]
        for i in range(1, len(self.point_mass)):
            self.nodal_point_masses.append(0.0)
            self.nodal_point_masses.append(self.point_mass[i])

        # calculate the total center of mass
        cg_x, cg_y, cg_z = 0, 0, 0
        for i, beam in enumerate(self.beams):
            cg_x += beam.m1 * beam.node_first.position.x1 + beam.m2 * beam.node_mid.position.x1 + beam.m3 * beam.node_last.position.x1
            cg_y += beam.m1 * beam.node_first.position.x2 + beam.m2 * beam.node_mid.position.x2 + beam.m3 * beam.node_last.position.x2
            cg_z += beam.m1 * beam.node_first.position.x3 + beam.m2 * beam.node_mid.position.x3 + beam.m3 * beam.node_last.position.x3
        cg_x /= self.total_mass
        cg_y /= self.total_mass
        cg_z /= self.total_mass
        self.center_of_mass = Vec3(cg_x, cg_y, cg_z) - self.mip

    def _preprocess_nodes(self):
        # add the midpoint nodes for the elements
        total_coords = [self.coordinate_list[0]]
        for i in range(1, len(self.coordinate_list)):
            prev = self.coordinate_list[i - 1]
            this = self.coordinate_list[i]
            total_coords.append((this + prev) / 2)
            total_coords.append(this)

        nodes = np.array([])
        for i, c in enumerate(total_coords):
            nodes = np.append(
                nodes,
                StructuralNode(
                    parent_component=self.component_name,
                    root_offset_index=i,
                    position=c
                )
            )

        return nodes, nodes.shape[0]

    def _preprocess_beams(self):

        # add mid points for the masses
        total_masses = [self.mass_distribution[0]]
        for i in range(1, len(self.mass_distribution)):
            prev = self.mass_distribution[i - 1]
            this = self.mass_distribution[i]
            mid = (this + prev) / 2
            total_masses.append(mid)
            total_masses.append(this)

        # populate the added mass array...
        # this does not require interpolation
        # but it does need to be the correct length
        total_added_mass = [self.point_mass[0]]
        for i in range(1, len(self.point_mass)):
            total_added_mass.append(0)
            total_added_mass.append(self.point_mass[i])

        # add mid points for the cm_offsets
        total_cm_offsets = [self.cm_offset[0]]
        for i in range(1, len(self.cm_offset)):
            prev = self.cm_offset[i - 1]
            this = self.cm_offset[i]
            mid = [
                (this[0] + prev[0]) / 2,
                (this[1] + prev[1]) / 2
            ]
            total_cm_offsets.append(mid)
            total_cm_offsets.append(this)

        # add mid points for the inertias
        total_inertias = [self.inertias[0]]
        for i in range(1, len(self.inertias)):
            prev = self.inertias[i - 1]
            this = self.inertias[i]
            mid = [sum(x) / 2 for x in zip(prev, this)]
            total_inertias.append(mid)
            total_inertias.append(this)

        beams = []
        beam_count = self.node_count // 2
        for i in range(0, beam_count):
            identifier = self.component_name + "_beam + " + str(10 * i)

            # these are the physical nodes
            node1 = self.nodes[i * 2]
            node2 = self.nodes[i * 2 + 1]
            node3 = self.nodes[i * 2 + 2]
            mass1 = total_masses[i * 2]
            mass2 = total_masses[i * 2 + 1]
            mass3 = total_masses[i * 2 + 2]
            addedmass1 = total_added_mass[i * 2]
            addedmass2 = total_added_mass[i * 2 + 1]
            addedmass3 = total_added_mass[i * 2 + 2]
            inertia1 = total_inertias[i * 2]
            inertia2 = total_inertias[i * 2 + 1]
            inertia3 = total_inertias[i * 2 + 2]
            cm_offset1 = total_cm_offsets[i * 2]
            cm_offset2 = total_cm_offsets[i * 2 + 1]
            cm_offset3 = total_cm_offsets[i * 2 + 2]

            # these are at the gauss nodes
            twist1 = self.twist[i]
            twist2 = self.twist[i + 1]
            stiff1 = self.stiffness_matrix[i]
            stiff2 = self.stiffness_matrix[i + 1]

            # create the Beam3
            this_beam = Beam3(identifier,
                              self.primary_axis,
                              node1, node2, node3,
                              mass1, mass2, mass3,
                              addedmass1, addedmass2, addedmass3,
                              inertia1, inertia2, inertia3,
                              cm_offset1, cm_offset2, cm_offset3,
                              twist1, twist2,
                              stiff1, stiff2,
                              self.interpolator)
            beams.append(this_beam)

        return beams, beam_count

    ### export routines ###

    def export_all(self, output_directory):
        self.export_node_file(output_directory)
        self.export_beam_file(output_directory)
        self.export_element_file(output_directory)

    def export_node_file(self, output_directory):
        output = Output("{}/{}.nodes".format(output_directory, self.component_name))

        output.write_line(
            "# Structural nodes for the {}".format(self.component_name))
        output.write_empty_line()
        output.write_line("# *** nodes ***")
        for n in self.nodes:
            output.write_line(str(n))
            output.write_empty_line()
        output.end()

    def export_element_file(self, output_directory):
        output = Output("{}/{}.structural".format(output_directory, self.component_name))

        output.write_line("# Beam elements for the fuselage")
        output.write_empty_line()
        output.write_line("set: integer {}_beam = {};".format(self.component_name, self.mbdyn_ref_index))
        output.write_empty_line()
        output.write_line("# *** beam elements ***")
        for beam in self.beams:
            output.write_line(str(beam))
            output.write_line(
                "include: \"{}.beam\";".format(self.component_name))
            output.write_empty_line()
        output.end()

    def export_beam_file(self, output_directory):
        output = Output("{}/{}.beam".format(output_directory, self.component_name))
        output.write_line("# Generic beam element properties for the beams")
        output.write_empty_line()
        output.write_line("# *** inertial properties ***")
        output.write_line("body: curr_beam, beam_node1,")
        output.write_line("    m1,")
        output.write_line("    reference, node, cm_offx1, cm_offy1, cm_offz1,")
        output.write_line("    diag, Ixx1, Iyy1, Izz1;")
        output.write_line("body: curr_beam + 1, beam_node2,")
        output.write_line("    m2,")
        output.write_line("    reference, node, cm_offx2, cm_offy2, cm_offz2,")
        output.write_line("    diag, Ixx2, Iyy2, Izz2;")
        output.write_line("body: curr_beam + 2, beam_node3,")
        output.write_line("    m3,")
        output.write_line("    reference, node, cm_offx3, cm_offy3, cm_offz3,")
        output.write_line("    diag, Ixx3, Iyy3, Izz3;")
        output.write_empty_line()
        output.write_line("# *** elastic properties ***")
        output.write_line("beam3:  curr_beam,")
        output.write_line("    beam_node1, reference, node, null,")
        output.write_line("    beam_node2, reference, node, null,")
        output.write_line("    beam_node3, reference, node, null,")
        output.write_line("    reference, mip_rf, eye,")
        output.write_line("    linear elastic generic, sym,")
        output.write_line("        k1_11, k1_12, k1_13, k1_14, k1_15, k1_16,")
        output.write_line("               k1_22, k1_23, k1_24, k1_25, k1_26,")
        output.write_line("                      k1_33, k1_24, k1_25, k1_26,")
        output.write_line("                             k1_44, k1_25, k1_26,")
        output.write_line("                                    k1_55, k1_26,")
        output.write_line("                                           k1_66,")
        output.write_line("    reference, mip_rf, eye,")
        output.write_line("    linear elastic generic, sym,")
        output.write_line("        k2_11, k2_12, k2_13, k2_14, k2_15, k2_16,")
        output.write_line("               k2_22, k2_23, k2_24, k2_25, k2_26,")
        output.write_line("                      k2_33, k2_24, k2_25, k2_26,")
        output.write_line("                             k2_44, k2_25, k2_26,")
        output.write_line("                                    k2_55, k2_26,")
        output.write_line("                                           k2_66;")

        output.end()
