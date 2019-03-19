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
from .mbdyn_types import Beam3
import numpy as np


class BaseComponent():
    """
    BaseComponent is the super class from which all non-rotor components are created.
    It handles parsing the component dictionary and instantiating all subcomponents
    like Beam and StructuralNode. Finally, export routines for MBDyn input files
    are included here.
    """

    def __init__(self, component_name, model_dict, interpolator, mbdyn_ref_index, primary_axis):
        """
        inputs:
            component_name: str - a unique identifier for this component

            model_dict: dict - a description of this component as parsed from the input file;
                it should contain the following key-values:
                {
                    "keypoint": Vec3,

                    "element_end_nodes": [Vec3],

                    "twist": [float],

                    "component": [str],

                    "point_mass": [float],

                    "stiffness_matrix": [[float]],

                    "mass_distribution": [[float]],

                    "cm_offsets": [[float]],

                    "inertias": [[float]]
                }

            interpolator: func - the function to use when interpolating values between node locations;
                this allows subclasses to determine on which axis to do interpolations

            mbdyn_ref_index: int - a value to use when identifying this component in mbdyn
            
            primary_axis: str - string representation of this component's primary axis; [x1 | x2 | x3]

        outputs:
            self: BaseComponent - an instantiated BaseComponent object
        """

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
        self.bodies, self.body_count, self.beams, self.beam_count = self._preprocess_bodies_beams()

    def _postprocess(self):
        """
        Calculates the mass and center of mass.

        component_mass: the total mass of this physical component only
        added_mass: the total mass of any additional point masses
        total_mass: the sum of component and added masses

        The component's mip is subtracted from the center of mass
        so the resulting CG is relative to the component.
        """
        # sum the point masses to get total mass
        self.component_mass = sum([body.mass for body in self.bodies])
        self.added_mass = sum([body.added_mass for body in self.bodies])
        self.total_mass = self.component_mass + self.added_mass

        # place the point masses on the nodes and give the added nodes 0 point mass
        self.nodal_point_masses = [self.point_mass[0]]
        for i in range(1, len(self.point_mass)):
            self.nodal_point_masses.append(0.0)
            self.nodal_point_masses.append(self.point_mass[i])

        # calculate the total center of mass
        cg = Vec3(0.0, 0.0, 0.0)
        for i, beam in enumerate(self.beams):
            body_index = i * 4
            body1 = self.bodies[body_index]
            body2_1 = self.bodies[body_index + 1]
            body2_2 = self.bodies[body_index + 2]
            body3 = self.bodies[body_index + 3]
            cg += Vec3(
                body1.mass * beam.node_first.position.x1 + (body2_1.mass + body2_2.mass) * beam.node_mid.position.x1 + body3.mass * beam.node_last.position.x1,
                body1.mass * beam.node_first.position.x2 + (body2_1.mass + body2_2.mass) * beam.node_mid.position.x2 + body3.mass * beam.node_last.position.x2,
                body1.mass * beam.node_first.position.x3 + (body2_1.mass + body2_2.mass) * beam.node_mid.position.x3 + body3.mass * beam.node_last.position.x3
            )
        cg /= self.component_mass
        self.center_of_gravity = cg - self.mip

    def _preprocess_nodes(self):
        """
        Builds the StructuralNodes for this component.
        The central nodes are added here by linearly interpolating the position.
        output:
            nodes: [StructuralNode]
            node_count: len(nodes)
        """
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

    def _preprocess_bodies_beams(self):
        """
        Builds the Body and Beam3 for this component.

        The central node properties are computed here by linearly interpolating
        all properties.
        """
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
        bodies = []
        body_count = beam_count * 4
        for i in range(beam_count):
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

            beam_length = np.sqrt(
                np.power(node1.position.x1 - node3.position.x1, 2)
                + np.power(node1.position.x2 - node3.position.x2, 2)
                + np.power(node1.position.x3 - node3.position.x3, 2)
            )

            unit_vector = Vec3(
                node3.position.x1 - node1.position.x1,
                node3.position.x2 - node1.position.x2,
                node3.position.x3 - node1.position.x3) / beam_length

            # gauss point locations
            gauss_first_displacement = (beam_length / 2.0) * (1 - 1 / np.sqrt(3))
            gauss_last_displacement = (beam_length / 2.0) * (1 + 1 / np.sqrt(3))
            gauss_first_point = (node1.position + gauss_first_displacement) * unit_vector
            gauss_last_point = (node1.position + gauss_last_displacement) * unit_vector
            
            # guass point twist
            gauss_first_twist = self.interpolator(
                node1.position,
                node3.position,
                twist1,
                twist2,
                gauss_first_point
            )
            gauss_last_twist = self.interpolator(
                node1.position,
                node3.position,
                twist1,
                twist2,
                gauss_last_point
            )

            # guass point stiffness
            gauss_first_stiffness, gauss_last_stiffness = [], []
            for j in range(21):
                gauss_first_stiffness.append(
                    self.interpolator(
                        node1.position,
                        node3.position,
                        stiff1[j],
                        stiff2[j],
                        gauss_first_point
                    )
                )
                gauss_last_stiffness.append(
                    self.interpolator(
                        node1.position,
                        node3.position,
                        stiff1[j],
                        stiff2[j],
                        gauss_last_point
                    )
                )

            # lumped mass and inertia calculation
            if self.primary_axis == "x1":
                # global - local beam
                # x (x1) = x (x1)
                # y (x2) = y (x2)
                # z (x3) = z (x3)
                m1, cmx1, cmy1, cmz1, \
                    ixx1, iyy1, izz1, ixy1, ixz1, iyz1, \
                    m2_1, cmx2_1, cmy2_1, cmz2_1, \
                    ixx2_1, iyy2_1, izz2_1, ixy2_1, ixz2_1, iyz2_1 = self.mass_inertia_distribution(
                        node1.position.x1,
                        cm_offset1[0], cm_offset1[1],
                        node2.position.x1,
                        cm_offset2[0], cm_offset2[1],
                        mass1, mass2,
                        inertia1[0], inertia1[1], inertia1[2],
                        inertia1[3], inertia1[4], inertia1[5],
                        inertia2[0], inertia2[1], inertia2[2],
                        inertia2[3], inertia2[4], inertia2[5]
                    )

                m2_2, cmx2_2, cmy2_2, cmz2_2, \
                    ixx2_2, iyy2_2, izz2_2, ixy2_2, ixz2_2, iyz2_2, \
                    m3, cmx3, cmy3, cmz3, ixx3, iyy3, izz3, ixy3, ixz3, iyz3 = self.mass_inertia_distribution(
                        node2.position.x1,
                        cm_offset2[0], cm_offset2[1],
                        node3.position.x1,
                        cm_offset3[0], cm_offset3[1],
                        mass2, mass3,
                        inertia2[0], inertia2[1], inertia2[2],
                        inertia2[3], inertia2[4], inertia2[5],
                        inertia3[0], inertia3[1], inertia3[2],
                        inertia3[3], inertia3[4], inertia3[5]
                    )
                cmx1 -= node1.position.x1
                cmx2_1 -= node2.position.x1
                cmx2_2 -= node2.position.x1
                cmx3 -= node3.position.x1

            elif self.primary_axis == "x2":
                # permutate the indeces to pass the global coordinate into the beams reference frame
                # global - local beam
                # y (x2) = x (x1)
                # z (x3) = y (x2)
                # x (x1) = z (x3)
                #
                # in place of these => pass these
                # x y z => y z x
                # xy xz yz => yz xy xz
                m1, cmy1, cmz1, cmx1, \
                    iyy1, izz1, ixx1, iyz1, ixy1, ixz1, \
                    m2_1, cmy2_1, cmz2_1, cmx2_1, \
                    iyy2_1, izz2_1, ixx2_1, iyz2_1, ixy2_1, ixz2_1 = self.mass_inertia_distribution(
                        node1.position.x2,
                        cm_offset1[0], cm_offset1[1],
                        node2.position.x2,
                        cm_offset2[0], cm_offset2[1],
                        mass1, mass2,
                        inertia1[1], inertia1[2], inertia1[0],
                        inertia1[5], inertia1[3], inertia1[4],
                        inertia2[1], inertia2[2], inertia2[0],
                        inertia2[5], inertia2[3], inertia2[4]
                    )

                m2_2, cmy2_2, cmz2_2, cmx2_2, \
                    iyy2_2, izz2_2, ixx2_2, iyz2_2, ixy2_2, ixz2_2, \
                    m3, cmy3, cmz3, cmx3, \
                    iyy3, izz3, ixx3, iyz3, ixy3, ixz3 = self.mass_inertia_distribution(
                        node2.position.x2,
                        cm_offset2[0], cm_offset2[1],
                        node3.position.x2,
                        cm_offset3[0], cm_offset3[1],
                        mass2, mass3,
                        inertia2[1], inertia2[2], inertia2[0],
                        inertia2[5], inertia2[3], inertia2[4],
                        inertia3[1], inertia3[2], inertia3[0],
                        inertia3[5], inertia3[3], inertia3[4]
                    )
                cmy1 -= node1.position.x2
                cmy2_1 -= node2.position.x2
                cmy2_2 -= node2.position.x2
                cmy3 -= node3.position.x2

            elif self.primary_axis == "x3":
                # permutate the indeces to pass the global coordinate into the beams reference frame
                # global - local beam
                # z (x3) = x (x1)
                # x (x1) = y (x2)
                # y (x2) = z (x3)
                #
                # in place of these => pass these
                # x y z => z x y
                # xy xz yz => xz yz xy
                m1, cmz1, cmx1, cmy1, \
                    izz1, ixx1, iyy1, ixz1, iyz1, ixy1, \
                    m2_1, cmz2_1, cmx2_1, cmy2_1, \
                    izz2_1, ixx2_1, iyy2_1, ixz2_1, iyz2_1, ixy2_1 = self.mass_inertia_distribution(
                        node1.position.x3,
                        cm_offset1[0], cm_offset1[1],
                        node2.position.x3,
                        cm_offset2[0], cm_offset2[1],
                        mass1, mass2,
                        inertia1[2], inertia1[0], inertia1[1],
                        inertia1[4], inertia1[5], inertia1[3],
                        inertia2[2], inertia2[0], inertia2[1],
                        inertia2[4], inertia2[5], inertia2[3]
                    )

                m2_2, cmz2_2, cmx2_2, cmy2_2, \
                    izz2_2, ixx2_2, iyy2_2, ixz2_2, iyz2_2, ixy2_2, \
                    m3, cmz3, cmx3, cmy3, \
                    izz3, ixx3, iyy3, ixz3, iyz3, ixy3 = self.mass_inertia_distribution(
                        node2.position.x3,
                        cm_offset2[0], cm_offset2[1],
                        node3.position.x3,
                        cm_offset3[0], cm_offset3[1],
                        mass2, mass3,
                        inertia2[2], inertia2[0], inertia2[1],
                        inertia2[4], inertia2[5], inertia2[3],
                        inertia3[2], inertia3[0], inertia3[1],
                        inertia3[4], inertia3[5], inertia3[3]
                    )
                cmz1 -= node1.position.x3
                cmz2_1 -= node2.position.x3
                cmz2_2 -= node2.position.x3
                cmz3 -= node3.position.x3

            # create the Bodies and Beam3
            id_base = self.component_name + "_beam + "
            index = 10 * i
            body1 = Body(
                id_base + str(index + 0),
                node1,
                m1,
                addedmass1,
                Vec3(cmx1, cmy1, cmz1),
                ixx1, iyy1, izz1, ixy1, ixz1, iyz1
            )
            body2_1 = Body(
                id_base + str(index + 1),
                node2,
                m2_1,
                addedmass2,  # this is always 0
                Vec3(cmx2_1, cmy2_1, cmz2_1),
                ixx2_1, iyy2_1, izz2_1, ixy2_1, ixz2_1, iyz2_1
            )
            body2_2 = Body(
                id_base + str(index + 2),
                node2,
                m2_2,
                addedmass2,  # this is always 0
                Vec3(cmx2_2, cmy2_2, cmz2_2),
                ixx2_2, iyy2_2, izz2_2, ixy2_2, ixz2_2, iyz2_2
            )
            body3 = Body(
                id_base + str(index + 3),
                node3,
                m3,
                addedmass3,
                Vec3(cmx3, cmy3, cmz3),
                ixx3, iyy3, izz3, ixy3, ixz3, iyz3
            )
            bodies += [body1, body2_1, body2_2, body3]

            this_beam = Beam3(id_base + str(index),
                              node1, node2, node3,
                              body1, body2_1, body2_2, body3,
                              gauss_first_twist,
                              gauss_last_twist,
                              gauss_first_stiffness,
                              gauss_last_stiffness)
            beams.append(this_beam)

        return bodies, body_count, beams, beam_count

    def mass_inertia_distribution(self,
                                  x1,
                                  yg1, zg1,
                                  x2,
                                  yg2, zg2,
                                  m1, m2,
                                  ixx1, iyy1, izz1,
                                  ixy1, ixz1, iyz1,
                                  ixx2, iyy2, izz2,
                                  ixy2, ixz2, iyz2):
        """
        See the preprocessor documentation; in particular, section 0.5.
        inputs:
            x1: Float - location in the primary axis direction of the first node
            yg1: Float - center of mass offset in a secondary axis direction for the first node
            zg1: Float - center of mass offset in a secondary axis direction for the first node
            x2: Float - location in the primary axis direction of the second node
            yg2: Float - center of mass offset in a secondary axis direction for the second node
            zg2: Float - center of mass offset in a secondary axis direction for the second node
            m1: Float - mass per unit length of the first node
            m2: Float - mass per unit length of the second node
            ixx1: Float - inertia per unit length of the first node
            iyy1: Float - inertia per unit length of the first node
            izz1: Float - inertia per unit length of the first node
            ixy1: Float - inertia per unit length of the first node
            ixz1: Float - inertia per unit length of the first node
            iyz1: Float - inertia per unit length of the first node
            ixx2: Float - inertia per unit length of the second node
            iyy2: Float - inertia per unit length of the second node
            izz2: Float - inertia per unit length of the second node
            ixy2: Float - inertia per unit length of the second node
            ixz2: Float - inertia per unit length of the second node
            iyz2: Float - inertia per unit length of the second node

        outputs:
            M1: Float - lumped mass of the first node
            Xg: Float - center of mass in the primary axis direction of the first node
            Yg: Float - center of mass in the secondary axis direction of the first node
            Zg: Float - center of mass in the secondary axis direction of the first node
            Ixx1G: Float - lumped inertia of the first node
            Iyy1G: Float - lumped inertia of the first node
            Izz1G: Float - lumped inertia of the first node
            Ixy1G: Float - lumped inertia of the first node
            Ixz1G: Float - lumped inertia of the first node
            Iyz1G: Float - lumped inertia of the first node
            M2: Float - lumped mass of the second node
            Xg: Float - center of mass in the primary axis direction of the second node
            Yg: Float - center of mass in the secondary axis direction of the second node
            Zg: Float - center of mass in the secondary axis direction of the second node
            Ixx2G: Float - lumped inertia of the second node
            Iyy2G: Float - lumped inertia of the second node
            Izz2G: Float - lumped inertia of the second node
            Ixy2G: Float - lumped inertia of the second node
            Ixz2G: Float - lumped inertia of the second node
            Iyz2G: Float - lumped inertia of the second node
        """

        Lb = x2 - x1
        M = (m1 + m2) / 2.0 * Lb

        XgM = (Lb / 6.0) * ((2 * m2 + m1) * Lb + 3 * x1 * (m1 + m2))
        YgM = (Lb / 6.0) * ((2 * m1 + m2) * yg1 + (2 * m2 + m1) * yg2)
        ZgM = (Lb / 6.0) * ((2 * m1 + m2) * zg1 + (2 * m2 + m1) * zg2)

        M1 = (3 * m1 + m2) / 8 * Lb
        M2 = M - M1

        Ixx = (Lb / 12.0) \
            * (
                6 * (ixx1 + ixx2)
                + m1 * (3 * yg1**2 + 2 * yg1 * yg2 + yg2**2 +
                        3 * zg1**2 + 2 * zg1 * zg2 + zg2**2)
                + m2 * (yg1**2 + 2 * yg1 * yg2 + 3 * yg2**2 +
                        zg1**2 + 2 * zg1 * zg2 + 3 * zg2**2)
            )
        Iyy = (Lb / 12.0) \
            * (
                6 * (iyy1 + iyy2)
                + m1 * (Lb**2 + 4 * Lb * x1 + 6 * x1**2 +
                        3 * zg1**2 + 2 * zg1 * zg2 + zg2**2)
                + m2 * (3 * Lb**2 + 8 * Lb * x1 + 6 * x1**2 +
                        zg1**2 + 2 * zg1 * zg2 + 3 * zg2**2)
            )
        Izz = (Lb / 12.0) \
            * (
                6 * (izz1 + izz2)
                + m1 * (Lb**2 + 4 * Lb * x1 + 6 * x1**2 +
                        3 * yg1**2 + 2 * yg1 * yg2 + yg2**2)
                + m2 * (3 * Lb**2 + 8 * Lb * x1 + 6 * x1**2 +
                        yg1**2 + 2 * yg1 * yg2 + 3 * yg2**2)
            )
        Ixy = (Lb / 12.0) \
            * (
                6 * (ixy1 + ixy2)
                + m1 * (Lb * (yg1 + yg2) + 2 * x1 * (2 * yg1 + yg2))
                + m2 * (Lb * (yg1 + 3 * yg2) + 2 * x1 * (yg1 + 2 * yg2))
            )
        Ixz = (Lb / 12.0) \
            * (
                6 * (ixz1 + ixz2)
                + m1 * (Lb * (zg1 + zg2) + 2 * x1 * (2 * zg1 + zg2))
                + m2 * (Lb * (zg1 + 3 * zg2) + 2 * x1 * (zg1 + 2 * zg2))
            )
        Iyz = (Lb / 12.0) \
            * (
                6 * (iyz1 + iyz2)
                + m1 * (yg2 * (zg1 + zg2) + yg1 * (3 * zg1 + zg2))
                + m2 * (yg1 * (zg1 + zg2) + yg2 * (zg1 + 3 * zg2))
            )

        Ixx1 = (Lb / 192.0) \
            * (
                24 * (3 * ixx1 + ixx2)
                + m1 * (45 * yg1**2 + 22 * yg1 * yg2 + 5 * yg2**2 +
                        45 * zg1**2 + 22 * zg1 * zg2 + 5 * zg2**2)
                + m2 * (11 * yg1**2 + 10 * yg1 * yg2 + 3 * yg2**2 +
                        11 * zg1**2 + 10 * zg1 * zg2 + 3 * zg2**2)
            )
        Iyy1 = (Lb / 192.0) \
            * (
                24 * (3 * iyy1 + iyy2)
                + m1 * (5 * Lb**2 + 32 * Lb * x1 + 72 * x1**2 +
                        45 * zg1**2 + 22 * zg1 * zg2 + 5 * zg2**2)
                + m2 * (3 * Lb**2 + 16 * Lb * x1 + 24 * x1**2 +
                        11 * zg1**2 + 10 * zg1 * zg2 + 3 * zg2**2)
            )
        Izz1 = (Lb / 192.0) \
            * (
                24 * (3 * izz1 + izz2)
                + m1 * (5 * Lb**2 + 32 * Lb * x1 + 72 * x1**2 +
                        45 * yg1**2 + 22 * yg1 * yg2 + 5 * yg2**2)
                + m2 * (3 * Lb**2 + 16 * Lb * x1 + 24 * x1**2 +
                        11 * yg1**2 + 10 * yg1 * yg2 + 3 * yg2**2)
            )
        Ixy1 = (Lb / 192.0) \
            * (
                24 * (3 * ixy1 + ixy2)
                + m1 * (11 * Lb * yg1 + 56 * x1 * yg1 +
                        5 * Lb * yg2 + 16 * x1 * yg2)
                + m2 * (5 * Lb * yg1 + 16 * x1 * yg1 + 3 * Lb * yg2 + 8 * x1 * yg2)
            )
        Ixz1 = (Lb / 192.0) \
            * (
                24 * (3 * ixz1 + ixz2)
                + m1 * (11 * Lb * zg1 + 56 * x1 * zg1 +
                        5 * Lb * zg2 + 16 * x1 * zg2)
                + m2 * (5 * Lb * zg1 + 16 * x1 * zg1 + 3 * Lb * zg2 + 8 * x1 * zg2)
            )
        Iyz1 = (Lb / 192.0) \
            * (
                24 * (3 * iyz1 + iyz2)
                + m1 * (45 * yg1 * zg1 + 11 * yg2 * zg1 +
                        11 * yg1 * zg2 + 5 * yg2 * zg2)
                + m2 * (11 * yg1 * zg1 + 5 * yg2 * zg1 +
                        5 * yg1 * zg2 + 3 * yg2 * zg2)
            )

        Xg = XgM / M
        Yg = YgM / M
        Zg = ZgM / M

        Xg_star = x1 + (Lb / 3.0) * (2 * m1 + m2) / (3 * m1 + m2)
        Yg_star = ((7 * m1 + 2 * m2) * yg1 + (2 * m1 + m2) * yg2)  / (9 * m1 + 3 * m2)
        Zg_star = ((7 * m1 + 2 * m2) * zg1 + (2 * m1 + m2) * zg2)  / (9 * m1 + 3 * m2)

        R = -1 * np.array([Xg, Yg, Zg])
        R_star = -1 * np.array([Xg_star, Yg_star, Zg_star])
        R_prime = np.array(
            [
                Xg - Xg_star,
                Yg - Yg_star,
                Zg - Zg_star
            ]
        )

        Ixx1G = Ixx1 + M1 \
            * (
                np.sum(R_prime**2) - np.sum(R_star**2) \
                + R_star[0] * R_star[0] 
                - R_prime[0] * R_prime[0]
            )
        Iyy1G = Iyy1 + M1 \
            * (
                np.sum(R_prime**2) - np.sum(R_star**2)
                + R_star[1] * R_star[1]
                - R_prime[1] * R_prime[1]
            )
        Izz1G = Izz1 + M1 \
            * (
                np.sum(R_prime**2) - np.sum(R_star**2)
                + R_star[2] * R_star[2]
                - R_prime[2] * R_prime[2]
            )
        Ixy1G = Ixy1 + M1 \
            * (
                R_star[0] * R_star[1]
                - R_prime[0] * R_prime[1]
            )
        Ixz1G = Ixz1 + M1 \
            * (
                R_star[0] * R_star[2]
                - R_prime[0] * R_prime[2]
            )
        Iyz1G = Iyz1 + M1 \
            * (
                R_star[1] * R_star[2]
                - R_prime[1] * R_prime[2]
            )

        Ixx2G = Ixx - Ixx1G - M * (np.sum(R**2) - R[0] * R[0])
        Iyy2G = Iyy - Iyy1G - M * (np.sum(R**2) - R[1] * R[1])
        Izz2G = Izz - Izz1G - M * (np.sum(R**2) - R[2] * R[2])
        Ixy2G = Ixy - Ixy1G - M * (-1 * R[0] * R[1])
        Ixz2G = Ixz - Ixz1G - M * (-1 * R[0] * R[2])
        Iyz2G = Iyz - Iyz1G - M * (-1 * R[1] * R[2])

        return M1, \
            Xg, Yg, Zg, \
            Ixx1G, Iyy1G, Izz1G, \
            Ixy1G, Ixz1G, Iyz1G, \
            M2, \
            Xg, Yg, Zg, \
            Ixx2G, Iyy2G, Izz2G, \
            Ixy2G, Ixz2G, Iyz2G

    ### export routines ###

    def export_all(self, output_directory):
        self.export_node_file(output_directory)
        self.export_body_file(output_directory)
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

        output.write_line("# Beam elements for the {}".format(self.component_name))        
        output.write_empty_line()
        output.write_line("set: integer {}_beam = {};".format(self.component_name, self.mbdyn_ref_index))
        output.write_line("set: integer {}_body = {};".format(self.component_name, self.mbdyn_ref_index))
        output.write_empty_line()
        output.write_line("# *** beam elements ***")
        for beam in self.beams:
            output.write_line(str(beam))
            output.write_line(
                "include: \"{}.beam\";".format(self.component_name))
            output.write_empty_line()
        output.end()

    def export_body_file(self, output_directory):
        output = Output("{}/{}.body".format(output_directory, self.component_name))
        output.write_line("# Bodies for the {}".format(self.component_name))        
        output.write_empty_line()
        for body in self.bodies:
            output.write_line(str(body))
            output.write_empty_line()
        output.end()

    def export_beam_file(self, output_directory):
        output = Output("{}/{}.beam".format(output_directory, self.component_name))
        output.write_line("# Generic beam element properties for the beams")
        output.write_empty_line()
        output.write_line("# *** elastic properties ***")
        output.write_line("beam3:  current_beam,")
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
