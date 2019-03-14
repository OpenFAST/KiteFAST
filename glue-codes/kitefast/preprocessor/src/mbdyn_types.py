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

import numpy as np


class Vec3():
    def __init__(self, x1, x2, x3):
        self.x1 = float(x1)
        self.x2 = float(x2)
        self.x3 = float(x3)

    def __str__(self):
        return "{:8.3f},{:8.3f},{:8.3f}".format(self.x1, self.x2, self.x3)

    def __add__(self, arg):
        if type(arg) is Vec3:
            return Vec3(self.x1 + arg.x1,
                        self.x2 + arg.x2,
                        self.x3 + arg.x3)
        else:
            return Vec3(self.x1 + arg,
                        self.x2 + arg,
                        self.x3 + arg)

    def __sub__(self, arg):
        if type(arg) is Vec3:
            return Vec3(self.x1 - arg.x1,
                        self.x2 - arg.x2,
                        self.x3 - arg.x3)
        else:
            return Vec3(self.x1 - arg,
                        self.x2 - arg,
                        self.x3 - arg)

    def __mul__(self, arg):
        if type(arg) is Vec3:
            return Vec3(self.x1 * arg.x1,
                        self.x2 * arg.x2,
                        self.x3 * arg.x3)
        else:
            return Vec3(self.x1 * arg,
                        self.x2 * arg,
                        self.x3 * arg)

    def __truediv__(self, arg):
        if type(arg) is Vec3:
            return Vec3(self.x1 / arg.x1,
                        self.x2 / arg.x2,
                        self.x3 / arg.x3)
        else:
            return Vec3(self.x1 / arg,
                        self.x2 / arg,
                        self.x3 / arg)

    def __eq__(self, arg):
        return self.x1 == arg.x1 \
            and self.x2 == arg.x2 \
            and self.x3 == arg.x3


class OrientationMatrix():
    def __init__(self, rotation_angles):
        self.alpha = np.deg2rad(rotation_angles.x1)  # roll
        self.beta = np.deg2rad(rotation_angles.x2)  # pitch
        self.gamma = np.deg2rad(rotation_angles.x3)  # yaw
        self.rotation_matrix = self.generalized_rotation(self.alpha, self.beta, self.gamma)
        self.row1 = ", ".join([str(n) for n in self.rotation_matrix[0]])
        self.row2 = ", ".join([str(n) for n in self.rotation_matrix[1]])
        self.row3 = None

    def generalized_rotation(self, alpha, beta, gamma):
        def _roll(angle):
            return np.array(
                [
                    [1,             0,                  0],
                    [0, np.cos(angle), -1 * np.sin(angle)],
                    [0, np.sin(angle),      np.cos(angle)]
                ]
            )
        def _pitch(angle):
            return np.array(
                [
                    [np.cos(angle),  0, -1 * np.sin(angle)],
                    [0,  1,                  0],
                    [np.sin(angle),  0,      np.cos(angle)]
                ]
            )
        def _yaw(angle):
            return np.array(
                [
                    [np.cos(angle), -1 * np.sin(angle),  0],
                    [np.sin(angle),      np.cos(angle),  0],
                    [0,                  0,  1]
                ]
            )

        return _roll(alpha) * _pitch(beta) * _yaw(gamma)
        
    def __str__(self):
        string = ""
        if self.row1 is not None:
            string += "{:8d},{},".format(1, self.row1)
        if self.row2 is not None:
            string += "{:8d},{}".format(2, self.row2)
        return string


class ReferenceFrame():
    def __init__(self,
                 name,
                 reference,
                 absolute_position,
                 absolute_orientation_matrix="null",
                 absolute_velocity="null",
                 absolute_angular_velocity="null"):
        self.name = name
        self.reference = reference
        self.absolute_position = absolute_position
        self.absolute_orientation_matrix = absolute_orientation_matrix
        self.absolute_velocity = absolute_velocity
        self.absolute_angular_velocity = absolute_angular_velocity

    def __str__(self):
        namestring = "reference: {},".format(self.name)
        posstring = "        reference, {},{},".format(self.reference, self.absolute_position)
        orientationstring = "        reference, {},{},".format(self.reference, self.absolute_orientation_matrix)
        velstring = "        reference, {},{},".format(self.reference, self.absolute_velocity)
        omegastring = "        reference, {},{};".format(self.reference, self.absolute_angular_velocity)
        return "\n".join([namestring, posstring, orientationstring, velstring, omegastring])


class TotalJoint():
    def __init__(self,
                 identifier,
                 node1,
                 node2):
        self.id = identifier
        self.node1 = node1
        self.node2 = node2
        self.offset = node2.position - node1.position

    def __str__(self):
        refid = "    joint: {}, total joint,".format(self.id)
        node1     = "        {},".format(self.node1.id)
        node2     = "        {},".format(self.node2.id)
        constant  = "\n".join([
            "            position, reference, other node, null,",
            "            rotation orientation, reference, other node, eye,",
            "        position constraint,",
            "            active, active, active,",
            # "            component, {},".format(str(self.offset)),
            "            null,",
            "        orientation constraint,",
            "            active, active, active,",
            "            null;"
        ])
        return "\n".join([refid, node1, node2, constant])


class RevoluteHinge():
    def __init__(self,
                 identifier,
                 node1,
                 node2):
        self.id = identifier
        self.node1 = node1
        self.node2 = node2
        self.offset = node2.position - node1.position

    def __str__(self):
        refid = "    joint: {}, revolute hinge,".format(self.id)
        node1 = "\n".join([
            "        {},".format(self.node1.id),
            "            position, reference, {}, null,".format(self.node2.reference_frame),
            "            orientation, reference, {},".format(self.node2.reference_frame),
            "                2, 0.0, 1.0, 0.0,",
            "                3, 1.0, 0.0, 0.0,"
        ])
        node2 = "\n".join([
            "        {},".format(self.node2.id),
            "            position, reference, {}, null,".format(self.node2.reference_frame),
            "            orientation, reference, {},".format(self.node2.reference_frame),
            "                2, 0.0, 1.0, 0.0,",
            "                3, 1.0, 0.0, 0.0,"
        ])
        constant = "        initial theta, 0.0;"
        return "\n".join([refid, node1, node2, constant])


class StructuralNode():
    def __init__(self,
                 parent_component,
                 root_offset_index,
                 node_type="dynamic",
                 reference_frame="mip_rf",
                 position=None,
                 orientation="eye",
                 velocity=Vec3(0.0, 0.0, 0.0),
                 angular_velocity=Vec3(0.0, 0.0, 0.0)):
        self.parent_component = parent_component
        self.root_offset_index = root_offset_index
        self.id = self.parent_component + "_root_node + " + str(self.root_offset_index)
        self.node_type = node_type
        self.reference_frame = reference_frame
        self.position = position
        self.orientation = orientation
        self.velocity = velocity
        self.angular_velocity = angular_velocity

    def __str__(self):
        refid            = "structural: {}, dynamic,".format(self.id)
        position         = "    reference, {},{},".format(self.reference_frame, self.position)
        orientation      = "    reference, {},{},".format(self.reference_frame, self.orientation)
        velocity         = "    reference, {},{},".format(self.reference_frame, self.velocity)
        angular_velocity = "    reference, {},{};".format(self.reference_frame, self.angular_velocity)
        return "\n".join([refid, position, orientation, velocity, angular_velocity])


class Body():
    def __init__(self,
                 identifier,
                 node,
                 mass,
                 cm_offset=Vec3(0.0, 0.0, 0.0),
                 Ixx=0.0,
                 Iyy=0.0,
                 Izz=0.0):
        self.id = identifier
        self.node = node
        self.mass = mass
        self.cm_offset = cm_offset
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz

    def __str__(self):
        refid   = "body: {}, {},".format(self.id, self.node.id)
        mass    = "    {},".format(self.mass)
        ref     = "    reference, node, {},".format(self.cm_offset)
        inertia = "    diag, {}, {}, {};".format(self.Ixx, self.Iyy, self.Izz)
        return "\n".join([refid, mass, ref, inertia])


class Beam3():
    def __init__(self,
                 identifier,
                 primary_direction,
                 node_first,
                 node_mid,
                 node_last,
                 mass_first,
                 mass_mid,
                 mass_last,
                 point_mass_first,
                 point_mass_mid,
                 point_mass_last,
                 inertia_first,
                 inertia_mid,
                 inertia_last,
                 cm_offset_first,
                 cm_offset_mid,
                 cm_offset_last,
                 twist_first,
                 twist_last,
                 stiff_first,
                 stiff_last,
                 interpolator):

        self.id = identifier
        self.primary_direction = primary_direction
        self.node_first = node_first
        self.node_mid = node_mid
        self.node_last = node_last
        self.mass_first = mass_first
        self.mass_mid = mass_mid
        self.mass_last = mass_last
        self.point_mass_first = point_mass_first
        self.point_mass_mid = point_mass_mid
        self.point_mass_last = point_mass_last
        self.cm_offset_first = cm_offset_first
        self.cm_offset_mid = cm_offset_mid
        self.cm_offset_last = cm_offset_last
        self.twist_first = twist_first
        self.twist_last = twist_last
        self.stiff_first = stiff_first
        self.stiff_last = stiff_last
        self.inertia_first = inertia_first
        self.inertia_mid = inertia_mid
        self.inertia_last = inertia_last

        # derived attributes
        self.length = np.sqrt(
            np.power(self.node_first.position.x1 - self.node_last.position.x1, 2)
            + np.power(self.node_first.position.x2 - self.node_last.position.x2, 2)
            + np.power(self.node_first.position.x3 - self.node_last.position.x3, 2))

        self.unit_vector = Vec3(
            self.node_last.position.x1 - self.node_first.position.x1,
            self.node_last.position.x2 - self.node_first.position.x2,
            self.node_last.position.x3 - self.node_first.position.x3) / self.length

        # gauss point locations
        gauss_first_displacement = (self.length / 2.0) * (1 - 1 / np.sqrt(3))
        gauss_last_displacement = (self.length / 2.0) * (1 + 1 / np.sqrt(3))

        gauss_first_point = (self.node_first.position + gauss_first_displacement) * self.unit_vector
        gauss_last_point = (self.node_first.position + gauss_last_displacement) * self.unit_vector

        # guass point twist
        self.gauss_first_twist = interpolator(self.node_first.position, self.node_last.position,
                                              self.twist_first, self.twist_last,
                                              gauss_first_point)
        self.gauss_last_twist = interpolator(self.node_first.position, self.node_last.position,
                                             self.twist_first, self.twist_last,
                                             gauss_last_point)

        # guass point stiffness
        self.gauss_first_stiffness, self.gauss_last_stiffness = [], []
        for i in range(21):
            self.gauss_first_stiffness.append(
                interpolator(self.node_first.position, self.node_last.position,
                             self.stiff_first[i], self.stiff_last[i],
                             gauss_first_point)
            )
            self.gauss_last_stiffness.append(
                interpolator(self.node_first.position, self.node_last.position,
                             self.stiff_first[i], self.stiff_last[i],
                             gauss_last_point)
            )

        # lumped mass and inertia calculation
        if self.primary_direction == "x1":
            # global - local beam
            # x (x1) = x (x1)
            # y (x2) = y (x2)
            # z (x3) = z (x3)
            self.m1, \
            self.cmx1, self.cmy1, self.cmz1, \
            self.ixx1, self.iyy1, self.izz1, \
            self.ixy1, self.ixz1, self.iyz1, \
            self.m2_1, \
            self.cmx2_1, self.cmy2_1, self.cmz2_1, \
            self.ixx2_1, self.iyy2_1, self.izz2_1, \
            self.ixy2_1, self.ixz2_1, self.iyz2_1 = beam_mass_distribution(
                self.node_first.position.x1, self.node_first.position.x2, self.node_first.position.x3,
                self.node_mid.position.x1, self.node_mid.position.x2, self.node_mid.position.x3,
                self.mass_first, self.mass_mid,
                self.inertia_first[0], self.inertia_first[1], self.inertia_first[2],
                self.inertia_first[3], self.inertia_first[4], self.inertia_first[5],
                self.inertia_mid[0], self.inertia_mid[1], self.inertia_mid[2],
                self.inertia_mid[3], self.inertia_mid[4], self.inertia_mid[5]
            )

            self.m2_2, \
            self.cmx2_2, self.cmy2_2, self.cmz2_2, \
            self.ixx2_2, self.iyy2_2, self.izz2_2, \
            self.ixy2_2, self.ixz2_2, self.iyz2_2, \
            self.m3, \
            self.cmx3, self.cmy3, self.cmz3, \
            self.ixx3, self.iyy3, self.izz3, \
            self.ixy3, self.ixz3, self.iyz3 = beam_mass_distribution(
                self.node_mid.position.x1, self.node_mid.position.x2, self.node_mid.position.x3,
                self.node_last.position.x1, self.node_last.position.x2, self.node_last.position.x3,
                self.mass_mid, self.mass_last,
                self.inertia_mid[0], self.inertia_mid[1], self.inertia_mid[2],
                self.inertia_mid[3], self.inertia_mid[4], self.inertia_mid[5],
                self.inertia_last[0], self.inertia_last[1], self.inertia_last[2],
                self.inertia_last[3], self.inertia_last[4], self.inertia_last[5]
            )

        elif self.primary_direction == "x2":
            # permutate the indeces to pass the global coordinate into the beams reference frame
            # global - local beam
            # y (x2) = x (x1)
            # z (x3) = y (x2)
            # x (x1) = z (x3)
            #
            # in place of these => pass these
            # x y z => y z x
            # xy xz yz => yz xy xz
            self.m1, \
            self.cmy1, self.cmz1, self.cmx1, \
            self.iyy1, self.izz1, self.ixx1, \
            self.iyz1, self.ixy1, self.ixz1, \
            self.m2_1, \
            self.cmy2_1, self.cmz2_1, self.cmx2_1, \
            self.iyy2_1, self.izz2_1, self.ixx2_1, \
            self.iyz2_1, self.ixy2_1, self.ixz2_1 = beam_mass_distribution(
                self.node_first.position.x2, self.node_first.position.x3, self.node_first.position.x1,
                self.node_mid.position.x2, self.node_mid.position.x3, self.node_mid.position.x1,
                self.mass_first, self.mass_mid,
                self.inertia_first[1], self.inertia_first[2], self.inertia_first[0],
                self.inertia_first[5], self.inertia_first[3], self.inertia_first[4],
                self.inertia_mid[1], self.inertia_mid[2], self.inertia_mid[0],
                self.inertia_mid[5], self.inertia_mid[3], self.inertia_mid[4]
            )

            self.m2_2, \
            self.cmy2_2, self.cmz2_2, self.cmx2_2, \
            self.iyy2_2, self.izz2_2, self.ixx2_2, \
            self.iyz2_2, self.ixy2_2, self.ixz2_2, \
            self.m3, \
            self.cmy3, self.cmz3, self.cmx3, \
            self.iyy3, self.izz3, self.ixx3, \
            self.iyz3, self.ixy3, self.ixz3 = beam_mass_distribution(
                self.node_mid.position.x2, self.node_mid.position.x3, self.node_mid.position.x1,
                self.node_last.position.x2, self.node_last.position.x3, self.node_last.position.x1,
                self.mass_mid, self.mass_last,
                self.inertia_mid[1], self.inertia_mid[2], self.inertia_mid[0],
                self.inertia_mid[5], self.inertia_mid[3], self.inertia_mid[4],
                self.inertia_last[1], self.inertia_last[2], self.inertia_last[0],
                self.inertia_last[5], self.inertia_last[3], self.inertia_last[4]
            )

        elif self.primary_direction == "x3":
            # permutate the indeces to pass the global coordinate into the beams reference frame
            # global - local beam
            # z (x3) = x (x1)
            # x (x1) = y (x2)
            # y (x2) = z (x3)
            # 
            # in place of these => pass these
            # x y z => z x y
            # xy xz yz => xz yz xy
            self.m1, \
            self.cmz1, self.cmx1, self.cmy1, \
            self.izz1, self.ixx1, self.iyy1, \
            self.ixz1, self.iyz1, self.ixy1, \
            self.m2_1, \
            self.cmz2_1, self.cmx2_1, self.cmy2_1, \
            self.izz2_1, self.ixx2_1, self.iyy2_1, \
            self.ixz2_1, self.iyz2_1, self.ixy2_1 = beam_mass_distribution(
                self.node_first.position.x3, self.node_first.position.x1, self.node_first.position.x2,
                self.node_mid.position.x3, self.node_mid.position.x1, self.node_mid.position.x2,
                self.mass_first, self.mass_mid,
                self.inertia_first[2], self.inertia_first[0], self.inertia_first[1],
                self.inertia_first[4], self.inertia_first[5], self.inertia_first[3],
                self.inertia_mid[2], self.inertia_mid[0], self.inertia_mid[1],
                self.inertia_mid[4], self.inertia_mid[5], self.inertia_mid[3]
            )

            self.m2_2, \
            self.cmz2_2, self.cmx2_2, self.cmy2_2, \
            self.izz2_2, self.ixx2_2, self.iyy2_2, \
            self.ixz2_2, self.iyz2_2, self.ixy2_2, \
            self.m3, \
            self.cmz3, self.cmx3, self.cmy3, \
            self.izz3, self.ixx3, self.iyy3, \
            self.ixz3, self.iyz3, self.ixy3 = beam_mass_distribution(
                self.node_mid.position.x3, self.node_mid.position.x1, self.node_mid.position.x2,
                self.node_last.position.x3, self.node_last.position.x1, self.node_last.position.x2,
                self.mass_mid, self.mass_last,
                self.inertia_mid[2], self.inertia_mid[0], self.inertia_mid[1],
                self.inertia_mid[4], self.inertia_mid[5], self.inertia_mid[3],
                self.inertia_last[2], self.inertia_last[0], self.inertia_last[1],
                self.inertia_last[4], self.inertia_last[5], self.inertia_last[3]
            )

        # add the lumped masses to the nodal masses
        self.m1 += self.point_mass_first
        self.m2 += self.point_mass_mid
        self.m3 += self.point_mass_last

    def __str__(self):
        ks = ["_11", "_12", "_13", "_14", "_15", "_16",
                     "_22", "_23", "_24", "_25", "_26",
                            "_33", "_34", "_35", "_36",
                                   "_44", "_45", "_46",
                                          "_55", "_56",
                                                 "_66"]
        refid = "set: curr_beam = {};".format(self.id)
        beamref1 = "set: beam_ref1 = mip_rf;"
        beamref2 = "set: beam_ref2 = mip_rf;"
        twist1 = "set: twist1 = {};".format(self.gauss_first_twist)
        twist2 = "set: twist2 = {};".format(self.gauss_last_twist)
        node1 = "set: beam_node1 = {};".format(self.node_first.id)
        node2 = "set: beam_node2 = {};".format(self.node_mid.id)
        node3 = "set: beam_node3 = {};".format(self.node_last.id)
        mass1 = "set: m1 = {};".format(self.m1)
        mass2 = "set: m2 = {};".format(self.m2)
        mass3 = "set: m3 = {};".format(self.m3)
        cmoffx1 = "set: cm_offx1 = {};".format(self.cm_offx1)
        cmoffy1 = "set: cm_offy1 = {};".format(self.cm_offy1)
        cmoffz1 = "set: cm_offz1 = {};".format(self.cm_offz1)
        cmoffx2 = "set: cm_offx2 = {};".format(self.cm_offx2)
        cmoffy2 = "set: cm_offy2 = {};".format(self.cm_offy2)
        cmoffz2 = "set: cm_offz2 = {};".format(self.cm_offz2)
        cmoffx3 = "set: cm_offx3 = {};".format(self.cm_offx3)
        cmoffy3 = "set: cm_offy3 = {};".format(self.cm_offy3)
        cmoffz3 = "set: cm_offz3 = {};".format(self.cm_offz3)
        ixx1 = "set: Ixx1 = {};".format(self.ixx1)
        ixx2 = "set: Ixx2 = {};".format(self.ixx2)
        ixx3 = "set: Ixx3 = {};".format(self.ixx3)
        iyy1 = "set: Iyy1 = {};".format(self.iyy1)
        iyy2 = "set: Iyy2 = {};".format(self.iyy2)
        iyy3 = "set: Iyy3 = {};".format(self.iyy3)
        izz1 = "set: Izz1 = {};".format(self.izz1)
        izz2 = "set: Izz2 = {};".format(self.izz2)
        izz3 = "set: Izz3 = {};".format(self.izz3)
        stiffness1, stiffness2 = "", ""
        for i in range(21):
            stiffness1 += "set: k1{} = {};\n".format(ks[i], self.gauss_first_stiffness[i])
            stiffness2 += "set: k2{} = {};\n".format(ks[i], self.gauss_last_stiffness[i])
        l_beam = "set: L_beam = {};".format(self.length)
        return "\n".join([refid, l_beam,
                          beamref1, beamref2,
                          twist1, twist2,
                          node1, node2, node3,
                          mass1, mass2, mass3,
                          cmoffx1, cmoffy1, cmoffz1,
                          cmoffx2, cmoffy2, cmoffz2,
                          cmoffx3, cmoffy3, cmoffz3,
                          ixx1, ixx2, ixx3,
                          iyy1, iyy2, iyy3,
                          izz1, izz2, izz3,
                          stiffness1, stiffness2
                          ])

def beam_mass_distribution(x1, y1, z1,
                           x2, y2, z2,
                           m1, m2,
                           ixx1, iyy1, izz1,
                           ixy1, ixz1, iyz1,
                           ixx2, iyy2, izz2,
                           ixy2, ixz2, iyz2):

    Lb = x2 - x1
    M = (m1 + m2) / 2.0 * Lb

    XgM = (Lb / 6.0) * ((2 * m2 + m1) * Lb + 3 * x1 * (m1 + m2))
    YgM = (Lb / 6.0) * ((2 * m1 + m2) * y1 + (2 * m2 + m1) * y2)
    ZgM = (Lb / 6.0) * ((2 * m1 + m2) * z1 + (2 * m2 + m1) * z2)

    M1 = (3 * m1 + m2) / 8 * Lb
    M2 = M - M1

    Ixx = (Lb / 12.0) \
        * (
            6 * (ixx1 + ixx2)
            + m1 * (3 * y1**2 + 2 * y1 * y2 + y2**2 + 3 * z1**2 + 2 * z1 * z2 + z2**2)
            + m2 * (y1**2 + 2 * y1 * y2 + 3 * y2**2 + z1**2 + 2 * z1 * z2 + 3 * z2**2)
        )
    Iyy = (Lb / 12.0) \
        * (
            6 * (iyy1 + iyy2)
            + m1 * (Lb**2 + 4 * Lb * x1 + 6 * x1**2 + 3 * z1**2 + 2 * z1 * z2 + z2**2)
            + m2 * (3 * Lb**2 + 8 * Lb * x1 + 6 * x1**2 + z1**2 + 2 * z1 * z2 + 3 * z2**2)
        )
    Izz = (Lb / 12.0) \
        * (
            6 * (izz1 + izz2)
            + m1 * (Lb**2 + 4 * Lb * x1 + 6 * x1**2 + 3 * y1**2 + 2 * y1 * y2 + y2**2)
            + m2 * (3 * Lb**2 + 8 * Lb * x1 + 6 * x1**2 + y1**2 + 2 * y1 * y2 + 3 * y2**2)
        )
    Ixy = (Lb / 12.0) \
        * (
            6 * (ixy1 + ixy2)
            + m1 * (Lb * (y1 + y2) + 2 * x1 * (2 * y1 + y2))
            + m2 * (Lb * (y1 + 3 * y2) + 2 * x1 * (y1 + 2 * y2))
        )
    Ixz = (Lb / 12.0) \
        * (
            6 * (ixz1 + ixz2)
            + m1 * (Lb * (z1 + z2) + 2 * x1 * (2 * z1 + z2))
            + m2 * (Lb * (z1 + 3 * z2) + 2 * x1 * (z1 + 2 * z2))
        )
    Iyz = (Lb / 12.0) \
        * (
            6 * (iyz1 + iyz2)
            + m1 * (y2 * (z1 + z2) + y1 * (3 * z1 + z2))
            + m2 * (y1 * (z1 + z2) + y2 * (z1 + 3 * z2))
    )

    Ixx1 = (Lb / 192) \
        * (
            24 * (3 * ixx1 + ixx2)
            + m1 * (45 * y1**2 + 22 * y1 * y2 + 5 * y2**2 + 45 * z1**2 + 22 * z1 * z2 + 5 * z2**2)
            + m2 * (11 * y1**2 + 10 * y1 * y2 + 3 * y2**2 + 11 * z1**2 + 10 * z1 * z2 + 3 * z2**2)
        )

    Iyy1 = (Lb / 192) \
        * (
            24 * (3 * iyy1 + iyy2)
            + m1 * (5 * Lb**2 + 32 * Lb * x1 + 72 * x1**2 + 45 * z1**2 + 22 * z1 * z2 + 5 * z2**2)
            + m2 * (3 * Lb**2 + 16 * Lb * x1 + 24 * x1**2 + 11 * z1**2 + 10 * z1 * z2 + 3 * z2**2)
        )

    Izz1 = (Lb / 192) \
        * (
            24 * (3 * izz1 + izz2)
            + m1 * (5 * Lb**2 + 32 * Lb * x1 + 72 * x1**2 + 45 * y1**2 + 22 * y1 * y2 + 5 * y2**2)
            + m2 * (3 * Lb**2 + 16 * Lb * x1 + 24 * x1**2 + 11 * y1**2 + 10 * y1 * y2 + 3 * y2**2)
        )

    Ixy1 = (Lb / 192) \
        * (
            24 * (3 * ixy1 + ixy2)
            + m1 * (11 * Lb * y1 + 56 * x1 * y1 + 5 * Lb * y2 + 16 * x1 * y2)
            + m2 * (5 * Lb * y1 + 16 * x1 * y1 + 3 * Lb * y2 + 8 * x1 * y2)
        )

    Ixz1 = (Lb / 192) \
        * (
            24 * (3 * ixz1 + ixz2)
            + m1 * (11 * Lb * z1 + 56 * x1 * z1 + 5 * Lb * z2 + 16 * x1 * z2)
            + m2 * (5 * Lb * z1 + 16 * x1 * z1 + 3 * Lb * z2 + 8 * x1 * z2)
        )
    
    Iyz1 = (Lb / 192) \
        * (
            24 * (3 * iyz1 + iyz2)
            + m1 * (45 * y1 * z1 + 11 * y2 * z1 + 11 * y1 * z2 + 5 * y2 * z2)
            + m2 * (11 * y1 * z1 + 5 * y2 * z1 + 5 * y1 * z2 + 3 * y2 * z2)
        )

    Xg = XgM / M
    Yg = YgM / M
    Zg = ZgM / M
    R = -1 * np.array([Xg, Yg, Zg])
    
    Ixx1G = Ixx1 - M1 * (np.sum(R**2) - R[0] * R[0])
    Iyy1G = Iyy1 - M1 * (np.sum(R**2) - R[1] * R[1])
    Izz1G = Izz1 - M1 * (np.sum(R**2) - R[2] * R[2])
    Ixy1G = Ixy1 - M1 * (-1 * R[0] * R[1])
    Ixz1G = Ixz1 - M1 * (-1 * R[0] * R[2])
    Iyz1G = Iyz1 - M1 * (-1 * R[1] * R[2])

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
