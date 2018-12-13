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
                 Ixx, Ixy, Ixz,
                 Iyy, Iyz,
                 Izz):
        self.id = identifier
        self.node = node
        self.mass = mass
        self.Ixx = Ixx
        self.Ixy = Ixy
        self.Ixz = Ixz
        self.Iyy = Iyy
        self.Iyz = Iyz
        self.Izz = Izz

    def __str__(self):
        refid   = "body: {}, {},".format(self.id, self.node.name)
        mass    = "    {}".format(self.mass)
        ref     = "    reference, node, null,"
        inertia = "    mat3, {}, {}, {},".format(self.Ixx, self.Ixy, self.Ixz)
        inertia = "          {}, {}, {},".format(self.Ixy, self.Iyy, self.Iyz)
        inertia = "          {}, {}, {};".format(self.Ixz, self.Iyz, self.Izz)
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

        # lumped mass calculation
        self.m1, \
            self.cm_offx1, self.cm_offy1, self.cm_offz1, \
            self.ixx1, self.iyy1, self.izz1, \
            self.ixy1, self.ixz1, self.iyz1, \
            self.m2_1, \
            self.cm_offx2_1, self.cm_offy2_1, self.cm_offz2_1, \
            self.ixx2_1, self.iyy2_1, self.izz2_1, \
            self.ixy2_1, self.ixz2_1, self.iyz2_1 = beam_mass_distribution(
                self.node_first.position.x1, self.node_first.position.x2, self.node_first.position.x3,
                self.node_mid.position.x1, self.node_mid.position.x2, self.node_mid.position.x3,
                self.mass_first, self.mass_mid,
                self.cm_offset_first, self.cm_offset_mid,
                self.inertia_first[0], self.inertia_first[1], self.inertia_first[2],
                self.inertia_first[3], self.inertia_first[4], self.inertia_first[5],
                self.inertia_mid[0], self.inertia_mid[1], self.inertia_mid[2],
                self.inertia_mid[3], self.inertia_mid[4], self.inertia_mid[5],
                self.primary_direction
            )

        self.m2_2, \
            self.cm_offx2_2, self.cm_offy2_2, self.cm_offz2_2, \
            self.ixx2_2, self.iyy2_2, self.izz2_2, \
            self.ixy2_2, self.ixz2_2, self.iyz2_2, \
            self.m3, \
            self.cm_offx3, self.cm_offy3, self.cm_offz3, \
            self.ixx3, self.iyy3, self.izz3, \
            self.ixy3, self.ixz3, self.iyz3 = beam_mass_distribution(
                self.node_mid.position.x1, self.node_mid.position.x2, self.node_mid.position.x3,
                self.node_last.position.x1, self.node_last.position.x2, self.node_last.position.x3,
                self.mass_mid, self.mass_last,
                self.cm_offset_mid, self.cm_offset_last,
                self.inertia_mid[0], self.inertia_mid[1], self.inertia_mid[2],
                self.inertia_mid[3], self.inertia_mid[4], self.inertia_mid[5],
                self.inertia_last[0], self.inertia_last[1], self.inertia_last[2],
                self.inertia_last[3], self.inertia_last[4], self.inertia_last[5],
                self.primary_direction
            )

        self.m2 = self.m2_1 + self.m2_2
        self.cm_offx2 = (self.m2_1 * self.cm_offx2_1 + self.m2_2 * self.cm_offx2_2) / self.m2
        self.cm_offy2 = (self.m2_1 * self.cm_offy2_1 + self.m2_2 * self.cm_offy2_2) / self.m2
        self.cm_offz2 = (self.m2_1 * self.cm_offz2_1 + self.m2_2 * self.cm_offz2_2) / self.m2

        # if self.primary_direction == "x1":
        #     beam_length = self.node_last.position.x1 - self.node_first.position.x1
        #     self.cm_offx2 = beam_length / 2
        #     self.cm_offy2 = (self.m2_1 * self.cm_offy2_1 + self.m2_2 * self.cm_offy2_2) / self.m2
        #     self.cm_offz2 = (self.m2_1 * self.cm_offz2_1 + self.m2_2 * self.cm_offz2_2) / self.m2
        # elif self.primary_direction == "x2":
        #     beam_length = self.node_last.position.x2 - self.node_first.position.x2
        #     self.cm_offx2 = (self.m2_1 * self.cm_offx2_1 + self.m2_2 * self.cm_offx2_2) / self.m2
        #     self.cm_offy2 = beam_length / 2
        #     self.cm_offz2 = (self.m2_1 * self.cm_offz2_1 + self.m2_2 * self.cm_offz2_2) / self.m2
        # elif self.primary_direction == "x3":
        #     beam_length = self.node_last.position.x3 - self.node_first.position.x3
        #     self.cm_offx2 = (self.m2_1 * self.cm_offx2_1 + self.m2_2 * self.cm_offx2_2) / self.m2
        #     self.cm_offy2 = (self.m2_1 * self.cm_offz2_1 + self.m2_2 * self.cm_offz2_2) / self.m2
        #     self.cm_offz2 = beam_length / 2

        # self.ixx2_1 = self.ixx2_1 + self.m2_1 * beam_length**2
        # self.ixx2_2 = self.ixx2_2 + 3 * self.m2_2 * beam_length**2
        self.ixx2 = self.ixx2_1 + self.ixx2_2 + self.m2_1 * (self.cm_offy2_1**2 + self.cm_offz2_1**2) + self.m2_2 * (self.cm_offy2_2**2 + self.cm_offz2_2**2)
        self.iyy2 = self.iyy2_1 + self.iyy2_2 + self.m2_1 * (self.cm_offx2_1**2 + self.cm_offz2_1**2) + self.m2_2 * (self.cm_offx2_2**2 + self.cm_offz2_2**2)
        self.izz2 = self.izz2_1 + self.izz2_2 + self.m2_1 * (self.cm_offx2_1**2 + self.cm_offy2_1**2) + self.m2_2 * (self.cm_offx2_2**2 + self.cm_offy2_2**2)
        # self.ixy2 = self.ixy2_1 + self.ixy2_2
        # self.ixz2 = self.ixz2_1 + self.ixz2_2
        # self.iyz2 = self.iyz2_1 + self.iyz2_2

        # print("{:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}".format("Mass", "Cm1", "Cm2", "Cm3", "Ixx1", "Ixx2", "Ixx3"))
        # print("{:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4}".format(self.m1, self.cm_offx1, self.cm_offy1, self.cm_offz1, self.ixx1, self.iyy1, self.izz1))
        # print("{:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4}".format(self.m2_1, self.cm_offx2_1, self.cm_offy2_1, self.cm_offz2_1, self.ixx2_1, self.iyy2_1, self.izz2_1))
        # print("{:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4}".format(self.m2_2, self.cm_offx2_2, self.cm_offy2_2, self.cm_offz2_2, self.ixx2_2, self.iyy2_2, self.izz2_2))
        # print("{:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4} {:>8.4}".format(self.m3, self.cm_offx3, self.cm_offy3, self.cm_offz3, self.ixx3, self.iyy3, self.izz3))

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


def IntegralN(n1, n2, l1, l2):
    # N = integral(mx + b) = mx^2/2 + bx
    slope = (n2 - n1) / (l2 - l1)
    intercept = n1

    def eval_at(x):
        return slope * x**2 / 2 + intercept * x
    return eval_at(l2) - eval_at(l1)


def IntegralNx(n1, n2, l1, l2):
    # Nx = integral(m(x-a)^2 + b(x-a)) = integral(m(x-a)^2 + b(x-a))    m(x-a)^3/3 + max^2/2  b(x-a)^2/2
    slope = (n2 - n1) / (l2 - l1)
    intercept = n1

    def eval_at(x):
        return slope * x**3 / 3.0 + intercept * x**2 / 2.0
    return eval_at(l2) - eval_at(l1)


def IntegralNP(n1, n2, p1, p2, l1, l2):
    # NP = integral( (ax + b) * (cx + d) )
    #    = integral( acx^2 + (ad + bc) * x + bd )
    #    = acx^3/3 + (ad + bc) * x^2/2 + bdx
    a = (n2 - n1) / (l2 - l1)
    b = n1
    c = (p2 - p1) / (l2 - l1)
    d = p1

    def eval_at(x):
        return a * c * x**3 / 3 + (x**2 / 2) * (a * d + b * c) + b * d * x
    return eval_at(l2) - eval_at(l1)


def IntegralN_times_P2plusR2(n1, n2, p1, p2, r1, r2, l1, l2):
    # N*(P^2+R^2) = integral( (ax + b) * ((cx + d)^2 + (ex + f)^2) )
    #             = integral( (ax + b) * ((c^2 + e^2)x^2 + 2(cd + ef)x + d^2 + f^2) )
    a = (n2 - n1) / (l2 - l1)
    b = n1
    c = (p2 - p1) / (l2 - l1)
    d = p1
    e = (r2 - r1) / (l2 - l1)
    f = r1

    def eval_at(x):
        return x**4 * a * (c**2 + e**2) / 4 \
             + x**3 * a * 2 * (c * d + e * f) / 3 \
             + x**2 * a * (d**2 + f **2) / 2 \
             + x**3 * b * (c**2 + e**2) / 3 \
             + x**2 * b * 2 * (c * d + e * f) / 2 \
             + x * b * (d**2 + f ** 2)
    return eval_at(l2) - eval_at(l1)


def IntegralNPR(n1, n2, p1, p2, r1, r2, l1, l2):
    # N*P*R = integral( (ax + b) * (cx + d) * (ex + f) )
    a = (n2 - n1) / (l2 - l1)
    b = n1
    c = (p2 - p1) / (l2 - l1)
    d = p1
    e = (r2 - r1) / (l2 - l1)
    f = r1

    def eval_at(x):
        return x**4 * (a * c * e) / 4 \
             + x**3 * (a * c * f + a * d * e + b * c * e) / 3 \
             + x**2 * (a * d * f + b * c * f + b * d * e) / 2 \
             + x * (b * d * f)
    return eval_at(l2) - eval_at(l1)


def beam_mass_distribution(x1, y1, z1,
                           x2, y2, z2,
                           m_l1, m_l2,
                           cm_offset1, cm_offset2,
                           ixx_l1, iyy_l1, izz_l1,
                           ixx_l2, iyy_l2, izz_l2,
                           ixy_l1, ixz_l1, iyz_l1,
                           ixy_l2, ixz_l2, iyz_l2,
                           primary_direction):

    # move the input coordinates to 0, 0, 0
    # so that the calculation is relative to the element, not the kite
    x1, x2 = x1 - x1, x2 - x1
    y1, y2 = y1 - y1, y2 - y1
    z1, z2 = z1 - z1, z2 - z1

    # volume quanitities
    if primary_direction == "x1":
        a, b = x1, x2
        cmx_offset1, cmx_offset2 = 0, 0
        cmy_offset1, cmy_offset2 = cm_offset1[0], cm_offset2[0]
        cmz_offset1, cmz_offset2 = cm_offset1[1], cm_offset2[1]
    elif primary_direction == "x2":
        a, b = y1, y2
        cmx_offset1, cmx_offset2 = cm_offset1[0], cm_offset2[0]
        cmy_offset1, cmy_offset2 = 0, 0
        cmz_offset1, cmz_offset2 = cm_offset1[1], cm_offset2[1]
    elif primary_direction == "x3":
        a, b = z1, z2
        cmx_offset1, cmx_offset2 = cm_offset1[0], cm_offset2[0]
        cmy_offset1, cmy_offset2 = cm_offset1[1], cm_offset2[1]
        cmz_offset1, cmz_offset2 = 0, 0

    # element properties
    L = b - a
    M = IntegralN(m_l1, m_l2, a, b)

    # these centers of gravity are local to this element
    Xg = IntegralNP(m_l1, m_l2, x1 + cmx_offset1, x2 + cmx_offset2, a, b) / M
    Yg = IntegralNP(m_l1, m_l2, y1 + cmy_offset1, y2 + cmy_offset2, a, b) / M
    Zg = IntegralNP(m_l1, m_l2, z1 + cmz_offset1, z2 + cmz_offset2, a, b) / M
    
    Ixx = IntegralN(ixx_l1, ixx_l2, a, b) + IntegralN_times_P2plusR2(m_l1, m_l2, y1, y2, z1, z2, a, b)
    Iyy = IntegralN(iyy_l1, iyy_l2, a, b) + IntegralN_times_P2plusR2(m_l1, m_l2, x1, x2, z1, z2, a, b)
    Izz = IntegralN(izz_l1, izz_l2, a, b) + IntegralN_times_P2plusR2(m_l1, m_l2, x1, x2, y1, y2, a, b)

    # print("should be: ", 5**3 / 3)
    # print("{:>8} {:>8} {:>8}".format("I11", "I22", "I33"))
    # print("{:>8.4} {:>8.4} {:>8.4}".format(Ixx, Iyy, Izz))

    Ixy = IntegralN(ixy_l1, ixy_l2, b, a) + IntegralNPR(m_l1, m_l2, x1, x2, y1, y2, b, a)
    Ixz = IntegralN(ixz_l1, ixz_l2, b, a) + IntegralNPR(m_l1, m_l2, x1, x2, z1, z2, b, a)
    Iyz = IntegralN(iyz_l1, iyz_l2, b, a) + IntegralNPR(m_l1, m_l2, y1, y2, z1, z2, b, a)

    # nodal quantities
    if primary_direction == "x1":
        M2 = (Xg - x1) * M / L
        Xg1, Xg2 = 0.0, 0.0
        Yg1, Yg2 = Yg, Yg
        Zg1, Zg2 = Zg, Zg
    elif primary_direction == "x2":
        M2 = (Yg - y1) * M / L
        Xg1, Xg2 = Xg, Xg
        Yg1, Yg2 = 0.0, 0.0
        Zg1, Zg2 = Zg, Zg
    elif primary_direction == "x3":
        M2 = (Zg - z1) * M / L
        Xg1, Xg2 = Xg, Xg
        Yg1, Yg2 = Yg, Yg
        Zg1, Zg2 = 0.0, 0.0
    M1 = M - M2

    def _calculate_node_inertia_component(I11, I22, I33, M, L, m2, dCG1, dCG2, dCG3):
        # return (I11 - M * (dCG2**2 + dCG3**2)) / 2, \
        #        (I22 - M * dCG3**2 - M2 * L**2) / 2, \
        #        (I33 - M * dCG2**2 - M2 * L**2) / 2
        return I11 / 2, I22 / 2, I33 / 2

    if primary_direction == "x1":
        Ixx1, Iyy1, Izz1 = _calculate_node_inertia_component(Ixx, Iyy, Izz, M, L, M2, Xg, Yg, Zg)
    elif primary_direction == "x2":
        Iyy1, Ixx1, Izz1 = _calculate_node_inertia_component(Iyy, Ixx, Izz, M, L, M2, Yg, Xg, Zg)
    elif primary_direction == "x3":
        Izz1, Iyy1, Ixx1 = _calculate_node_inertia_component(Izz, Iyy, Ixx, M, L, M2, Zg, Yg, Xg)
    Ixx2, Iyy2, Izz2 = Ixx1, Iyy1, Izz1

    Ixy1 = 0 #(Ixy - M * x1 * y1) / 2
    Ixy2 = Ixy1
    Ixz1 = 0 #(Ixz - M * x1 * z1) / 2
    Ixz2 = Ixz1
    Iyz1 = 0 #(Iyz - M * y1 * z1) / 2
    Iyz2 = Iyz1
    Ixx1, Iyy1, Izz1 = 10.0, 10.0, 10.0
    Ixx2, Iyy2, Izz2 = 10.0, 10.0, 10.0
    return M1, \
        Xg1, Yg1, Zg1, \
        Ixx1, Iyy1, Izz1, \
        Ixy1, Ixz1, Iyz1, \
        M2, \
        Xg2, Yg2, Zg2, \
        Ixx2, Iyy2, Izz2, \
        Ixy2, Ixz2, Iyz2
