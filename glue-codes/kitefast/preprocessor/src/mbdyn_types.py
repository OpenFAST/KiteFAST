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
from scipy.spatial.transform import Rotation as R


class Vec3():
    """
    Vec3 is a three-component vector with overloaded mathematical operators.
    All operations support element-wise or scalar math. For example,
        Vec3(1, 2, 3) + Vec3(4, 5, 6) = Vec3(5, 7, 9)
        Vec3(1, 2, 3) + 3 = Vec3(4, 5, 6)
    """
    def __init__(self, x1, x2=None, x3=None):
        if isinstance(x1, list):
            self.x1, self.x2, self.x3 = [float(x) for x in x1]
        else:
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
        self.alpha = rotation_angles.x1  # roll
        self.beta = rotation_angles.x2   # pitch
        self.gamma = rotation_angles.x3  # yaw

        # Upper case XYZ are intrinsic rotations (body-reference-frame)
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_euler.html#scipy.spatial.transform.Rotation.from_euler
        rotation = R.from_euler("XYZ", [self.alpha, self.beta, self.gamma], degrees=True)
        self.rotation_matrix = rotation.as_dcm()
        self.row1 = ", ".join([str(n) for n in self.rotation_matrix[0]])
        self.row2 = ", ".join([str(n) for n in self.rotation_matrix[1]])
        self.row3 = ", ".join([str(n) for n in self.rotation_matrix[2]])

    def __str__(self):
        string = ""
        if self.row1 is not None:
            string += "{:8d},{},".format(1, self.row1)
        if self.row3 is not None:
            string += "{:8d},{}".format(3, self.row3)
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
                 added_mass,
                 cm_offset=Vec3(0.0, 0.0, 0.0),
                 Ixx=0.0,
                 Iyy=0.0,
                 Izz=0.0,
                 Ixy=0.0,
                 Ixz=0.0,
                 Iyz=0.0):
        self.id = identifier
        self.node = node
        self.mass = mass
        self.added_mass = added_mass
        self.cm_offset = cm_offset
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.Ixy = Ixy
        self.Ixz = Ixz
        self.Iyz = Iyz
        self.total_mass = self.mass + self.added_mass

    def __str__(self):
        refid    = "body: {}, {},".format(self.id, self.node.id)
        mass = "    {:8.3},".format(self.mass + self.added_mass)
        ref      = "    reference, node, {},".format(self.cm_offset)
        inertia1 = "    sym, {:8.3}, {:8.3}, {:8.3},".format(self.Ixx, self.Ixy, self.Ixz)
        inertia2 = "                   {:8.3}, {:8.3},".format(self.Iyy, self.Iyz)
        inertia3 = "                             {:8.3};".format(self.Izz)
        return "\n".join([refid, mass, ref, inertia1, inertia2, inertia3])


class Beam3():
    def __init__(self,
                 identifier,
                 node_first,
                 node_mid,
                 node_last,
                 body_first,
                 body_mid_first,
                 body_mid_last,
                 body_last,
                 gauss_first_twist,
                 gauss_last_twist,
                 gauss_first_stiffness,
                 gauss_last_stiffness
                ):
        self.id = identifier
        self.node_first = node_first
        self.node_mid = node_mid
        self.node_last = node_last
        self.gauss_first_twist = gauss_first_twist
        self.gauss_last_twist = gauss_last_twist
        self.gauss_first_stiffness = gauss_first_stiffness
        self.gauss_last_stiffness = gauss_last_stiffness
        
    def __str__(self):
        ks = ["_11", "_12", "_13", "_14", "_15", "_16",
                     "_22", "_23", "_24", "_25", "_26",
                            "_33", "_34", "_35", "_36",
                                   "_44", "_45", "_46",
                                          "_55", "_56",
                                                 "_66"]
        refid = "set: current_beam = {};".format(self.id)
        twist1 = "set: twist1 = {};".format(self.gauss_first_twist)
        twist2 = "set: twist2 = {};".format(self.gauss_last_twist)
        node1 = "set: beam_node1 = {};".format(self.node_first.id)
        node2 = "set: beam_node2 = {};".format(self.node_mid.id)
        node3 = "set: beam_node3 = {};".format(self.node_last.id)
        stiffness1, stiffness2 = "", ""
        for i in range(21):
            stiffness1 += "set: k1{} = {};\n".format(ks[i], self.gauss_first_stiffness[i])
            stiffness2 += "set: k2{} = {};\n".format(ks[i], self.gauss_last_stiffness[i])
        return "\n".join(
            [
                refid,
                twist1, twist2,
                node1, node2, node3,
                stiffness1, stiffness2
            ]
        )

def cos(angle):
    return np.cos(angle)

def sin(angle):
    return np.sin(angle)
