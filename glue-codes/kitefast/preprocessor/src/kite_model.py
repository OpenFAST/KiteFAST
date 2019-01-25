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

from .base_model import BaseModel
from .base_model import ModelException
from .main_mbd import MainMBD
from .main_set import MainSet
from .components import Fuselage
from .components import Wing
from .components import VerticalStabilizer
from .components import HorizontalStabilizer
from .components import VerticalStabilizer
from .components import Pylon
from .components import RotorAssembly
from .iohandler import Output
from .mbdyn_types import Vec3
from .mbdyn_types import OrientationMatrix
from .mbdyn_types import ReferenceFrame
from .mbdyn_types import TotalJoint, RevoluteHinge


class KiteModel(BaseModel):
    def __init__(self, simulation_dict, model_dict):
        super().__init__()

        # constants
        self.title = simulation_dict["title"]
        initial_conditions = simulation_dict["initial_conditions"]
        self.mip_reference_frame = ReferenceFrame(
            name="mip_rf",
            reference="global",
            absolute_position=initial_conditions["position"],
            absolute_orientation_matrix=OrientationMatrix(initial_conditions["orientation"]),
            absolute_velocity=initial_conditions["translational_velocity"],
            absolute_angular_velocity=initial_conditions["rotational_velocity"],
        )

        # unpack and derive model constants
        number_of_pylons_per_wing = 0
        for key in model_dict["keypoints"].keys():
            if "pylon" in key:
                number_of_pylons_per_wing += 1
        self.number_of_pylons_per_wing = int(number_of_pylons_per_wing / 2)
        number_of_kite_components = 6 + 2 * self.number_of_pylons_per_wing
        number_of_flaps_per_wing = model_dict["wing"]["number_of_flaps_per_wing"]

        # verify required components exist
        self.required_components = [
            ["fuselage"],
            ["wing", "starboard"],
            ["wing", "port"],
            ["horizontal_stabilizer", "starboard"],
            ["horizontal_stabilizer", "port"],
            ["vertical_stabilizer"]
        ]
        for i in range(self.number_of_pylons_per_wing):
            self.required_components.append(["pylon", "starboard", i + 1])    
            self.required_components.append(["pylon", "port", i + 1])
            self.required_components.append(["rotor_assembly", "starboard", i + 1, "upper"])
            self.required_components.append(["rotor_assembly", "starboard", i + 1, "lower"])
            self.required_components.append(["rotor_assembly", "port", i + 1, "upper"])
            self.required_components.append(["rotor_assembly", "port", i + 1, "lower"])
        self._verify_component_list(model_dict)

        # model setup
        self.components = self._build_components(model_dict)
        self.fuselage = self.components[0]
        self.starboard_wing = self.components[1]
        self.port_wing = self.components[2]
        self.vstab = self.components[3]
        self.starboard_hstab = self.components[4]
        self.port_hstab = self.components[5]
        self.starboard_pylons = self.components[6]
        self.port_pylons = self.components[7]
        self.starboard_rotors = self.components[8]
        self.port_rotors = self.components[9]

        self.joints = self._build_joints()

        self._validate_model()

        # simulation setup
        self.main_mbd = MainMBD(simulation_dict,
                                model_dict["keypoints"],
                                self.joints,
                                self.fuselage,
                                self.starboard_wing, self.port_wing,
                                self.starboard_hstab, self.port_hstab,
                                self.vstab,
                                self.starboard_pylons, self.port_pylons,
                                self.starboard_rotors, self.port_rotors,
                                number_of_flaps_per_wing,
                                self.number_of_pylons_per_wing,
                                number_of_kite_components
                                )

        self.main_set = MainSet(self.mip_reference_frame,
                                self.fuselage,
                                self.starboard_wing, self.port_wing,
                                self.starboard_hstab, self.port_hstab,
                                self.vstab,
                                self.starboard_pylons, self.port_pylons,
                                self.starboard_rotors, self.port_rotors)

    def _build_components(self, model_dict):

        def _build_component(component_type, path, mbdyn_ref_index):
            name = "_".join([str(p) for p in path])
            return component_type(
                name,
                self._deep_get(model_dict, path),
                mbdyn_ref_index
            )

        fuselage = _build_component(Fuselage, ["fuselage"], 1000)
        starboard_wing = _build_component(Wing, ["wing", "starboard"], 2000)
        port_wing = _build_component(Wing, ["wing", "port"], 3000)
        vstab = _build_component(VerticalStabilizer, ["vertical_stabilizer"], 6000)
        starboard_hstab = _build_component(HorizontalStabilizer, ["horizontal_stabilizer", "starboard"], 4000)
        port_hstab = _build_component(HorizontalStabilizer, ["horizontal_stabilizer", "port"], 5000)

        # pylons
        mbdyn_ref_index = 10000
        starboard_pylons = []
        for i in range(self.number_of_pylons_per_wing):
            mbdyn_ref_index += i * 100
            pylon = _build_component(Pylon, ["pylon", "starboard", i + 1], mbdyn_ref_index)
            starboard_pylons.append(pylon)

        mbdyn_ref_index = 11000
        port_pylons = []
        for i in range(self.number_of_pylons_per_wing):
            mbdyn_ref_index += i * 100
            pylon = _build_component(Pylon, ["pylon", "port", i + 1], mbdyn_ref_index)
            port_pylons.append(pylon)

        # rotors
        mbdyn_ref_index = 20000
        starboard_rotors = []
        for i in range(self.number_of_pylons_per_wing):
            mbdyn_ref_index += i * 100
            rotor = _build_component(RotorAssembly, ["rotor_assembly", "starboard", i + 1, "upper"], mbdyn_ref_index)
            starboard_rotors.append(rotor)
            rotor = _build_component(RotorAssembly, ["rotor_assembly", "starboard", i + 1, "lower"], mbdyn_ref_index + 10)
            starboard_rotors.append(rotor)
        
        mbdyn_ref_index = 30000
        port_rotors = []
        for i in range(self.number_of_pylons_per_wing):
            mbdyn_ref_index += i * 100
            rotor = _build_component(RotorAssembly, ["rotor_assembly", "port", i + 1, "upper"], mbdyn_ref_index)
            port_rotors.append(rotor)
            rotor = _build_component(RotorAssembly, ["rotor_assembly", "port", i + 1, "lower"], mbdyn_ref_index + 10)
            port_rotors.append(rotor)

        return fuselage, starboard_wing, port_wing, vstab, starboard_hstab, port_hstab, starboard_pylons, port_pylons, starboard_rotors, port_rotors

    def _build_joints(self):

        def _build_joint(joint_type, node1, node2, joint_count):
            return joint_type(joint_count, node1, node2)

        joints = []

        # fuselage to starboard wing
        index = self.fuselage.component.index("wing")
        node1 = self.fuselage.nodes[2 * index]
        index = self.starboard_wing.component.index("fuselage")
        node2 = self.starboard_wing.nodes[2 * index]
        joints.append(_build_joint(TotalJoint, node1, node2, len(joints) + 1))

        # fuselage to port wing
        index = self.fuselage.component.index("wing")
        node1 = self.fuselage.nodes[2 * index]
        index = self.port_wing.component.index("fuselage")
        node2 = self.port_wing.nodes[2 * index]
        joints.append(_build_joint(TotalJoint, node1, node2, len(joints) + 1))

        # fuselage to vertical stabilizer
        index = self.fuselage.component.index("stabilizer/vertical")
        node1 = self.fuselage.nodes[2 * index]
        index = self.vstab.component.index("fuselage")
        node2 = self.vstab.nodes[2 * index]
        joints.append(_build_joint(TotalJoint, node1, node2, len(joints) + 1))

        # vertical stabilizer to starboard horizontal stabilizer
        index = self.vstab.component.index("stabilizer/horizontal")
        node1 = self.vstab.nodes[2 * index]
        index = self.starboard_hstab.component.index("stabilizer/vertical")
        node2 = self.starboard_hstab.nodes[2 * index]
        joints.append(_build_joint(TotalJoint, node1, node2, len(joints) + 1))

        # vertical stabilizer to port horizontal stabilizer
        index = self.vstab.component.index("stabilizer/horizontal")
        node1 = self.vstab.nodes[2 * index]
        index = self.port_hstab.component.index("stabilizer/vertical")
        node2 = self.port_hstab.nodes[2 * index]
        joints.append(_build_joint(TotalJoint, node1, node2, len(joints) + 1))

        # starboard wing to starboard pylons
        for i, pylon in enumerate(self.starboard_pylons):
            index = self.starboard_wing.component.index("pylon/starboard/{}".format(str(i+1)))
            node1 = self.starboard_wing.nodes[2 * index]
            index = pylon.component.index("wing/starboard")
            node2 = pylon.nodes[2 * index]
            joints.append(_build_joint(TotalJoint, node1, node2, len(joints) + 1))

        # port wing to port pylons
        for i, pylon in enumerate(self.port_pylons):
            index = self.port_wing.component.index("pylon/port/{}".format(str(i+1)))
            node1 = self.port_wing.nodes[2 * index]
            index = pylon.component.index("wing/port")
            node2 = pylon.nodes[2 * index]
            joints.append(_build_joint(TotalJoint, node1, node2, len(joints) + 1))

        # starboard rotors to starboard pylons
        for rotor in self.starboard_rotors:
            # rotor to pylon
            i = rotor.component_name.split("_")[3]
            ul = rotor.component_name.split("_")[4]
            pylon = self.starboard_pylons[int(i) - 1]
            index = pylon.component.index("rotor_assembly/starboard/{}/{}".format(str(i), ul))
            node1 = pylon.nodes[2 * index]
            node2 = rotor.nodes[0]
            joints.append(_build_joint(TotalJoint, node1, node2, len(joints) + 1))

            # nacelle to pylon
            node2 = rotor.nodes[1]
            joints.append(_build_joint(TotalJoint, node1, node2, len(joints) + 1))

        # port rotors to port pylons
        for rotor in self.port_rotors:
            # rotor to pylon
            i = rotor.component_name.split("_")[3]
            ul = rotor.component_name.split("_")[4]
            pylon = self.port_pylons[int(i) - 1]
            index = pylon.component.index("rotor_assembly/port/{}/{}".format(str(i), ul))
            node1 = pylon.nodes[2 * index]
            node2 = rotor.nodes[0]
            joints.append(_build_joint(TotalJoint, node1, node2, len(joints) + 1))

            # nacelle to pylon
            node2 = rotor.nodes[1]
            joints.append(_build_joint(TotalJoint, node1, node2, len(joints) + 1))
        
        return joints
    
    def _validate_model(self):
        
        # pylons should be given in order from inboard to outboard

        # port pylons should decrease in y as the array index gets larger
        inner = self.port_pylons[0]
        for pylon in self.port_pylons[1:]:
            outer = pylon
            if not inner.mip.x2 > outer.mip.x2:
                raise ModelException(pylon, "Invalid order - inboard to outboard")
            inner = outer
        
        # starboard pylons should increase in y as the array index gets larger
        inner = self.starboard_pylons[0]
        for pylon in self.starboard_pylons[1:]:
            outer = pylon
            if not inner.mip.x2 < outer.mip.x2:
                raise ModelException(pylon, "Invalid order - inboard to outboard")
            inner = outer

        # rotors should be given in order from upper to lower and inboard to outboard
        
        # even and 0 indexed rotors should have a z greater than odd indexed rotors
        # except z is flipped in our coordinate system so "up" is smaller z 
        for i in range(0, len(self.port_rotors), 2):
            even = self.port_rotors[i]
            odd = self.port_rotors[i + 1]
            if not even.mip.x3 < odd.mip.x3:
                raise ModelException(even, "Invalid order - upper to lower")

        # even and 0 indexed rotors should have a z greater than odd indexed rotors
        # except z is flipped in our coordinate system so "up" is smaller z
        for i in range(0, len(self.starboard_rotors), 2):
            even = self.starboard_rotors[i]
            odd = self.starboard_rotors[i + 1]
            if not even.mip.x3 < odd.mip.x3:
                raise ModelException(even, "Invalid order - upper to lower")

        # port side rotors should decrease in y as the array index gets larger
        inner_upper = self.port_rotors[0]
        inner_lower = self.port_rotors[1]
        for i in range(2, len(self.port_rotors), 2):
            outer_upper = self.port_rotors[i]
            outer_lower = self.port_rotors[i + 1]
            if not inner_upper.mip.x2 > outer_upper.mip.x2:
                raise ModelException(pylon, "Invalid order - inboard to outboard")
            if not inner_lower.mip.x2 > outer_lower.mip.x2:
                raise ModelException(pylon, "Invalid order - inboard to outboard")
            inner_upper = outer_upper
            inner_lower = outer_lower
        
        # starboard side rotors should increase in y as the array index gets larger
        inner_upper = self.starboard_rotors[0]
        inner_lower = self.starboard_rotors[1]
        for i in range(2, len(self.starboard_rotors), 2):
            outer_upper = self.starboard_rotors[i]
            outer_lower = self.starboard_rotors[i + 1]
            if not inner_upper.mip.x2 < outer_upper.mip.x2:
                raise ModelException(pylon, "Invalid order - inboard to outboard")
            if not inner_lower.mip.x2 < outer_lower.mip.x2:
                raise ModelException(pylon, "Invalid order - inboard to outboard")
            inner_upper = outer_upper
            inner_lower = outer_lower

    def export(self, output_directory):
        self.main_set.export_set_file(output_directory)
        self.main_mbd.export_mbd_file(output_directory)
        self.fuselage.export_all(output_directory)
        self.starboard_wing.export_all(output_directory)
        self.port_wing.export_all(output_directory)
        self.vstab.export_all(output_directory)
        self.starboard_hstab.export_all(output_directory)
        self.port_hstab.export_all(output_directory)
        for pylon in self.starboard_pylons:
            pylon.export_all(output_directory)
        for pylon in self.port_pylons:
            pylon.export_all(output_directory)
        for rotor in self.starboard_rotors:
            rotor.export_all(output_directory)
        for rotor in self.port_rotors:
            rotor.export_all(output_directory)
