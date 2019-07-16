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

import yaml


class Output():
    """
    Output is a helper class for generating output files.
    """
    def __init__(self, filename):
        self.filename = filename
        self.file = open(self.filename, "w")
        self.ln = "\n"

    def write_empty_line(self):
        self.write_line("")

    def write_line(self, line):
        self.file.write(line + self.ln)

    def end(self):
        self.file.close()


class Input():
    """
    Input handles reading and parsing the yaml input file.
    It unpacks all of the dictionaries and casts the data to the necessary types.
    """
    def __init__(self, input_name):
        self.input_dict = yaml.load(open(input_name, "r"), Loader=yaml.FullLoader)
        self.simulation_dict = self._parse_simulation_dict(self.input_dict)
        if "simulation_type" in self.simulation_dict:
            self.simulation_type = self.simulation_dict["simulation_type"]
        else:
            self.simulation_type = 1
        self.model_dict = self._parse_model_dict(self.input_dict)

    def _string_to_list(self, string, delimiter=" "):
        dirty = string.split(delimiter)
        clean = filter(None, dirty)
        return list(clean)

    def _parse_simulation_dict(self, input_dict):
        simulation_controls = input_dict["simulation_controls"]
        simulation_dict = {
            "title": input_dict["title"],
            "output": input_dict["output"]
        }
        simulation_dict.update(simulation_controls)

        return simulation_dict 

    def _parse_model_dict(self, input_dict):
        model_dict = {
            "keypoints": {
                "fuselage": [0.000, 0.000, 0.000]
            }
        }
        for key, value in input_dict["keypoints"].items():
            model_dict["keypoints"][key] = value

        # unpack component info
        # component = {
        #     "keypoint": [float],
        #     "element_end_nodes": [float],
        #     "twist": [float],
        #     "component": [str],
        #     "point_mass": [float],
        #     "stiffness_matrix": [[float]],
        #     "mass_distribution": [[float]],
        #     "cm_offsets": [[float]],
        #     "inertias": [[float]]
        # }

        def _deep_get(_dict, keys, default=None):
            for key in keys:
                if isinstance(_dict, dict):
                    _dict = _dict.get(key, default)
                else:
                    return default
            return _dict

        def _unpack_component_info(path):
            nodes, twist, attached_component, point_mass = [], [], [], []
            stiffness_matrix = []
            mass_distribution, cm_offset, inertias = [], [], []
            input_component = _deep_get(input_dict, path)

            if input_component is None:
                return None

            for i, end_node in enumerate(input_component["element_end_nodes"]):
                # element end nodes
                this_node = end_node

                nodes.append(float(this_node[0]))
                twist.append(float(this_node[1]))
                attached_component.append(this_node[2])
                point_mass.append(float(this_node[3]))

                # stiffness matrix
                this_stiff = input_component["stiffness_matrix"][i]
                stiffness_matrix.append([float(stiff) for stiff in this_stiff])

                # mass distribution
                this_mass = input_component["mass_distribution"][i]
                mass_distribution.append(float(this_mass[0]))
                cm_offset.append([float(m) for m in this_mass[1:3]])
                inertias.append([float(m) for m in this_mass[3:9]])

            stiffness_constant = input_component["proportional_stiffness_constant"]

            return {
                "keypoint": model_dict["keypoints"]["/".join([str(p) for p in path])],
                "element_end_nodes": nodes,
                "twist": twist,
                "component": attached_component,
                "point_mass": point_mass,
                "stiffness_constant": stiffness_constant,
                "stiffness_matrix": stiffness_matrix,
                "mass_distribution": mass_distribution,
                "cm_offset": cm_offset,
                "inertias": inertias
            }
        
        if self.simulation_type < 3:
            model_dict["fuselage"] = _unpack_component_info(["fuselage"])
            model_dict["wing"] = {
                "number_of_flaps_per_wing": _deep_get(input_dict, ["wing", "number_of_flaps_per_wing"]),
                "starboard": _unpack_component_info(["wing", "starboard"]),
                "port": _unpack_component_info(["wing", "port"])
            }
            model_dict["horizontal_stabilizer"] = {
                "starboard": _unpack_component_info(["stabilizer", "horizontal", "starboard"]),
                "port": _unpack_component_info(["stabilizer", "horizontal", "port"])
            }
            model_dict["vertical_stabilizer"] = _unpack_component_info(["stabilizer", "vertical"])

            pylon = {}
            component = "pylon"
            for direction in ["starboard", "port"]:
                this_side = _deep_get(input_dict, [component, direction])
                if this_side == None:
                    pylon[direction] = None
                else:
                    pylon[direction] = {}
                    for position_number in this_side.keys():
                        path = [component, direction, position_number]
                        pylon[direction][position_number] = _unpack_component_info(path)
            model_dict[component] = pylon

            component = "rotor_assembly"
            rotor_assembly = {}
            for direction in ["starboard", "port"]:
                this_side = _deep_get(input_dict, [component, direction])
                if this_side == None:
                    rotor_assembly[direction] = None
                else:
                    rotor_assembly[direction] = {}
                    for position in this_side:
                        this_position = this_side[position]
                        rotor_assembly[direction][position] = {}
                        for level in ["upper", "lower"]:
                            this_component = this_position[level]

                            keypoint = model_dict["keypoints"]["rotor_assembly"
                                                            + "/" + direction
                                                            + "/" + str(position)
                                                            + "/" + level]

                            rotor_mass_props = this_component["rotor"]["mass_properties"]
                            rotor_mass = float(rotor_mass_props[0])
                            rotor_cm_offset = float(rotor_mass_props[1])
                            rotor_rot_inertia = float(rotor_mass_props[2])
                            rotor_trans_inertia = float(rotor_mass_props[3])

                            nacelle_mass_props = this_component["nacelle"]["mass_properties"]
                            nacelle_mass = float(nacelle_mass_props[0])
                            nacelle_cm_offset = [float(m) for m in nacelle_mass_props[1:4]]
                            nacelle_inertias = [float(m) for m in nacelle_mass_props[4:10]]

                            rotor_assembly[direction][position][level] = {
                                "keypoint": keypoint,
                                "rotor": {
                                    "mass": rotor_mass,
                                    "cm_offset": rotor_cm_offset,
                                    "rotational_inertia": rotor_rot_inertia,
                                    "translational_inertia": rotor_trans_inertia
                                },
                                "nacelle": {
                                    "mass": nacelle_mass,
                                    "cm_offset": nacelle_cm_offset,
                                    "inertia": nacelle_inertias
                                }
                            }
            model_dict[component] = rotor_assembly

        if self.simulation_type > 1:
            model_dict["platform"] = {
                "mass": input_dict["platform"]["mass_properties"][0],
                "inertia": input_dict["platform"]["mass_properties"][1:4],
                "node_location": input_dict["platform"]["node_location"],
                "imu_location": input_dict["platform"]["imu_location"],
                "wind_reference_station_location": input_dict["wind_reference_station"]["location"],
                "ground_station_location": input_dict["ground_station"]["location"]
            }

        return model_dict
