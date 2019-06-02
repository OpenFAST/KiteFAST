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

import sys
from .iohandler import Output
from .mbdyn_types import Vec3

class BaseModel():
    def __init__(self):
        self.title = "BaseModel"
        # initialize with an empty component list
        self.required_components = []
        self.components = []

    def _list_to_vec3(self, list):
        return Vec3(list[0], list[1], list[2])

    def _deep_get(self, _dict, keys, default=None):
        """
        Recursive function for finding a nested dict.

        _dict: the dictionary to search over
        keys: list of keys defining the nested path
        default: optional value to return when the given path is not found

        returns the nested dictionary
        """
        for key in keys:
            if isinstance(_dict, dict):
                _dict = _dict.get(key, default)
            else:
                return default
        return _dict

    def _deep_put(self, _dict, keys, value):
        """
        A function for putting a given value at a key path.

        _dict: the input dictionary to modify
        keys: list of keys defining the nested path
        value: the value to add at the nested path

        returns the modified input dictionary

        NOTE: this takes advantage of the face that Python stores values by
        reference. Since `traverse_dict` is referencing the same memory as the
        input `_dict`, modifying it also modifies `_dict`.
        """
        traverse_dict = _dict
        for i, key in enumerate(keys[:-1]):
            traverse_dict = traverse_dict.get(key)
            if i == len(keys) - 2:
                traverse_dict[keys[-1]] = value
                break
        return _dict

    def _verify_component_list(self, model_dict):
        for path in self.required_components:
            if self._deep_get(model_dict, path) is None:
                raise ModelException(path, "expected component not given")

    def _preprocess_model_dict(self, model_dict):
        keypoints = model_dict["keypoints"]
        for keypoint in keypoints:
            keypoints[keypoint] = self._list_to_vec3(keypoints[keypoint])
        model_dict["keypoints"] = keypoints

        for component_path in self.required_components:
            component_dict = self._deep_get(model_dict, component_path)
            keypoint = self._list_to_vec3(component_dict["keypoint"])
            self._deep_put(model_dict, component_path + ["keypoint"], keypoint)
        
        reference_point_paths = [
            ["platform", "node_location"],
            ["platform", "imu_location"],
            ["platform", "wind_reference_station_location"],
            ["platform", "ground_station_location"]
        ]
        for path in reference_point_paths:
            point = self._deep_get(model_dict, path)
            self._deep_put(model_dict, path, self._list_to_vec3(point))

        return model_dict

    def _preprocess_simulation_dict(self, simulation_dict):
        simulation_dict["initial_conditions"]["location"] = self._list_to_vec3(simulation_dict["initial_conditions"]["location"])
        simulation_dict["initial_conditions"]["orientation"] = self._list_to_vec3(simulation_dict["initial_conditions"]["orientation"])
        simulation_dict["initial_conditions"]["velocity"]["translational"] = self._list_to_vec3(simulation_dict["initial_conditions"]["velocity"]["translational"])
        simulation_dict["initial_conditions"]["velocity"]["rotational"] = self._list_to_vec3(simulation_dict["initial_conditions"]["velocity"]["rotational"])
        return simulation_dict

    def print_model_info(self):
        """
        Calculates the total mass and center of mass of the model and exports 
        this information into a file.
        """
        # put all of the component elements into a 1d array
        flat_list = []
        for element in self.components:
            if type(element) is list:
                flat_list += element
            else:
                flat_list.append(element)

        # calculate the total mass and cg
        total_mass = 0
        cg = Vec3(0.0, 0.0, 0.0)
        for element in flat_list:
            total_mass += element.total_mass
            cg += Vec3(
                element.total_mass * (element.center_of_gravity.x1 + element.mip.x1),
                element.total_mass * (element.center_of_gravity.x2 + element.mip.x2),
                element.total_mass * (element.center_of_gravity.x3 + element.mip.x3)
            )
        cg /= total_mass

        # export the model info
        info_file = Output("KiteMain.preprocessor")
        info_file.write_empty_line()
        info_file.write_line("MBDyn preprocessor model information")
        info_file.write_empty_line()
        info_file.write_line("{:>32} | {:>10} | {:<16}".format("model", "total mass", "center of gravity (relative to model mip)"))
        center_of_gravity_string = "{}".format(cg)
        info_file.write_line("{:>32} | {:>10.3f} | <{}>".format(self.title, total_mass, center_of_gravity_string))
        info_file.write_empty_line()

        # write the component mass info
        info_file.write_line("{:>32} | {:>10} | {:<16}".format("component", "mass", "center of gravity (relative to component)"))
        for element in flat_list:
            info_file.write_line("{:>32} | {:>10.3f} | <{}>".format(
                element.component_name, element.component_mass, element.center_of_gravity))
        info_file.write_empty_line()

        # write the component point masses excluding rotors
        info_file.write_line("{:>32} | {:<16}".format("component", "point masses (listed in ascending order of node location in component primary direction)"))
        for element in flat_list:
            if "rotor" in element.component_name:
                continue
            point_mass_list_string = "".join(["{:>10.3f}".format(i) for i in element.nodal_point_masses])
            info_file.write_line("{:>32} | {}".format(element.component_name, point_mass_list_string))
        info_file.write_empty_line()

        # write a notice about rotor masses not being included in mbdyn
        info_file.write_line("NOTE: Due to the design of the rotor load calculation, rotor masses are not included in the mbdyn model")
        info_file.write_line("      and the gravity loads are instead calculated in KiteFAST. Therefore, mass, center of mass, and inertia")
        info_file.write_line("      reported above may not match what is reported in MBDyn.")

        info_file.end()

class ModelException(Exception):
    """
    Exception raised for modeling errors.

    Attributes:
        component -- the preprocessor component type which contains the error
        message -- explanation of the error
    """

    def __init__(self, component, message):
        super().__init__(message)
        self.component = component
        self.message = message

    def __str__(self):
        return "Error in {}: {}".format(self.component.component_name, self.message)
