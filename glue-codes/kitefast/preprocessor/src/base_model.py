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

class BaseModel():
    def __init__(self):
        # initialize with an empty component list
        self.required_components = []
        self.components = []

    def _deep_get(self, _dict, keys, default=None):
        for key in keys:
            if isinstance(_dict, dict):
                _dict = _dict.get(key, default)
            else:
                return default
        return _dict

    def _verify_component_list(self, model_dict):
        for path in self.required_components:
            if self._deep_get(model_dict, path) is None:
                print("ERROR expected component not given: {}".format(path))
                sys.exit(7)

    def print_component_info(self):
        info_file = Output("KiteMain.preprocessor")
        info_file.write_line("MBDyn preprocessor model information")
        info_file.write_empty_line()

        total_mass = 0
        cm_x = 0
        cm_y = 0
        cm_z = 0

        # put all of the component elements into a 1d array
        flat_list = []
        for element in self.components:
            if type(element) is list:
                flat_list += element
            else:
                flat_list.append(element)

        # write the component mass info
        info_file.write_line("{:>32} | {:>10} | {:<16}".format("component", "total mass", "center of mass (relative to component)"))
        for element in flat_list:
            total_mass += element.total_mass
            cm_x += element.total_mass * element.center_of_mass.x1
            cm_y += element.total_mass * element.center_of_mass.x2
            cm_z += element.total_mass * element.center_of_mass.x3
            info_file.write_line("{:>32} | {:>10.3f} | <{}>".format(element.component_name, element.total_mass, element.center_of_mass))
        info_file.write_empty_line()

        # write the component point masses excluding rotors
        info_file.write_line("{:>32} | {:<16}".format("component", "point masses (listed in ascending order of node location in component primary direction)"))
        for element in flat_list:
            if "rotor" in element.component_name:
                continue
            info_file.write_line("{:>32} | {}".format(element.component_name, element.nodal_point_masses))
        info_file.write_empty_line()

        # finish calculating and write the kite mass info
        cm_x /= total_mass
        cm_y /= total_mass
        cm_z /= total_mass
        info_file.write_line("total mass: {:8.3f}".format(total_mass))
        info_file.write_line("center of mass: <{:8.3f},{:8.3f},{:8.3f}>".format(cm_x, cm_y, cm_z))
        info_file.end()
