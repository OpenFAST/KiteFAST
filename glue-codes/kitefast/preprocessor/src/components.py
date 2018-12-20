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

from .base_component import BaseComponent
from .base_rotor import BaseRotor


class Fuselage(BaseComponent):

    def __init__(self, component_name, model_dict, mbdyn_ref_index):
        self.primary_axis = "x1"
        super().__init__(component_name, model_dict,
                         self._interpolator, mbdyn_ref_index,
                         self.primary_axis)

    def _principal_axis_sort(self, vec3):
        return vec3.x1

    def _interpolator(self, pos0, pos1, val0, val1, location_vec3):
        return (val0 * (pos1.x1 - location_vec3.x1) + val1 * (location_vec3.x1 - pos0.x1)) / (pos1.x1 - pos0.x1)


class Wing(BaseComponent):
    def __init__(self, component_name, model_dict, mbdyn_ref_index):
        self.primary_axis = "x2"
        super().__init__(component_name, model_dict,
                         self._interpolator, mbdyn_ref_index,
                         self.primary_axis)

    def _principal_axis_sort(self, vec3):
        return vec3.x2

    def _interpolator(self, pos0, pos1, val0, val1, location_vec3):
        return (val0 * (pos1.x2 - location_vec3.x2) + val1 * (location_vec3.x2 - pos0.x2)) / (pos1.x2 - pos0.x2)


class HorizontalStabilizer(BaseComponent):
    def __init__(self, component_name, model_dict, mbdyn_ref_index):
        self.primary_axis = "x2"
        super().__init__(component_name, model_dict,
                         self._interpolator, mbdyn_ref_index,
                         self.primary_axis)

    def _principal_axis_sort(self, vec3):
        return vec3.x2

    def _interpolator(self, pos0, pos1, val0, val1, location_vec3):
        return (val0 * (pos1.x2 - location_vec3.x2) + val1 * (location_vec3.x2 - pos0.x2)) / (pos1.x2 - pos0.x2)


class VerticalStabilizer(BaseComponent):
    def __init__(self, component_name, model_dict, mbdyn_ref_index):
        self.primary_axis = "x3"
        super().__init__(component_name, model_dict,
                         self._interpolator, mbdyn_ref_index,
                         self.primary_axis)

    def _principal_axis_sort(self, vec3):
        return vec3.x3

    def _interpolator(self, pos0, pos1, val0, val1, location_vec3):
        return (val0 * (pos1.x3 - location_vec3.x3) + val1 * (location_vec3.x3 - pos0.x3)) / (pos1.x3 - pos0.x3)


class Pylon(BaseComponent):
    def __init__(self, component_name, model_dict, mbdyn_ref_index):
        self.primary_axis = "x3"
        super().__init__(component_name, model_dict,
                         self._interpolator, mbdyn_ref_index,
                         self.primary_axis)

    def _principal_axis_sort(self, vec3):
        return vec3.x3

    def _interpolator(self, pos0, pos1, val0, val1, location_vec3):
        return (val0 * (pos1.x3 - location_vec3.x3) + val1 * (location_vec3.x3 - pos0.x3)) / (pos1.x3 - pos0.x3)


class RotorAssembly(BaseRotor):
    def __init__(self, component_name, model_dict, mbdyn_ref_index):
        super().__init__(component_name, model_dict, mbdyn_ref_index)
