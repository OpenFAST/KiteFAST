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
import os
import argparse
from src.iohandler import Input
from src.two_element_beam_model import TwoElementBeamModel
from src.kite_model import KiteModel

# Parse the input arguments
description = "Creates a set of MBDyn input files from a model definition."
parser = argparse.ArgumentParser(description=description)
parser.add_argument("-i",
                    "--input-file",
                    nargs=1,
                    required=True,
                    help="Path to the preprocessor input file")
parser.add_argument("-o",
                    "--output-directory",
                    nargs="?",
                    default=None,
                    help="Path to the directory where the output should be written")
parser.add_argument("-m",
                    "--model-type",
                    nargs="?",
                    type=str,
                    default="kite",
                    help="Available options: kite (default), beam")
parser.add_argument("-info",
                    "--show-component-info",
                    action="store_true",
                    help="Display mass, center of mass, and inertia for each component")
args = parser.parse_args()

# Read the inputs and instantiate the model
input_file = os.path.abspath(args.input_file[0])
inputs = Input(input_file)
if args.model_type == "kite":
    model = KiteModel(inputs.simulation_dict, inputs.model_dict, inputs.simulation_type)
elif args.model_type == "beam":
    model = TwoElementBeamModel(inputs.simulation_dict, inputs.model_dict)

# Export the MBDyn model files
if args.output_directory is None:
    output_directory = os.path.dirname(input_file)
else:
    output_directory = os.path.abspath(args.output_directory)
model.export(output_directory)

# Print component information if requested
if args.show_component_info:
    model.print_model_info()
