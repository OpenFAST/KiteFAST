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

import sys, os
srcdir = os.path.join(os.path.dirname(__file__), '..', 'src')
sys.path.insert(0, srcdir)
from mbdyn_types import IntegralN
from mbdyn_types import IntegralNx
from mbdyn_types import IntegralNP
from mbdyn_types import IntegralN_times_P2plusR2
from mbdyn_types import IntegralNPR


def _assert(value, baseline, test_name, tolerance=1e-5):
    try:
        assert abs(value - baseline) < tolerance
    except AssertionError:
        print("Error in {}:".format(test_name))
        print("    expected: {}".format(baseline))
        print("    received: {}".format(value))


def test_IntegralN():
    """
    integral(mx + b)
    m = (1 - 3) / (1 - 0) = -2
    b = 3
    integral( -2x + 3 ) [0, 1] = 2
    """
    test_name = "IntegralN_times_P2plusR2"
    baseline = 2.00000
    value = IntegralN(3, 1, 0, 1)
    _assert(value, baseline, test_name)


def test_IntegralNx():
    """
    integral( (mx + b) * x )
    m = (1 - 3) / (1 - 0) = -2
    b = 3
    integral( -2x^2 + 3x ) [0, 1] = 0.8333
    """
    test_name = "IntegralNx"
    baseline = 0.83333
    value = IntegralNx(3, 1, 0, 1)
    _assert(value, baseline, test_name)


def test_IntegralNP():
    """
    integral( (ax + b) * (cx + d) )
    a = (1 - 3) / (1 - 0) = -2
    b = 3
    c = (1 - 5) / (1 - 0) = -4
    d = 5
    integral( (-2x + 3) * (-4x + 5) )
    integral( 8x^2 - 22x + 15 ) [0, 1] = 6.6667
    """
    test_name = "IntegralNP"
    baseline = 6.66666
    value = IntegralNP(3, 1, 5, 1, 0, 1)
    _assert(value, baseline, test_name)


def test_IntegralN_times_P2plusR2():
    """
    integral( (ax + b) * ((cx + d)^2 + (ex + f)^2) )
    a = (1 - 3) / (1 - 0) = -2
    b = 3
    c = (1 - 5) / (1 - 0) = -4
    d = 5
    e = (1 - 7) / (1 - 0) = -6
    f = 7
    integral( (-2x + 3) * ((-4x + 5)^2 + (-6x + 7)^2)
    integral( (-2x + 3) * (48x^2 - 124x +74) )
    integral( (-104x^3 + 404x^2 - 520x + 222) ) [0, 1] = 
    """
    test_name = "IntegralN_times_P2plusR2"
    baseline = 70.66666
    value = IntegralN_times_P2plusR2(3, 1, 5, 1, 7, 1, 0, 1) 
    _assert(value, baseline, test_name)


def test_IntegralNPR():
    """
    integral( (ax + b) * (cx + d) * (ex + f) )
    a = (1 - 3) / (1 - 0) = -2
    b = 3
    c = (1 - 5) / (1 - 0) = -4
    d = 5
    e = (1 - 7) / (1 - 0) = -6
    f = 7
    """
    test_name = "IntegralNPR"
    baseline = 33.66666
    value = IntegralNPR(3, 1, 5, 1, 7, 1, 0, 1)
    _assert(value, baseline, test_name)

if __name__=="__main__":
    test_IntegralN()
    test_IntegralNx()
    test_IntegralNP()
    test_IntegralN_times_P2plusR2()
    test_IntegralNPR()
