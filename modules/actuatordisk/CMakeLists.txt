#
# Copyright 2016 National Renewable Energy Laboratory
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

if (GENERATE_TYPES)
  generate_f90_types(src/ActuatorDisk_Registry.txt ${CMAKE_CURRENT_LIST_DIR}/src/ActuatorDisk_Types.f90 -noextrap)
endif()

set(LIB_SOURCES
  src/ActuatorDisk.f90
  tests/ActuatorDisk_Test.f90
  src/ActuatorDisk_Types.f90
)

add_library(actuatordisklib ${LIB_SOURCES})
target_link_libraries(actuatordisklib nwtclibs)

set(DRIVER_SOURCES
  src/ActuatorDisk_Driver.f90
)

add_executable(actuatordisk_driver ${DRIVER_SOURCES})
target_link_libraries(actuatordisk_driver actuatordisklib nwtclibs ${CMAKE_DL_LIBS})

install(TARGETS actuatordisk_driver actuatordisklib
  EXPORT "${CMAKE_PROJECT_NAME}Libraries"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
