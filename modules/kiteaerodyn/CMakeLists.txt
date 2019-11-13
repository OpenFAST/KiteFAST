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
  generate_f90_types(src/KiteAeroDyn_Registry.txt ${CMAKE_CURRENT_LIST_DIR}/src/KiteAeroDyn_Types.f90)
endif()

set(LIB_SOURCES
  src/KiteAeroDyn.f90
  src/KiteAeroDyn_IO.f90
  src/KiteAeroDyn_Types.f90
)

add_library(kiteaerodynlib ${LIB_SOURCES})
target_link_libraries(kiteaerodynlib airfoilinfolib actuatordisklib vsmlib nwtclibs)

set(DRIVER_SOURCES
  src/KiteAeroDyn_Driver.f90
  src/KAD_Dvr_Subs.f90
  tests/KiteAeroDyn_Test.f90
  )

add_executable(kiteaerodyn_driver ${DRIVER_SOURCES})
target_link_libraries(kiteaerodyn_driver kiteaerodynlib actuatordisklib nwtclibs ${CMAKE_DL_LIBS})

install(TARGETS kiteaerodyn_driver kiteaerodynlib
  EXPORT "${CMAKE_PROJECT_NAME}Libraries"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
