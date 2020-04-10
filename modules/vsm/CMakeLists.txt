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
  generate_f90_types(src/VSM_Registry.txt ${CMAKE_CURRENT_LIST_DIR}/src/VSM_Types.f90)
endif()


set(LIB_SOURCES
  src/VSM.f90
  src/VSM_Types.f90
)

add_library(vsmlib ${LIB_SOURCES})
target_link_libraries(vsmlib airfoilinfolib nwtclibs)

# set(DRIVER_SOURCES
#   src/VSM_Driver.f90
#   src/VSM_Dvr_Subs.f90
#   tests/VSM_Test.f90
#   )

# add_executable(vsm_driver ${DRIVER_SOURCES})
# target_link_libraries(vsm_driver vsmlib kiteaerodynlib actuatordisklib nwtclibs ${CMAKE_DL_LIBS})

install(TARGETS vsmlib # vsm_driver
  EXPORT "${CMAKE_PROJECT_NAME}Libraries"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
