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

if(APPLE OR UNIX)
  add_definitions(-DIMPLICIT_DLLEXPORT)
endif()

generate_f90_types(src/KiteFAST_Registry.txt KiteFAST_Types.f90 -noextrap)

add_library(kitefast_prelib KiteFAST_Types.f90)
target_link_libraries(kitefast_prelib
  actuatordisklib
  airfoilinfolib
  ifwlib
  kiteaerodynlib
  moordynlib
  )

add_library(kitefastlib src/KiteFAST.f90)
target_link_libraries(kitefastlib kitefast_prelib)

install(TARGETS kitefastlib kitefast_prelib
  EXPORT ${CMAKE_PROJECT_NAME}Libraries
  RUNTIME DESTINATION lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

# install(FILES
#   src/FAST_Library.h
#   DESTINATION include)
