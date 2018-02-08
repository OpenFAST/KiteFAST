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

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CMAKE_MACOSX_RPATH 1)
endif()

generate_f90_types(src/KiteFastController_Registry.txt KiteFastController_Types.f90 -noextrap)

# kitefastcontroller-lib
set(LIB_SOURCES src/KiteFastController.f90 KiteFastController_Types.f90 )
add_library(kitefastcontrollerlib ${LIB_SOURCES})
target_link_libraries(kitefastcontrollerlib nwtclibs)

# kitefastcontroller-controller
set(DYLIB_SOURCES src/KiteFastController.c)
add_library(kitefastcontroller_controller SHARED ${DYLIB_SOURCES})
# set_target_properties(kitefastcontroller_controller PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1)

# kitefastcontroller-driver
set(DRIVER_SOURCES src/KiteFastController_Driver.f90)
add_executable(kitefastcontroller_driver ${DRIVER_SOURCES})
target_link_libraries(kitefastcontroller_driver kitefastcontrollerlib nwtclibs ${CMAKE_DL_LIBS})

# installation
install(TARGETS kitefastcontrollerlib kitefastcontroller_controller kitefastcontroller_driver
  EXPORT "${CMAKE_PROJECT_NAME}Libraries"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
