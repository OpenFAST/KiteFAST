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

# required to build the dependencies (C_MATH)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -g")
find_package(GSL REQUIRED)

generate_f90_types(src/KiteFastController_Registry.txt KiteFastController_Types.f90 -noextrap)

# kitefastcontroller-lib
set(LIB_SOURCES src/KiteFastController.f90 KiteFastController_Types.f90 )
add_library(kitefastcontrollerlib ${LIB_SOURCES})
target_link_libraries(kitefastcontrollerlib nwtclibs)

# kitefastcontroller-controller
file(GLOB C_MATH_SOURCES "src/common/c_math/*")
add_library(C_MATH ${C_MATH_SOURCES})

set(DYLIB_SOURCES
	src/KiteFastController.c 
	src/control/control_log.c
	src/control/control_params.c
	src/control/controller_conversion.c
#	src/control/controller_util.c
	src/control/ground_station_frame.c
	src/control/kfc.c
	src/control/perch_frame.c
	src/control/sensor_util.c
	src/control/simple_aero.c
	src/control/system_params.c
	src/control/system_types.c
	src/control/tether_util.c
	src/control/crosswind/crosswind_frame.c
	src/control/crosswind/crosswind_inner.c
	src/control/crosswind/crosswind_playbook.c
	src/control/crosswind/crosswind_power.c
	src/control/crosswind/crosswind_types.c
	src/control/crosswind/crosswind_util.c
	src/control/crosswind/crosswind.c
	src/control/estimator/estimator_types.c
) 
include_directories(
#	"src/common/"
#	"src/control/"
#	"src/control/crosswind/"
#	"src/control/estimator/"
#	"src/system/"
#	"dependencies/c_math/"
	"src/"
#	"dependencies/"
)
add_library(kitefastcontroller_controller SHARED ${DYLIB_SOURCES})
target_link_libraries(kitefastcontroller_controller C_MATH ${GSL_LIBRARIES})

# kitefastcontroller-testexe
set(TESTEXE_SOURCES src/main.c src/KiteFastController.c)
add_executable(kitefastcontroller_testexe ${TESTEXE_SOURCES})
target_link_libraries(kitefastcontroller_testexe kitefastcontroller_controller)

# kitefastcontroller-driver
set(DRIVER_SOURCES src/KiteFastController_Driver.f90)
add_executable(kitefastcontroller_driver ${DRIVER_SOURCES})
target_link_libraries(kitefastcontroller_driver kitefastcontroller_controller kitefastcontrollerlib ${CMAKE_DL_LIBS})

# installation
install(TARGETS kitefastcontrollerlib kitefastcontroller_controller kitefastcontroller_driver C_MATH kitefastcontroller_testexe
  EXPORT "${CMAKE_PROJECT_NAME}Libraries"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
