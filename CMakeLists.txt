cmake_minimum_required(VERSION 2.8)
project(ESO_Simulation)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

find_package(Eigen3 3.1.0 REQUIRED)
message("Eigen Include Directories: " ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

IF(TEST_ALL)
  add_definitions(-D TEST_ALL)
  set(ALL_SRC 
    src/PIDController.cpp
    src/toy_ESO.cpp
    src/toy_PID.cpp
    src/twoMass_ESO.cpp
    src/twoMass_PID.cpp
  )
  add_executable(test ${ALL_SRC} test/main.cpp)
ELSE()
  add_executable(toy_eso src/toy_ESO.cpp)
  add_executable(toy_pid src/toy_PID.cpp src/PIDController.cpp)
  add_executable(twomass_eso src/twoMass_ESO.cpp)
  add_executable(twomass_pid src/twoMass_PID.cpp src/PIDController.cpp)
ENDIF()

