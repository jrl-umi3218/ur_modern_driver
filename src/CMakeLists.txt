cmake_minimum_required(VERSION 3.1)

set(ur_modern_driver_SRC
  do_output.cpp
  robot_state.cpp
  robot_state_RT.cpp
  ur_communication.cpp
  ur_driver.cpp
  ur_realtime_communication.cpp
)

add_library(ur_modern_driver SHARED ${ur_modern_driver_SRC})
target_include_directories(ur_modern_driver PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

install(TARGETS ur_modern_driver
  EXPORT ur_modern_driverTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(EXPORT ur_modern_driverTargets
  FILE ur_modern_driverTargets.cmake
  NAMESPACE ur_modern_driver::
  DESTINATION lib/cmake/ur_modern_driver)
