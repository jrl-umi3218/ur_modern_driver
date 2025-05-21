set(ur_modern_driver_SRC
  do_output.cpp
  robot_state.cpp
  robot_state_RT.cpp
  ur_communication.cpp
  ur_driver.cpp
  ur_realtime_communication.cpp
)

set(ur_modern_driver_HDR
  ${PROJECT_SOURCE_DIR}/include/ur_modern_driver/do_output.h
  ${PROJECT_SOURCE_DIR}/include/ur_modern_driver/robot_state.h
  ${PROJECT_SOURCE_DIR}/include/ur_modern_driver/robot_state_RT.h
  ${PROJECT_SOURCE_DIR}/include/ur_modern_driver/ur_communication.h
  ${PROJECT_SOURCE_DIR}/include/ur_modern_driver/ur_driver.h
  ${PROJECT_SOURCE_DIR}/include/ur_modern_driver/ur_realtime_communication.h
)

add_library(ur_modern_driver SHARED ${ur_modern_driver_SRC} ${ur_modern_driver_HDR})
target_include_directories(ur_modern_driver PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

install(FILES ${ur_modern_driver_HDR} DESTINATION include/ur_modern_driver)

install(TARGETS ur_modern_driver
  EXPORT ur_modern_driverTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(EXPORT ur_modern_driverTargets
  FILE ur_modern_driverTargets.cmake
  NAMESPACE ur_modern_driver::
  DESTINATION lib/cmake/ur_modern_driver)
