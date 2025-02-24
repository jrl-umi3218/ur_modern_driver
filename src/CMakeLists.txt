cmake_minimum_required(VERSION 3.1)

set(ur_modern_driver_SRC
  src/do_output.cpp
  src/robot_state.cpp
  src/robot_state_RT.cpp
  src/ur_communication.cpp
  src/ur_driver.cpp
  src/ur_realtime_communication.cpp
)

set(ur_modern_driver_HEADERS
  include/ur_modern_driver/do_output.h
  include/ur_modern_driver/robot_state.h
  include/ur_modern_driver/robot_state_RT.h
  include/ur_modern_driver/ur_communication.h
  include/ur_modern_driver/ur_driver.h
  include/ur_modern_driver/ur_realtime_communication.h
)

add_library(ur_modern_driver SHARED ${ur_modern_driver_SRC} ${ur_modern_driver_HEADERS})
target_include_directories(ur_modern_driver PUBLIC include)

install(TARGETS ur_modern_driver
  EXPORT ur_modern_driverTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
