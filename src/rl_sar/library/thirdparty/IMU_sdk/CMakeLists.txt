cmake_minimum_required(VERSION 3.10)
project(fdilink_ahrs_standalone)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Serial library is built from the serial subdirectory in the root project
# No need to find_library, we will link against the serial target directly

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Build static library for AHRS interface
add_library(fdilink_ahrs STATIC
  src/ahrs_interface.cpp
  src/crc_table.cpp
)

target_link_libraries(fdilink_ahrs
  serial
  pthread
)

# Build example executable
add_executable(print_imu
  examples/print_imu.cpp
)

target_link_libraries(print_imu
  fdilink_ahrs
)

# Install targets
install(TARGETS fdilink_ahrs print_imu
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/fdilink_ahrs
  FILES_MATCHING PATTERN "*.h"
)