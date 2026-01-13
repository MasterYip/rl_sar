cmake_minimum_required(VERSION 3.10)
project(fdilink_ahrs_standalone)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find serial library (you may need to install wjwwood/serial)
# For demonstration, we assume it's available via find_package or as a system library
# If not available, you can build it from source or use pkg-config
find_library(SERIAL_LIB serial REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Build static library for AHRS interface
add_library(fdilink_ahrs STATIC
  src/ahrs_interface.cpp
  src/crc_table.cpp
)

target_link_libraries(fdilink_ahrs
  ${SERIAL_LIB}
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