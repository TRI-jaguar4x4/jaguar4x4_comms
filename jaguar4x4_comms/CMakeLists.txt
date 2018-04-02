cmake_minimum_required(VERSION 3.5)
project(jaguar4x4_comms)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

include_directories(include)

add_library(jaguar4x4_comms SHARED
  src/Communication.cpp
  src/MotorParse.cpp
  src/Utils.cpp
)

install(TARGETS
  jaguar4x4_comms
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(jaguar4x4_comms)

ament_package()
