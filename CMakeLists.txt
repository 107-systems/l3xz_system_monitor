#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(l3xz_watchdog)
set(L3XZ_WATCHDOG_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#######################################################################################
add_subdirectory(external/json/current)
#######################################################################################
include_directories(${L3XZ_WATCHDOG_TARGET}
  include
)
#######################################################################################
add_executable(${L3XZ_WATCHDOG_TARGET}
  src/HeartbeatMonitor.cpp
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
target_compile_features(${L3XZ_WATCHDOG_TARGET} PRIVATE cxx_std_17)
#######################################################################################
target_link_libraries(${L3XZ_WATCHDOG_TARGET} NLOHMANN_JSON)
#######################################################################################
ament_target_dependencies(${L3XZ_WATCHDOG_TARGET} rclcpp std_msgs)
#######################################################################################
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
