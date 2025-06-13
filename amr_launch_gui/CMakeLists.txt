cmake_minimum_required(VERSION 3.8)
project(amr_launch_gui)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui UiTools)
find_package(amr_launcher REQUIRED)

#qt_standard_project_setup()

#enable meta object compilation
set(CMAKE_AUTOMOC TRUE)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${amr_launcher_INCLUDE_DIRS})

add_library(debug_tools ${PROJECT_SOURCE_DIR}/src/debug_tools.cpp)
ament_target_dependencies(debug_tools PUBLIC ament_cmake rclcpp)
target_link_libraries(debug_tools PRIVATE Qt5::Core Qt5::Widgets Qt5::Gui Qt5::UiTools)

add_library(launch_data ${PROJECT_SOURCE_DIR}/src/launch_data.cpp)
target_link_libraries(launch_data PRIVATE Qt5::Core Qt5::Widgets Qt5::Gui Qt5::UiTools debug_tools)

add_library(central_data ${PROJECT_SOURCE_DIR}/src/central_data.cpp)
target_link_libraries(central_data PRIVATE Qt5::Core Qt5::Widgets Qt5::Gui Qt5::UiTools launch_data debug_tools)

add_library(launch_strip ${PROJECT_SOURCE_DIR}/src/launch_strip.cpp)
ament_target_dependencies(launch_strip PUBLIC ament_cmake rclcpp)
target_link_libraries(launch_strip PRIVATE Qt5::Core Qt5::Widgets Qt5::Gui Qt5::UiTools debug_tools)

add_library(device_tab ${PROJECT_SOURCE_DIR}/src/device_tab.cpp)
ament_target_dependencies(device_tab PUBLIC ament_cmake rclcpp)
target_link_libraries(device_tab PRIVATE Qt5::Core Qt5::Widgets Qt5::Gui Qt5::UiTools launch_strip debug_tools launch_data)

add_library(central_tab ${PROJECT_SOURCE_DIR}/src/central_tab.cpp)
ament_target_dependencies(central_tab PUBLIC ament_cmake rclcpp)
target_link_libraries(central_tab PRIVATE Qt5::Core Qt5::Widgets Qt5::Gui Qt5::UiTools device_tab debug_tools launch_data central_data)

add_library(server_wrapper ${PROJECT_SOURCE_DIR}/src/server_wrapper.cpp)
ament_target_dependencies(server_wrapper PUBLIC ament_cmake rclcpp amr_launcher)
target_link_libraries(server_wrapper PRIVATE Qt5::Core Qt5::Widgets Qt5::Gui Qt5::UiTools debug_tools)

add_library(main_window ${PROJECT_SOURCE_DIR}/src/mainwindow.cpp)
ament_target_dependencies(main_window PUBLIC ament_cmake rclcpp amr_launcher)
target_link_libraries(main_window PRIVATE Qt5::Core Qt5::Widgets Qt5::Gui Qt5::UiTools device_tab central_tab debug_tools server_wrapper)

add_executable(launch_gui src/main.cpp)
ament_target_dependencies(launch_gui PUBLIC ament_cmake rclcpp)
target_link_libraries(launch_gui PRIVATE Qt5::Core Qt5::Widgets Qt5::Gui main_window)

file(GLOB ui_files RELATIVE ${PROJECT_SOURCE_DIR} ui/*.ui)
file(GLOB resource_files RELATIVE ${PROJECT_SOURCE_DIR} ui/resource/*)

install(TARGETS
  launch_gui
  DESTINATION lib/${PROJECT_NAME}
)

#install ui files
install(PROGRAMS 
  ${ui_files}
  DESTINATION share/${PROJECT_NAME}/ui
)

#install resource files
install(PROGRAMS 
  ${resource_files}
  DESTINATION share/${PROJECT_NAME}/ui/resource
)

ament_package()
