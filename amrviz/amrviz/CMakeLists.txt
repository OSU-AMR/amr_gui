cmake_minimum_required(VERSION 3.8)
project(amrviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find general dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tinyxml2_vendor REQUIRED)

# find message deps
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(amr_meshes REQUIRED)
find_package(amr_msgs REQUIRED)

# rviz specific deps
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# pick up headerfiles
include_directories(include)

# pick up soirce files to build
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

# pick up headers to moc
file(GLOB MOC_HEADERS ${PROJECT_SOURCE_DIR}/include/riptide_rviz/*.hpp)
file(GLOB MOC_HEADERS2 ${PROJECT_SOURCE_DIR}/include/amrviz/*.hpp)

file(GLOB UIS ${PROJECT_SOURCE_DIR}/src/*.ui)

# moc the headers
foreach(header "${MOC_HEADERS}")
  qt5_wrap_cpp(MOC_FILES "${header}") # this should spit out the moc files for each header
endforeach()

foreach(header "${MOC_HEADERS2}")
  qt5_wrap_cpp(MOC_FILES "${header}") # this should spit out the moc files for each header
endforeach()


QT5_WRAP_UI( UI_HEADERS ${UIS} )

# add all of the source files here
add_library(${PROJECT_NAME} SHARED
            ${SRC_FILES}
            ${MOC_FILES}
            ${UI_HEADERS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_NS="talos")

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Link ament packages
set(deps
  rclcpp 
  rclcpp_action
  ament_index_cpp
  tf2
  rviz_common 
  rviz_rendering 
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  std_srvs
  nav_msgs
  diagnostic_msgs
  interactive_markers
  amr_msgs
)

ament_target_dependencies(${PROJECT_NAME} 
  ${deps}
)

# Link non ament packages
target_link_libraries(${PROJECT_NAME} 
  Qt5::Widgets
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  tinyxml2
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  rviz_ogre_vendor
  resource_retriever
  ament_index_cpp
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

file(GLOB python_programs python/*.py)
install(PROGRAMS 
  ${python_programs}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
  
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config"
  DESTINATION "share/${PROJECT_NAME}"
)

#Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
