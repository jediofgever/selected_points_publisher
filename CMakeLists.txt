cmake_minimum_required(VERSION 3.5)
project(selected_points_publisher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(QT_LIBRARIES Qt5::Widgets)

include_directories(include)
qt5_wrap_cpp(MOC_FILES
  include/selected_points_publisher/selected_points_publisher.hpp
)

set(dependencies
geometry_msgs
pluginlib
Qt5
rclcpp
rviz_common
rviz_default_plugins
rviz_ogre_vendor
rviz_rendering
)

add_library(${PROJECT_NAME} SHARED
  src/selected_points_publisher.cpp
  ${MOC_FILES}
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}   
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)     

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()