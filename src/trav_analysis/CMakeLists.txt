cmake_minimum_required(VERSION 3.5)
project(trav_analysis)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(cloud_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
#find_package(GTSAM REQUIRED QUIET)

find_package(OpenCV REQUIRED )
#find_package(Boost REQUIRED COMPONENTS thread serialization timer)

set(dependencies
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
#        nav_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        pcl_conversions
        pcl_ros
#        cloud_msgs
        )

include_directories( include
        ${OpenCV_INCLUDE_DIRS}
        )


add_library(CellManagement src/utils/CellManagement.cpp )
add_library(GridManagement src/utils/GridManagement.cpp )

#set ( CUSTOM_LIBRARIES  Stats  DatasetStats  IOUtils  CellManagement GridManagement Features  Camera )
set ( CUSTOM_LIBRARIES  CellManagement  GridManagement)

target_link_libraries(GridManagement ${PCL_LIBRARIES}  CellManagement )


ament_target_dependencies(CellManagement  ${dependencies})
ament_target_dependencies(GridManagement  ${dependencies})

add_executable(${PROJECT_NAME}_predicter src/predicter.cpp)
target_link_libraries(${PROJECT_NAME}_predicter ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CUSTOM_LIBRARIES} )
ament_target_dependencies(${PROJECT_NAME}_predicter ${dependencies} Eigen3 )

install(TARGETS
        ${PROJECT_NAME}_predicter  ${CUSTOM_LIBRARIES}
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        config
        launch
        DESTINATION share/${PROJECT_NAME}
)



install(DIRECTORY include/
        DESTINATION include/
        )

ament_export_include_directories(include)
ament_export_libraries(${CUSTOM_LIBRARIES})
ament_export_dependencies(${dependencies})

ament_package()