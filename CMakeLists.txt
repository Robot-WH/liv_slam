cmake_minimum_required(VERSION 2.8.3)
project(liv_slam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
roscpp
pcl_conversions
pcl_msgs
pcl_ros
sensor_msgs
message_generation
ndt_omp
)

find_package(catkin REQUIRED )
find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(G2O REQUIRED)
#find_package(Eigen3)
find_package(OpenCV REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR} 
  ${G2O_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${G2O_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# link_libraries(${G2O_LIBRARIES})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

# ?????
catkin_package(
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

# ????
set(ALL_TARGET_LIBRARIES "")
# ????
include(cmake/geographic.cmake)

file(GLOB_RECURSE ALL_SRCS "*.cpp")     # 所有的.cpp文件  放置到 ALL_SRCS 
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")     # 所有的node cpp 文件
file(GLOB_RECURSE other_SRCS "src/apps/feature_based_FrontEnd/*.cpp")     # 所有的node cpp 文件
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_part/*.cpp") # 所有第三方库 cpp文件
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})                # ALL_SRCS 中剔除node cpp文件
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})         # ALL_SRCS 中剔除库cpp文件 
list(REMOVE_ITEM ALL_SRCS ${other_SRCS}) 


###########
## Build ##
###########

add_executable(PointCloudProcess
  src/apps/LidarPreprocessing_node.cpp
)
add_dependencies(PointCloudProcess ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(PointCloudProcess
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(Segmentation
  src/apps/Segmentation_node.cpp
)
add_dependencies(Segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Segmentation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


add_executable( basic_lidar_odometry
  src/apps/basic_FrontEnd/lidarOdometry_node.cpp
)
add_dependencies(basic_lidar_odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(basic_lidar_odometry
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable( MapOptimator_node
   src/apps/basic_FrontEnd/MapOptimator_node.cpp
)
add_dependencies(MapOptimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MapOptimator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(ascanRegistration src/apps/feature_based_FrontEnd/scanRegistration.cpp)
add_dependencies(ascanRegistration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserOdometry src/apps/feature_based_FrontEnd/laserOdometry.cpp)
add_dependencies(alaserOdometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/apps/feature_based_FrontEnd/laserMapping.cpp)
add_dependencies(alaserMapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})



add_executable(BackEnd_node
  src/apps/BackEnd_node.cpp
  src/lib/keyframe.cpp
  src/lib/information_matrix_calculator.cpp
  src/lib/robust_kernel_io.cpp
  src/lib/map_cloud_generator.cpp
  ${ALL_SRCS}
)
add_dependencies(BackEnd_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(BackEnd_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
  ${ALL_TARGET_LIBRARIES}
)



