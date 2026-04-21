cmake_minimum_required(VERSION 3.16.3)
project(manifold_test) #The name of your choice for the project comes here

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_subdirectory(manifold)
add_subdirectory(point_cloud_data_loader)
add_subdirectory(monte_carlo)
add_subdirectory(experiment_manager)
add_subdirectory(trajectory_optimization_on_manifold)
add_compile_options(-std=c++14) #optional to use c++11
option(MANIFOLD_BUILD_LEGACY_APP "Build legacy manifold_test executable" OFF)


find_package(Eigen3 REQUIRED)
find_package(voxblox QUIET)
link_directories(${Eigen_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)

set(VOXBLOX_LOCAL_INCLUDE_DIRS
  /home/shekoufeh/fov/FIF_ws/src/voxblox/voxblox/include
  /home/shekoufeh/fov/FIF_ws/src/minkindr/minkindr/include
  /home/shekoufeh/fov/FIF_ws/devel/include
  /home/shekoufeh/fov/FIF_ws/devel/.private/voxblox/include
)
set(VOXBLOX_LOCAL_LIBS
  /home/shekoufeh/fov/FIF_ws/devel/lib/libvoxblox.so
  /home/shekoufeh/fov/FIF_ws/devel/lib/libvoxblox_proto.so
  /usr/lib/x86_64-linux-gnu/libglog.so.0
  /usr/lib/x86_64-linux-gnu/libgflags.so.2.2
)
set(VOXBLOX_LOCAL_AVAILABLE FALSE)
if(EXISTS "/home/shekoufeh/fov/FIF_ws/src/voxblox/voxblox/include/voxblox/core/esdf_map.h"
   AND EXISTS "/home/shekoufeh/fov/FIF_ws/devel/lib/libvoxblox.so")
  set(VOXBLOX_LOCAL_AVAILABLE TRUE)
endif()



#add manifold executable
if(MANIFOLD_BUILD_LEGACY_APP)
  add_executable(${PROJECT_NAME} "manifold_test.cpp") #The name of the cpp file and its path can vary
  target_include_directories(${PROJECT_NAME}
     PRIVATE manifold point_cloud_data_loader monte_carlo experiment_manager
  )
  target_link_directories (${PROJECT_NAME}
   PRIVATE manifold point_cloud_data_loader experiment_manager
  )
  target_link_libraries(${PROJECT_NAME}
     ${Eigen_LIBRARIES} manifold cloud_loader montecarlo e_manager
     )
endif()




#add trajectory optimizer executable


add_executable(${PROJECT_NAME}_trajectory "trajectory_test.cpp") #The name of the cpp file and its path can vary
target_include_directories(${PROJECT_NAME}_trajectory
   PRIVATE point_cloud_data_loader trajectory_optimization_on_manifold  monte_carlo experiment_manager manifold
)
target_link_directories (${PROJECT_NAME}_trajectory
 PRIVATE point_cloud_data_loader trajectory_optimization_on_manifold experiment_manager manifold 
)
target_link_libraries(${PROJECT_NAME}_trajectory
   ${Eigen_LIBRARIES}  manifold cloud_loader trajectory_optimizer e_manager montecarlo
   )

if(voxblox_FOUND)
  target_include_directories(${PROJECT_NAME}_trajectory PRIVATE ${voxblox_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_trajectory ${voxblox_LIBRARIES})
  message(STATUS "Voxblox found - ESDF occlusion filtering enabled")
elseif(VOXBLOX_LOCAL_AVAILABLE)
  target_include_directories(${PROJECT_NAME}_trajectory PRIVATE ${VOXBLOX_LOCAL_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_trajectory ${VOXBLOX_LOCAL_LIBS})
  message(STATUS "Using local FIF_ws Voxblox fallback - ESDF occlusion filtering enabled")
else()
  message(STATUS "Voxblox not found - ESDF occlusion filtering will be disabled")
endif()
