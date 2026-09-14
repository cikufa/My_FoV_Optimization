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

# Workspace-local standalone Voxblox fallback.  The original ROS/catkin
# workspace is not available on the current machine, but trajectory ESDF
# filtering only needs the Voxblox core map reader.
get_filename_component(FOV_WS_ROOT "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)
set(VOXBLOX_STANDALONE_INCLUDE_DIRS
  ${FOV_WS_ROOT}/voxblox_standalone_src/voxblox/include
  ${FOV_WS_ROOT}/minkindr_standalone_src/minkindr/include
  ${FOV_WS_ROOT}/voxblox_standalone_build/generated
)
set(VOXBLOX_STANDALONE_LIB
  ${FOV_WS_ROOT}/voxblox_standalone_build/libvoxblox_standalone.so
)
set(VOXBLOX_STANDALONE_AVAILABLE FALSE)
if(EXISTS "${FOV_WS_ROOT}/voxblox_standalone_src/voxblox/include/voxblox/core/esdf_map.h"
   AND EXISTS "${VOXBLOX_STANDALONE_LIB}")
  set(VOXBLOX_STANDALONE_AVAILABLE TRUE)
endif()

set(ACT_MAP_LOCAL_INCLUDE_DIRS
  /home/shekoufeh/fov/FIF_ws/src/rpg_information_field/act_map/include
  /home/shekoufeh/fov/FIF_ws/src/rpg_information_field/third_party/rpg_common/include
  /home/shekoufeh/fov/FIF_ws/src/rpg_information_field/third_party/rpg_vi_utils/include
  /home/shekoufeh/fov/FIF_ws/devel/include
  /home/shekoufeh/fov/FIF_ws/build/act_map
  /usr/include/opencv
)
set(ACT_MAP_LOCAL_LIBS
  /home/shekoufeh/fov/FIF_ws/devel/lib/libact_map.so
  /home/shekoufeh/fov/FIF_ws/devel/lib/libact_map_proto.so
)
set(ACT_MAP_LOCAL_AVAILABLE FALSE)
if(EXISTS "/home/shekoufeh/fov/FIF_ws/src/rpg_information_field/act_map/include/act_map/visibility_checker.h"
   AND EXISTS "/home/shekoufeh/fov/FIF_ws/devel/lib/libact_map.so")
  set(ACT_MAP_LOCAL_AVAILABLE TRUE)
endif()

# Workspace-local source fallback for the historical act_map depth-map
# visibility backend.  This machine no longer has the original ROS1 catkin
# workspace under /home/shekoufeh/fov/FIF_ws, but the exact act_map sources and
# depth-map protobuf format are present in the adjacent FIF repository.
set(ACT_MAP_EMBEDDED_ROOT
  ${FOV_WS_ROOT}/my_FIF-perception-aware-planning
)
if(NOT ACT_MAP_LOCAL_AVAILABLE
   AND EXISTS "${ACT_MAP_EMBEDDED_ROOT}/act_map/include/act_map/visibility_checker.h")
  find_package(Protobuf REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core)
  protobuf_generate_cpp(ACT_MAP_PROTO_SRCS ACT_MAP_PROTO_HDRS
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/proto/voxblox/Block_act_map.proto
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/proto/voxblox/Layer_act_map.proto
  )
  file(GLOB ACT_MAP_VOXBLOX_SRCS
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/src/voxblox/*.cc
  )
  add_library(fov_act_map_depthmap STATIC
    ${ACT_MAP_PROTO_SRCS}
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/src/blocks_serialization.cpp
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/src/conversion.cpp
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/src/depth_voxel.cpp
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/src/depth_map.cpp
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/src/quadratic_factor_voxel.cpp
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/src/sampler.cpp
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/src/visibility_checker.cpp
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/src/voxblox_utils.cpp
    ${ACT_MAP_EMBEDDED_ROOT}/third_party/rpg_common/src/fs.cpp
    ${ACT_MAP_EMBEDDED_ROOT}/third_party/rpg_vi_utils/src/cam_min.cpp
    ${ACT_MAP_EMBEDDED_ROOT}/third_party/rpg_vi_utils/src/common_utils.cpp
    ${ACT_MAP_EMBEDDED_ROOT}/third_party/rpg_vi_utils/src/map.cpp
    ${ACT_MAP_VOXBLOX_SRCS}
  )
  target_include_directories(fov_act_map_depthmap PUBLIC
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/include
    ${ACT_MAP_EMBEDDED_ROOT}/third_party/rpg_common/include
    ${ACT_MAP_EMBEDDED_ROOT}/third_party/rpg_vi_utils/include
    ${FOV_WS_ROOT}/minkindr_standalone_src/minkindr/include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
  )
  target_compile_definitions(fov_act_map_depthmap PUBLIC GLOG_USE_GLOG_EXPORT=1)
  target_link_libraries(fov_act_map_depthmap PUBLIC
    ${OpenCV_LIBS}
    ${Protobuf_LIBRARIES}
    /usr/local/lib/libglog.so
    /usr/lib/x86_64-linux-gnu/libgflags.so.2.2
  )
  set(ACT_MAP_LOCAL_INCLUDE_DIRS
    ${ACT_MAP_EMBEDDED_ROOT}/act_map/include
    ${ACT_MAP_EMBEDDED_ROOT}/third_party/rpg_common/include
    ${ACT_MAP_EMBEDDED_ROOT}/third_party/rpg_vi_utils/include
    ${FOV_WS_ROOT}/minkindr_standalone_src/minkindr/include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${OpenCV_INCLUDE_DIRS}
  )
  set(ACT_MAP_LOCAL_LIBS fov_act_map_depthmap)
  set(ACT_MAP_LOCAL_AVAILABLE TRUE)
  message(STATUS "Using workspace source act_map fallback - prebuilt visibility depth-map filtering enabled")
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
  target_compile_definitions(${PROJECT_NAME}_trajectory PRIVATE FOV_HAVE_VOXBLOX=1)
  target_link_libraries(${PROJECT_NAME}_trajectory ${voxblox_LIBRARIES})
  message(STATUS "Voxblox found - ESDF occlusion filtering enabled")
elseif(VOXBLOX_LOCAL_AVAILABLE)
  target_include_directories(${PROJECT_NAME}_trajectory PRIVATE ${VOXBLOX_LOCAL_INCLUDE_DIRS})
  target_compile_definitions(${PROJECT_NAME}_trajectory PRIVATE FOV_HAVE_VOXBLOX=1)
  target_link_libraries(${PROJECT_NAME}_trajectory ${VOXBLOX_LOCAL_LIBS})
  message(STATUS "Using local FIF_ws Voxblox fallback - ESDF occlusion filtering enabled")
elseif(VOXBLOX_STANDALONE_AVAILABLE)
  target_include_directories(${PROJECT_NAME}_trajectory PRIVATE ${VOXBLOX_STANDALONE_INCLUDE_DIRS})
  target_compile_definitions(${PROJECT_NAME}_trajectory PRIVATE
    FOV_HAVE_VOXBLOX=1 GLOG_USE_GLOG_EXPORT=1)
  target_link_libraries(${PROJECT_NAME}_trajectory
    ${VOXBLOX_STANDALONE_LIB}
    /usr/local/lib/libglog.so
    /usr/lib/x86_64-linux-gnu/libgflags.so.2.2
    /usr/lib/x86_64-linux-gnu/libprotobuf.so)
  set_target_properties(${PROJECT_NAME}_trajectory PROPERTIES
    BUILD_RPATH "${FOV_WS_ROOT}/voxblox_standalone_build;/usr/local/lib")
  message(STATUS "Using workspace standalone Voxblox - ESDF occlusion filtering enabled")
else()
  message(STATUS "Voxblox not found - ESDF occlusion filtering will be disabled")
endif()

if(ACT_MAP_LOCAL_AVAILABLE)
  target_include_directories(${PROJECT_NAME}_trajectory PRIVATE ${ACT_MAP_LOCAL_INCLUDE_DIRS})
  target_compile_definitions(${PROJECT_NAME}_trajectory PRIVATE FOV_HAVE_ACT_MAP_DEPTHMAP=1)
  target_link_libraries(${PROJECT_NAME}_trajectory ${ACT_MAP_LOCAL_LIBS})
  message(STATUS "Using local FIF_ws act_map fallback - prebuilt visibility depth-map filtering enabled")
else()
  message(STATUS "act_map not found - prebuilt visibility depth-map filtering will be disabled")
endif()
