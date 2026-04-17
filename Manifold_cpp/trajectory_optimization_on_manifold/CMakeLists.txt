find_package(voxblox QUIET)

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

add_library(trajectory_optimizer trajectory_optimizer.h)
set_target_properties(trajectory_optimizer PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(trajectory_optimizer PUBLIC /usr/include/eigen3)

if(voxblox_FOUND)
  target_link_libraries(trajectory_optimizer PUBLIC ${voxblox_LIBRARIES})
  target_include_directories(trajectory_optimizer PUBLIC ${voxblox_INCLUDE_DIRS})
elseif(EXISTS "/home/shekoufeh/fov/FIF_ws/src/voxblox/voxblox/include/voxblox/core/esdf_map.h"
       AND EXISTS "/home/shekoufeh/fov/FIF_ws/devel/lib/libvoxblox.so")
  target_link_libraries(trajectory_optimizer PUBLIC ${VOXBLOX_LOCAL_LIBS})
  target_include_directories(trajectory_optimizer PUBLIC ${VOXBLOX_LOCAL_INCLUDE_DIRS})
endif()
