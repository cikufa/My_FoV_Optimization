cmake_minimum_required(VERSION 3.16.3)
project(manifold_test) #The name of your choice for the project comes here

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_subdirectory(manifold)
add_subdirectory(point_cloud_data_loader)
add_subdirectory(monte_carlo)
add_subdirectory(experiment_manager)
add_subdirectory(trajectory_optimization_on_manifold)
add_compile_options(-std=c++14) #optional to use c++11


find_package(Eigen3 REQUIRED)
link_directories(${Eigen_INCLUDE_DIRS})



#add manifold executable

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