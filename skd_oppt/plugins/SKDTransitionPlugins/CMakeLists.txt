#find cppzmq wrapper, installed by make of cppzmq
#find_package(cppzmq REQUIRED)

# Options to use OPENCV
include_directories(${MLPACK_LIBRARIES})

find_package( mlpack REQUIRED)



# Safe Traj Gen Plugins
ADD_TRANSITION_PLUGIN(SafeTrajGenTransitionPlugin ${CMAKE_CURRENT_LIST_DIR}/SafeTrajGenTransitionPlugin.cpp)
target_include_directories(SafeTrajGenTransitionPlugin PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(SafeTrajGenTransitionPlugin ${MLPACK_LIBRARIES})


# Kamikaze Trajectory generation with Basic Controller as a model 	
ADD_TRANSITION_PLUGIN(KamikazeTrajGenTransitionPlugin ${CMAKE_CURRENT_LIST_DIR}/KamikazeTrajGenTransitionPlugin.cpp)
target_include_directories(KamikazeTrajGenTransitionPlugin PRIVATE ${PROJECT_SOURCE_DIR})




