
cmake_minimum_required(VERSION 3.5)
project(NCAPPlugins)

set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=gnu++11 -pg")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og -std=gnu++11 -pg")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=gnu++11 -DEIGEN_NO_DEBUG")



# RelWithDebInfo is the default build type
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo) 
endif()

find_package(oppt)


# Include the CMakeLists of the different plugins
include(${CMAKE_CURRENT_LIST_DIR}/SKDInitialBeliefPlugins/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/SKDHeuristicPlugins/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/SKDObservationPlugins/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/SKDRewardPlugins/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/SKDTerminalPlugins/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/SKDTransitionPlugins/CMakeLists.txt)



