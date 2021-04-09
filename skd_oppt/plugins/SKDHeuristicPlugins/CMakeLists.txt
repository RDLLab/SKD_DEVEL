
# Kamikaze Heuristics
ADD_HEURISTIC_PLUGIN(KamikazeTrajGenHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/KamikazeTrajGenHeuristicPlugin.cpp)
target_include_directories(KamikazeTrajGenHeuristicPlugin PRIVATE ${PROJECT_SOURCE_DIR})


# Safety Heuristics
ADD_HEURISTIC_PLUGIN(SafeTrajGenHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/SafeTrajGenHeuristicPlugin.cpp)
target_include_directories(SafeTrajGenHeuristicPlugin PRIVATE ${PROJECT_SOURCE_DIR})

