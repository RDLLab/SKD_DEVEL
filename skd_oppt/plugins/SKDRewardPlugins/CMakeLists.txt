# Kamikaze Plugins
ADD_REWARD_PLUGIN(KamikazeTrajGenRewardPlugin ${CMAKE_CURRENT_LIST_DIR}/KamikazeTrajGenRewardPlugin.cpp)
target_include_directories(KamikazeTrajGenRewardPlugin PRIVATE ${PROJECT_SOURCE_DIR})

# Basic Safety Plugins
ADD_REWARD_PLUGIN(SafeTrajGenRewardPlugin ${CMAKE_CURRENT_LIST_DIR}/SafeTrajGenRewardPlugin.cpp)
target_include_directories(SafeTrajGenRewardPlugin PRIVATE ${PROJECT_SOURCE_DIR})


