# Collision Plugins


# Basic Safety Plugins
ADD_TERMINAL_PLUGIN(KamikazeTrajGenTerminalPlugin ${CMAKE_CURRENT_LIST_DIR}/KamikazeTrajGenTerminalPlugin.cpp)
target_include_directories(KamikazeTrajGenTerminalPlugin PRIVATE ${PROJECT_SOURCE_DIR})



# Safety Plugins
ADD_TERMINAL_PLUGIN(SafeTrajGenTerminalPlugin ${CMAKE_CURRENT_LIST_DIR}/SafeTrajGenTerminalPlugin.cpp)
target_include_directories(SafeTrajGenTerminalPlugin PRIVATE ${PROJECT_SOURCE_DIR})