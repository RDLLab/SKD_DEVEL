# Observation plugin for kamikazeTrajGen experiments
ADD_OBSERVATION_PLUGIN(SKDGenObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/SKDGenObservationPlugin.cpp)
target_include_directories(SKDGenObservationPlugin PRIVATE ${PROJECT_SOURCE_DIR})


# Observation plugin for kamikaze experiments
ADD_OBSERVATION_PLUGIN(SafeTrajGenObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/SafeTrajGenObservationPlugin.cpp)
target_include_directories(SafeTrajGenObservationPlugin PRIVATE ${PROJECT_SOURCE_DIR})

