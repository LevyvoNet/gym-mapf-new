include_directories(
        .
        ../third-party/hopscotch-map/include
)

# Testing
add_subdirectory(tests)

add_library(gym_mapf SHARED
        gym_mapf.h
        gym_mapf.cpp
        grid/grid.h
        grid/grid.cpp
        mapf_env/mapf_env.h
        mapf_env/mapf_env.cpp
        utils/parsing/parsing_utils.h
        utils/parsing/parsing_utils.cpp
        multiagent_action/multiagent_action.h
        multiagent_action/multiagent_action.cpp
        multiagent_state/multiagent_state.h
        multiagent_state/multiagent_state.cpp
        utils/state_storage/state_storage.h
        utils/state_storage/state_storage.cpp
        utils/dictionary/dictionary.h
        utils/dictionary/dictionary.cpp
        )

target_include_directories(gym_mapf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

