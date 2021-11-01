# Import gtest
include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
enable_testing()


include_directories(
        .
        ../third-party/hopscotch-map/include
)


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
        )

target_include_directories(gym_mapf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


# unit tests
add_executable(
        gym_mapf_ut

        tests/grid_test.cpp
        tests/mapf_env_tests.cpp
        tests/utils_tests.cpp
)
target_link_libraries(
        gym_mapf_ut

        gtest_main
        $<TARGET_OBJECTS:gym_mapf>
        tsl::hopscotch_map
)

include(GoogleTest)
