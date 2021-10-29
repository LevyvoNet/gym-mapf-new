cmake_minimum_required(VERSION 3.20)
project(gym_mapf)

set(CMAKE_CXX_STANDARD 20)


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
)

add_library(gym_mapf SHARED
        gym_mapf/gym_mapf.h
        gym_mapf/gym_mapf.cpp
        gym_mapf/grid/grid.h
        gym_mapf/grid/grid.cpp
        gym_mapf/mapf_env/mapf_env.h
        gym_mapf/mapf_env/mapf_env.cpp
        gym_mapf/utils/utils.h
        gym_mapf/utils/utils.cpp
        )

add_library(gym_mapf_solvers SHARED
        solvers/utils/policy/policy.h
        solvers/utils/policy/policy.cpp
        solvers/value_iteartion/value_iteration.h
        solvers/value_iteartion/value_iteration.cpp
        solvers/utils/value_table/value_table.h
        solvers/utils/value_table/value_table.cpp
        )

add_executable(
        main

        main.cpp
        gym_mapf/gym_mapf.h
        solvers/value_iteartion/value_iteration.h
)
target_link_libraries(
        main

        $<TARGET_OBJECTS:gym_mapf>
        $<TARGET_OBJECTS:gym_mapf_solvers>
)

# unit tests
add_executable(
        gym_mapf_ut

        gym_mapf/gym_mapf.h
        gym_mapf/tests/mapf_env_tests.cpp
        gym_mapf/tests/grid_test.cpp
        gym_mapf/tests/utils_tests.cpp
)
target_link_libraries(
        gym_mapf_ut

        gtest_main
        $<TARGET_OBJECTS:gym_mapf>
)

include(GoogleTest)
