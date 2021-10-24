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
        grid
        mapf_env
        tests
)

add_library(gym_mapf SHARED
        gym_mapf.h
        gym_mapf.cpp
        grid/grid.h
        grid/grid.cpp
        mapf_env/mapf_env.h
        mapf_env/mapf_env.cpp
        )

add_executable(
        main

        main.cpp
)
target_link_libraries(
        main

        $<TARGET_OBJECTS:gym_mapf>
)

# unit tests
add_executable(
        ut

        tests/mapf_env_tests.cpp
        tests/grid_test.cpp
)
target_link_libraries(
        ut

        gtest_main
        $<TARGET_OBJECTS:gym_mapf>
)

include(GoogleTest)
