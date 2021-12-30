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

include_directories(. ..)

# unit tests
add_executable(
        gym_mapf_ut

        grid_test.cpp
        mapf_env_tests.cpp
        utils_tests.cpp
        state_storage_tests.cpp
)

target_link_libraries(
        gym_mapf_ut

        gtest_main
        $<TARGET_OBJECTS:gym_mapf>
        tsl::hopscotch_map
)

include(GoogleTest)