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

include_directories(. .. ../../gym_mapf)

# unit tests
add_executable(
        solvers_ut

        heuristics_tests.cpp
        online_tests.cpp
)

target_link_libraries(
        solvers_ut

        gtest_main
        $<TARGET_OBJECTS:gym_mapf>
        $<TARGET_OBJECTS:gym_mapf_solvers>
        tsl::hopscotch_map
)


include(GoogleTest)