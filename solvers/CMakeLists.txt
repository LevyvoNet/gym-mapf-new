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



include_directories(. .. ../gym_mapf ../third-party/hopscotch-map/include)

add_library(gym_mapf_solvers SHARED
        solvers.h
        utils/policy/policy.h
        utils/policy/policy.cpp
        utils/policy/value_function_policy.h
        utils/policy/value_function_policy.cpp
        utils/policy/crossed_policy.h
        utils/policy/crossed_policy.cpp
        value_iteartion/value_iteration.h
        value_iteartion/value_iteration.cpp
        heuristics/heuristic.h
        heuristics/heuristic.cpp
        heuristics/dijkstra_heuristic.h
        heuristics/dijkstra_heuristic.cpp
        heuristics/rtdp_dijkstra_heuristic.h
        heuristics/rtdp_dijkstra_heuristic.cpp
        rtdp/rtdp.h
        rtdp/rtdp.cpp
        id/id.h
        id/id.cpp
        online_replan/online_replan.h
        online_replan/online_replan.cpp
        utils/conflict/conflict.h
        utils/conflict/conflict.cpp
        utils/utils.h
        )
target_include_directories(gym_mapf_solvers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})




# unit tests
add_executable(
        solvers_ut

        tests/heuristics_tests.cpp
)

target_link_libraries(
        solvers_ut

        gtest_main
        $<TARGET_OBJECTS:gym_mapf>
        $<TARGET_OBJECTS:gym_mapf_solvers>
        tsl::hopscotch_map
)


include(GoogleTest)