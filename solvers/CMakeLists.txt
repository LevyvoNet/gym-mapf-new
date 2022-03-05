include_directories(. .. ../gym_mapf ../third-party/hopscotch-map/include)

# Testing
add_subdirectory(tests)

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
        heuristics/solution_sum_heuristic.h
        heuristics/solution_sum_heuristic.cpp
        rtdp/rtdp.h
        rtdp/rtdp.cpp
        id/id.h
        id/id.cpp
        utils/conflict/conflict.h
        utils/conflict/conflict.cpp
        utils/utils.h
        dijkstra_baseline/dijkstra_baseline.h
        dijkstra_baseline/dijkstra_baseline.cpp
        online_window/online_window.h
        online_window/online_window.cpp
        )
target_include_directories(gym_mapf_solvers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
