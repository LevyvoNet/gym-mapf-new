include_directories(. .. ../gym_mapf)

add_library(gym_mapf_solvers SHARED
        solvers.h
        utils/policy/policy.h
        utils/policy/policy.cpp
        value_iteartion/value_iteration.h
        value_iteartion/value_iteration.cpp
        )
target_include_directories(gym_mapf_solvers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
