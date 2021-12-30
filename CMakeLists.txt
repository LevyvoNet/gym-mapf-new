cmake_minimum_required(VERSION 3.20)
project(gym_mapf)

set(CMAKE_CXX_STANDARD 20)

# Mine
add_subdirectory(gym_mapf)
add_subdirectory(solvers)
#add_subdirectory(visualization)

# Third Party
add_subdirectory(third-party/hopscotch-map)


include_directories(. gym_mapf solvers third-party/hopscotch-map/include)

add_executable(
        main
        main.cpp
)


target_link_libraries(
        main

        $<TARGET_OBJECTS:gym_mapf>
        $<TARGET_OBJECTS:gym_mapf_solvers>

        tsl::hopscotch_map
)
