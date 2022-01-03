include_directories(. .. ../gym_mapf ../solvers)


add_executable(
        main_visualize
        main_visualize.cpp
        visualization.cpp
        ../benchmark.h
)


target_link_libraries(
        main_visualize

        $<TARGET_OBJECTS:gym_mapf>
        $<TARGET_OBJECTS:gym_mapf_solvers>
        -lsfml-graphics
        -lsfml-window
        -lsfml-system

        tsl::hopscotch_map
)