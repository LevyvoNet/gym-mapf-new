include_directories(. .. ../gym_mapf ../solvers)


add_executable(
        main_visualize
        main_visualize.cpp
        visualization.cpp
        ../benchmark/available_solvers_envs.h
        ../benchmark/available_solvers_envs.cpp
        ../benchmark/utils.h
        ../benchmark/utils.cpp
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