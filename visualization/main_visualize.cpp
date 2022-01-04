//
// Created by levyvonet on 30/12/2021.
//

#include <gym_mapf/gym_mapf.h>
#include <solvers/solvers.h>
#include <visualization/visualization.h>
#include <benchmark.h>
#include <unistd.h>

#define LONG_TIME_SEC (60)
#define LONG_TIME_MS (LONG_TIME_SEC * 1000)
#define SLEEP_TIME_nS (200000)

void render_solver_on_env(EnvCreator *env_creator, SolverCreator *solver_creator) {
    MapfEnv *env = (*env_creator)();
    add_mountains_to_env(env);
    Policy *policy = (*solver_creator)(env, 1.0);
    policy->train(LONG_TIME_MS);

    int reward = 0;
    bool done = false;
    bool is_collision = false;
    MultiAgentAction *selected_action = nullptr;
    MultiAgentState next_state(policy->env->start_state->locations, -1);

    unsigned int window_size = min(sf::VideoMode::getDesktopMode().width,
                                   sf::VideoMode::getDesktopMode().height);

    sf::RenderWindow window(sf::VideoMode(window_size, window_size), "MAPF");

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        done = false;
        policy->reset();

        do {
            /* Render */
            window.clear();
            render(policy->env, &window);
            window.display();
            usleep(SLEEP_TIME_nS);

            /* Execute new action */
            selected_action = policy->act(*(policy->env->s), LONG_TIME_MS);
            policy->env->step(*selected_action, &next_state, &reward, &done, &is_collision);
        } while (!done);

        /* Render for the last time */
        window.clear();
        render(env, &window);
        window.display();

        sleep(2);
    }

}


int main(int argc, char **argv) {
    render_solver_on_env( new MazeEnv("maze-128-128-10_scen_2_5-agents", 128, 10, 2, 5),
                         new online_replan("online_replan_rtdp_2", 2, new rtdp_dijkstra_rtdp("")));

    return 0;
}