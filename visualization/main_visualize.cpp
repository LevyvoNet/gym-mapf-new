//
// Created by levyvonet on 30/12/2021.
//

#include <gym_mapf/gym_mapf.h>
#include <solvers/solvers.h>
#include <visualization/visualization.h>
#include <unistd.h>

#define LONG_TIME_SEC (60)
#define LONG_TIME_MS (LONG_TIME_SEC * 1000)

void render_stuff(){
    MapfEnv *env = create_mapf_env("empty-8-8", 1, 2, 0, -1000, -1, -1);
    Policy* policy = new RtdpPolicy(env, 1.0, "rtdp", new RtdpDijkstraHeuristic(1.0));
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

        do {
            /* Render */
            window.clear();
            render(policy->env, &window);
            window.display();
            sleep(1);

            /* Execute new action */
            selected_action = policy->act(*(policy->env->s), LONG_TIME_MS);
            policy->env->step(*selected_action, &next_state, &reward, &done, &is_collision);
        } while (!done);

        /* Render for the last time */
        window.clear();
        render(env, &window);
        window.display();

    }
}

int main(int argc, char **argv) {
    render_stuff();

    return 0;
}