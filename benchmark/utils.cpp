//
// Created by levyvonet on 16/01/2022.
//

#include "utils.h"

void add_mountains_to_env(MapfEnv *env) {
    int grid_area = (env->grid->max_row + 1) * ((env->grid->max_col + 1));
    vector<Location> mountain_centers;


    vector<vector<size_t>> groups(env->n_agents);
    for (size_t i = 0; i < env->n_agents; ++i) {
        groups[i] = {i};
    }
    CrossedPolicy *p = solve_local_and_cross(env,
                                             1.0,
                                             BENCHMARK_LONG_TIME_MS,
                                             new dijkstra_baseline(""),
                                             &groups);

    for (size_t agent_idx = 0; agent_idx < env->n_agents; ++agent_idx) {
        MultiAgentState *s = p->env->locations_to_state({p->env->start_state->locations[agent_idx]});
        Location l = s->locations[0];
        int path_length = ((DijkstraBaselinePolicy *) (p->policies[agent_idx]))->h->distance[0][l.id];

        /* Add a mountain in middle of path */
        size_t j = 0;
        for (; j < path_length / 2; ++j) {
            MultiAgentAction *a = p->policies[agent_idx]->act(MultiAgentState({l}, l.id), BENCHMARK_LONG_TIME_MS);
            l = p->env->grid->execute(l, a->actions[0]);
            delete a;
        }
        int mountain_dim = min(3, path_length / 8);

        int mountain_top = max(0, l.row - mountain_dim);
        int mountain_bottom = min((int) env->grid->max_row, l.row + mountain_dim);
        int mountain_left = max(0, l.col - mountain_dim);
        int mountain_right = min((int) env->grid->max_col, l.col + mountain_dim);
        env->add_mountain(GridArea(mountain_top, mountain_bottom, mountain_left, mountain_right));
    }
}