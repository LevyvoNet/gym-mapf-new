//
// Created by levyvonet on 22/11/2021.
//

#include "rtdp_dijkstra_heuristic.h"

RtdpDijkstraHeuristic::RtdpDijkstraHeuristic(float gamma) : gamma(gamma) {
    this->env = nullptr;
}

void RtdpDijkstraHeuristic::init(MapfEnv *env_param, double timeout_ms) {
    MEASURE_TIME;
    MapfEnv *local_env = nullptr;
    RtdpPolicy *local_policy = nullptr;

    this->env = env_param;

    /* Calculate local policies for each agent */
    for (size_t agent = 0; agent < this->env->n_agents; ++agent) {
        local_env = get_local_view(this->env, {agent});
        local_policy = new RtdpPolicy(local_env, this->gamma, "", new DijkstraHeuristic());
        local_policy->train(timeout_ms - ELAPSED_TIME_MS);
        this->local_policies.push_back(local_policy);
    }

}

RtdpDijkstraHeuristic::~RtdpDijkstraHeuristic() {
    for (RtdpPolicy *local_policy: this->local_policies) {
        delete local_policy->env;
        delete local_policy;
    }
}

double RtdpDijkstraHeuristic::operator()(MultiAgentState *s) {
    double sum = 0;
    size_t count = 0;
    MapfEnv *local_env = nullptr;
    MultiAgentState *local_state = nullptr;

    for (size_t agent = 0; agent < this->env->n_agents; ++agent) {
        if (s->locations[agent] != this->env->goal_state->locations[agent]) {
            local_env = this->local_policies[agent]->env;
            local_state = local_env->locations_to_state(
                    {local_env->grid->get_location(s->locations[agent].row, s->locations[agent].col)});
            if (!(*local_state == *local_env->goal_state)) {
                sum += this->local_policies[agent]->get_value(local_state);
                ++count;
            }
            delete local_state;
        }
    }

    /* If everyone is in goal_definition, return 0 and don't add the goal_definition reward in case there is one */
    if (0 == count) {
        return 0;
    }

    return sum - (count - 1) * this->env->reward_of_goal;
}


