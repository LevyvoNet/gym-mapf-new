//
// Created by levyvonet on 5/3/23.
//

#include "any_goal_heurisitc.h"

AnyGoalHeuristic::AnyGoalHeuristic(vector<bool> is_important) : is_important_(is_important), DijkstraHeuristic() {}

double AnyGoalHeuristic::operator()(MultiAgentState *s) {
    double max_value = -std::numeric_limits<double>::infinity();;
    for (size_t agent = 0; agent < this->env->n_agents; ++agent) {
        if (!is_important_[agent]) {
            continue;
        }
        double curr_value =
                this->env->reward_of_living * this->distance[agent][s->locations[agent].id] + this->env->reward_of_goal;
        max_value = max(max_value, curr_value);
    }

    return max_value;

}




