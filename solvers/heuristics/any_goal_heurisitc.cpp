//
// Created by levyvonet on 5/3/23.
//

#include "any_goal_heurisitc.h"

AnyGoalHeuristic::AnyGoalHeuristic(const vector<double> &values) :
        DijkstraHeuristic(), local_goals_v_(values) {}

double AnyGoalHeuristic::operator()(MultiAgentState *s) {
    double max_value = std::numeric_limits<double>::lowest();
    for (size_t agent = 0; agent < this->env->n_agents; ++agent) {
        double curr_value = this->local_goals_v_[agent] - this->distance[agent][s->locations[agent].id];
        max_value = max(max_value, curr_value);
    }

    return max_value;

}




