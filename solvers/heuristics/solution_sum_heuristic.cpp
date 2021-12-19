//
// Created by levyvonet on 05/12/2021.
//

#include "solution_sum_heuristic.h"


SolutionSumHeuristic::SolutionSumHeuristic(vector<ValueFunctionPolicy *> policies, vector<vector<size_t>> groups) :
        policies(policies), groups(groups) {}

SolutionSumHeuristic::~SolutionSumHeuristic() {
    for (Policy *p: this->policies) {
        delete p->env;
        delete p;
    }
}

void SolutionSumHeuristic::init(MapfEnv *env_param) {
    this->env = env_param;
}


double SolutionSumHeuristic::operator()(MultiAgentState *s) {
    double sum = 0;
    size_t relevant_values = 0;
    vector<vector<Location>> local_locations(this->groups.size());
    MultiAgentState *local_state;
    size_t curr_group = 0;

    /* Extract the local states of the two groups from the merged state */
    for (size_t i = 0; i < s->locations.size(); ++i) {
        local_locations[curr_group].push_back(s->locations[i]);
        if (local_locations[curr_group].size() == this->groups[curr_group].size()) {
            ++curr_group;
        }
    }

    /* Add each value to the sum if it is not the local goal state */
    for (size_t i = 0; i < this->groups.size(); ++i) {
        local_state = this->policies[i]->env->locations_to_state(local_locations[i]);
        if (*local_state != *this->policies[i]->env->goal_state) {
            sum += this->policies[i]->get_value(local_state);
            ++relevant_values;
        }
        delete local_state;
    }

    if (0 == relevant_values) {
        return 0;
    }

    return sum - (relevant_values - 1) * this->env->reward_of_goal;
}

