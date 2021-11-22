//
// Created by levyvonet on 19/11/2021.
//

#include "crossed_policy.h"

/** Crossed Policy ********************************************************************************************/
CrossedPolicy::CrossedPolicy(MapfEnv *env, float gamma, const string &name, vector<vector<size_t>> groups,
                             vector<Policy *> policies) : Policy(env, gamma, name), groups(groups),
                                                          policies(policies) {}

void CrossedPolicy::train() {}

MultiAgentAction *CrossedPolicy::act(const MultiAgentState &state) {
    vector<Action> actions(this->env->n_agents);
    MultiAgentAction *group_action = nullptr;
    vector<Location> group_locations;
    MultiAgentState *group_state = nullptr;
    int64_t action_id = 0;
    int action_space_size = 5;

    for (size_t i = 0; i < this->groups.size(); ++i) {
        for (size_t a: this->groups[i]) {
            group_locations.push_back(state.locations[a]);
        }
        group_state = this->policies[i]->env->locations_to_state(group_locations);
        group_action = this->policies[i]->act(*group_state);
        for (size_t j = 0; j < this->groups[i].size(); ++j) {
            actions[this->groups[i][j]] = group_action->actions[j];
        }
        group_locations.clear();
    }

    /* Calculate the new action ID */
    for (size_t i = 0; i < actions.size(); ++i) {
        action_id += actions[i] * (pow(5, i));
    }

    return new MultiAgentAction(actions, action_id);
}

CrossedPolicy::~CrossedPolicy() {
    for (Policy* p:this->policies){
        delete p;
    }

}

/** Utility Functions ********************************************************************************************/

CrossedPolicy *solve_local_and_cross(MapfEnv *env, float gamma, SolverCreator *low_level_planner_creator, vector<vector<size_t>> *groups) {
    vector<Policy *> policies;
    MapfEnv *local_env = nullptr;
    Policy *group_policy = nullptr;

    for (vector<size_t> group: *groups) {
        local_env = get_local_view(env, group);
        group_policy = (*low_level_planner_creator)(local_env, gamma);
        group_policy->train();
        policies.push_back(group_policy);
    }

    return new CrossedPolicy(env, gamma, "", *groups, policies);
}

MapfEnv *merge_groups_envs(CrossedPolicy *joint_policy, size_t g1, size_t g2) {
    vector<size_t> merged_agents;
    vector<Location> new_locations;


    for (size_t i = 0; i < joint_policy->groups[g1].size(); ++i) {
        merged_agents.push_back(joint_policy->groups[g1][i]);
        new_locations.push_back(joint_policy->policies[g1]->env->start_state->locations[i]);
    }
    for (size_t i = 0; i < joint_policy->groups[g2].size(); ++i) {
        merged_agents.push_back(joint_policy->groups[g2][i]);
        new_locations.push_back(joint_policy->policies[g2]->env->start_state->locations[i]);
    }


    return get_local_view(joint_policy->env, merged_agents);
}

int find_group_of_agent(size_t agent, const vector<vector<size_t>> &groups) {
    for (size_t i = 0; i < groups.size(); ++i) {
        for (size_t j = 0; j < groups[i].size(); ++j) {
            if (groups[i][j] == agent) {
                return i;
            }
        }
    }

    return -1;
}