//
// Created by levyvonet on 07/11/2021.
//

#include "id.h"

/** Default Policy Merger***************************************************************************************/
Policy *DefaultPolicyMerger::operator()(MapfEnv *env,
                                        float gamma,
                                        vector<vector<size_t>> groups,
                                        size_t group1,
                                        size_t group2,
                                        Policy *policy1,
                                        Policy *policy2) {
    CrossedPolicy joint_policy(env, gamma, "", groups, {policy1, policy2});
    MapfEnv *merged_env = merge_groups_envs(&joint_policy, group1, group2);
    joint_policy.policies.clear();

    Policy *policy = (*this->low_level_planner_creator)(merged_env, gamma);
    policy->train();

    return policy;
}

DefaultPolicyMerger::DefaultPolicyMerger(SolverCreator *low_level_planner_creator) :
        low_level_planner_creator(low_level_planner_creator) {}

/** ID *******************************************************************************************************/

IdPolicy::IdPolicy(MapfEnv *env, float gamma, const string &name,
                   SolverCreator *low_level_planner_creator, PolicyMerger *low_level_merger) :
        Policy(env, gamma, name),
        low_level_planner_creator(low_level_planner_creator), low_level_merger(low_level_merger) {
    if (nullptr == low_level_merger) {
        this->low_level_merger = new DefaultPolicyMerger(low_level_planner_creator);
    }
    this->joint_policy = nullptr;

}


CrossedPolicy *merge_groups(MapfEnv *env,
                            float gamma,
                            PolicyMerger *low_level_merger,
                            CrossedPolicy *joint_policy,
                            Conflict *conflict) {
    Policy *new_joint_policy = nullptr;
    vector<size_t> new_group;
    vector<vector<size_t>> new_groups;
    vector<Policy *> new_policies;

    /* Find the group of each agent */
    int a1_group = get_group_of_agent(joint_policy->groups, conflict->agent1);
    int a2_group = get_group_of_agent(joint_policy->groups, conflict->agent2);

    /* Merge the policies of both groups to a single one */
    new_joint_policy = (*low_level_merger)(env,
                                           gamma,
                                           joint_policy->groups,
                                           a1_group,
                                           a2_group,
                                           joint_policy->policies[a1_group],
                                           joint_policy->policies[a2_group]);

    /* Create the new groups */
    for (size_t i = 0; i < joint_policy->groups.size(); ++i) {
        if (i != a1_group && i != a2_group) {
            /* Just copy */
            new_groups.push_back(joint_policy->groups[i]);
            new_policies.push_back(joint_policy->policies[i]);

        } else if (i == a1_group) {
            /* Create the new group */
            new_group = joint_policy->groups[a1_group];
            new_group.insert(new_group.end(),
                             joint_policy->groups[a2_group].begin(),
                             joint_policy->groups[a2_group].end());

            /* Push the new group and its policy */
            new_groups.push_back(new_group);
            new_policies.push_back(new_joint_policy);
        }
    }

    return new CrossedPolicy(env, gamma, "", new_groups, new_policies);

}


void IdPolicy::train() {
    Conflict *conflict = nullptr;
    CrossedPolicy *curr_joint_policy = nullptr;
    CrossedPolicy *prev_joint_policy = nullptr;
    vector<vector<size_t>> groups(env->n_agents);
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;
    size_t conflicts_count = 0;
    std::chrono::steady_clock::time_point conflict_begin;
    float conflict_detection_time_milliseconds = 0;

    /* Solve Independently for each agent */
    for (size_t i = 0; i < env->n_agents; ++i) {
        groups[i] = {i};
    }
    curr_joint_policy = solve_local_and_cross(this->env, this->gamma, this->low_level_planner_creator, &groups);

    /* Search for conflicts and merge iteratively */
    do {
        /* Check for conflict */
        conflict_begin = std::chrono::steady_clock::now();
        conflict = detect_conflict(curr_joint_policy);
        conflict_detection_time_milliseconds += std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - conflict_begin).count();


        if (nullptr != conflict) {
            /* Merge the groups of the agents in the conflict */
            prev_joint_policy = curr_joint_policy;
            curr_joint_policy = merge_groups(env, gamma, low_level_merger, prev_joint_policy, conflict);
            ++conflicts_count;
        }
    } while (groups.size() != 1 || nullptr != conflict);

    /* Update train time measurement */
    end = std::chrono::steady_clock::now();
    auto elapsed_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    float elapsed_time_seconds = float(elapsed_time_milliseconds) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;

    /* Set additional training info */
    (*(this->train_info->additional_data))["n_conflicts"] = std::to_string(conflicts_count);
    float conflict_time = conflict_detection_time_milliseconds / 1000;
    (*(this->train_info->additional_data))["conflicts_time"] = std::to_string((int) round(conflict_time * 100) / 100);

    this->joint_policy = curr_joint_policy;
}

MultiAgentAction *IdPolicy::act(const MultiAgentState &state) {
    return this->joint_policy->act(state);
}

IdPolicy::~IdPolicy() {
    delete this->joint_policy;
}



