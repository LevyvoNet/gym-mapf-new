//
// Created by levyvonet on 07/11/2021.
//

#include "id.h"

/** Default Policy Merger***************************************************************************************/
Policy *DefaultPolicyMerger::operator()(MapfEnv *env,
                                        float gamma,
                                        double timeout_ms,
                                        size_t group1,
                                        size_t group2,
                                        CrossedPolicy *joint_policy) {
    MapfEnv *merged_env = merge_groups_envs(joint_policy, group1, group2);

    Policy *policy = (*this->low_level_planner_creator)(merged_env, gamma);
    policy->train(timeout_ms);

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
                            double timeout_ms,
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
                                           timeout_ms,
                                           a1_group,
                                           a2_group,
                                           joint_policy);

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


void IdPolicy::train(double timeout_ms) {
    Conflict *conflict = nullptr;
    CrossedPolicy *curr_joint_policy = nullptr;
    CrossedPolicy *prev_joint_policy = nullptr;
    vector<vector<size_t>> groups(env->n_agents);
    size_t conflicts_count = 0;
    float conflict_detection_ms = 0;
    clock_t conflict_begin = 0;
    MEASURE_TIME;


    /* Solve Independently for each agent */
    for (size_t i = 0; i < env->n_agents; ++i) {
        groups[i] = {i};
    }
    curr_joint_policy = solve_local_and_cross(this->env,
                                              this->gamma,
                                              timeout_ms,
                                              this->low_level_planner_creator,
                                              &groups);
    if (ELAPSED_TIME_MS >= timeout_ms) {
        return;
    }

    /* Search for conflicts and merge iteratively */
    do {
        /* Check for conflict */
        conflict_begin = clock();
        conflict = detect_conflict(curr_joint_policy, timeout_ms - ELAPSED_TIME_MS);
        if (ELAPSED_TIME_MS >= timeout_ms) {
            return;
        }
        conflict_detection_ms += (((double) (clock() - conflict_begin)) / CLOCKS_PER_SEC) * 1000;

        if (nullptr != conflict) {
            /* Merge the groups of the agents in the conflict */
            prev_joint_policy = curr_joint_policy;
            curr_joint_policy = merge_groups(env,
                                             gamma,
                                             timeout_ms - ELAPSED_TIME_MS,
                                             low_level_merger,
                                             prev_joint_policy,
                                             conflict);
            ++conflicts_count;
        }
    } while (groups.size() != 1 && nullptr != conflict);

    /* Update train time measurement */
    float elapsed_time_seconds = float(ELAPSED_TIME_MS) / 1000;
    this->train_info->time = round(elapsed_time_seconds * 100) / 100;

    /* Set additional training info */
    (*(this->train_info->additional_data))["n_conflicts"] = std::to_string(conflicts_count);
    float conflict_time_sec = conflict_detection_ms / 1000;
    (*(this->train_info->additional_data))["conflicts_time"] = std::to_string(
            (int) round(conflict_time_sec * 100) / 100);

    this->joint_policy = curr_joint_policy;
}

MultiAgentAction *IdPolicy::act(const MultiAgentState &state, double timeout_ms) {
    return this->joint_policy->act(state, timeout_ms);
}

IdPolicy::~IdPolicy() {
    delete this->joint_policy;
}



