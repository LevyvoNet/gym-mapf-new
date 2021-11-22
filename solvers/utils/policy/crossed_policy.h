//
// Created by levyvonet on 19/11/2021.
//

#ifndef GYM_MAPF_CROSSED_POLICY_H
#define GYM_MAPF_CROSSED_POLICY_H

#include <gym_mapf/gym_mapf.h>
#include <solvers/utils/policy/policy.h>
#include <solvers/utils/utils.h>


class CrossedPolicy : public Policy {
public:
    vector<Policy *> policies;

    vector<vector<size_t>> groups;

    CrossedPolicy(MapfEnv *env, float gamma, const string &name, vector<vector<size_t>> groups,
                  vector<Policy *> policies);

    ~CrossedPolicy();

    virtual MultiAgentAction *act(const MultiAgentState &state);

    virtual void train();

};

CrossedPolicy *solve_local_and_cross(MapfEnv *env,
                                     float gamma,
                                     SolverCreator *low_level_planner_creator,
                                     vector<vector<size_t>> *groups);

MapfEnv *merge_groups_envs(CrossedPolicy *joint_policy, size_t g1, size_t g2);

#endif //GYM_MAPF_CROSSED_POLICY_H
