//
// Created by levyvonet on 07/11/2021.
//

#ifndef GYM_MAPF_ID_H
#define GYM_MAPF_ID_H

#include <gym_mapf/gym_mapf.h>
#include <solvers/utils/policy/policy.h>
#include <solvers/utils/utils.h>
#include <solvers/utils/policy/crossed_policy.h>
#include <solvers/utils/conflict/conflict.h>


class PolicyMerger {
public:
    virtual Policy *operator()(MapfEnv *env,
                               float gamma,
                               vector<vector<size_t>> groups,
                               size_t group1,
                               size_t group2,
                               Policy* policy1,
                               Policy* policy2) = 0;
};

class DefaultPolicyMerger:public PolicyMerger {
public:
    SolverCreator *low_level_planner_creator;

    DefaultPolicyMerger(SolverCreator *low_level_planner_creator);

    virtual Policy *operator()(MapfEnv *env,
                               float gamma,
                               vector<vector<size_t>> groups,
                               size_t group1,
                               size_t group2,
                               Policy* policy1,
                               Policy* policy2);
};

class IdPolicy : public Policy {
public:
    SolverCreator *low_level_planner_creator;
    CrossedPolicy *joint_policy;
    PolicyMerger *low_level_merger;

    IdPolicy(MapfEnv *env, float gamma, const string &name,
             SolverCreator *low_level_planner_creator, PolicyMerger *low_level_merger);

    virtual MultiAgentAction *act(const MultiAgentState &state);

    virtual void train();

};


#endif //GYM_MAPF_ID_H
