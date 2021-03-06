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
                               double timeout_ms,
                               size_t group1,
                               size_t group2,
                               CrossedPolicy *joint_policy) = 0;
};

class DefaultPolicyMerger : public PolicyMerger {
public:
    SolverCreator *low_level_planner_creator;

    DefaultPolicyMerger(SolverCreator *low_level_planner_creator);

    virtual Policy *operator()(MapfEnv *env,
                               float gamma,
                               double timeout_ms,
                               size_t group1,
                               size_t group2,
                               CrossedPolicy *joint_policy);
};

class IdPolicy : public Policy {
public:
    SolverCreator *low_level_planner_creator;
    CrossedPolicy *joint_policy;
    PolicyMerger *low_level_merger;

    IdPolicy(MapfEnv *env, float gamma, const string &name,
             SolverCreator *low_level_planner_creator, PolicyMerger *low_level_merger);

    ~IdPolicy();

    virtual MultiAgentAction *act(const MultiAgentState &state, double timeout_ms);

    virtual void train(double timeout_milliseconds);

};


#endif //GYM_MAPF_ID_H
