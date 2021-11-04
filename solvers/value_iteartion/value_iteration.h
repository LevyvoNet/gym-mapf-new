//
// Created by levyvonet on 26/10/2021.
//

#ifndef GYM_MAPF_VALUE_ITERATION_H
#define GYM_MAPF_VALUE_ITERATION_H

#include <unordered_map>
#include <chrono>
#include <cmath>

#include <gym_mapf/gym_mapf.h>
#include <solvers/utils/policy/policy.h>

class ValueIterationPolicy : public Policy {
public:
    double default_value;
    double *v;


    ValueIterationPolicy(MapfEnv *env, float gamma, const string &name);

    virtual MultiAgentAction *act(const MultiAgentState &state) override;

    virtual void train() override;

    virtual TrainInfo *get_train_info() override;
};

#endif //GYM_MAPF_VALUE_ITERATION_H

