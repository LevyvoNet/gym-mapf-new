//
// Created by levyvonet on 26/10/2021.
//

#ifndef GYM_MAPF_VALUE_ITERATION_H
#define GYM_MAPF_VALUE_ITERATION_H

#include <unordered_map>
#include <chrono>
#include <cmath>

#include <gym_mapf/gym_mapf.h>
#include <solvers/utils/policy/value_function_policy.h>

class ValueIterationPolicy : public ValueFunctionPolicy {
public:
    double default_value;
    double *v;


    ValueIterationPolicy(MapfEnv *env, float gamma, const string &name);

    virtual ~ValueIterationPolicy() override;

    virtual double get_value(MultiAgentState *s) override;

    virtual void train() override;
};

#endif //GYM_MAPF_VALUE_ITERATION_H

