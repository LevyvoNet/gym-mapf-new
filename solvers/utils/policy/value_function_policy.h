//
// Created by levyvonet on 06/11/2021.
//

#ifndef GYM_MAPF_VALUE_FUNCTION_POLICY_H
#define GYM_MAPF_VALUE_FUNCTION_POLICY_H

#include <gym_mapf/gym_mapf.h>
#include "solvers/utils/policy/policy.h"

class ValueFunctionPolicy : public Policy {
public:
    ValueFunctionPolicy(MapfEnv *env, float gamma, const std::string &name = "");

    MultiAgentAction* select_max_value_action(const MultiAgentState& s);
    virtual MultiAgentAction *act(const MultiAgentState &state) override;
    virtual double get_value(MultiAgentState *s)=0;
};

#endif //GYM_MAPF_VALUE_FUNCTION_POLICY_H
