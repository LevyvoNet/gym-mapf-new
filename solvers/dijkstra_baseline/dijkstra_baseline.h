//
// Created by levyvonet on 23/12/2021.
//

#ifndef GYM_MAPF_DIJSKTRA_BASELINE_H
#define GYM_MAPF_DIJSKTRA_BASELINE_H

#include <gym_mapf/gym_mapf.h>
#include "solvers/utils/policy/policy.h"
#include <solvers/utils/policy/value_function_policy.h>
#include "solvers/heuristics/dijkstra_heuristic.h"
#include "solvers/rtdp/rtdp.h"

class DijkstraBaselinePolicy : public ValueFunctionPolicy {
public:
    DijkstraHeuristic* h;

    DijkstraBaselinePolicy(MapfEnv *env, float gamma, const string &name);

    virtual ~DijkstraBaselinePolicy();

    virtual void train(double timeout_ms) override;

    virtual double get_value(MultiAgentState *s) override;

};


#endif //GYM_MAPF_DIJSKTRA_BASELINE_H
