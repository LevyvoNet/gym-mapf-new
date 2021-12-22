//
// Created by levyvonet on 05/12/2021.
//

#ifndef GYM_MAPF_SOLUTION_SUM_HEURISTIC_H
#define GYM_MAPF_SOLUTION_SUM_HEURISTIC_H

#include <solvers/heuristics/heuristic.h>
#include <solvers/utils/policy/value_function_policy.h>

/* TODO: convert this to multiple groups and policies (not just two) */

class SolutionSumHeuristic : public Heuristic {
public:
    vector<ValueFunctionPolicy *> policies;
    vector<vector<size_t>> groups;
    MapfEnv *env;

    SolutionSumHeuristic(vector<ValueFunctionPolicy *> policies, vector<vector<size_t>> groups);

    ~SolutionSumHeuristic();

    virtual void init(MapfEnv *env_param, double timeout_milliseconds);


    virtual double operator()(MultiAgentState *s);


};

#endif //GYM_MAPF_SOLUTION_SUM_HEURISTIC_H
