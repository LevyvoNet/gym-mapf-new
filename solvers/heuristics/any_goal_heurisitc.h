//
// Created by levyvonet on 5/3/23.
//

#ifndef GYM_MAPF_ANY_GOAL_HEURISITC_H
#define GYM_MAPF_ANY_GOAL_HEURISITC_H

#include <gym_mapf/gym_mapf.h>
#include "solvers/heuristics/heuristic.h"
#include <solvers/heuristics/dijkstra_heuristic.h>

class AnyGoalHeuristic : public DijkstraHeuristic {
public:
    AnyGoalHeuristic(vector<bool> is_important);

    virtual double operator()(MultiAgentState *s) override;

private:
    const vector<bool> is_important_;
};


#endif //GYM_MAPF_ANY_GOAL_HEURISITC_H
