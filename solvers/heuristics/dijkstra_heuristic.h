//
// Created by levyvonet on 03/11/2021.
//

#ifndef GYM_MAPF_DIJKSTRA_HEURISTIC_H
#define GYM_MAPF_DIJKSTRA_HEURISTIC_H

#include <queue>

#include <gym_mapf/gym_mapf.h>
#include "solvers/heuristics/heuristic.h"

class DijkstraHeuristic : public Heuristic {
private:
    int **distance;
    MapfEnv *env;

    void dijkstra_single_agent(size_t agent_idx);

public:

    virtual ~DijkstraHeuristic() override;

    virtual void init(MapfEnv *env) override;

    virtual double operator()(MultiAgentState *s);
};

#endif //GYM_MAPF_DIJKSTRA_HEURISTIC_H
