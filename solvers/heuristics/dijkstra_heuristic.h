//
// Created by levyvonet on 03/11/2021.
//

#ifndef GYM_MAPF_DIJKSTRA_HEURISTIC_H
#define GYM_MAPF_DIJKSTRA_HEURISTIC_H

#include <queue>

#include <gym_mapf/gym_mapf.h>
#include "solvers/heuristics/heuristic.h"

class DijkstraHeuristic : public Heuristic {
public:
    int **distance;
    size_t n_agents;
    MapfEnv *env;

    void dijkstra_single_agent(size_t agent_idx);

    virtual ~DijkstraHeuristic() override;

    virtual void init(MapfEnv *env, double timeout_milliseconds) override;

    virtual double operator()(MultiAgentState *s);
};

#endif //GYM_MAPF_DIJKSTRA_HEURISTIC_H
