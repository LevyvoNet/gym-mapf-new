//
// Created by levyvonet on 03/11/2021.
//

#ifndef GYM_MAPF_DIJKSTRA_HEURISTIC_H
#define GYM_MAPF_DIJKSTRA_HEURISTIC_H

#include <queue>

#include <gym_mapf/gym_mapf.h>
#include "solvers/heuristics/heuristic.h"

// Utility function for running dijkstra's algorithm on envs.
// Should be exported to some heuristic_utils module in the future.
int *dijkstra_single_agent(size_t agent_idx, const MapfEnv *env);

class DijkstraHeuristic : public Heuristic {
public:
    int **distance;
    size_t n_agents;
    MapfEnv *env;


    virtual ~DijkstraHeuristic() override;

    virtual void init(MapfEnv *env, double timeout_milliseconds) override;

    virtual double operator()(MultiAgentState *s);
};

#endif //GYM_MAPF_DIJKSTRA_HEURISTIC_H
