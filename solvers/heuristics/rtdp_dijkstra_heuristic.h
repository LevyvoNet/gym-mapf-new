//
// Created by levyvonet on 22/11/2021.
//

#ifndef GYM_MAPF_RTDP_DIJKSTRA_HEURISTIC_H
#define GYM_MAPF_RTDP_DIJKSTRA_HEURISTIC_H

#include "solvers/heuristics/dijkstra_heuristic.h"
#include "solvers/rtdp/rtdp.h"

class RtdpDijkstraHeuristic : public Heuristic {
private:
    MapfEnv *env;
    float gamma;
    vector<RtdpPolicy*> local_policies;

public:

    RtdpDijkstraHeuristic(float gamma);

    virtual ~RtdpDijkstraHeuristic() override;

    virtual void init(MapfEnv *env, double timeout_milliseconds) override;

    virtual double operator()(MultiAgentState *s);
};

#endif //GYM_MAPF_RTDP_DIJKSTRA_HEURISTIC_H
