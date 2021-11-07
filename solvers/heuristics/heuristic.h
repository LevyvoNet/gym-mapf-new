//
// Created by levyvonet on 03/11/2021.
//

#ifndef GYM_MAPF_HEURISTIC_H
#define GYM_MAPF_HEURISTIC_H

#include <gym_mapf/gym_mapf.h>

class Heuristic {
public:
    virtual void init(MapfEnv *env)=0;

    virtual ~Heuristic();

    virtual double operator()(MultiAgentState *s)=0;
};

#endif //GYM_MAPF_HEURISTIC_H
