//
// Created by levyvonet on 07/11/2021.
//

#ifndef GYM_MAPF_CONFLICT_H
#define GYM_MAPF_CONFLICT_H


#include <gym_mapf/gym_mapf.h>
#include <solvers/utils/policy/crossed_policy.h>
#include <set>


class Conflict {
public:
    size_t g1;
    size_t g2;

    MultiAgentState prev_state;
    MultiAgentState next_state;

    Conflict(size_t g1, size_t g2, MultiAgentState prev_state, MultiAgentState next_state);
};


Conflict *detect_single_conflict_couple(CrossedPolicy *joint_policy, size_t a1, size_t a2);

Conflict *detect_single_conflict(CrossedPolicy *joint_policy);

#endif //GYM_MAPF_CONFLICT_H
