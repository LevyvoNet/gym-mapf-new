//
// Created by levyvonet on 07/11/2021.
//

#ifndef GYM_MAPF_CONFLICT_H
#define GYM_MAPF_CONFLICT_H

#include <tsl/hopscotch_set.h>

#include <gym_mapf/gym_mapf.h>
#include <solvers/utils/policy/crossed_policy.h>
#include <set>


//class ConflictOld {
//public:
//    size_t g1;
//    size_t g2;
//
//    MultiAgentState prev_state;
//    MultiAgentState next_state;
//
//    ConflictOld(size_t g1, size_t g2, MultiAgentState prev_state, MultiAgentState next_state);
//};
//
//ConflictOld *detect_single_conflict_couple(CrossedPolicy *joint_policy, size_t a1, size_t a2);
//
//ConflictOld *detect_single_conflict(CrossedPolicy *joint_policy);

class Conflict {
public:
    size_t agent1;
    size_t agent2;

    Conflict(size_t agent1, size_t agent2);
};

tsl::hopscotch_set<Location> *get_reachable_locations(CrossedPolicy *joint_policy, size_t agent, double timeout_ms);

bool are_intersected(tsl::hopscotch_set<Location> *locs1, tsl::hopscotch_set<Location> *locs2);

bool are_sharing_states(CrossedPolicy* joint_policy, size_t agent1, size_t agent2, double timeout_ms);

Conflict* detect_conflict(CrossedPolicy* joint_policy, double timeout_ms);

#endif //GYM_MAPF_CONFLICT_H
