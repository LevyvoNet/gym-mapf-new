//
// Created by levyvonet on 07/11/2021.
//

#include "conflict.h"

int get_group_of_agent(const vector<vector<size_t>> &groups, size_t agent) {
    for (size_t group_idx = 0; group_idx < groups.size(); ++group_idx) {
        for (size_t a: groups[group_idx]) {
            if (a == agent) {
                return group_idx;
            }
        }
    }

    return -1;
}


Conflict *detect_single_conflict_couple(CrossedPolicy *joint_policy, size_t g1, size_t g2) {
    vector<Location> new_locations;
    vector<MultiAgentState> states_to_expand;
    MapfEnv *merged_env = nullptr;
    MultiAgentState curr_state({}, 0);
    vector<Location> l1;
    vector<Location> l2;
    MultiAgentState *s1 = nullptr;
    MultiAgentState *s2 = nullptr;
    MultiAgentAction *action1 = nullptr;
    MultiAgentAction *action2 = nullptr;
    MultiAgentState *new_state = nullptr;
    TransitionsList *transitions1 = nullptr;
    TransitionsList *transitions2 = nullptr;
    MultiAgentStateStorage<bool *> *expanded_states = nullptr;
    bool *expanded = nullptr;
    Conflict *conflict = nullptr;

    /* Merge into a single group and get its local view */
    merged_env = merge_groups_envs(joint_policy, g1, g2);

    /* Initialize with start state */
    new_state = merged_env->start_state;
    expanded_states = new MultiAgentStateStorage<bool *>(merged_env->n_agents, nullptr);
    expanded = new bool;
    *expanded = false;
    expanded_states->set(*new_state, expanded);
    states_to_expand.push_back(*new_state);

    while (states_to_expand.size() > 0) {
        /* Get the next state to expand and mark it */
        curr_state = states_to_expand.back();
        states_to_expand.pop_back();
        expanded = new bool;
        *expanded = true;
        expanded_states->set(curr_state, expanded);

        /* Extract the local group state from the merged state */
        for (size_t i = 0; i < curr_state.locations.size(); ++i) {
            if (i < joint_policy->groups[g1].size()) {
                l1.push_back(curr_state.locations[i]);
            } else {
                l2.push_back(curr_state.locations[i]);
            }
        }
        s1 = joint_policy->policies[g1]->env->locations_to_state(l1);
        s2 = joint_policy->policies[g2]->env->locations_to_state(l2);
        l1.clear();
        l2.clear();

        /* Get the action of each policy to the current state */
        action1 = joint_policy->policies[g1]->act(*s1);
        action2 = joint_policy->policies[g2]->act(*s2);


        /* Get the transitions for the actions */
        transitions1 = joint_policy->policies[g1]->env->get_transitions(*s1, *action1);
        transitions2 = joint_policy->policies[g2]->env->get_transitions(*s2, *action2);
        delete s1;
        delete s2;
        delete action1;
        delete action2;

        /* Iterate over the transitions, merge each state pair and add it to expansion if needed */
        for (Transition *t1: *transitions1->transitions) {
            for (Transition *t2: *transitions2->transitions) {
                /* Calculate the new merged state derived from the current transition */
                new_locations.clear();
                for (Location l: t1->next_state->locations) {
                    new_locations.push_back(l);
                }
                for (Location l: t2->next_state->locations) {
                    new_locations.push_back(l);
                }
                new_state = merged_env->locations_to_state(new_locations);

                /* Check if the current transition represents a conflict */
                if (is_collision_transition(&curr_state, new_state)) {
                    conflict = new Conflict(g1, g2, curr_state, *new_state);
                    goto l_cleanup;
                }

                /* Add the new_state to expansion if needed */
                expanded = expanded_states->get(*new_state);
                if (nullptr == expanded) {
                    states_to_expand.push_back(*new_state);
                }

                delete new_state;
                new_state = nullptr;
            }
        }
    }

l_cleanup:
    if (nullptr!=new_state){
        delete new_state;
    }
    delete merged_env;
    return conflict;
}

Conflict *detect_single_conflict(CrossedPolicy *joint_policy) {
    Conflict *conflict = nullptr;

    for (size_t g1 = 0; g1 < joint_policy->groups.size(); ++g1) {
        for (size_t g2 = 0; g2 < joint_policy->groups.size(); ++g2) {
            if (g1 < g2) {
                conflict = detect_single_conflict_couple(joint_policy, g1, g2);
                if (nullptr != conflict) {
                    return conflict;
                }
            }
        }
    }

    return conflict;
}

Conflict::Conflict(size_t g1, size_t g2, MultiAgentState prev_state, MultiAgentState next_state) :
        g1(g1), g2(g2), prev_state(prev_state), next_state(next_state) {}
