//
// Created by levyvonet on 24/10/2021.
//

#include <gtest/gtest.h>
#include <cmath>

#include <gym_mapf.h>

#define REWARD_OF_COLLISION (-1000)
#define REWARD_OF_GOAL (100)
#define REWARD_OF_LIVING (-1)

using namespace std;

template<typename T>
bool contains(list<T *> l, T *t) {
    for (T *lt: l) {
        if (*lt == *t) {
            return true;
        }
    }
    return false;
}

template<typename T>
bool list_equal_no_order(list<T *> l1, list<T *> l2) {
    if (l1.size() != l2.size()) {
        return false;
    }

    for (T *t: l1) {
        if (!contains(l2, t)) {
            return false;
        }
    }

    for (T *t: l2) {
        if (!contains(l1, t)) {
            return false;
        }
    }

    return true;
}

TEST(MapfEnvTests, EmptyGridTransitionFunction) {
    std::vector<std::vector<char>> empty_8_8{{'.', '.', '.', '.', '.', '.', '.', '.'},
                                             {'.', '.', '.', '.', '.', '.', '.', '.'},
                                             {'.', '.', '.', '.', '.', '.', '.', '.'},
                                             {'.', '.', '.', '.', '.', '.', '.', '.'},
                                             {'.', '.', '.', '.', '.', '.', '.', '.'},
                                             {'.', '.', '.', '.', '.', '.', '.', '.'},
                                             {'.', '.', '.', '.', '.', '.', '.', '.'},
                                             {'.', '.', '.', '.', '.', '.', '.', '.'}};
    Grid g(empty_8_8);
    MultiAgentState start_state = MultiAgentState({Location(0, 0), Location(7, 7)});
    MultiAgentState goal_state = MultiAgentState({Location(0, 2), Location(5, 7)});

    MapfEnv env(&g, 2, &start_state, &goal_state, 0.2, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);
    MultiAgentAction action;

    /* Set the action */
    action.actions.push_back(RIGHT);
    action.actions.push_back(UP);

    /* Set the expected transitions */
    list<Transition *> expected_transitions(
            {
                    new Transition(0.64,
                                   MultiAgentState({Location(0, 1), Location(6, 7)}),
                                   -1, false, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(1, 0), Location(6, 7)}),
                                   -1, false, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(0, 0), Location(6, 7)}),
                                   -1, false, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(0, 1), Location(7, 7)}),
                                   -1, false, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(0, 1), Location(7, 6)}),
                                   -1, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(1, 0), Location(7, 7)}),
                                   -1, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(1, 0), Location(7, 6)}),
                                   -1, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(0, 0), Location(7, 7)}),
                                   -1, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(0, 0), Location(7, 6)}),
                                   -1, false, false),});
    list<Transition *> *transitions = env.get_transitions(*env.s, action);

    /* Round the probabilities to 2 decimal points */
    for (Transition *t: *transitions) {
        t->p = round(t->p * 100) / 100;
    }


    /* Test transitions from a state near the goal */

    MultiAgentState wish_state = MultiAgentState({Location(0, 1), Location(6, 7)});
    /* Set the expected transitions */
    list<Transition *> expected_transitions_from_wish_state(
            {
                    new Transition(0.64,
                                   MultiAgentState({Location(0, 2), Location(5, 7)}),
                                   REWARD_OF_LIVING + REWARD_OF_GOAL, true, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(1, 1), Location(5, 7)}),
                                   REWARD_OF_LIVING, false, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(0, 1), Location(5, 7)}),
                                   REWARD_OF_LIVING, false, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(0, 2), Location(6, 7)}),
                                   REWARD_OF_LIVING, false, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(0, 2), Location(6, 6)}),
                                   REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(1, 1), Location(6, 7)}),
                                   REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(1, 1), Location(6, 6)}),
                                   REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(0, 1), Location(6, 7)}),
                                   REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(0, 1), Location(6, 6)}),
                                   REWARD_OF_LIVING, false, false),});
    transitions = env.get_transitions(wish_state, action);

    /* Round the probabilities to 2 decimal points */
    for (Transition *t: *transitions) {
        t->p = round(t->p * 100) / 100;
    }


    ASSERT_TRUE(list_equal_no_order(*transitions, expected_transitions_from_wish_state));
}