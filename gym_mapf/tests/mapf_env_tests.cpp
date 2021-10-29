//
// Created by levyvonet on 24/10/2021.
//

#include <gtest/gtest.h>
#include <cmath>

#include <gym_mapf/gym_mapf.h>

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
    std::vector<std::string> empty_8_8{{'.', '.', '.', '.', '.', '.', '.', '.'},
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
    MultiAgentAction action({RIGHT, UP});


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
                                   2 * REWARD_OF_LIVING + REWARD_OF_GOAL, true, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(1, 1), Location(5, 7)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(0, 1), Location(5, 7)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(0, 2), Location(6, 7)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.08,
                                   MultiAgentState({Location(0, 2), Location(6, 6)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(1, 1), Location(6, 7)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(1, 1), Location(6, 6)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(0, 1), Location(6, 7)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   MultiAgentState({Location(0, 1), Location(6, 6)}),
                                   2 * REWARD_OF_LIVING, false, false),});
    transitions = env.get_transitions(wish_state, action);

    /* Round the probabilities to 2 decimal points */
    for (Transition *t: *transitions) {
        t->p = round(t->p * 100) / 100;
    }


    ASSERT_TRUE(list_equal_no_order(*transitions, expected_transitions_from_wish_state));
}

TEST(MapfEnvTests, CollisionStateTerminalNegativeReward) {
    std::vector<std::string> empty_8_8{{'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'}};

    Grid g(empty_8_8);
    MultiAgentState start_state = MultiAgentState({Location(0, 0), Location(0, 2)});
    MultiAgentState goal_state = MultiAgentState({Location(7, 7), Location(5, 5)});

    MapfEnv env(&g, 2, &start_state, &goal_state, 0.2, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    MultiAgentAction action({RIGHT, LEFT});
    list<Transition *> *transitions = env.get_transitions(*env.s, action);

    /* Round the probabilities to 2 decimal points */
    for (Transition *t: *transitions) {
        t->p = round(t->p * 100) / 100;
    }
    Transition *target_transition = new Transition(0.64, MultiAgentState({Location(0, 1), Location(0, 1)}),
                                                   2 * REWARD_OF_LIVING + REWARD_OF_COLLISION, true, true);

    ASSERT_TRUE(contains(*transitions, target_transition));
}

TEST(MapfEnvTests, ActionFromTerminalStateHasNoEffect) {
    /* Initialize env */
    std::vector<std::string> empty_2_2{{'.', '.'},
                                       {'.', '.'}};
    Grid g(empty_2_2);
    MultiAgentState start_state = MultiAgentState({Location(0, 0)});
    MultiAgentState goal_state = MultiAgentState({Location(1, 1)});
    MapfEnv env(&g, 1, &start_state, &goal_state, 0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    /* Step results */
    MultiAgentState next_state({Location(0, 0)});
    int reward = 0;
    bool done = false;
    bool collision = false;


    /* Execute the first step */
    env.step(MultiAgentAction({RIGHT}), &next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, REWARD_OF_LIVING);
    ASSERT_EQ(done, false);
    ASSERT_EQ(next_state, MultiAgentState({Location(0, 1)}));

    /* Execute the second step - this one reaches the goal */
    env.step(MultiAgentAction({DOWN}), &next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, REWARD_OF_LIVING + REWARD_OF_GOAL);
    ASSERT_EQ(done, true);
    ASSERT_EQ(next_state, MultiAgentState({Location(1, 1)}));

    /* Now, after the game is finished - do another step and make sure it has not effect. */
    env.step(MultiAgentAction({UP}), &next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, 0);
    ASSERT_EQ(done, true);
    ASSERT_EQ(next_state, MultiAgentState({Location(1, 1)}));

    /* Another time like I'm trying to reach the goal */
    env.step(MultiAgentAction({DOWN}), &next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, 0);
    ASSERT_EQ(done, true);
    ASSERT_EQ(next_state, MultiAgentState({Location(1, 1)}));

}

TEST(MapfEnvTests, SwitchStopIsCollision) {
    std::vector<std::string> empty_2{{'.', '.'}};
    Grid g(empty_2);
    MultiAgentState start_state = MultiAgentState({Location(0, 0), Location(0, 1)});
    MultiAgentState goal_state = MultiAgentState({Location(0, 1), Location(0, 0)});

    MapfEnv env(&g, 2, &start_state, &goal_state, 0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    /* Step results */
    MultiAgentState next_state({Location(0, 0), Location(0, 0)});
    int reward = 0;
    bool done = false;
    bool collision = false;

    /* Execute an action which switches the locations of the agents - this should be a collision */
    env.step(MultiAgentAction({RIGHT, LEFT}), &next_state, &reward, &done, &collision);
    ASSERT_EQ(next_state, MultiAgentState({Location(0, 1), Location(0, 0)}));
    ASSERT_EQ(reward, 2 * REWARD_OF_LIVING + REWARD_OF_COLLISION);
    ASSERT_EQ(done, true);

}

/* TODO: is it really necessary? */
TEST(MapfEnvTests, SameTestTransitionsProbabilitySummed) {
    GTEST_SKIP();
    std::vector<std::string> empty_2_2{{'.', '.'},
                                       {'.', '.'}};
    Grid g(empty_2_2);
    MultiAgentState start_state = MultiAgentState({Location(0, 0)});
    MultiAgentState goal_state = MultiAgentState({Location(1, 1)});
    MapfEnv env(&g, 1, &start_state, &goal_state, 0.1, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    list<Transition *> *transitions = env.get_transitions(*env.s, MultiAgentAction({STAY, STAY}));

    list<Transition *> expected_transitions(
            {
                    new Transition(1,
                                   MultiAgentState({Location(0, 0)}),
                                   REWARD_OF_LIVING, false, false)
            });

    ASSERT_TRUE(list_equal_no_order(*transitions, expected_transitions));
}

TEST(MapfEnvTests, RewardMultiagentSoc) {
    /* Initialize env */
    std::vector<std::string> empty_4_4{{'.', '.', '.', '.',},
                                       {'.', '.', '.', '.',},
                                       {'.', '.', '.', '.',},
                                       {'.', '.', '.', '.',}};
    Grid g(empty_4_4);
    MultiAgentState start_state = MultiAgentState({Location(0, 0), Location(3, 3), Location(1, 1)});
    MultiAgentState goal_state = MultiAgentState({Location(0, 1), Location(1, 3), Location(1, 2)});

    MapfEnv env(&g, 3, &start_state, &goal_state, 0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    int total_reward = 0;

    /* Step results */
    MultiAgentState next_state({Location(0, 0), Location(0, 0), Location(0, 0)});
    int reward = 0;
    bool done = false;
    bool collision = false;

    /* First step */
    env.step(MultiAgentAction({RIGHT, UP, RIGHT}), &next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, 3 * REWARD_OF_LIVING);
    ASSERT_EQ(done, false);
    total_reward += reward;

    /* Second step */
    env.step(MultiAgentAction({STAY, UP, STAY}), &next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, REWARD_OF_LIVING + REWARD_OF_GOAL);
    ASSERT_EQ(done, true);
    ASSERT_EQ(next_state, *env.goal_state);
    total_reward += reward;

    ASSERT_EQ(total_reward, 4 * REWARD_OF_LIVING + REWARD_OF_GOAL);

}


TEST(MapfEnvTests, RewardMultiagentSocStayBeforeGoal) {
    /* Initialize env */
    std::vector<std::string> empty_4_4{{'.', '.', '.', '.',},
                                       {'.', '.', '.', '.',},
                                       {'.', '.', '.', '.',},
                                       {'.', '.', '.', '.',}};
    Grid g(empty_4_4);
    MultiAgentState start_state = MultiAgentState({Location(0, 0), Location(3, 3), Location(1, 1)});
    MultiAgentState goal_state = MultiAgentState({Location(0, 1), Location(1, 3), Location(1, 2)});

    MapfEnv env(&g, 3, &start_state, &goal_state, 0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    int total_reward = 0;

    /* Step results */
    MultiAgentState next_state({Location(0, 0), Location(0, 0), Location(0, 0)});
    int reward = 0;
    bool done = false;
    bool collision = false;

    /* First step */
    env.step(MultiAgentAction({RIGHT, STAY, STAY}), &next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, 3 * REWARD_OF_LIVING);
    ASSERT_EQ(done, false);
}

TEST(MapfEnvTests, RewardMultiagentSocSingleAgent) {
    /* Initialize env */
    std::vector<std::string> empty_4_5{{'.', '.', '.', '.',},
                                       {'.', '.', '.', '.',},
                                       {'.', '.', '.', '.',},
                                       {'.', '.', '.', '.',},
                                       {'.', '.', '.', '.',}};
    Grid g(empty_4_5);
    MultiAgentState start_state = MultiAgentState({Location(0, 0)});
    MultiAgentState goal_state = MultiAgentState({Location(4, 0)});

    MapfEnv env(&g, 1, &start_state, &goal_state, 0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    int total_reward = 0;

    /* Step results */
    MultiAgentState next_state({Location(0, 0)});
    int reward = 0;
    bool done = false;
    bool collision = false;

    /* Step down 4 times */
    env.step(MultiAgentAction({DOWN}), &next_state, &reward, &done, &collision);
    total_reward += reward;
    env.step(MultiAgentAction({DOWN}), &next_state, &reward, &done, &collision);
    total_reward += reward;
    env.step(MultiAgentAction({DOWN}), &next_state, &reward, &done, &collision);
    total_reward += reward;
    env.step(MultiAgentAction({DOWN}), &next_state, &reward, &done, &collision);
    total_reward += reward;

    ASSERT_EQ(next_state, *env.goal_state);
    ASSERT_EQ(reward, REWARD_OF_GOAL + REWARD_OF_LIVING);
    ASSERT_EQ(total_reward, REWARD_OF_GOAL + 4 * REWARD_OF_LIVING);
}


TEST(MafpEnvTests, StateSpaceIteration) {
    std::vector<std::string> small_grid_with_obstacles{{'.', '@', '.'},
                                                       {'.', '@', '.'},
                                                       {'.', '.', '.'}};
    Grid g(small_grid_with_obstacles);
    MultiAgentState start_state = MultiAgentState({Location(0, 0), Location(0, 2)});
    MultiAgentState goal_state = MultiAgentState({Location(0, 2), Location(0, 0)});

    MapfEnv env(&g, 2, &start_state, &goal_state, 0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    list<MultiAgentState *> states;

    for (MultiAgentStateIterator iter = env.observation_space->begin(); iter != env.observation_space->end(); ++iter) {
        states.push_back(new MultiAgentState(iter->locations));
    }


    list<MultiAgentState *> expected_states{
            new MultiAgentState({Location(0, 0), Location(0, 0)}),
            new MultiAgentState({Location(0, 2), Location(0, 0)}),
            new MultiAgentState({Location(1, 0), Location(0, 0)}),
            new MultiAgentState({Location(1, 2), Location(0, 0)}),
            new MultiAgentState({Location(2, 0), Location(0, 0)}),
            new MultiAgentState({Location(2, 1), Location(0, 0)}),
            new MultiAgentState({Location(2, 2), Location(0, 0)}),

            new MultiAgentState({Location(0, 0), Location(0, 2)}),
            new MultiAgentState({Location(0, 2), Location(0, 2)}),
            new MultiAgentState({Location(1, 0), Location(0, 2)}),
            new MultiAgentState({Location(1, 2), Location(0, 2)}),
            new MultiAgentState({Location(2, 0), Location(0, 2)}),
            new MultiAgentState({Location(2, 1), Location(0, 2)}),
            new MultiAgentState({Location(2, 2), Location(0, 2)}),

            new MultiAgentState({Location(0, 0), Location(1, 0)}),
            new MultiAgentState({Location(0, 2), Location(1, 0)}),
            new MultiAgentState({Location(1, 0), Location(1, 0)}),
            new MultiAgentState({Location(1, 2), Location(1, 0)}),
            new MultiAgentState({Location(2, 0), Location(1, 0)}),
            new MultiAgentState({Location(2, 1), Location(1, 0)}),
            new MultiAgentState({Location(2, 2), Location(1, 0)}),

            new MultiAgentState({Location(0, 0), Location(1, 2)}),
            new MultiAgentState({Location(0, 2), Location(1, 2)}),
            new MultiAgentState({Location(1, 0), Location(1, 2)}),
            new MultiAgentState({Location(1, 2), Location(1, 2)}),
            new MultiAgentState({Location(2, 0), Location(1, 2)}),
            new MultiAgentState({Location(2, 1), Location(1, 2)}),
            new MultiAgentState({Location(2, 2), Location(1, 2)}),

            new MultiAgentState({Location(0, 0), Location(2, 0)}),
            new MultiAgentState({Location(0, 2), Location(2, 0)}),
            new MultiAgentState({Location(1, 0), Location(2, 0)}),
            new MultiAgentState({Location(1, 2), Location(2, 0)}),
            new MultiAgentState({Location(2, 0), Location(2, 0)}),
            new MultiAgentState({Location(2, 1), Location(2, 0)}),
            new MultiAgentState({Location(2, 2), Location(2, 0)}),

            new MultiAgentState({Location(0, 0), Location(2, 1)}),
            new MultiAgentState({Location(0, 2), Location(2, 1)}),
            new MultiAgentState({Location(1, 0), Location(2, 1)}),
            new MultiAgentState({Location(1, 2), Location(2, 1)}),
            new MultiAgentState({Location(2, 0), Location(2, 1)}),
            new MultiAgentState({Location(2, 1), Location(2, 1)}),
            new MultiAgentState({Location(2, 2), Location(2, 1)}),

            new MultiAgentState({Location(0, 0), Location(2, 2)}),
            new MultiAgentState({Location(0, 2), Location(2, 2)}),
            new MultiAgentState({Location(1, 0), Location(2, 2)}),
            new MultiAgentState({Location(1, 2), Location(2, 2)}),
            new MultiAgentState({Location(2, 0), Location(2, 2)}),
            new MultiAgentState({Location(2, 1), Location(2, 2)}),
            new MultiAgentState({Location(2, 2), Location(2, 2)}),
    };

    ASSERT_TRUE(list_equal_no_order(states, expected_states));

}

TEST(MafpEnvTests, ActionSpaceIteration) {
    std::vector<std::string> small_grid_with_obstacles{{'.', '@', '.'},
                                                       {'.', '@', '.'},
                                                       {'.', '.', '.'}};
    Grid g(small_grid_with_obstacles);
    MultiAgentState start_state = MultiAgentState({Location(0, 0), Location(0, 2)});
    MultiAgentState goal_state = MultiAgentState({Location(0, 2), Location(0, 0)});

    MapfEnv env(&g, 2, &start_state, &goal_state, 0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    list<MultiAgentAction *> actions;

    for (MultiAgentActionIterator iter = env.action_space->begin(); iter != env.action_space->end(); ++iter) {
        actions.push_back(new MultiAgentAction(iter->actions));
    }


    list<MultiAgentAction *> expected_actions{
            new MultiAgentAction({STAY, STAY}),
            new MultiAgentAction({UP, STAY}),
            new MultiAgentAction({RIGHT, STAY}),
            new MultiAgentAction({DOWN, STAY}),
            new MultiAgentAction({LEFT, STAY}),

            new MultiAgentAction({STAY, UP}),
            new MultiAgentAction({UP, UP}),
            new MultiAgentAction({RIGHT, UP}),
            new MultiAgentAction({DOWN, UP}),
            new MultiAgentAction({LEFT, UP}),

            new MultiAgentAction({STAY, RIGHT}),
            new MultiAgentAction({UP, RIGHT}),
            new MultiAgentAction({RIGHT, RIGHT}),
            new MultiAgentAction({DOWN, RIGHT}),
            new MultiAgentAction({LEFT, RIGHT}),

            new MultiAgentAction({STAY, DOWN}),
            new MultiAgentAction({UP, DOWN}),
            new MultiAgentAction({RIGHT, DOWN}),
            new MultiAgentAction({DOWN, DOWN}),
            new MultiAgentAction({LEFT, DOWN}),

            new MultiAgentAction({STAY, LEFT}),
            new MultiAgentAction({UP, LEFT}),
            new MultiAgentAction({RIGHT, LEFT}),
            new MultiAgentAction({DOWN, LEFT}),
            new MultiAgentAction({LEFT, LEFT}),

    };

    ASSERT_TRUE(list_equal_no_order(actions, expected_actions));

}