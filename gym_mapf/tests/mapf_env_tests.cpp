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
    std::vector<std::string> empty_8_8{{'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'}};
    Grid g(empty_8_8);
    vector<Location> start_locations{g.get_location(0, 0), g.get_location(7, 7)};
    vector<Location> goal_locations = {g.get_location(0, 2), g.get_location(5, 7)};

    MapfEnv env(&g, 2, start_locations, goal_locations, 0.2, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);
    MultiAgentAction *action = env.actions_to_action({RIGHT, UP});



    /* Set the expected transitions */
    list<Transition *> expected_transitions(
            {
                    new Transition(0.64,
                                   env.locations_to_state({g.get_location(0, 1), g.get_location(6, 7)}),
                                   -1, false, false),
                    new Transition(0.08,
                                   env.locations_to_state({g.get_location(1, 0), g.get_location(6, 7)}),
                                   -1, false, false),
                    new Transition(0.08,
                                   env.locations_to_state({g.get_location(0, 0), g.get_location(6, 7)}),
                                   -1, false, false),
                    new Transition(0.08,
                                   env.locations_to_state({g.get_location(0, 1), g.get_location(7, 7)}),
                                   -1, false, false),
                    new Transition(0.08,
                                   env.locations_to_state({g.get_location(0, 1), g.get_location(7, 6)}),
                                   -1, false, false),
                    new Transition(0.01,
                                   env.locations_to_state({g.get_location(1, 0), g.get_location(7, 7)}),
                                   -1, false, false),
                    new Transition(0.01,
                                   env.locations_to_state({g.get_location(1, 0), g.get_location(7, 6)}),
                                   -1, false, false),
                    new Transition(0.01,
                                   env.locations_to_state({g.get_location(0, 0), g.get_location(7, 7)}),
                                   -1, false, false),
                    new Transition(0.01,
                                   env.locations_to_state({g.get_location(0, 0), g.get_location(7, 6)}),
                                   -1, false, false),});
    list<Transition *> *transitions = env.get_transitions(*env.s, *action);

    /* Round the probabilities to 2 decimal points */
    for (Transition *t: *transitions) {
        t->p = round(t->p * 100) / 100;
    }


    /* Test transitions from a state near the goal */

    MultiAgentState *wish_state = env.locations_to_state({g.get_location(0, 1), g.get_location(6, 7)});
    /* Set the expected transitions */
    list<Transition *> expected_transitions_from_wish_state(
            {
                    new Transition(0.64,
                                   env.locations_to_state({g.get_location(0, 2), g.get_location(5, 7)}),
                                   2 * REWARD_OF_LIVING + REWARD_OF_GOAL, true, false),
                    new Transition(0.08,
                                   env.locations_to_state({g.get_location(1, 1), g.get_location(5, 7)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.08,
                                   env.locations_to_state({g.get_location(0, 1), g.get_location(5, 7)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.08,
                                   env.locations_to_state({g.get_location(0, 2), g.get_location(6, 7)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.08,
                                   env.locations_to_state({g.get_location(0, 2), g.get_location(6, 6)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   env.locations_to_state({g.get_location(1, 1), g.get_location(6, 7)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   env.locations_to_state({g.get_location(1, 1), g.get_location(6, 6)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   env.locations_to_state({g.get_location(0, 1), g.get_location(6, 7)}),
                                   2 * REWARD_OF_LIVING, false, false),
                    new Transition(0.01,
                                   env.locations_to_state({g.get_location(0, 1), g.get_location(6, 6)}),
                                   2 * REWARD_OF_LIVING, false, false),});
    transitions = env.get_transitions(*wish_state, *action);

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

    MapfEnv env(&g,
                2,
                {g.get_location(0, 0), g.get_location(0, 2)},
                {g.get_location(7, 7), g.get_location(5, 5)},
                0.2,
                REWARD_OF_COLLISION,
                REWARD_OF_GOAL,
                REWARD_OF_LIVING);

    MultiAgentAction *action = env.actions_to_action({RIGHT, LEFT});
    list<Transition *> *transitions = env.get_transitions(*env.s, *action);

    /* Round the probabilities to 2 decimal points */
    for (Transition *t: *transitions) {
        t->p = round(t->p * 100) / 100;
    }
    Transition *target_transition = new Transition(0.64,
                                                   env.locations_to_state({g.get_location(0, 1), g.get_location(0, 1)}),
                                                   2 * REWARD_OF_LIVING + REWARD_OF_COLLISION, true, true);

    ASSERT_TRUE(contains(*transitions, target_transition));
}

TEST(MapfEnvTests, ActionFromTerminalStateHasNoEffect) {
    /* Initialize env */
    std::vector<std::string> empty_2_2{{'.', '.'},
                                       {'.', '.'}};
    Grid g(empty_2_2);
    MapfEnv env(&g, 1, {g.get_location(0, 0)}, {g.get_location(1, 1)}, 0, REWARD_OF_COLLISION, REWARD_OF_GOAL,
                REWARD_OF_LIVING);

    /* Step results */
    MultiAgentState *next_state = env.locations_to_state({g.get_location(0, 0)});
    int reward = 0;
    bool done = false;
    bool collision = false;


    /* Execute the first step */
    env.step(*env.actions_to_action({RIGHT}), next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, REWARD_OF_LIVING);
    ASSERT_EQ(done, false);
    ASSERT_EQ(*next_state, *env.locations_to_state({g.get_location(0, 1)}));

    /* Execute the second step - this one reaches the goal */
    env.step(*env.actions_to_action({DOWN}), next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, REWARD_OF_LIVING + REWARD_OF_GOAL);
    ASSERT_EQ(done, true);
    ASSERT_EQ(*next_state, *env.locations_to_state({g.get_location(1, 1)}));

    /* Now, after the game is finished - do another step and make sure it has not effect. */
    env.step(*env.actions_to_action({UP}), next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, 0);
    ASSERT_EQ(done, true);
    ASSERT_EQ(*next_state, *env.locations_to_state({g.get_location(1, 1)}));

    /* Another time like I'm trying to reach the goal */
    env.step(*env.actions_to_action({DOWN}), next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, 0);
    ASSERT_EQ(done, true);
    ASSERT_EQ(*next_state, *env.locations_to_state({g.get_location(1, 1)}));

}

TEST(MapfEnvTests, SwitchStopIsCollision) {
    std::vector<std::string> empty_2{{'.', '.'}};
    Grid g(empty_2);

    MapfEnv env(&g, 2,
                {g.get_location(0, 0), g.get_location(0, 1)}, {g.get_location(0, 1), g.get_location(0, 0)},
                0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    /* Step results */
    MultiAgentState *next_state = env.locations_to_state({g.get_location(0, 0), g.get_location(0, 0)});
    int reward = 0;
    bool done = false;
    bool collision = false;

    /* Execute an action which switches the locations of the agents - this should be a collision */
    env.step(*env.actions_to_action({RIGHT, LEFT}), next_state, &reward, &done, &collision);
    ASSERT_EQ(*next_state, *env.locations_to_state({g.get_location(0, 1), g.get_location(0, 0)}));
    ASSERT_EQ(reward, 2 * REWARD_OF_LIVING + REWARD_OF_COLLISION);
    ASSERT_EQ(done, true);

}

/* TODO: is it really necessary? */
TEST(MapfEnvTests, SameTestTransitionsProbabilitySummed) {
    GTEST_SKIP();
    std::vector<std::string> empty_2_2{{'.', '.'},
                                       {'.', '.'}};
    Grid g(empty_2_2);
    MapfEnv env(&g, 1, {g.get_location(0, 0)}, {g.get_location(1, 1)}, 0.1, REWARD_OF_COLLISION, REWARD_OF_GOAL,
                REWARD_OF_LIVING);

    list<Transition *> *transitions = env.get_transitions(*env.s, *env.actions_to_action({STAY, STAY}));

    list<Transition *> expected_transitions(
            {
                    new Transition(1,
                                   *env.locations_to_state({g.get_location(0, 0)}),
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

    MapfEnv env(&g, 3,
                {g.get_location(0, 0), g.get_location(3, 3), g.get_location(1, 1)},
                {g.get_location(0, 1), g.get_location(1, 3), g.get_location(1, 2)},
                0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    int total_reward = 0;

    /* Step results */
    MultiAgentState *next_state = env.locations_to_state({g.get_location(0, 0), g.get_location(0, 0), g.get_location(0, 0)});
    int reward = 0;
    bool done = false;
    bool collision = false;

    /* First step */
    env.step(*env.actions_to_action({RIGHT, UP, RIGHT}), next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, 3 * REWARD_OF_LIVING);
    ASSERT_EQ(done, false);
    total_reward += reward;

    /* Second step */
    env.step(*env.actions_to_action({STAY, UP, STAY}), next_state, &reward, &done, &collision);
    ASSERT_EQ(reward, REWARD_OF_LIVING + REWARD_OF_GOAL);
    ASSERT_EQ(done, true);
    ASSERT_EQ(*next_state, *env.goal_state);
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

    MapfEnv env(&g, 3,
                {g.get_location(0, 0), g.get_location(3, 3), g.get_location(1, 1)},
                {g.get_location(0, 1), g.get_location(1, 3), g.get_location(1, 2)},
                0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    int total_reward = 0;

    /* Step results */
    MultiAgentState *next_state = env.locations_to_state({g.get_location(0, 0), g.get_location(0, 0), g.get_location(0, 0)});
    int reward = 0;
    bool done = false;
    bool collision = false;

    /* First step */
    env.step(*env.actions_to_action({RIGHT, STAY, STAY}), next_state, &reward, &done, &collision);
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

    MapfEnv env(&g, 1,
                {g.get_location(0, 0)},
                {g.get_location(4, 0)},
                0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    int total_reward = 0;

    /* Step results */
    MultiAgentState *next_state = env.locations_to_state({g.get_location(0, 0)});
    int reward = 0;
    bool done = false;
    bool collision = false;

    /* Step down 4 times */
    env.step(*env.actions_to_action({DOWN}), next_state, &reward, &done, &collision);
    total_reward += reward;
    env.step(*env.actions_to_action({DOWN}), next_state, &reward, &done, &collision);
    total_reward += reward;
    env.step(*env.actions_to_action({DOWN}), next_state, &reward, &done, &collision);
    total_reward += reward;
    env.step(*env.actions_to_action({DOWN}), next_state, &reward, &done, &collision);
    total_reward += reward;

    ASSERT_EQ(*next_state, *env.goal_state);
    ASSERT_EQ(reward, REWARD_OF_GOAL + REWARD_OF_LIVING);
    ASSERT_EQ(total_reward, REWARD_OF_GOAL + 4 * REWARD_OF_LIVING);
}


TEST(MafpEnvTests, StateSpaceIteration) {
    std::vector<std::string> small_grid_with_obstacles{{'.', '@', '.'},
                                                       {'.', '@', '.'},
                                                       {'.', '.', '.'}};
    Grid g(small_grid_with_obstacles);

    MapfEnv env(&g,
                2,
                {g.get_location(0, 0), g.get_location(0, 2)},
                {g.get_location(0, 2), g.get_location(0, 0)},
                0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    list<MultiAgentState *> states;

    for (MultiAgentStateIterator iter = env.observation_space->begin(); iter != env.observation_space->end(); ++iter) {
        states.push_back(new MultiAgentState(iter->locations, iter->id));
    }


    list<MultiAgentState *> expected_states{
            env.locations_to_state({g.get_location(0, 0), g.get_location(0, 0)}),
            env.locations_to_state({g.get_location(0, 2), g.get_location(0, 0)}),
            env.locations_to_state({g.get_location(1, 0), g.get_location(0, 0)}),
            env.locations_to_state({g.get_location(1, 2), g.get_location(0, 0)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(0, 0)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(0, 0)}),
            env.locations_to_state({g.get_location(2, 2), g.get_location(0, 0)}),

            env.locations_to_state({g.get_location(0, 0), g.get_location(0, 2)}),
            env.locations_to_state({g.get_location(0, 2), g.get_location(0, 2)}),
            env.locations_to_state({g.get_location(1, 0), g.get_location(0, 2)}),
            env.locations_to_state({g.get_location(1, 2), g.get_location(0, 2)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(0, 2)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(0, 2)}),
            env.locations_to_state({g.get_location(2, 2), g.get_location(0, 2)}),

            env.locations_to_state({g.get_location(0, 0), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(0, 2), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(1, 0), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(1, 2), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(2, 2), g.get_location(1, 0)}),

            env.locations_to_state({g.get_location(0, 0), g.get_location(1, 2)}),
            env.locations_to_state({g.get_location(0, 2), g.get_location(1, 2)}),
            env.locations_to_state({g.get_location(1, 0), g.get_location(1, 2)}),
            env.locations_to_state({g.get_location(1, 2), g.get_location(1, 2)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(1, 2)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(1, 2)}),
            env.locations_to_state({g.get_location(2, 2), g.get_location(1, 2)}),

            env.locations_to_state({g.get_location(0, 0), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(0, 2), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(1, 0), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(1, 2), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(2, 2), g.get_location(2, 0)}),

            env.locations_to_state({g.get_location(0, 0), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(0, 2), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(1, 0), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(1, 2), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(2, 2), g.get_location(2, 1)}),

            env.locations_to_state({g.get_location(0, 0), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(0, 2), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(1, 0), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(1, 2), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(2, 2), g.get_location(2, 2)}),
    };

    ASSERT_TRUE(list_equal_no_order(states, expected_states));

}

TEST(MafpEnvTests, ActionSpaceIteration) {
    std::vector<std::string> small_grid_with_obstacles{{'.', '@', '.'},
                                                       {'.', '@', '.'},
                                                       {'.', '.', '.'}};
    Grid g(small_grid_with_obstacles);

    MapfEnv env(&g, 2,
                {g.get_location(0, 0), g.get_location(0, 2)},
                {g.get_location(0, 2), g.get_location(0, 0)},
                0, REWARD_OF_COLLISION, REWARD_OF_GOAL, REWARD_OF_LIVING);

    list<MultiAgentAction *> actions;

    for (MultiAgentActionIterator iter = env.action_space->begin(); iter != env.action_space->end(); ++iter) {
        actions.push_back(new MultiAgentAction(iter->actions, iter->id));
    }


    list<MultiAgentAction *> expected_actions{
            new MultiAgentAction({STAY, STAY}, 0),
            new MultiAgentAction({UP, STAY}, 1),
            new MultiAgentAction({RIGHT, STAY}, 2),
            new MultiAgentAction({DOWN, STAY}, 3),
            new MultiAgentAction({LEFT, STAY}, 4),

            new MultiAgentAction({STAY, UP}, 5),
            new MultiAgentAction({UP, UP}, 6),
            new MultiAgentAction({RIGHT, UP}, 7),
            new MultiAgentAction({DOWN, UP}, 8),
            new MultiAgentAction({LEFT, UP}, 9),

            new MultiAgentAction({STAY, RIGHT}, 10),
            new MultiAgentAction({UP, RIGHT}, 11),
            new MultiAgentAction({RIGHT, RIGHT}, 12),
            new MultiAgentAction({DOWN, RIGHT}, 13),
            new MultiAgentAction({LEFT, RIGHT}, 14),

            new MultiAgentAction({STAY, DOWN}, 15),
            new MultiAgentAction({UP, DOWN}, 16),
            new MultiAgentAction({RIGHT, DOWN}, 17),
            new MultiAgentAction({DOWN, DOWN}, 18),
            new MultiAgentAction({LEFT, DOWN}, 19),

            new MultiAgentAction({STAY, LEFT}, 20),
            new MultiAgentAction({UP, LEFT}, 21),
            new MultiAgentAction({RIGHT, LEFT}, 22),
            new MultiAgentAction({DOWN, LEFT}, 23),
            new MultiAgentAction({LEFT, LEFT}, 24),

    };

    ASSERT_TRUE(list_equal_no_order(actions, expected_actions));

}

