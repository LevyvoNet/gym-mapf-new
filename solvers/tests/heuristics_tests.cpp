//
// Created by levyvonet on 04/11/2021.
//

#include <gtest/gtest.h>

#include <gym_mapf/gym_mapf.h>
#include "heuristics/heuristic.h"
#include "heuristics/dijkstra_heuristic.h"
#include "value_iteartion/value_iteration.h"

#define LONG_TIME_MS (60*1000)

TEST(HeuristicsTest, DijkstraSimpleEnv) {
    std::vector<std::string> lines{{'.', '.', '@', '.', '.',},
                                   {'.', '.', '@', '.', '.',},
                                   {'.', '.', '.', '.', '.',}};
    Grid g(lines);

    MapfEnv env(&g, 1,
                {g.get_location(0, 0)},
                {g.get_location(0, 4)},
                0, -1000, 100, -1);

    /* Initialize the heuristic */
    DijkstraHeuristic h = DijkstraHeuristic();
    h.init(&env, LONG_TIME_MS);

    /* Calculate the expected values by value iteration */
    ValueIterationPolicy vi_policy = ValueIterationPolicy(&env, 1.0, "vi");
    vi_policy.train(LONG_TIME_MS);

    MultiAgentStateIterator s_iter = *env.observation_space->begin();
    MultiAgentStateIterator s_end = *env.observation_space->end();

    for (s_iter.reach_begin(); s_iter != s_end; ++s_iter) {
        MultiAgentState s = *s_iter;
        ASSERT_EQ(h(&s), vi_policy.get_value(&s));
    }

}

TEST(HeuristicsTest, DijkstraLargeGoalReward) {
    std::vector<std::string> lines{{'.', '.', '@', '.', '.',},
                                   {'.', '.', '@', '.', '.',},
                                   {'.', '.', '.', '.', '.',}};
    Grid g(lines);

    MapfEnv env(&g, 1,
                {g.get_location(0, 0)},
                {g.get_location(0, 4)},
                0, -1000, 0, -1);

    /* Initialize the heuristic */
    DijkstraHeuristic h = DijkstraHeuristic();
    h.init(&env, LONG_TIME_MS);

    /* Calculate the expected values by value iteration */
    ValueIterationPolicy vi_policy = ValueIterationPolicy(&env, 1.0, "vi");
    vi_policy.train(LONG_TIME_MS);

    MultiAgentStateIterator s_iter = *env.observation_space->begin();
    MultiAgentStateIterator s_end = *env.observation_space->end();

    for (s_iter.reach_begin(); s_iter != s_end; ++s_iter) {
        MultiAgentState s = *s_iter;
        ASSERT_EQ(h(&s), vi_policy.get_value(&s));
    }

}


TEST(HeuristicsTest, DijkstraRoomEnv) {
    MapfEnv *env = create_mapf_env("room-32-32-4", 1, 1, 0, -1000, -1, -1);


    /* Initialize the heuristic */
    DijkstraHeuristic h = DijkstraHeuristic();
    h.init(env, LONG_TIME_MS);

    /* Calculate the expected values by value iteration */
    ValueIterationPolicy vi_policy = ValueIterationPolicy(env, 1.0, "vi");
    vi_policy.train(LONG_TIME_MS);

    for (MultiAgentStateIterator s_iter = *env->observation_space->begin();
         s_iter != *env->observation_space->end(); ++s_iter) {
        MultiAgentState s = *s_iter;
        ASSERT_EQ(h(&s), vi_policy.get_value(&s));
    }

}


TEST(HeuristicsTest, DijkstraTwoAgents) {
    std::vector<std::string> lines{{'.', '.', '@', '.', '.',},
                                   {'.', '.', '@', '.', '.',},
                                   {'.', '.', '.', '.', '.',}};

    Grid g(lines);

    MapfEnv env(&g, 2,
                {g.get_location(0, 0), g.get_location(0, 4)},
                {g.get_location(2, 1), g.get_location(2, 3)},
                0, -1000, 100, -1);

    MapfEnv env0(&g, 1,
                 {g.get_location(0, 0),},
                 {g.get_location(2, 1),},
                 0, -1000, 100, -1);


    MapfEnv env1(&g, 1,
                 {g.get_location(0, 4)},
                 {g.get_location(2, 3)},
                 0, -1000, 100, -1);


    /* Initialize the heuristic */
    DijkstraHeuristic h = DijkstraHeuristic();
    h.init(&env, LONG_TIME_MS);

    /* Calculate the expected_reward values by value iteration */
    ValueIterationPolicy vi_policy0 = ValueIterationPolicy(&env0, 1.0, "");
    vi_policy0.train(LONG_TIME_MS);

    /* Calculate the expected_reward values by value iteration */
    ValueIterationPolicy vi_policy1 = ValueIterationPolicy(&env1, 1.0, "");
    vi_policy1.train(LONG_TIME_MS);

    int expected_reward = 0;
    bool all_in_goal = true;
    for (MultiAgentStateIterator s_iter = *env.observation_space->begin();
         s_iter != *env.observation_space->end(); ++s_iter) {

        expected_reward = 0;
        all_in_goal = true;

        if (s_iter->locations[0] != env.goal_state->locations[0]) {
            all_in_goal = false;
            expected_reward += vi_policy0.get_value(vi_policy0.env->locations_to_state({s_iter->locations[0]})) -
                               env.reward_of_goal;
        }

        if (s_iter->locations[1] != env.goal_state->locations[1]) {
            all_in_goal = false;
            expected_reward += vi_policy1.get_value(vi_policy1.env->locations_to_state({s_iter->locations[1]})) -
                               env.reward_of_goal;
        }

        if (!all_in_goal) {
            expected_reward += env.reward_of_goal;
        }

        MultiAgentState s = *s_iter;
        ASSERT_EQ(h(&s), expected_reward);
    }

}