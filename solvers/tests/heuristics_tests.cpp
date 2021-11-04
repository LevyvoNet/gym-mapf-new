//
// Created by levyvonet on 04/11/2021.
//

#include <gtest/gtest.h>

#include <gym_mapf/gym_mapf.h>
#include "heuristics/heuristic.h"
#include "heuristics/dijkstra_heuristic.h"
#include "value_iteartion/value_iteration.h"

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
    h.init(&env);

    /* Calculate the expected values by value iteration */
    ValueIterationPolicy vi_policy = ValueIterationPolicy(&env, 1.0, "vi");
    vi_policy.train();

    for (MultiAgentStateIterator s_iter = env.observation_space->begin();
         s_iter != env.observation_space->end(); ++s_iter) {
        MultiAgentState s = *s_iter;
        ASSERT_EQ(h(&s), vi_policy.v[s_iter->id]);
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
    h.init(&env);

    /* Calculate the expected values by value iteration */
    ValueIterationPolicy vi_policy = ValueIterationPolicy(&env, 1.0, "vi");
    vi_policy.train();

    for (MultiAgentStateIterator s_iter = env.observation_space->begin();
         s_iter != env.observation_space->end(); ++s_iter) {
        MultiAgentState s = *s_iter;
        ASSERT_EQ(h(&s), vi_policy.v[s_iter->id]);
    }

}


TEST(HeuristicsTest, DijkstraRoomEnv) {
    MapfEnv *env = create_mapf_env("room-32-32-4", 1, 1, 0, -1000, -1, -1);


    /* Initialize the heuristic */
    DijkstraHeuristic h = DijkstraHeuristic();
    h.init(env);

    /* Calculate the expected values by value iteration */
    ValueIterationPolicy vi_policy = ValueIterationPolicy(env, 1.0, "vi");
    vi_policy.train();

    for (MultiAgentStateIterator s_iter = env->observation_space->begin();
         s_iter != env->observation_space->end(); ++s_iter) {
        MultiAgentState s = *s_iter;
        ASSERT_EQ(h(&s), vi_policy.v[s_iter->id]);
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
    h.init(&env);

    /* Calculate the expected values by value iteration */
    ValueIterationPolicy vi_policy0 = ValueIterationPolicy(&env0, 1.0, "");
    vi_policy0.train();

    /* Calculate the expected values by value iteration */
    ValueIterationPolicy vi_policy1 = ValueIterationPolicy(&env1, 1.0, "");
    vi_policy1.train();

    int exptected_reward = 0;
    bool all_in_goal = true;
    for (MultiAgentStateIterator s_iter = env.observation_space->begin();
         s_iter != env.observation_space->end(); ++s_iter) {

        exptected_reward = 0;
        all_in_goal = true;

        if (s_iter->locations[0] != env.goal_state->locations[0]) {
            all_in_goal = false;
            exptected_reward += vi_policy0.v[s_iter->id] - env.reward_of_goal;
        }

        if (s_iter->locations[1] != env.goal_state->locations[1]) {
            all_in_goal = false;
            exptected_reward += vi_policy1.v[s_iter->id] - env.reward_of_goal;
        }

        if (!all_in_goal) {
            exptected_reward += env.reward_of_goal;
        }

        MultiAgentState s = *s_iter;
        ASSERT_EQ(h(&s), exptected_reward);
    }

}