//
// Created by levyvonet on 05/12/2021.
//

#include <list>

#include <gtest/gtest.h>
#include <gym_mapf/gym_mapf.h>

#include <solvers/solvers.h>

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

class rtdp_dijkstra_rtdp : public SolverCreator {
public:
    rtdp_dijkstra_rtdp(string name) : SolverCreator(name) {}

    virtual Policy *operator()(MapfEnv *env, float gamma) {
        return new RtdpPolicy(env, gamma, this->name, new RtdpDijkstraHeuristic(gamma));
    }
};

TEST(OnlineReplanTest, ConflictAreaConstructionIteration
) {
    std::vector<std::string> lines{{'.', '.', '.', '.', '.', '.', '.', '.'},
                                   {'.', '@', '.', '.', '.', '.', '.', '.'},
                                   {'.', '.', '.', '.', '.', '.', '.', '.'},
                                   {'.', '.', '.', '.', '.', '.', '.', '.'},
                                   {'.', '.', '.', '.', '.', '.', '.', '.'},
                                   {'.', '.', '.', '.', '.', '.', '.', '.'},
                                   {'.', '.', '.', '.', '.', '.', '.', '.'},
                                   {'.', '.', '.', '.', '.', '.', '.', '.'}};
    Grid grid(lines);
    vector<size_t> group{0, 1, 3};
    MultiAgentState s = MultiAgentState({
                                                grid.get_location(1, 2),
                                                grid.get_location(3, 1),
                                                grid.get_location(7, 7),
                                                grid.get_location(5, 1),
                                        }, -1);


    GridArea area = construct_conflict_area(&grid, group, s);

    GridArea expected_area = GridArea(1, 5, 1, 2);
    ASSERT_EQ(area, expected_area
    );

    AreaMultiAgentStateSpace area_space = AreaMultiAgentStateSpace(&grid, area, group.size());
    AreaMultiAgentStateIterator *area_iter = area_space.begin();
    MultiAgentState init_state = **area_iter;

    vector<Location> expected_init_locations{grid.get_location(1, 2),
                                             grid.get_location(1, 2),
                                             grid.get_location(1, 2)};
    area_iter->
            set_locations(expected_init_locations);
    MultiAgentState expected_init_state = **area_iter;
    ASSERT_EQ(init_state, expected_init_state
    );

}

//TEST(OnlineReplanTest, CorridorSwitch) {
//    std::vector<std::string> lines{{'.', '.', '.', '.', '.'},
//                                   {'@', '@', '.', '@', '@'}};
//    Grid grid(lines);
//
//    MapfEnv env = MapfEnv(&grid,
//                          2,
//                          {grid.get_location(0, 0), grid.get_location(0, 4)},
//                          {grid.get_location(0, 4), grid.get_location(0, 0)},
//                          0.2,
//                          1,
//                          0,
//                          -1);
//    int k = 2;
//
//    OnlineReplanPolicy online_policy = OnlineReplanPolicy(&env, 1.0, "", new rtdp_dijkstra_rtdp(""), k);
//
//    online_policy.train();
//
//    EvaluationInfo *eval_info = online_policy.evaluate(1, 100, 1);
//
//    /* Assert a single re-plan was made */
//
//}
//
//TEST(OnineReplanTest, WholeEnvConflictArea) {
//    std::vector<std::string> lines{{'.', '.', '.', '.', '.'},
//                                   {'@', '@', '.', '@', '@'}};
//    Grid grid(lines);
//
//    MapfEnv env = MapfEnv(&grid,
//                          2,
//                          {grid.get_location(0, 0), grid.get_location(0, 4)},
//                          {grid.get_location(0, 4), grid.get_location(0, 0)},
//                          0.2,
//                          1,
//                          0,
//                          -1);
//    int k = 10;
//
//    OnlineReplanPolicy online_policy = OnlineReplanPolicy(&env, 1.0, "", new rtdp_dijkstra_rtdp(""), k);
//
//    online_policy.train();
//
//    EvaluationInfo *eval_info = online_policy.evaluate(1, 100, 1);
//
//    /* Assert a single re-plan was made */
//}

TEST(OnlineReplanTest, GirthStatesNormalIteration
) {
    std::vector<std::string> lines{{'.', '.', '.', '.'},
                                   {'.', '.', '.', '.'},
                                   {'.', '.', '.', '.'},
                                   {'.', '.', '.', '.'},
                                   {'.', '.', '.', '.'}};
    Grid grid(lines);
    MapfEnv aux_env = MapfEnv(&grid,
                              2,
                              {grid.get_location(1, 1), grid.get_location(3, 2)},
                              {grid.get_location(4, 4), grid.get_location(0, 0)},
                              0.2,
                              -1000,
                              0,
                              -1);

    GridArea area = GridArea(1, 3, 1, 2);

    GirthMultiAgentStateSpace girth_space = GirthMultiAgentStateSpace(&grid, area, 2);

    list<MultiAgentState *> states;
    GirthMultiAgentStateIterator *girth_iter = girth_space.begin();
    for (; *girth_iter != *girth_space.

            end();

           ++*girth_iter) {
        MultiAgentState *s = new MultiAgentState((*girth_iter)->locations, (*girth_iter)->id);
        states.
                push_back(s);
    }

    list<MultiAgentState *> expected_states{
            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(0, 0)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(0, 0)}),


            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(0, 1)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(0, 1)}),


            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(0, 2)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(0, 2)}),

            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(0, 3)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(0, 3)}),

            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(1, 3)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(1, 3)}),

            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(2, 3)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(2, 3)}),

            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(3, 3)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(3, 3)}),

            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(4, 3)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(4, 3)}),

            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(4, 2)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(4, 2)}),

            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(4, 1)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(4, 1)}),

            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(4, 0)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(4, 0)}),

            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(3, 0)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(3, 0)}),

            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(2, 0)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(2, 0)}),

            aux_env.locations_to_state({grid.get_location(0, 0), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 1), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 2), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(0, 3), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(1, 3), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(2, 3), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(3, 3), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 3), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 2), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 1), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(4, 0), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(3, 0), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(2, 0), grid.get_location(1, 0)}),
            aux_env.locations_to_state({grid.get_location(1, 0), grid.get_location(1, 0)}),


    };

    ASSERT_TRUE(list_equal_no_order(states, expected_states)
    );

}

TEST(OnlineReplanTest, SymmetricalBottleneckAreaGirth
) {
    vector<std::string> map_lines({
                                          "..@...",
                                          "..@...",
                                          "......",
                                          "..@...",
                                          "..@..."
                                  });

    Grid g = Grid(map_lines);

    MapfEnv env = MapfEnv(&g,
                          2,
                          {g.get_location(2, 0), g.get_location(2, 5)},
                          {g.get_location(2, 5), g.get_location(2, 0)},
                          0.21,
                          -1000,
                          100,
                          -1);

    GridArea conflict_area = GridArea(2, 2, 1, 4);

/* Calculate girth states */
    GirthMultiAgentStateSpace girth_space = GirthMultiAgentStateSpace(&g, conflict_area, 2);
    list<MultiAgentState *> girth_states;
    GirthMultiAgentStateIterator *girth_iter = girth_space.begin();
    for (; *girth_iter != *girth_space.

            end();

           ++*girth_iter) {
        MultiAgentState *s = new MultiAgentState((*girth_iter)->locations, (*girth_iter)->id);
        girth_states.
                push_back(s);
    }

    list<MultiAgentState *> expected_girth_states{
            env.locations_to_state({g.get_location(1, 0), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(1, 0)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(1, 0)}),

            env.locations_to_state({g.get_location(1, 0), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(1, 1)}),

            env.locations_to_state({g.get_location(1, 0), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(1, 3)}),

            env.locations_to_state({g.get_location(1, 0), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(1, 4)}),

            env.locations_to_state({g.get_location(1, 0), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(1, 5)}),

            env.locations_to_state({g.get_location(1, 0), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(2, 5)}),

            env.locations_to_state({g.get_location(1, 0), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(3, 5)}),

            env.locations_to_state({g.get_location(1, 0), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(3, 4)}),

            env.locations_to_state({g.get_location(1, 0), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(3, 3)}),

            env.locations_to_state({g.get_location(1, 0), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(3, 1)}),

            env.locations_to_state({g.get_location(1, 0), g.get_location(3, 0)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(3, 0)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(3, 0)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(3, 0)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(3, 0)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(3, 0)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(3, 0)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(3, 0)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(3, 0)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(3, 0)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(3, 0)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(3, 0)}),

            env.locations_to_state({g.get_location(1, 0), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(1, 1), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(3, 0), g.get_location(2, 0)}),
            env.locations_to_state({g.get_location(2, 0), g.get_location(2, 0)}),
    };


    ASSERT_TRUE(list_equal_no_order(girth_states, expected_girth_states)
    );

/* Calculate area states */
    AreaMultiAgentStateSpace area_space = AreaMultiAgentStateSpace(&g, conflict_area, 2);
    list<MultiAgentState *> area_states;
    AreaMultiAgentStateIterator *area_iter = area_space.begin();
    for (; *area_iter != *area_space.end();++*area_iter) {
        MultiAgentState *s = new MultiAgentState((*area_iter)->locations, (*area_iter)->id);
        area_states.push_back(s);
    }

    list<MultiAgentState *> expected_area_states{
            env.locations_to_state({g.get_location(2, 1), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(2, 2), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(2, 3), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(2, 4), g.get_location(2, 1)}),

            env.locations_to_state({g.get_location(2, 1), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(2, 2), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(2, 3), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(2, 4), g.get_location(2, 2)}),

            env.locations_to_state({g.get_location(2, 1), g.get_location(2, 3)}),
            env.locations_to_state({g.get_location(2, 2), g.get_location(2, 3)}),
            env.locations_to_state({g.get_location(2, 3), g.get_location(2, 3)}),
            env.locations_to_state({g.get_location(2, 4), g.get_location(2, 3)}),

            env.locations_to_state({g.get_location(2, 1), g.get_location(2, 4)}),
            env.locations_to_state({g.get_location(2, 2), g.get_location(2, 4)}),
            env.locations_to_state({g.get_location(2, 3), g.get_location(2, 4)}),
            env.locations_to_state({g.get_location(2, 4), g.get_location(2, 4)}),

    };

    ASSERT_TRUE(list_equal_no_order(area_states, expected_area_states)
    );
}

TEST(OnlineReplanTest, SymmetricalBottleneckAreaGirthDifferentRows
) {
    vector<std::string> map_lines({
                                          "..@...",
                                          "..@...",
                                          "......",
                                          "..@...",
                                          "..@..."
                                  });

    Grid g = Grid(map_lines);

    MapfEnv env = MapfEnv(&g,
                          2,
                          {g.get_location(2, 0), g.get_location(2, 5)},
                          {g.get_location(2, 5), g.get_location(2, 0)},
                          0.21,
                          -1000,
                          100,
                          -1);

    GridArea conflict_area = GridArea(2, 3, 2, 4);

/* Calculate girth states */
    GirthMultiAgentStateSpace girth_space = GirthMultiAgentStateSpace(&g, conflict_area, 2);
    list<MultiAgentState *> girth_states;
    GirthMultiAgentStateIterator *girth_iter = girth_space.begin();
    for (; *girth_iter != *girth_space.end(); ++*girth_iter) {
        MultiAgentState *s = new MultiAgentState((*girth_iter)->locations, (*girth_iter)->id);
        girth_states.push_back(s);
    }

    list<MultiAgentState *> expected_girth_states{
            env.locations_to_state({g.get_location(1, 1), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(1, 1)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(1, 1)}),

            env.locations_to_state({g.get_location(1, 1), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(1, 3)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(1, 3)}),

            env.locations_to_state({g.get_location(1, 1), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(1, 4)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(1, 4)}),

            env.locations_to_state({g.get_location(1, 1), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(1, 5)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(1, 5)}),

            env.locations_to_state({g.get_location(1, 1), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(2, 5)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(2, 5)}),

            env.locations_to_state({g.get_location(1, 1), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(3, 5)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(3, 5)}),

            env.locations_to_state({g.get_location(1, 1), g.get_location(4, 5)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(4, 5)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(4, 5)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(4, 5)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(4, 5)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(4, 5)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(4, 5)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(4, 5)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(4, 5)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(4, 5)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(4, 5)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(4, 5)}),

            env.locations_to_state({g.get_location(1, 1), g.get_location(4, 4)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(4, 4)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(4, 4)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(4, 4)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(4, 4)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(4, 4)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(4, 4)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(4, 4)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(4, 4)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(4, 4)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(4, 4)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(4, 4)}),

            env.locations_to_state({g.get_location(1, 1), g.get_location(4, 3)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(4, 3)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(4, 3)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(4, 3)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(4, 3)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(4, 3)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(4, 3)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(4, 3)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(4, 3)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(4, 3)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(4, 3)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(4, 3)}),

            env.locations_to_state({g.get_location(1, 1), g.get_location(4, 1)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(4, 1)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(4, 1)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(4, 1)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(4, 1)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(4, 1)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(4, 1)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(4, 1)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(4, 1)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(4, 1)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(4, 1)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(4, 1)}),

            env.locations_to_state({g.get_location(1, 1), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(3, 1)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(3, 1)}),

            env.locations_to_state({g.get_location(1, 1), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(1, 3), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(1, 4), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(1, 5), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(2, 5), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(3, 5), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(4, 5), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(4, 4), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(4, 3), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(4, 1), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(3, 1), g.get_location(2, 1)}),
            env.locations_to_state({g.get_location(2, 1), g.get_location(2, 1)}),


    };


    ASSERT_TRUE(list_equal_no_order(girth_states, expected_girth_states));

/* Calculate area states */
    AreaMultiAgentStateSpace area_space = AreaMultiAgentStateSpace(&g, conflict_area, 2);
    list<MultiAgentState *> area_states;
    AreaMultiAgentStateIterator *area_iter = area_space.begin();
    for (; *area_iter != *area_space.end(); ++*area_iter) {
        MultiAgentState *s = new MultiAgentState((*area_iter)->locations, (*area_iter)->id);
        area_states.push_back(s);
    }

    list<MultiAgentState *> expected_area_states{
            env.locations_to_state({g.get_location(2, 2), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(2, 3), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(2, 4), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(2, 2)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(2, 2)}),

            env.locations_to_state({g.get_location(2, 2), g.get_location(2, 3)}),
            env.locations_to_state({g.get_location(2, 3), g.get_location(2, 3)}),
            env.locations_to_state({g.get_location(2, 4), g.get_location(2, 3)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(2, 3)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(2, 3)}),

            env.locations_to_state({g.get_location(2, 2), g.get_location(2, 4)}),
            env.locations_to_state({g.get_location(2, 3), g.get_location(2, 4)}),
            env.locations_to_state({g.get_location(2, 4), g.get_location(2, 4)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(2, 4)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(2, 4)}),

            env.locations_to_state({g.get_location(2, 2), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(2, 3), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(2, 4), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(3, 3)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(3, 3)}),

            env.locations_to_state({g.get_location(2, 2), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(2, 3), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(2, 4), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(3, 3), g.get_location(3, 4)}),
            env.locations_to_state({g.get_location(3, 4), g.get_location(3, 4)}),


    };

    ASSERT_TRUE(list_equal_no_order(area_states, expected_area_states));
}