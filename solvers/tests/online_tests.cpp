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

TEST(OnlineReplanTest, ConflictAreaConstructionIteration) {
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
    ASSERT_EQ(area, expected_area);

    AreaMultiAgentStateSpace area_space = AreaMultiAgentStateSpace(&grid, area, group.size());
    AreaMultiAgentStateIterator *area_iter = area_space.begin();
    MultiAgentState init_state = **area_iter;

    vector<Location> expected_init_locations{grid.get_location(1, 2),
                                             grid.get_location(1, 2),
                                             grid.get_location(1, 2)};
    area_iter->set_locations(expected_init_locations);
    MultiAgentState expected_init_state = **area_iter;
    ASSERT_EQ(init_state, expected_init_state);

}


TEST(OnlineReplanTest, SingleLocationGirthArea) {
    std::vector<std::string> lines{{'.', '.', '.', '.', '.'},
                                   {'@', '@', '.', '@', '@'}};
    Grid grid(lines);

    MapfEnv env = MapfEnv(&grid,
                          2,
                          {grid.get_location(0, 0), grid.get_location(0, 4)},
                          {grid.get_location(0, 4), grid.get_location(0, 0)},
                          0.2,
                          1,
                          0,
                          -1);

    MultiAgentState *interesting_state = env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 4)});
    GridArea area = construct_conflict_area(&grid, {0, 1}, *interesting_state);


    GirthMultiAgentStateSpace girth_space = GirthMultiAgentStateSpace(&grid, area, 2);
    list<MultiAgentState *> girth_states;
    GirthMultiAgentStateIterator *girth_iter = girth_space.begin();
    for (; *girth_iter != *girth_space.end(); ++*girth_iter) {
        MultiAgentState *s = new MultiAgentState((*girth_iter)->locations, (*girth_iter)->id);
        girth_states.push_back(s);
    }

    list<MultiAgentState *> girth_expected_states{
            env.locations_to_state({grid.get_location(1, 2), grid.get_location(1, 2)})
    };

    ASSERT_TRUE(list_equal_no_order(girth_states, girth_expected_states));


    AreaMultiAgentStateSpace area_space = AreaMultiAgentStateSpace(&grid, area, 2);
    list<MultiAgentState *> area_states;
    AreaMultiAgentStateIterator *area_iter = area_space.begin();
    for (; *area_iter != *area_space.end(); ++*area_iter) {
        MultiAgentState *s = new MultiAgentState((*area_iter)->locations, (*area_iter)->id);
        area_states.push_back(s);
    }

    list<MultiAgentState *> area_expected_states{
            env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 0)}),
            env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 0)}),
            env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 0)}),
            env.locations_to_state({grid.get_location(0, 3), grid.get_location(0, 0)}),
            env.locations_to_state({grid.get_location(0, 4), grid.get_location(0, 0)}),

            env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 1)}),
            env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 1)}),
            env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 1)}),
            env.locations_to_state({grid.get_location(0, 3), grid.get_location(0, 1)}),
            env.locations_to_state({grid.get_location(0, 4), grid.get_location(0, 1)}),

            env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 2)}),
            env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 2)}),
            env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 2)}),
            env.locations_to_state({grid.get_location(0, 3), grid.get_location(0, 2)}),
            env.locations_to_state({grid.get_location(0, 4), grid.get_location(0, 2)}),

            env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 3)}),
            env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 3)}),
            env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 3)}),
            env.locations_to_state({grid.get_location(0, 3), grid.get_location(0, 3)}),
            env.locations_to_state({grid.get_location(0, 4), grid.get_location(0, 3)}),

            env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 4)}),
            env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 4)}),
            env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 4)}),
            env.locations_to_state({grid.get_location(0, 3), grid.get_location(0, 4)}),
            env.locations_to_state({grid.get_location(0, 4), grid.get_location(0, 4)}),
    };

    ASSERT_TRUE(list_equal_no_order(area_states, area_expected_states));


}

TEST(OnlineReplanTest, EmptyGirthWholeEnvConflictArea) {
    std::vector<std::string> lines{{'.', '.', '.'},
                                   {'.', '@', '@'}};
    Grid grid(lines);

    MapfEnv env = MapfEnv(&grid,
                          2,
                          {grid.get_location(1, 0), grid.get_location(0, 2)},
                          {grid.get_location(0, 0), grid.get_location(0, 1)},
                          0.2,
                          1,
                          0,
                          -1);

    MultiAgentState *interesting_state = env.locations_to_state({grid.get_location(1, 0), grid.get_location(0, 2)});
    GridArea area = construct_conflict_area(&grid, {0, 1}, *interesting_state);


    GirthMultiAgentStateSpace girth_space = GirthMultiAgentStateSpace(&grid, area, 2);
    list<MultiAgentState *> girth_states;
    GirthMultiAgentStateIterator *girth_iter = girth_space.begin();
    for (; *girth_iter != *girth_space.end(); ++*girth_iter) {
        MultiAgentState *s = new MultiAgentState((*girth_iter)->locations, (*girth_iter)->id);
        girth_states.push_back(s);
    }

    list<MultiAgentState *> girth_expected_states{};

    ASSERT_TRUE(list_equal_no_order(girth_states, girth_expected_states));


    AreaMultiAgentStateSpace area_space = AreaMultiAgentStateSpace(&grid, area, 2);
    list<MultiAgentState *> area_states;
    AreaMultiAgentStateIterator *area_iter = area_space.begin();
    for (; *area_iter != *area_space.end(); ++*area_iter) {
        MultiAgentState *s = new MultiAgentState((*area_iter)->locations, (*area_iter)->id);
        area_states.push_back(s);
    }

    list<MultiAgentState *> area_expected_states{

            env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 0)}),
            env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 0)}),
            env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 0)}),
            env.locations_to_state({grid.get_location(1, 0), grid.get_location(0, 0)}),

            env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 1)}),
            env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 1)}),
            env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 1)}),
            env.locations_to_state({grid.get_location(1, 0), grid.get_location(0, 1)}),

            env.locations_to_state({grid.get_location(0, 0), grid.get_location(0, 2)}),
            env.locations_to_state({grid.get_location(0, 1), grid.get_location(0, 2)}),
            env.locations_to_state({grid.get_location(0, 2), grid.get_location(0, 2)}),
            env.locations_to_state({grid.get_location(1, 0), grid.get_location(0, 2)}),

            env.locations_to_state({grid.get_location(0, 0), grid.get_location(1, 0)}),
            env.locations_to_state({grid.get_location(0, 1), grid.get_location(1, 0)}),
            env.locations_to_state({grid.get_location(0, 2), grid.get_location(1, 0)}),
            env.locations_to_state({grid.get_location(1, 0), grid.get_location(1, 0)}),
    };

    ASSERT_TRUE(list_equal_no_order(area_states, area_expected_states));


}

TEST(OnlineReplanTest, DeterministicSymmetricalEnvGirthStatesBug) {
    vector<std::string> map_lines({
                                          "..@...",
                                          "..@...",
                                          "......",
                                          "..@...",
                                          "..@...",
                                          "..@..."
                                  });

    Grid *grid = new Grid(map_lines);

    MapfEnv *env = new MapfEnv(grid,
                               2,
                               {grid->get_location(2, 0), grid->get_location(2, 5)},
                               {grid->get_location(2, 5), grid->get_location(2, 0)},
                               0,
                               -1000,
                               0,
                               -1);

    GridArea area = GridArea(1, 3, 3, 4);
    GirthMultiAgentStateSpace girth_space = GirthMultiAgentStateSpace(grid, area, 1);
    list<MultiAgentState *> girth_states;
    GirthMultiAgentStateIterator *girth_iter = girth_space.begin();
    for (; *girth_iter != *girth_space.end(); ++*girth_iter) {
        MultiAgentState *s = new MultiAgentState((*girth_iter)->locations, (*girth_iter)->id);
        girth_states.push_back(s);
    }

    list<MultiAgentState *> girth_expected_states{
            env->locations_to_state({grid->get_location(0, 3)}),
            env->locations_to_state({grid->get_location(0, 4)}),
            env->locations_to_state({grid->get_location(0, 5)}),
            env->locations_to_state({grid->get_location(1, 5)}),
            env->locations_to_state({grid->get_location(2, 5)}),
            env->locations_to_state({grid->get_location(3, 5)}),
            env->locations_to_state({grid->get_location(4, 5)}),
            env->locations_to_state({grid->get_location(4, 4)}),
            env->locations_to_state({grid->get_location(4, 3)}),
            env->locations_to_state({grid->get_location(2, 2)}),
    };

    ASSERT_TRUE(list_equal_no_order(girth_states, girth_expected_states));

}

