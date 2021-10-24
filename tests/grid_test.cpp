//
// Created by levyvonet on 21/10/2021.
//

#include <gtest/gtest.h>

#include <gym_mapf.h>

TEST(GridTests, ObstacleCellTest) {
    std::vector<std::vector<char>> map_lines{{'.', '.', '.'},
                                             {'@', '.', '@'}};

    Grid *g = new Grid(map_lines);

    ASSERT_EQ(g->map[0][0].is_obstacle, false);
    ASSERT_EQ(g->map[1][0].is_obstacle, true);

}


TEST(GridTests, SingleAgentAction) {
    std::vector<std::vector<char>> map_lines{{'.', '.', '.'},
                                             {'@', '.', '@'}};

    Grid *g = new Grid(map_lines);

    /* Test regular actions */
    ASSERT_EQ(g->execute(Location(1, 1), UP), Location(0, 1));
    ASSERT_EQ(g->execute(Location(0, 0), RIGHT), Location(0, 1));
    ASSERT_EQ(g->execute(Location(0, 1), LEFT), Location(0, 0));
    ASSERT_EQ(g->execute(Location(0, 1), DOWN), Location(1, 1));

    /* Test obstacles */
    ASSERT_EQ(g->execute(Location(0, 0), DOWN), Location(0, 0));
    ASSERT_EQ(g->execute(Location(1, 1), LEFT), Location(1, 1));
    ASSERT_EQ(g->execute(Location(1, 1), RIGHT), Location(1, 1));
}