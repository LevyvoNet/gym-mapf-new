//
// Created by levyvonet on 21/10/2021.
//

#include <gtest/gtest.h>

#include <gym-mapf.h>

TEST(GridTests, ObstacleCellTest) {
    std::vector<std::vector<char>> map_lines{{'.', '.', '.'},
                                             {'@', '.', '@'}};

    Grid *g = new Grid(map_lines);

    EXPECT_EQ(g->map[0][0].is_obstacle, false);
    EXPECT_EQ(g->map[1][0].is_obstacle, true);

}