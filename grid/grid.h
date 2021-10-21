//
// Created by levyvonet on 21/10/2021.
//

#ifndef GYM_MAPF_GRID_H
#define GYM_MAPF_GRID_H

#include <vector>

class Cell {
public:
    bool is_obstacle;

    Cell(bool is_obstacle);
};

class Grid {
public:
    std::vector<std::vector<Cell>> map;
    std::size_t max_row;
    std::size_t max_col;

    Grid(std::vector<std::vector<char>> &map_lines);
};

#endif //GYM_MAPF_GRID_H
