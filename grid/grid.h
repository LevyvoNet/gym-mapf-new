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

class Location {
public:
    std::size_t row;
    std::size_t col;

    Location(std::size_t row, std::size_t col);

    bool operator==(const Location &other_loc) const;
};

enum Action {
    STAY, UP, RIGHT, LEFT, DOWN
};

class Grid {
private:
    Location _execute_aux(const Location &l, Action a);


    Location _execute_up(const Location &l);

    Location _execute_right(const Location &l);

    Location _execute_down(const Location &l);

    Location _execute_left(const Location &l);


public:
    std::vector<std::vector<Cell>> map;
    std::size_t max_row;
    std::size_t max_col;

    Grid(std::vector<std::vector<char>> &map_lines);

    Location execute(const Location &l, Action a);
};


#endif //GYM_MAPF_GRID_H
