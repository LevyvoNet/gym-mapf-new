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
    int row;
    int col;

    Location(int row, int col);

    bool operator==(const Location &other_loc) const;
    bool operator!=(const Location &other_loc) const;

};

enum Action {
    STAY, UP, RIGHT, DOWN, LEFT
};

class Grid {
private:
    Location _execute_aux(const Location &l, Action a) const;


    Location _execute_up(const Location &l) const;

    Location _execute_right(const Location &l) const;

    Location _execute_down(const Location &l) const;

    Location _execute_left(const Location &l) const;


public:
    std::vector<std::vector<Cell>> map;
    std::size_t max_row;
    std::size_t max_col;

    Grid(std::vector<std::vector<char>> &map_lines);
    Grid(const Grid& g);


    Location execute(const Location &l, Action a) const;
};


#endif //GYM_MAPF_GRID_H
