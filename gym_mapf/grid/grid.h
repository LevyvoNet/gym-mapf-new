//
// Created by levyvonet on 21/10/2021.
//

#ifndef GYM_MAPF_GRID_H
#define GYM_MAPF_GRID_H

#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>

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
    STAY, UP, RIGHT, DOWN, LEFT, LAST_INVALID_ACTION
};

/* Forward declaration */
class GridIterator;

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

    Grid(std::vector<std::string> &map_lines);


    Location execute(const Location &l, Action a);

    GridIterator begin() const;

    GridIterator end() const;
};


template<>
class std::hash<Location> {
public:
    inline size_t operator()(const Location &l) const;
};

inline size_t std::hash<Location>::operator()(const Location &l) const {
    return pow(2, l.row) + pow(3, l.col);
}

class GridIterator {
private:
    Location *ptr;
    const Grid *grid;

public:

    GridIterator(const Grid *grid);

    GridIterator(const Grid *grid, Location *loc);

    Location *operator->();

    Location operator*() const;

    GridIterator operator++();

    bool operator==(const GridIterator &other) const;

    bool operator!=(const GridIterator &other) const;
};

#endif //GYM_MAPF_GRID_H
