//
// Created by levyvonet on 21/10/2021.
//

#ifndef GYM_MAPF_GRID_H
#define GYM_MAPF_GRID_H

#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>

#define ILLEGAL_LOCATION (-1)
#define ACTIONS_COUNT (5)

class Cell {
public:
    bool is_obstacle;

    Cell(bool is_obstacle);
};

class Location {
public:
    int row;
    int col;
    int64_t id;

    Location(int row, int col, int64_t id);

    bool operator==(const Location &other_loc) const;

    bool operator!=(const Location &other_loc) const;

};

enum Action {
    STAY = 0,
    UP = 1,
    RIGHT = 2,
    DOWN = 3,
    LEFT = 4,
    LAST_INVALID_ACTION = 5
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
    std::vector<Location *> id_to_loc;
    std::vector<std::vector<int64_t>> loc_to_id;

    Grid(std::vector<std::string> &map_lines);

    Location get_location(int row, int col);

    Location execute(const Location &l, Action a);

    GridIterator begin() const;

    GridIterator end() const;
};


class GridIterator {
private:
    Location *ptr;
    const Grid *grid;

public:

    GridIterator(const Grid *grid);

    GridIterator(const Grid *grid, Location *loc);

    Location *operator->();

    Location operator*() const;

    GridIterator& operator++();

    bool operator==(const GridIterator &other) const;

    bool operator!=(const GridIterator &other) const;
};

#endif //GYM_MAPF_GRID_H
