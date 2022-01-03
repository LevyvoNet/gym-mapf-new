//
// Created by levyvonet on 21/10/2021.
//

#ifndef GYM_MAPF_GRID_H
#define GYM_MAPF_GRID_H

#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <iostream>

#define ILLEGAL_LOCATION (-1)
#define ACTIONS_COUNT (5)

using namespace std;

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

    friend std::ostream& operator<<(std::ostream& os, const Location& l);

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

    ~Grid();

    Location get_location(int row, int col) const;

    Location execute(const Location &l, Action a);

    GridIterator begin() const;

    GridIterator end() const;

    bool is_legal(const Location& l) const;

    uint64_t calculate_multi_locations_id(vector<Location> locations) const;
};


class GridIterator {
public:
    Location *ptr;
    const Grid *grid;

    GridIterator(const Grid *grid);

    GridIterator(const Grid *grid, Location *loc);

    GridIterator(const Grid *grid, int64_t id);

    Location *operator->();

    Location operator*() const;

    GridIterator &operator++();

    bool operator==(const GridIterator &other) const;

    bool operator!=(const GridIterator &other) const;
};


class GridArea {
public:
    int top_row;
    int bottom_row;
    int left_col;
    int right_col;

    GridArea(int top_row, int bottom_row, int left_col, int right_col);

    bool contains(const Location &l);

    bool operator==(const GridArea &other) const;
};

template<>
class std::hash<GridArea> {
public:
    size_t operator()(const GridArea &area) const;
};

#endif //GYM_MAPF_GRID_H
