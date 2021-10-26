//
// Created by levyvonet on 21/10/2021.
//

#include "grid.h"

/** Constants *************************************************************************************************/
#define OBSTACLE_CELL_CHAR  ('@')

Cell::Cell(bool is_obstacle) {
    this->is_obstacle = is_obstacle;
}


Grid::Grid(std::vector<std::string> &map_lines) {
    std::size_t i = 0;
    std::size_t j = 0;
    std::size_t n_rows = map_lines.size();
    std::size_t n_cols = map_lines[0].size();

    /* Initialize the array of cells */
    for (i = 0; i < n_rows; ++i) {
        this->map.emplace_back(std::vector<Cell>{});
        for (j = 0; j < n_cols; ++j) {
            this->map[i].push_back(Cell(map_lines[i][j] == OBSTACLE_CELL_CHAR));
        }
    }

    this->max_row = i - 1;
    this->max_col = j - 1;
}


Location Grid::execute(const Location &l, Action a) const {
    Location new_loc{0, 0};

    new_loc = this->_execute_aux(l, a);
    if (this->map[new_loc.row][new_loc.col].is_obstacle) {
        return l;
    }

    return new_loc;
}

int max(const int &val1, const int &val2) {
    if (val2 > val1) {
        return val2;
    }

    return val1;
}

int min(const int &val1, const int &val2) {
    if (val2 < val1) {
        return val2;
    }

    return val1;
}

Location Grid::_execute_aux(const Location &l, Action a) const {
    switch (a) {
        case STAY:
            return l;
        case UP:
            return this->_execute_up(l);
        case RIGHT:
            return this->_execute_right(l);
        case DOWN:
            return this->_execute_down(l);
        case LEFT:
            return this->_execute_left(l);
    }
}

Location Grid::_execute_up(const Location &l) const {
    return Location{max(0, l.row - 1), l.col};
}

Location Grid::_execute_right(const Location &l) const {
    return Location{l.row, min(this->max_col, l.col + 1)};
}

Location Grid::_execute_down(const Location &l) const {
    return Location{min(this->max_row, l.row + 1), l.col};
}

Location Grid::_execute_left(const Location &l) const {
    return Location{l.row, max(0, l.col - 1)};
}


bool Location::operator==(const Location &other_loc) const {
    return (this->row == other_loc.row) && (this->col == other_loc.col);
}

Location::Location(int row, int col) {
    this->row = row;
    this->col = col;
}

bool Location::operator!=(const Location &other_loc) const {
    return (this->row != other_loc.row) || (this->col != other_loc.col);
}
