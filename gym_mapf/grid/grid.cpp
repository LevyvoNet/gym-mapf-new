//
// Created by levyvonet on 21/10/2021.
//

#include "grid.h"

/** Constants *************************************************************************************************/
#define OBSTACLE_CELL_CHAR  ('@')

Cell::Cell(bool is_obstacle) {
    this->is_obstacle = is_obstacle;
}


/** GridIterator************************************************************/
GridIterator::GridIterator(const Grid *grid) {
    this->grid = grid;

    /* 'Guess' the first valid location of the map, if it is not valid, increment to next valid one */
    this->ptr = new Location(0, 0);

    if ((this->grid->map[this->ptr->row][this->ptr->col].is_obstacle) ||
        (this->ptr->col >= this->grid->max_col) ||
        (this->ptr->row >= this->grid->max_row)) {
        ++(*this);
    }
}

Location *GridIterator::operator->() {
    return this->ptr;
}

Location GridIterator::operator*() const {
    return *(this->ptr);
}

GridIterator GridIterator::operator++() {

    /* Advance at least one time */
    if (this->ptr->col < this->grid->max_col) {
        this->ptr->col++;
    } else {
        this->ptr->col = 0;
        this->ptr->row++;
    }

    /* If we have exhausted all of the locations, return the end */
    if (this->ptr->row > this->grid->max_row) {
        this->ptr = NULL;
        return *this;
    }

    /* Keep advance until the next non-obstacle cell (a valid one) or until we reach the end of the grid */
    while (this->grid->map[this->ptr->row][this->ptr->col].is_obstacle) {

        /* Advance again */
        if (this->ptr->col < this->grid->max_col) {
            this->ptr->col++;
        } else {
            this->ptr->col = 0;
            this->ptr->row++;
        }

        /* Check again if we have reached the end of the grid */
        if (this->ptr->row > this->grid->max_row) {
            this->ptr = NULL;
        }

    }

    return *this;

}

bool GridIterator::operator==(const GridIterator &other) const {
    if (!(this->ptr)) {
        if (!(other.ptr)) {
            return this->grid == other.grid;
        }
    }

    if (!other.ptr) {
        return false;
    }

    return *(this->ptr) == *(other.ptr);
}

bool GridIterator::operator!=(const GridIterator &other) const {
    return !((*this) == other);
}

GridIterator::GridIterator(const Grid *grid, Location *loc) {
    this->grid = grid;
    this->ptr = loc;
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


Location Grid::execute(const Location &l, Action a) {
    if (this->movement_cache[l].find(a) != this->movement_cache[l].end()) {
        return *this->movement_cache[l][a];
    }

    Location new_loc = this->_execute_aux(l, a);
    if (this->map[new_loc.row][new_loc.col].is_obstacle) {
        this->movement_cache[l][a] = new Location( l.row,  l.col);
        return l;
    }


    this->movement_cache[l][a] = new Location(new_loc.row, new_loc.col);
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


GridIterator Grid::begin() const {
    return GridIterator(this);

}

GridIterator Grid::end() const {
    return GridIterator(this, NULL);
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


