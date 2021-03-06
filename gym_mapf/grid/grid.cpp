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
    this->ptr = this->grid->id_to_loc[0];
}

Location *GridIterator::operator->() {
    return this->ptr;
}

Location GridIterator::operator*() const {
    return *(this->ptr);
}

GridIterator &GridIterator::operator++() {
    if (this->grid->id_to_loc.size() <= this->ptr->id + 1) {
        this->ptr = NULL;
    } else {

        this->ptr = this->grid->id_to_loc[this->ptr->id + 1];
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

GridIterator::GridIterator(const Grid *grid, int64_t id) {
    this->grid = grid;
    this->ptr = this->grid->id_to_loc[id];
}


Grid::Grid(std::vector<std::string> &map_lines) {
    std::size_t i = 0;
    std::size_t j = 0;
    std::size_t n_rows = map_lines.size();
    std::size_t n_cols = map_lines[0].size();
    bool is_obstacle = false;
    int64_t loc_id = 0;

    /* Initialize the array of cells */
    for (i = 0; i < n_rows; ++i) {
        this->loc_to_id.push_back(std::vector<int64_t>());
        this->map.emplace_back(std::vector<Cell>{});
        for (j = 0; j < n_cols; ++j) {
            is_obstacle = map_lines[i][j] == OBSTACLE_CELL_CHAR;
            this->map[i].push_back(Cell(is_obstacle));
            if (!is_obstacle) {
                id_to_loc.push_back(new Location(i, j, loc_id));
                this->loc_to_id[i].push_back(loc_id);
                loc_id++;
            } else {
                this->loc_to_id[i].push_back(ILLEGAL_LOCATION);
            }
        }
    }

    this->max_row = i - 1;
    this->max_col = j - 1;
}


Location Grid::execute(const Location &l, Action a) {
    Location new_loc = this->_execute_aux(l, a);
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
    int new_row = max(0, l.row - 1);
    int new_col = l.col;
    return Location{new_row, new_col, this->loc_to_id[new_row][new_col]};
}

Location Grid::_execute_right(const Location &l) const {
    int new_row = l.row;
    int new_col = min(this->max_col, l.col + 1);
    return Location{new_row, new_col, this->loc_to_id[new_row][new_col]};
}

Location Grid::_execute_down(const Location &l) const {
    int new_row = min(this->max_row, l.row + 1);
    int new_col = l.col;
    return Location{new_row, new_col, this->loc_to_id[new_row][new_col]};
}

Location Grid::_execute_left(const Location &l) const {
    int new_row = l.row;
    int new_col = max(0, l.col - 1);
    return Location{new_row, new_col, this->loc_to_id[new_row][new_col]};
}


GridIterator Grid::begin() const {
    return GridIterator(this);

}

GridIterator Grid::end() const {
    return GridIterator(this, (Location *) NULL);
}

Location Grid::get_location(int row, int col) const {
    return *this->id_to_loc[this->loc_to_id[row][col]];
}

Grid::~Grid() {
    for (Location *l: this->id_to_loc) {
        delete l;
    }

}

bool Grid::is_legal(const Location &l) const {
    if (l.row < 0 || l.row > this->max_row || l.col < 0 || l.col > this->max_col) {
        return 0;
    }

    return this->loc_to_id[l.row][l.col] != ILLEGAL_LOCATION;
}

uint64_t Grid::calculate_multi_locations_id(vector<Location> locations) const {
    uint64_t mul = 1;
    uint64_t sum = 0;
    int n_options = this->id_to_loc.size();

    sum += locations[0].id * mul;

    for (size_t i = 1; i < locations.size(); ++i) {
        mul *= n_options;
        sum += locations[i].id * mul;
    }

    return sum;
}


bool Location::operator==(const Location &other_loc) const {
    return (this->row == other_loc.row) && (this->col == other_loc.col);
}


Location::Location(int row, int col, int64_t id) {
    this->row = row;
    this->col = col;
    this->id = id;
}


bool Location::operator!=(const Location &other_loc) const {
    return (this->row != other_loc.row) || (this->col != other_loc.col);
}

std::ostream &operator<<(std::ostream &os, const Location &l) {
    os << "(" << l.row << "," << l.col << ")";
    return os;
}


size_t hash<GridArea>::operator()(const GridArea &area) const {
    size_t h = 0;
    h += pow(2, area.top_row);
    h += pow(3, area.bottom_row);
    h += pow(5, area.left_col);
    h += pow(7, area.right_col);
    return h;
}

bool GridArea::contains(const Location &l) {
    return ((l.row <= this->bottom_row) &&
            (l.row >= this->top_row) &&
            (l.col <= this->right_col) &&
            (l.col >= this->left_col));
}

GridArea::GridArea(int top_row, int bottom_row, int left_col, int right_col) :
        top_row(top_row), bottom_row(bottom_row), left_col(left_col), right_col(right_col) {}

bool GridArea::operator==(const GridArea &other) const {
    return ((this->top_row == other.top_row) &&
            (this->bottom_row == other.bottom_row) &&
            (this->left_col == other.left_col) &&
            (this->right_col == other.right_col));
}

std::ostream &operator<<(ostream &os, const GridArea &a) {
    os << "rows: " << a.top_row << "->" << a.bottom_row;
    os << " cols: " << a.left_col << "->" << a.right_col;

    return os;
}
