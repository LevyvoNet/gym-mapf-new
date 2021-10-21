//
// Created by levyvonet on 21/10/2021.
//

#include "grid.h"

/** Constants *************************************************************************************************/
#define OBSTACLE_CELL_CHAR  ('@')

Cell::Cell(bool is_obstacle) {
    this->is_obstacle = is_obstacle;
}

Grid::Grid(std::vector<std::vector<char>>& map_lines) {
    std::size_t i = 0;
    std::size_t j = 0;
    std::size_t n_rows = map_lines.size();
    std::size_t n_cols = map_lines[0].size();


    for (i = 0; i < n_rows; ++i) {
        this->map.push_back(std::vector<Cell>{});
        for (j = 0; j < n_cols; ++j) {
            this->map[i].push_back(Cell(map_lines[i][j] == OBSTACLE_CELL_CHAR));
        }
    }

    this->max_row = i - 1;
    this->max_col = j - 1;
}