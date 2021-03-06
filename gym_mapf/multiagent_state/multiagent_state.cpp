//
// Created by levyvonet on 31/10/2021.
//

#include "multiagent_state.h"


/** MultiAgentState *************************************************************************************************/

MultiAgentState::MultiAgentState(const vector<Location> &locations, int64_t id) {
    this->locations = locations;
    this->id = id;
}

bool MultiAgentState::operator==(const MultiAgentState &other) const {
    return this->id == other.id;
}

bool MultiAgentState::operator!=(const MultiAgentState &other) const {
    return this->id != other.id;
}

std::ostream &operator<<(ostream &os, const MultiAgentState &s) {
    os << "[";
    size_t i=0;
    for (;i<s.locations.size()-1; ++i){
        os << s.locations[i] << ", ";
    }

    os << s.locations[i] << "]";
    return os;
}


/** MultiAgentStateIterator ****************************************************************************************/
MultiAgentStateIterator::MultiAgentStateIterator(const Grid *grid, size_t n_agents) {
    this->grid = grid;
    this->n_agents = n_agents;
    vector<Location> locs;
    for (size_t i = 0; i < this->n_agents; ++i) {
        this->iters.push_back(this->grid->begin());
        locs.push_back(*(this->iters[i]));
    }


    this->ptr = new MultiAgentState(locs, 0);
}

MultiAgentStateIterator::~MultiAgentStateIterator() {
    delete this->ptr;
}

MultiAgentState *MultiAgentStateIterator::operator->() const {
    return this->ptr;
}

MultiAgentState MultiAgentStateIterator::operator*() const {
    return *(this->ptr);
}

MultiAgentStateIterator &MultiAgentStateIterator::operator++() {
    size_t agent_idx = 0;
    bool carry = false;

    /* Increment the first agent, then handle the "carry" */

    do {
        ++(this->iters[agent_idx]);
        if (this->iters[agent_idx] == this->grid->end()) {
            this->iters[agent_idx] = this->grid->begin();
            carry = true;
        } else {
            carry = false;
        }

        this->ptr->locations[agent_idx] = *(this->iters[agent_idx]);
        agent_idx++;

    } while ((agent_idx < this->n_agents) && carry);



    /* Check if we are out because of reaching the last state. If so, return the end */
    if (agent_idx == this->n_agents && carry) {
        this->reach_end();
    }

    this->ptr->id++;

    return *this;

}

bool MultiAgentStateIterator::operator==(const MultiAgentStateIterator &other) const {
    size_t i = 0;

    if (this->iters.size() != other.iters.size()) {
        return false;
    }

    for (i = 0; i < this->iters.size(); ++i) {
        if (this->iters[i] != other.iters[i]) {
            return false;
        }
    }

    return true;
}

bool MultiAgentStateIterator::operator!=(const MultiAgentStateIterator &other) const {
    return !(*this == other);
}

void MultiAgentStateIterator::reach_end() {
    size_t agent_idx = 0;

    for (agent_idx = 0; agent_idx < this->n_agents; agent_idx++) {
        this->iters[agent_idx] = this->grid->end();
    }
}

void MultiAgentStateIterator::reach_begin() {
    vector<Location> locs;
    for (size_t i = 0; i < this->n_agents; ++i) {
        this->iters[i] = this->grid->begin();
        locs.push_back(*(this->iters[i]));
    }

    *(this->ptr) = MultiAgentState(locs, 0);
}

void MultiAgentStateIterator::set_locations(vector<Location> locations) {
    this->ptr->locations = locations;
    this->ptr->id = this->grid->calculate_multi_locations_id(locations);
}

/** MultiAgentStateSpace ***************************************************************************************/
MultiAgentStateIterator *MultiAgentStateSpace::begin() {
    return new MultiAgentStateIterator(this->grid, this->n_agents);
}

MultiAgentStateIterator *MultiAgentStateSpace::end() {
    MultiAgentStateIterator *iter = new MultiAgentStateIterator(this->grid, this->n_agents);
    iter->reach_end();

    return iter;
}

MultiAgentStateSpace::MultiAgentStateSpace(const Grid *grid, size_t n_agents) {
    this->grid = grid;
    this->n_agents = n_agents;
}

/** Area state iterator ******************************************************************************************/
#define END_OF_AREA_ID (-222)
#define IS_END_OF_AREA(l) (l.id == END_OF_AREA_ID)

AreaMultiAgentStateIterator::AreaMultiAgentStateIterator(const Grid *grid,
                                                         GridArea area,
                                                         size_t n_agents) :
        MultiAgentStateIterator(grid, n_agents), area(area) {
    this->_reach_begin();
}

Location inc_area_iterator(const Location *l, GridArea area, const Grid *grid) {
    int row = l->row;
    int col = l->col;

    do {
        ++col;
        if (col > area.right_col) {
            col = area.left_col;
            ++row;
        }
    } while (row <= area.bottom_row && grid->loc_to_id[row][col] == ILLEGAL_LOCATION);

    /* We have reached the end */
    if (row > area.bottom_row) {
        return Location(area.top_row, area.left_col, END_OF_AREA_ID);
    }

    return grid->get_location(row, col);
}

Location area_first_legal_location(const Grid *grid, GridArea area) {
    int first_location_row = area.top_row;
    int first_location_col = area.left_col;

    if (grid->loc_to_id[first_location_row][first_location_col] == ILLEGAL_LOCATION) {
        Location temp_loc = Location(first_location_row, first_location_col, -1);
        return inc_area_iterator(&temp_loc, area, grid);
    } else {

        return grid->get_location(first_location_row, first_location_col);
    }
}

void AreaMultiAgentStateIterator::_reach_begin() {
    /* Start from the first legal location of the area */
    Location first_location = area_first_legal_location(this->grid, this->area);

    vector<Location> locations;
    for (size_t i = 0; i < this->n_agents; ++i) {
        locations.push_back(first_location);
        this->iters[i] = GridIterator(this->grid, first_location.id);
    }
    this->set_locations(locations);
}

void AreaMultiAgentStateIterator::reach_begin() {
    this->_reach_begin();
}

MultiAgentStateIterator &AreaMultiAgentStateIterator::operator++() {
    size_t agent_idx = 0;
    bool carry = false;

    /* Increment the first agent, then handle the "carry" */
    do {
        Location new_location = inc_area_iterator(this->iters[agent_idx].ptr, this->area, this->grid);
        if IS_END_OF_AREA(new_location) {
            Location first_location = area_first_legal_location(this->grid, this->area);
            new_location = first_location;
            carry = true;
        } else {
            carry = false;
        }
        this->iters[agent_idx] = GridIterator(this->grid, new_location.id);

        this->ptr->locations[agent_idx] = *(this->iters[agent_idx]);
        agent_idx++;

    } while ((agent_idx < this->n_agents) && carry);

    /* Check if we are out because of reaching the last state. If so, return the end */
    if (agent_idx == this->n_agents && carry) {
        this->reach_end();
    }

    this->set_locations(this->ptr->locations);

    return *this;
}


AreaMultiAgentStateSpace::AreaMultiAgentStateSpace(const Grid *grid, GridArea area, size_t n_agents) :
        MultiAgentStateSpace(grid, n_agents), area(area) {}

AreaMultiAgentStateIterator *AreaMultiAgentStateSpace::begin() {
    return new AreaMultiAgentStateIterator(this->grid, this->area, this->n_agents);
}

AreaMultiAgentStateIterator *AreaMultiAgentStateSpace::end() {
    AreaMultiAgentStateIterator *iter = new AreaMultiAgentStateIterator(this->grid, this->area, this->n_agents);
    iter->reach_end();

    return iter;
}

/** Girth state iterator *****************************************************************************************/

Location inc_grid_iterator_by_girth_aux(const Location *l, GridArea area, const Grid *grid) {
    int orig_row = l->row;
    int orig_col = l->col;

    Location loc = *l;

    do {
        /* top row, move right */
        if (loc.row == area.top_row - 1 && loc.col < area.right_col + 1) {
            ++loc.col;
            continue;
        }

        /* top row right corner, move down */
        if (loc.row == area.top_row - 1 && loc.col == area.right_col + 1) {
            ++loc.row;
            continue;
        }

        /* right col, move down */
        if (loc.col == area.right_col + 1 && loc.row < area.bottom_row + 1) {
            ++loc.row;
            continue;
        }

        /* right col bottom corner, move left */
        if (loc.col == area.right_col + 1 && loc.row == area.bottom_row + 1) {
            --loc.col;
            continue;
        }

        /* bottom row, move left */
        if (loc.row == area.bottom_row + 1 && loc.col > area.left_col - 1) {
            --loc.col;
            continue;
        }

        /* bottom row left corner, move up */
        if (loc.row == area.bottom_row + 1 && loc.col == area.left_col - 1) {
            --loc.row;
            continue;
        }

        /* left col, move up */
        if (loc.col == area.left_col - 1 && loc.row > area.top_row - 1) {
            --loc.row;
            continue;
        }
    } while (!grid->is_legal(loc) && !(loc.row == orig_row && loc.col == orig_col));

    if (loc.row == orig_row && loc.col == orig_col && !grid->is_legal(loc)) {
        return Location(area.top_row - 1, area.left_col - 1, -1);
    }

    return grid->get_location(loc.row, loc.col);
}

Location girth_first_legal_location(const Grid *grid, GridArea area) {
    Location first_location = Location(area.top_row - 1, area.left_col - 1, -1);
    if (area.top_row - 1 < 0 || area.left_col - 1 < 0 || area.top_row - 1 > grid->max_row ||
        area.left_col - 1 > grid->max_col) {
        first_location = inc_grid_iterator_by_girth_aux(&first_location, area, grid);
    }

    return first_location;

}

void GirthMultiAgentStateIterator::_reach_begin() {
    Location first_location = girth_first_legal_location(this->grid, this->area);

    if ((first_location.row == this->area.top_row - 1) &&
        (first_location.col == this->area.left_col - 1) &&
        (!this->grid->is_legal(first_location))) {
        this->reach_end();
        return;
    }

    /* Set the ID for the first location properly */
    first_location = this->grid->get_location(first_location.row, first_location.col);

    vector<Location> locations;
    for (size_t i = 0; i < this->n_agents; ++i) {
        locations.push_back(first_location);
        this->iters[i] = GridIterator(this->grid, first_location.id);
    }
    this->set_locations(locations);
}

GirthMultiAgentStateIterator::GirthMultiAgentStateIterator(const Grid *grid, GridArea area, size_t n_agents) :
        MultiAgentStateIterator(grid, n_agents), area(area) {
    this->_reach_begin();
}

void GirthMultiAgentStateIterator::reach_begin() {
    this->_reach_begin();
}

MultiAgentStateIterator &GirthMultiAgentStateIterator::operator++() {
    size_t agent_idx = 0;
    bool carry = false;
    Location first_location = girth_first_legal_location(this->grid, this->area);
    GridIterator end = GridIterator(this->grid, &first_location);
    /* Increment the first agent, then handle the "carry" */

    do {
        Location new_location = inc_grid_iterator_by_girth_aux(this->iters[agent_idx].ptr, this->area, this->grid);
        this->iters[agent_idx] = GridIterator(this->grid, new_location.id);
        if (this->iters[agent_idx] == end) {
            carry = true;
        } else {
            carry = false;
        }

        this->ptr->locations[agent_idx] = *(this->iters[agent_idx]);
        agent_idx++;

    } while ((agent_idx < this->n_agents) && carry);

    /* Check if we are out because of reaching the last state. If so, return the end */
    if (agent_idx == this->n_agents && carry) {
        this->reach_end();
    }

    this->set_locations(this->ptr->locations);

    return *this;
}

GirthMultiAgentStateSpace::GirthMultiAgentStateSpace(const Grid *grid, GridArea area, size_t n_agents) :
        MultiAgentStateSpace(grid, n_agents), area(area) {

}

GirthMultiAgentStateIterator *GirthMultiAgentStateSpace::begin() {
    return new GirthMultiAgentStateIterator(this->grid, this->area, this->n_agents);
}

GirthMultiAgentStateIterator *GirthMultiAgentStateSpace::end() {
    GirthMultiAgentStateIterator *iter = new GirthMultiAgentStateIterator(this->grid, this->area, this->n_agents);
    iter->reach_end();

    return iter;
}