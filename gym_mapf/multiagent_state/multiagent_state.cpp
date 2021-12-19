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