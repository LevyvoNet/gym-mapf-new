//
// Created by levyvonet on 31/10/2021.
//

#include "multiagent_action.h"

/** MultiAgentAction *********************************************************************************************/
MultiAgentAction::MultiAgentAction(const vector<Action> &actions, int64_t id) {
    this->actions = actions;
    this->id = id;
}

bool MultiAgentAction::operator==(const MultiAgentAction &other) const {
//    size_t agent_idx = 0;
//
//    if (this->actions.size() != other.actions.size()) {
//        return false;
//    }
//
//    for (agent_idx = 0; agent_idx < this->actions.size(); ++agent_idx) {
//
//        if (this->actions[agent_idx] != other.actions[agent_idx]) {
//            return false;
//        }
//    }
//
//    return true;
    return this->id == other.id;
}

MultiAgentAction::MultiAgentAction(size_t n_agents) {
    this->actions = vector<Action>(n_agents);
    for (size_t i = 0; i < n_agents; ++i) {
        this->actions[i] = STAY;
    }
    this->id = 0;
}

/** MultiAgentActionIterator *************************************************************************************/
MultiAgentActionIterator::MultiAgentActionIterator(size_t n_agents, vector<bool> is_effective_agent) {
    this->n_agents = n_agents;
    this->is_effective_agent = is_effective_agent;

    vector<Action> all_stay(n_agents);

    for (size_t i = 0; i < n_agents; ++i) {
        all_stay[i] = STAY;
    }

    this->ptr = new MultiAgentAction(all_stay, 0);
}

MultiAgentActionIterator::~MultiAgentActionIterator() {
    delete this->ptr;
}

void MultiAgentActionIterator::reach_end() {
    for (size_t i = 0; i < this->n_agents; ++i) {
        this->ptr->actions[i] = LAST_INVALID_ACTION;
    }
}

void MultiAgentActionIterator::reach_begin() {
    vector<Action> all_stay(n_agents);

    for (size_t i = 0; i < n_agents; ++i) {
        all_stay[i] = STAY;
    }

    this->ptr->actions = all_stay;
    this->ptr->id = 0;
}

MultiAgentAction *MultiAgentActionIterator::operator->() const {
    return this->ptr;
}

MultiAgentAction MultiAgentActionIterator::operator*() const {
    return *(this->ptr);
}

MultiAgentActionIterator &MultiAgentActionIterator::operator++() {
    size_t agent_idx = 0;
    bool carry = false;

    /* Increment the first and handle the "carry" */
    do {
        carry = false;

        /* Inc if this agent is effective, else "fake" it */
        if (!is_effective_agent[agent_idx]) {
            this->ptr->actions[agent_idx] = LAST_INVALID_ACTION;
        } else {
            this->ptr->actions[agent_idx] = (Action) (int(this->ptr->actions[agent_idx]) + 1);
        }

        if (this->ptr->actions[agent_idx] == LAST_INVALID_ACTION) {
            this->ptr->actions[agent_idx] = STAY;
            carry = true;
        }

        agent_idx++;


    } while (carry && (agent_idx < this->n_agents));

    if (agent_idx >= this->n_agents && carry) {
        this->reach_end();
    }

    this->ptr->id++;
    return *this;
}

bool MultiAgentActionIterator::operator==(const MultiAgentActionIterator &other) const {
    for (size_t i = 0; i < this->n_agents; ++i) {
        if (this->ptr->actions[i] != other.ptr->actions[i]) {
            return false;
        }
    }

    return true;
}

bool MultiAgentActionIterator::operator!=(const MultiAgentActionIterator &other) const {
    for (size_t i = 0; i < this->n_agents; ++i) {
        if (this->ptr->actions[i] == other.ptr->actions[i]) {
            return false;
        }
    }

    return true;
}


/** MultiAgentActionSpace ****************************************************************************************/
MultiAgentActionSpace::MultiAgentActionSpace(size_t n_agents) {
    this->n_agents = n_agents;
    for (size_t i = 0; i < this->n_agents; ++i) {
        this->is_effective_agent.push_back(true);
    }
}

MultiAgentActionIterator *MultiAgentActionSpace::begin() {
    return new MultiAgentActionIterator(this->n_agents, this->is_effective_agent);
}

MultiAgentActionIterator *MultiAgentActionSpace::end() {
    MultiAgentActionIterator *iter = new MultiAgentActionIterator(this->n_agents, this->is_effective_agent);
    iter->reach_end();

    return iter;
}

MultiAgentActionSpace::MultiAgentActionSpace(size_t n_agents, vector<bool> is_effective_agent) {
    this->n_agents = n_agents;
    this->is_effective_agent = is_effective_agent;
}
