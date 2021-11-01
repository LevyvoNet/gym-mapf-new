//
// Created by levyvonet on 31/10/2021.
//

#include "state_storage.h"

/** Hash functions implementations *******************************************************************************/
size_t std::hash<Location>::operator()(const Location &l) const {
//    return pow(2, l.row) + pow(3, l.col);
    return std::hash<int64_t>()(l.id);
}

size_t hash<MultiAgentState>::operator()(const MultiAgentState &s) const {
//    size_t h = 0;
//    int i = 0;
//
//    for (i = 0; i < s.locations.size(); i++) {
//        h += i ^ hash<Location>()(s.locations[i]);
//    }
//
//    return h;
    return std::hash<int64_t>()(s.id);
}

size_t hash<MultiAgentAction>::operator()(const MultiAgentAction &a) const {
//    size_t h = 0;
//    for (size_t i = 0; i < a.actions.size(); ++i) {
//        h += i ^ a.actions[i];
//    }
//
//    return h;
    return std::hash<int64_t>()(a.id);
}


/** ValueTable *****************************************************************************************************/
ValueTable::ValueTable(double default_value) {
    this->default_value = default_value;
    this->v = new tsl::hopscotch_map<MultiAgentState, double>();
}

double ValueTable::operator[](const MultiAgentState &s) {
    if (this->v->find(s) == this->v->end()) {
        return this->default_value;
    }

    return (*this->v)[s];
}

void ValueTable::set(const MultiAgentState &s, double value) {
    (*(this->v))[s] = value;
}

ValueTable::ValueTable(const ValueTable &other) {
    this->default_value = other.default_value;
    this->v = new tsl::hopscotch_map<MultiAgentState, double>();

    for (const auto &entry: *other.v) {
        (*(this->v))[entry.first] = entry.second;
    }
}