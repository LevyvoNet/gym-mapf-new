//
// Created by levyvonet on 27/10/2021.
//

#include "value_table.h"

ValueTable::ValueTable(double default_value) {
    this->default_value = default_value;
    this->v = new std::unordered_map<MultiAgentState, double>();
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
    this->v = new std::unordered_map<MultiAgentState, double>();

    for (const auto &entry: *other.v) {
        (*(this->v))[entry.first] = entry.second;
    }
}
