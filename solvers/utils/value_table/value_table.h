//
// Created by levyvonet on 27/10/2021.
//

#ifndef GYM_MAPF_VALUE_TABLE_H
#define GYM_MAPF_VALUE_TABLE_H

#include <unordered_map>

#include <gym_mapf/gym_mapf.h>

class ValueTable {
private:
    double default_value;
    std::unordered_map<MultiAgentState, double> *v;

public:
    ValueTable(double default_value);

    ValueTable(const ValueTable& other);

    double operator[](const MultiAgentState &s);

    void set(const MultiAgentState &s, double value);
};

#endif //GYM_MAPF_VALUE_TABLE_H
