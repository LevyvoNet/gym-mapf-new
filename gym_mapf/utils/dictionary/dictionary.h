//
// Created by levyvonet on 02/12/2021.
//

#ifndef GYM_MAPF_DICTIONARY_H
#define GYM_MAPF_DICTIONARY_H

#include <tsl/hopscotch_map.h>

class Dictionary {
public:
    tsl::hopscotch_map<int64_t, double> *d;
    double default_value = 0;

    Dictionary(double default_value);

    ~Dictionary();

    double get(int64_t key);

    void set(int64_t key, double value);

    bool contains(int64_t key);

    Dictionary* clone();
};

#endif //GYM_MAPF_DICTIONARY_H
