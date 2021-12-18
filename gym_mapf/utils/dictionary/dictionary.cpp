//
// Created by levyvonet on 02/12/2021.
//

#include "dictionary.h"

Dictionary::Dictionary(double default_value) : default_value(default_value) {
    this->d = new tsl::hopscotch_map<int64_t, double>();
}

double Dictionary::get(int64_t key) {
    if (!this->d->contains(key)) {
        return this->default_value;
    }

    return (*this->d)[key];
}

void Dictionary::set(int64_t key, double value) {
    (*this->d)[key] = value;
}

Dictionary::~Dictionary() {
    delete this->d;
}

bool Dictionary::contains(int64_t key) {
    return this->d->contains(key);
}

Dictionary *Dictionary::clone() {
    Dictionary *res = new Dictionary(this->default_value);

    for (auto iter = this->d->begin(); iter != this->d->end(); ++iter) {
        res->set(iter->first, iter->second);
    }

    return res;
}


