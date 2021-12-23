//
// Created by levyvonet on 23/12/2021.
//

#include "dijkstra_baseline.h"

DijkstraBaselinePolicy::DijkstraBaselinePolicy(MapfEnv *env, float gamma, const string &name) :
        ValueFunctionPolicy(env,
                            gamma,
                            name) {
    this->h = new DijkstraHeuristic();
}

DijkstraBaselinePolicy::~DijkstraBaselinePolicy() {
    delete this->h;
}

void DijkstraBaselinePolicy::train(double timeout_ms) {
    MEASURE_TIME;

    this->h->init(this->env, timeout_ms);

    this->train_info->time = round((ELAPSED_TIME_MS / 1000) * 100) / 100;
}

double DijkstraBaselinePolicy::get_value(MultiAgentState *s) {
    return (*(this->h))(s);
}
