//
// Created by levyvonet on 03/11/2021.
//

#include "dijkstra_heuristic.h"


/** Private *******************************************************************************/
size_t min_not_visited(int *distance, bool *visited, size_t size) {
    size_t min_idx = 0;
    int min_val = std::numeric_limits<int>::max();
    for (size_t i = 0; i < size; ++i) {
        if (!visited[i] && distance[i] < min_val) {
            min_val = distance[i];
            min_idx = i;
        }
    }

    return min_idx;
}

void DijkstraHeuristic::dijkstra_single_agent(size_t agent_idx) {
    Location goal_location = this->env->goal_state->locations[agent_idx];
    int *d = this->distance[agent_idx];
    MultiAgentActionSpace single_action_space = MultiAgentActionSpace(1);
    std::vector<Location> neighbours;
    MultiAgentActionIterator a = single_action_space.begin();
    MultiAgentActionIterator action_space_end = single_action_space.end();
    Location *curr_location = &goal_location;
    int curr_location_id = curr_location->id;
    unsigned long n_locs = this->env->grid->id_to_loc.size();
    bool *visited = new bool[n_locs];

    /* Initialize visited to false */
    std::fill(visited, visited + n_locs, false);

    /* Start with the goal state */
    d[goal_location.id] = 0;

    for (size_t i = 1; i <= n_locs; ++i) {
        curr_location_id = min_not_visited(d, visited, n_locs);
        curr_location = this->env->grid->id_to_loc[curr_location_id];


        /* Find the neighbours of the current node */
        neighbours.clear();
        for (a.reach_begin(); a != action_space_end; ++a) {
            neighbours.push_back(this->env->grid->execute(*curr_location, a->actions[0]));
        }

        /* Update each neighbour and push it to the queue if we did not visit it yet */
        for (Location neighbour: neighbours) {
            d[neighbour.id] = min(d[curr_location_id] + 1, d[neighbour.id]);
        }

        visited[curr_location_id] = true;
    }
l_cleanup:
    delete[] visited;
}


/** Public *******************************************************************************/
void DijkstraHeuristic::init(MapfEnv *env_param, double timeout_milliseconds) {
    this->env = env_param;
    this->n_agents = this->env->n_agents;
    this->distance = new int *[env->n_agents];

    /* Allocate the distance for each agent */
    for (size_t i = 0; i < this->env->n_agents; ++i) {
        this->distance[i] = new int[this->env->grid->id_to_loc.size()];
        std::fill(this->distance[i],
                  this->distance[i] + this->env->grid->id_to_loc.size(),
                  std::numeric_limits<int>::max());
    }

    /* Compute the distance for each agent */
    for (size_t i = 0; i < this->env->n_agents; ++i) {
        this->dijkstra_single_agent(i);
    }
}

double DijkstraHeuristic::operator()(MultiAgentState *s) {
    double sum = 0;
    size_t count = 0;

    for (size_t i = 0; i < this->env->n_agents; ++i) {
        if (s->locations[i] != this->env->goal_state->locations[i]) {
            sum += this->distance[i][this->env->grid->get_location(s->locations[i].row, s->locations[i].col).id];
            ++count;
        }
    }

    /* If everyone is in goal, return 0 and don't add the goal reward in case there is one */
    if (0 == count) {
        return 0;
    }


    return sum * this->env->reward_of_living + this->env->reward_of_goal;
}

DijkstraHeuristic::~DijkstraHeuristic() {
    for (size_t i=0;i<this->n_agents;++i){
        delete[] this->distance[i];
    }
//    delete[] this->distance;
}
