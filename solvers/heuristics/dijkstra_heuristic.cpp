//
// Created by levyvonet on 03/11/2021.
//

#include "dijkstra_heuristic.h"


/** Private *******************************************************************************/
class LocationNode {
public:
    Location loc;
    int distance;

    LocationNode(Location loc, size_t distance) : loc(loc), distance(distance) {}
};

template<>
class std::greater<LocationNode> {
public:

    bool operator()(const LocationNode &n1, const LocationNode &n2) {
        return (n1.distance > n2.distance);
    }
};

void DijkstraHeuristic::dijkstra_single_agent(size_t agent_idx) {
    Location goal_location = this->env->goal_state->locations[agent_idx];
    bool *visited = new bool[this->env->grid->id_to_loc.size()];
    int *d = this->distance[agent_idx];
    std::priority_queue<LocationNode, vector<LocationNode>, std::greater<LocationNode>> q;
    MultiAgentActionSpace single_action_space = MultiAgentActionSpace(1);
    std::vector<Location> neighbours;
    MultiAgentActionIterator a =single_action_space.begin();
    MultiAgentActionIterator action_space_end = single_action_space.end();

    /* Initialize visited to false */
    std::fill(visited, visited + this->env->grid->id_to_loc.size(), false);

    /* Start with the goal state */
    d[goal_location.id] = 0;
    q.push(LocationNode(goal_location, 0));

    while (q.size() > 0) {
        /* Select the node with the minimum distance */
        LocationNode curr_node = q.top();
        q.pop();

        /* Find the neighbours of the current node */
        neighbours = vector<Location>();
        for (a =single_action_space.begin(); a != action_space_end; ++a) {
            neighbours.push_back(this->env->grid->execute(curr_node.loc, a->actions[0]));
        }

        /* Update each neighbour and push it to the queue if we did not visit it yet */
        for (Location neighbour: neighbours) {
            d[neighbour.id] = min(curr_node.distance + 1, d[neighbour.id]);
            if (!visited[neighbour.id]) {
                q.push(LocationNode(neighbour, d[neighbour.id]));
            }
        }

        visited[curr_node.loc.id] = true;
    }
}


/** Public *******************************************************************************/
void DijkstraHeuristic::init(MapfEnv *env_param) {
    this->env = env_param;
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

    for (size_t i = 0; i < this->env->n_agents; ++i) {
        if (s->locations[i] != this->env->goal_state->locations[i]) {
            sum += this->distance[i][this->env->grid->get_location(s->locations[i].row, s->locations[i].col).id];
        }
    }

    /* If everyone is in goal, return 0 and don't add the goal reward in case there is one */
    if (0 == sum) {
        return 0;
    }


    return sum * this->env->reward_of_living + this->env->reward_of_goal;
}

DijkstraHeuristic::~DijkstraHeuristic() {
    delete[] this->distance;
}
