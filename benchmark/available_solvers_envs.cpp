//
// Created by levyvonet on 03/01/2022.
//

#include "available_solvers_envs.h"

/** Envs **********************************************************************************************************/
EmptyGrid::EmptyGrid(string name, size_t grid_size, size_t n_agents, int goal_reward) : EnvCreator(name),
                                                                                        grid_size(grid_size),
                                                                                        n_agents(n_agents),
                                                                                        goal_reward(goal_reward) {}

MapfEnv *EmptyGrid::operator()() {
    std::ostringstream map_name;
    map_name << "empty-" << this->grid_size << "-" << this->grid_size;

    return create_mapf_env(map_name.str(), 3, this->n_agents, FAIL_PROB, -1000, this->goal_reward, -1);
}

SymmetricalBottleneck::SymmetricalBottleneck(string name, int goal_reward) : EnvCreator(name),
                                                                             goal_reward(goal_reward) {}

MapfEnv *SymmetricalBottleneck::operator()() {
    vector<std::string> map_lines({
                                          "..@...",
                                          "..@...",
                                          "......",
                                          "..@...",
                                          "..@...",
                                          "..@..."
                                  });

    Grid *g = new Grid(map_lines);

    return new MapfEnv(g,
                       2,
                       {g->get_location(2, 0), g->get_location(2, 5)},
                       {g->get_location(2, 5), g->get_location(2, 0)},
                       FAIL_PROB,
                       -1000,
                       goal_reward,
                       -1);

}

ASymmetricalBottleneck::ASymmetricalBottleneck(string name, int goal_reward) : EnvCreator(name),
                                                                               goal_reward(goal_reward) {}

MapfEnv *ASymmetricalBottleneck::operator()() {
    vector<std::string> map_lines({
                                          "..@..",
                                          "..@..",
                                          ".....",
                                          "..@..",
                                          "..@.."
                                  });

    Grid *g = new Grid(map_lines);

    return new MapfEnv(g,
                       2,
                       {g->get_location(2, 0), g->get_location(2, 4)},
                       {g->get_location(2, 4), g->get_location(2, 0)},
                       FAIL_PROB,
                       -1000,
                       goal_reward,
                       -1);

}

RoomEnv::RoomEnv(string name, size_t room_size, size_t n_rooms, size_t scen_id, size_t n_agents) : EnvCreator(name),
                                                                                                   room_size(room_size),
                                                                                                   scen_id(scen_id),
                                                                                                   n_agents(n_agents),
                                                                                                   n_rooms(n_rooms) {}

MapfEnv *RoomEnv::operator()() {
    std::ostringstream map_name;
    map_name << "room-" << this->room_size << "-" << this->room_size << "-" << this->n_rooms;
    return create_mapf_env(map_name.str(), this->scen_id, this->n_agents, FAIL_PROB, -1000, 0, -1);
}

SanityEnv::SanityEnv(string name, size_t n_rooms, size_t room_size, size_t n_agents) : EnvCreator(name),
                                                                                       room_size(room_size),
                                                                                       n_agents(n_agents),
                                                                                       n_rooms(n_rooms) {}

MapfEnv *SanityEnv::operator()() {
    return create_sanity_mapf_env(this->n_rooms, this->room_size, this->n_agents, FAIL_PROB, -1000, 0, -1);
}

MazeEnv::MazeEnv(string name, size_t maze_size, size_t n_rooms, size_t scen_id, size_t n_agents) : EnvCreator(name),
                                                                                                   maze_size(maze_size),
                                                                                                   scen_id(scen_id),
                                                                                                   n_agents(n_agents),
                                                                                                   n_rooms(n_rooms) {}

MapfEnv *MazeEnv::operator()() {
    std::ostringstream map_name;
    map_name << "maze-" << this->maze_size << "-" << this->maze_size << "-" << this->n_rooms;
    return create_mapf_env(map_name.str(), this->scen_id, this->n_agents, FAIL_PROB, -1000, 0, -1);
}

BerlinEnv::BerlinEnv(string name, size_t scen_id, size_t n_agents) : EnvCreator(name), n_agents(n_agents),
                                                                     scen_id(scen_id) {}

MapfEnv *BerlinEnv::operator()() {
    return create_mapf_env("Berlin_1_256", this->scen_id, this->n_agents, FAIL_PROB, -1000, 0, -1);
}

/** Policies *******************************************************************************************************/
vi::vi(string name) : SolverCreator(name) {}

Policy *vi::operator()(MapfEnv *env, float gamma) {
    return new ValueIterationPolicy(env, gamma, this->name);
}

rtdp_dijkstra::rtdp_dijkstra(string name) : SolverCreator(name) {}

Policy *rtdp_dijkstra::operator()(MapfEnv *env, float gamma) {
    return new RtdpPolicy(env, gamma, this->name, new DijkstraHeuristic());
}

rtdp_dijkstra_rtdp::rtdp_dijkstra_rtdp(string name) : SolverCreator(name) {}

Policy *rtdp_dijkstra_rtdp::operator()(MapfEnv *env, float gamma) {
    return new RtdpPolicy(env, gamma, this->name, new RtdpDijkstraHeuristic(gamma));
}

id_rtdp::id_rtdp(string name) : SolverCreator(name) {}

Policy *id_rtdp::operator()(MapfEnv *env, float gamma) {
    return new IdPolicy(env, gamma, this->name, new rtdp_dijkstra_rtdp(""), new RtdpMerger());
}

id_rtdp_default::id_rtdp_default(string name) : SolverCreator(name) {}

Policy *id_rtdp_default::operator()(MapfEnv *env, float gamma) {
    return new IdPolicy(env, gamma, this->name, new rtdp_dijkstra_rtdp(""), nullptr);
}

online_replan::online_replan(string name, int k, SolverCreator *low_level_planner) : SolverCreator(name),
                                                                                     k(k), low_level_planner(
                low_level_planner) {}

Policy *online_replan::operator()(MapfEnv *env, float gamma) {
    return new OnlineReplanPolicy(env, gamma, this->name, this->low_level_planner, this->k);
}

dijkstra_baseline::dijkstra_baseline(string name) : SolverCreator(name) {}

Policy *dijkstra_baseline::operator()(MapfEnv *env, float gamma) {
    return new DijkstraBaselinePolicy(env, gamma, this->name);
}