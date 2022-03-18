//
// Created by levyvonet on 03/01/2022.
//

#include "available_solvers_envs.h"

/** Envs **********************************************************************************************************/
EmptyGrid::EmptyGrid(string name, size_t grid_size, size_t n_agents, int goal_reward) : EnvCreator(name),
                                                                                        grid_size(grid_size),
                                                                                        goal_reward(goal_reward) {
    this->n_agents = n_agents;
    this->scen_id = 0;
    std::ostringstream map_name;
    map_name << "empty-" << this->grid_size << "-" << this->grid_size;
    this->map_name = map_name.str();
}

MapfEnv *EmptyGrid::operator()() {


    return create_mapf_env(this->map_name, 3, this->n_agents, FAIL_PROB, -1000, this->goal_reward, -1);
}

SymmetricalBottleneck::SymmetricalBottleneck(string name, int goal_reward) : EnvCreator(name),
                                                                             goal_reward(goal_reward) {
    std::ostringstream map_name;
    map_name << "SymmetricalBottleneck-goal=" << goal_reward;
    this->map_name = map_name.str();
    this->scen_id = 0;
    this->n_agents = 2;
}

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
                       this->n_agents,
                       {g->get_location(2, 0), g->get_location(2, 5)},
                       {g->get_location(2, 5), g->get_location(2, 0)},
                       FAIL_PROB,
                       -1000,
                       goal_reward,
                       -1);

}

PaperExample::PaperExample(string name) : EnvCreator(name) {
    std::ostringstream map_name;
    map_name << "PaperExample";
    this->map_name = map_name.str();
    this->scen_id = 0;
    this->n_agents = 2;
}

MapfEnv *PaperExample::operator()() {
    vector<std::string> map_lines({
                                          "..@...",
                                          "......",
                                          "..@...",
                                          "..@...",
                                          "..@...",
                                          "..@...",
                                  });

    Grid *g = new Grid(map_lines);

    return new MapfEnv(g,
                       this->n_agents,
                       {
                               g->get_location(1, 0),
                               g->get_location(1, 5),
//                               g->get_location(5, 0)
                       },
                       {
                               g->get_location(1, 5),
                               g->get_location(1, 0),
//                               g->get_location(5, 5)
                       },
                       FAIL_PROB,
                       -1000,
                       0,
                       -1);
}

ASymmetricalBottleneck::ASymmetricalBottleneck(string name, int goal_reward) : EnvCreator(name),
                                                                               goal_reward(goal_reward) {
    std::ostringstream map_name;
    map_name << "AsymmetricalBottleneck-goal=" << goal_reward;
    this->map_name = map_name.str();
    this->scen_id = 0;
    this->n_agents = 2;
}

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
                       this->n_agents,
                       {g->get_location(2, 0), g->get_location(2, 4)},
                       {g->get_location(2, 4), g->get_location(2, 0)},
                       FAIL_PROB,
                       -1000,
                       goal_reward,
                       -1);

}

RoomEnv::RoomEnv(string name, size_t room_size, size_t n_rooms, size_t scen_id, size_t n_agents) : EnvCreator(name),
                                                                                                   room_size(room_size),
                                                                                                   n_rooms(n_rooms) {
    this->n_agents = n_agents;
    this->scen_id = scen_id;
    std::ostringstream map_name;
    map_name << "room-" << this->room_size << "-" << this->room_size << "-" << this->n_rooms;
    this->map_name = map_name.str();
}

MapfEnv *RoomEnv::operator()() {
    std::ostringstream map_name;
    map_name << "room-" << this->room_size << "-" << this->room_size << "-" << this->n_rooms;
    return create_mapf_env(map_name.str(), this->scen_id, this->n_agents, FAIL_PROB, -1000, 0, -1);
}

SanityEnv::SanityEnv(string name, size_t n_rooms, size_t room_size, size_t n_agents) : EnvCreator(name),
                                                                                       room_size(room_size),
                                                                                       n_rooms(n_rooms) {
    this->n_agents = n_agents;
    this->map_name = "SanityEnv";
    this->scen_id = 0;
}

MapfEnv *SanityEnv::operator()() {
    return create_sanity_mapf_env(this->n_rooms, this->room_size, this->n_agents, FAIL_PROB, -1000, 0, -1);
}

MazeEnv::MazeEnv(string name, size_t maze_size, size_t n_rooms, size_t scen_id, size_t n_agents) : EnvCreator(name),
                                                                                                   maze_size(maze_size),
                                                                                                   n_rooms(n_rooms) {
    this->n_agents = n_agents;
    this->scen_id = scen_id;
    std::ostringstream map_name;
    map_name << "maze-" << this->maze_size << "-" << this->maze_size << "-" << this->n_rooms;
    this->map_name = map_name.str();
}

MapfEnv *MazeEnv::operator()() {

    return create_mapf_env(this->map_name, this->scen_id, this->n_agents, FAIL_PROB, -1000, 0, -1);
}

BerlinEnv::BerlinEnv(string name, size_t scen_id, size_t n_agents) : EnvCreator(name) {
    this->n_agents = n_agents;
    this->scen_id = scen_id;
    this->map_name = "Berlin_1_256";
}

MapfEnv *BerlinEnv::operator()() {
    return create_mapf_env(this->map_name, this->scen_id, this->n_agents, FAIL_PROB, -1000, 0, -1);
}

MapfEnv *GeneralEnv::operator()() {
    return create_mapf_env(this->map_name, this->scen_id, this->n_agents, FAIL_PROB, -1000, 0, -1);
}

GeneralEnv::GeneralEnv(string name, string map_name, size_t scen_id, size_t n_agents) : EnvCreator(name) {
    this->map_name = map_name;
    this->n_agents = n_agents;
    this->scen_id = scen_id;
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

dijkstra_baseline::dijkstra_baseline(string name) : SolverCreator(name) {}

Policy *dijkstra_baseline::operator()(MapfEnv *env, float gamma) {
    return new DijkstraBaselinePolicy(env, gamma, this->name);
}

online_window::online_window(string name, int d, SolverCreator *low_level_planner, window_planner window_planner_func) :
        SolverCreator(name),
        d(d), low_level_planner(low_level_planner), window_planner_func(window_planner_func) {}

Policy *online_window::operator()(MapfEnv *env, float gamma) {
    return new OnlineWindowPolicy(env, gamma, this->name, this->low_level_planner, this->d, this->window_planner_func);
}


