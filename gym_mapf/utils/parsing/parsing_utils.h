//
// Created by levyvonet on 26/10/2021.
//

#ifndef GYM_MAPF_PARSING_UTILS_H
#define GYM_MAPF_PARSING_UTILS_H

#include <fstream>
#include <string>
#include <sstream>

#include <mapf_env/mapf_env.h>
#include <grid/grid.h>

std::vector<std::string> parse_map_file(string file_path);

void parse_scen_file(std::string file_path, size_t n_agents, Grid *grid, vector<Location> *start_locations,
                     vector<Location> *goal_locations);

MapfEnv* create_mapf_env(std::string map_name,
                         size_t scen_id,
                         size_t n_agents,
                         float fail_prob,
                         int collision_reward,
                         int goal_reward,
                         int living_reward);

#endif //GYM_MAPF_PARSING_UTILS_H
