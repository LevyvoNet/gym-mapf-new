//
// Created by levyvonet on 26/10/2021.
//

#include "parsing_utils.h"

std::vector<std::string> parse_map_file(string file_path) {
    std::string line;
    std::vector<std::string> lines;
    std::ifstream infile(file_path);

    /* Read the first redundant 4 lines */
    for (int i = 0; i < 4; i++) {
        std::getline(infile, line);
    }

    /* Now read the lines and return them */
    while (std::getline(infile, line)) {
        lines.push_back(line);
    }

    return lines;
}

std::string get_maps_dir() {
    std::string file_path = __FILE__;
    std::string dir_path = file_path.substr(0, file_path.find_last_of("/\\"));
    std::ostringstream maps_dir;
    maps_dir << dir_path << "/../../maps/";

    return maps_dir.str();
}

void parse_scen_file(std::string file_path,
                     size_t n_agents,
                     MultiAgentState *start_state,
                     MultiAgentState *goal_state) {
    std::vector<Location> start_locations;
    std::vector<Location> goal_locations;
    std::ifstream infile(file_path);
    std::string line;
    std::string start_row;
    std::string start_col;
    std::string goal_row;
    std::string goal_col;
    std::string delimiter = "\t";
    size_t i = 0;
    size_t j = 0;
    size_t pos = 0;
    Location start_loc(0, 0,DONT_CARE_ID);
    Location goal_loc(0, 0, DONT_CARE_ID);

    /* Dump the first line */
    std::getline(infile, line);

    for (i = 0; i < n_agents; i++) {
        std::getline(infile, line);

        /* Skip on the first 4 redundant parts of the line */
        for (j = 1; j <= 4; j++) {
            pos = line.find(delimiter);

            line.erase(0, pos + delimiter.length());
        }

        /* cut start row */
        pos = line.find(delimiter);
        start_row = line.substr(0, pos);
        line.erase(0, pos + delimiter.length());

        /* cut start col */
        pos = line.find(delimiter);
        start_col = line.substr(0, pos);
        line.erase(0, pos + delimiter.length());

        /* cut goal row */
        pos = line.find(delimiter);
        goal_row = line.substr(0, pos);
        line.erase(0, pos + delimiter.length());

        /* cut goal col */
        pos = line.find(delimiter);
        goal_col = line.substr(0, pos);
        line.erase(0, pos + delimiter.length());

        start_loc = Location(std::stoi(start_row), std::stoi(start_col), DONT_CARE_ID);
        goal_loc = Location(std::stoi(goal_row), std::stoi(goal_col), DONT_CARE_ID);

        start_state->locations.push_back(start_loc);
        goal_state->locations.push_back(goal_loc);
    }
}

MapfEnv *create_mapf_env(std::string map_name,
                         size_t scen_id,
                         size_t n_agents,
                         float fail_prob,
                         int collision_reward,
                         int goal_reward,
                         int living_reward) {
    /* Create the grid */
    std::ostringstream map_file_string_stream;
    map_file_string_stream << get_maps_dir() << "/" << map_name << "/" << map_name << ".map";
    std::vector<std::string> map_lines = parse_map_file(map_file_string_stream.str());

    /* Create the start and goal states */
    MultiAgentState start_state({});
    MultiAgentState goal_state({});
    std::ostringstream scen_file_string_stream;
    scen_file_string_stream << get_maps_dir() << "/" << map_name << "/" << map_name << "-even-" << scen_id << ".scen";
    parse_scen_file(scen_file_string_stream.str(), n_agents, &start_state, &goal_state);

    return new MapfEnv(new Grid(map_lines),
                       n_agents,
                       &start_state,
                       &goal_state,
                       fail_prob,
                       collision_reward,
                       goal_reward,
                       living_reward
    );
}