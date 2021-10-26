//
// Created by levyvonet on 21/10/2021.
//
#include <gym_mapf.h>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
    std::vector<std::string> empty_8_8{{'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'},
                                       {'.', '.', '.', '.', '.', '.', '.', '.'}};
    Grid g(empty_8_8);
    MultiAgentState start_state = MultiAgentState({Location(0, 0), Location(7, 7)});
    MultiAgentState goal_state = MultiAgentState({Location(0, 2), Location(5, 7)});


    std::cout << "hello" << std::endl;

}