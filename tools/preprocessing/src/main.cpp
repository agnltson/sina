#include <iostream>
#include <string>

#include "config.hpp"
#include "preprocessing.hpp"

int main(int argc, char** argv) {
    // argv[0] is the program name, so 3 real arguments means argc == 4 --
    // the original `argc != 3` check would reject every valid invocation.
    if (argc != 4) {
        std::cerr << "usage: " << argv[0] << " <config_path> <vrs_path> <trajectory_path>\n";
        return 1;
    }

    std::string config_path = argv[1];
    std::string vrs_path = argv[2];
    std::string trajectory_path = argv[3];

    try {
        preprocessing::Config config = preprocessing::load_config(config_path);
        preprocessing::preprocess(config, vrs_path, trajectory_path);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
