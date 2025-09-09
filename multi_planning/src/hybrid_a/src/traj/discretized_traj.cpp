#include "traj/discretized_traj.h"
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>


bool readCoarsePath(std::vector<TrajectoryPoint>& coarse_path,
            std::vector<TrajectoryPoint>& middle_path,int& size){
    std::string path="/home/jovaa/Desktop/code/record/planning/my_planning/src/traj_opti/example/path.yaml";
    YAML::Node node;
    try{
        node=YAML::LoadFile(path);
    } catch (std::exception& e){
        std::cerr<< "\033[1m\033[31mERROR: Failed to load map file: " << path
              << "\033[0m \n";
        return false;
    }
    int i=0;
    for(const auto& n:node["agent"]){
        // if(i>=size) break;
        TrajectoryPoint tp;
        tp.x=n["x"].as<double>();
        tp.y=n["y"].as<double>();
        tp.theta=n["theta"].as<double>();
        coarse_path.push_back(tp);
        i++;
    }
    for(const auto& n:node["middle"]){
        TrajectoryPoint tp;
        tp.x=n["x"].as<double>();
        tp.y=n["y"].as<double>();
        tp.theta=n["theta"].as<double>();
        middle_path.push_back(tp);
    }
    size=i;
    return true;
}

