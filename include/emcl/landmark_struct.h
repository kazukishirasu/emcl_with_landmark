#ifndef LANDMARK_STRUCT_H_
#define LANDMARK_STRUCT_H_

#include <string>
#include <yaml-cpp/yaml.h>

struct Pos
{
    float x, y, z;
};
struct Landmark
{
    std::string class_;
    Pos pos_;
    int clusterID_;
    bool enable_;
    YAML::Node option_;
};

#endif