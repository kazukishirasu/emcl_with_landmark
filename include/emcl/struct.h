#ifndef LANDMARK_STRUCT_H_
#define LANDMARK_STRUCT_H_

#include <string>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

struct Pos
{
    float x, y, z;
};

struct Landmark
{
    Pos pos_;
    int clusterID_;
    bool enable_;
    YAML::Node option_;
};

struct Distribution
{
    Pos mean_;
    Eigen::Matrix2f cov_;
    float pi_;
    int clusterID_;
};

#endif