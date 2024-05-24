#ifndef LANDMARK_STRUCT_H_
#define LANDMARK_STRUCT_H_

#include <iostream>
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

struct Data_point
{
    Eigen::MatrixXd point_;
    Eigen::MatrixXd responsibility_;
    std::vector<float> init_id;
};

struct Distribution
{
    Eigen::MatrixXd mean_;
    std::vector<Eigen::Matrix2d> cov_;
    std::vector<float> mixture_weight_;
};

#endif