#ifndef GMM_H_
#define GMM_H_

#include <ros/ros.h>
#include <random>
// #include <Eigen/Dense>
#include "emcl/struct.h"

namespace emcl {

class GMM
{
public:
    GMM();
    ~GMM();
    void main(std::vector<std::vector<Landmark>>&, std::vector<std::vector<Landmark>>&);
    void init_list(std::vector<std::vector<Landmark>>&);
    void clustering(std::vector<Landmark>&, std::vector<Landmark>&, std::vector<Distribution>&);
    void initialization(std::vector<Landmark>&, std::vector<Distribution>&);
    void init_mean(std::vector<Landmark>&, std::vector<Distribution>&);
    void init_cov(std::vector<Landmark>&, std::vector<Distribution>&);
    void allocate_id(std::vector<Landmark>&, std::vector<Distribution>&);
    int random(int, int);
private:
    std::vector<std::vector<Distribution>> distr_list;
    int cluster_n;
};

}
#endif