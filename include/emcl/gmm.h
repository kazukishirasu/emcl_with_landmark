#ifndef GMM_H_
#define GMM_H_

#include <ros/ros.h>
#include <random>
#include <algorithm>
#include <iterator>
#include "emcl/struct.h"

namespace emcl {

class GMM
{
public:
    GMM();
    ~GMM();
    void main(std::vector<std::vector<Landmark>>&, std::vector<std::vector<Landmark>>&);
    void clustering(std::vector<Landmark>&, std::vector<Landmark>&);
    void initialization(std::vector<Landmark>&, std::vector<Landmark>&, Data_point&, Distribution&);
    void init_mean(Data_point&, Distribution&);
    // void init_cov(std::vector<Landmark>&, std::vector<Distribution>&);
    // void allocate_id(std::vector<Landmark>&, std::vector<Distribution>&);
    int random(int, int);
private:
    int cluster_n;
};

}
#endif