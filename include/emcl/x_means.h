#ifndef X_MEANS_H_
#define X_MEANS_H_

#include <ros/ros.h>
#include <string>
#include <random>
#include <vector>
#include <algorithm>
#include <math.h>
#include "emcl/landmark_struct.h"

namespace emcl {

class x_means
{
public:
    x_means();
    ~x_means();
    void main(std::vector<std::vector<Landmark>>&, std::vector<std::vector<Landmark>>&);
    void k_means(std::vector<Landmark>&, std::vector<Landmark>&);
    void init(std::vector<Landmark>&, std::vector<Landmark>&);
    void select_centroid(std::vector<Landmark>&, std::vector<Landmark>&);
    void calc_bic();
    void allocate_id(std::vector<Landmark>&, std::vector<Landmark>&, bool&);
    void calc_centroid(std::vector<Landmark>&, std::vector<Landmark>&);
    int random(int, int);
private:
    int cluster_n;
};

}
#endif