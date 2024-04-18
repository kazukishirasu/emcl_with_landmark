#ifndef X_MEANS_H_
#define X_MEANS_H_

#include "emcl/register_landmark_node.h"
#include <random>

namespace emcl {

class x_means : public register_landmark
{
public:
    x_means();
    ~x_means();
    int main(const ros::TimerEvent&);
    void initialization(std::vector<Landmark>&);
    int random(int, int);
private:
};

}
#endif