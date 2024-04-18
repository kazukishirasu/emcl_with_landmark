#include "emcl/x_means.h"

namespace emcl {

x_means::x_means()
{
}

x_means::~x_means()
{
}

int x_means::main(const ros::TimerEvent& e)
{
    initialization(lm_list_);
}

void x_means::initialization(std::vector<Landmark>& lm_list)
{
    for (auto &ll:lm_list)
    {
        ll.clusterID_ = random(0, 1);
    }
}

int x_means::random(int min, int max)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(min, max);
    return dist(gen);
}

}