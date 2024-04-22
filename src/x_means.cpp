#include "emcl/x_means.h"

namespace emcl {

x_means::x_means()
{
}

x_means::~x_means()
{
}

int x_means::random(int min, int max)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(min, max);
    return dist(gen);
}

}