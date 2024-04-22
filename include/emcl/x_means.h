#ifndef X_MEANS_H_
#define X_MEANS_H_

#include <ros/ros.h>
#include <string>
#include <random>
#include <vector>
#include "emcl/landmark_struct.h"

namespace emcl {

class x_means
{
public:
    x_means();
    ~x_means();
    int random(int, int);
private:
};

}
#endif