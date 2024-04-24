#include "emcl/x_means.h"

namespace emcl {

x_means::x_means()
{
}

x_means::~x_means()
{
}

void x_means::main(std::vector<Landmark>& bc, std::vector<Landmark>& ac)
{
    k_means(bc, ac);
}

void x_means::k_means(std::vector<Landmark>& bc, std::vector<Landmark>& ac)
{
    if (!bc.empty())
    {
        initialization(bc, ac);
        for (size_t i = 0; i < 100; i++)
        {
            calc_centroid(bc, ac);
            allocate_id(bc, ac);
            if (i < 99)
            {
                ac.clear();
            }
        }
    }
    ROS_INFO("size of bc: %ld", bc.size());
}

void x_means::initialization(std::vector<Landmark>& bc, std::vector<Landmark>& ac)
{
    struct Landmark lm;
    for (auto &b:bc)
    {
        b.clusterID_ = random(0, cluster_n);
    }
    for (size_t i = 0; i < cluster_n; i++)
    {
        int num = random(0, bc.size());
        lm = bc[num];
        ac.push_back(lm);
    }
}

void x_means::calc_centroid(std::vector<Landmark>& bc, std::vector<Landmark>& ac)
{
    for (size_t i = 0; i < cluster_n; i++)
    {
        struct Landmark lm;
        float sum_x = 0, sum_y = 0;
        int num = 0;
        for (const auto &b:bc)
        {
            if (b.clusterID_ == i)
            {
                lm.class_ = b.class_;
                sum_x += b.pos_.x;
                sum_y += b.pos_.y;
                num++;
            }
        }
        ROS_INFO("cluster ID: %ld, centroid_x: %lf, centroid_y: %lf, num: %d", i, sum_x/num, sum_y/num, num);
        // ROS_INFO("cluster ID: %ld, sum_x: %lf, sum_y: %lf, num: %d", i, sum_x, sum_y, num);
        lm.clusterID_ = i;
        lm.pos_.x = sum_x/num;
        lm.pos_.y = sum_y/num;
        ac.push_back(lm);
    }
    ROS_INFO("--------------------");
}

void x_means::allocate_id(std::vector<Landmark>& bc, std::vector<Landmark>& ac)
{
    for (auto &b:bc)
    {
        float dist, min_dist = 1000000;
        for (const auto &a:ac)
        {
            dist = std::hypot(a.pos_.x - b.pos_.x, a.pos_.y - b.pos_.y);
            if (dist < min_dist)
            {
                min_dist = dist;
                b.clusterID_ = a.clusterID_;
            }
        }
    }
}

// int x_means::random()
// {
//     static std::random_device rd;
//     static std::mt19937 gen(rd());
//     std::uniform_int_distribution<> dist(0, cluster_n - 1);
//     return dist(gen);
// }

int x_means::random(int min, int max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(min, max - 1);
    return dist(gen);
}

}