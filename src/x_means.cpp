#include "emcl/x_means.h"

namespace emcl {

x_means::x_means()
{
}

x_means::~x_means()
{
}

void x_means::main(std::vector<std::vector<Landmark>>& data, std::vector<std::vector<Landmark>>& result)
{
    for (size_t i = 0; i < data.size(); i++)
    {
        if (i == 0)
        {
            cluster_n = 10;
        }else if (i == 1)
        {
            cluster_n = 6;
        }else
        {
            cluster_n = 2;
        }
        k_means(data[i], result[i]);
        ROS_INFO("====================");
    }
}

void x_means::k_means(std::vector<Landmark>& data, std::vector<Landmark>& result)
{
    bool finish = false;
    init(data, result);
    while (!finish)
    {
        calc_centroid(data, result);
        allocate_id(data, result, finish);
    }
    calc_centroid(data, result);
    for (const auto &r:result)
    {
        ROS_INFO("cluster ID: %d, centroid_x: %lf, centroid_y: %lf", r.clusterID_, r.pos_.x, r.pos_.y);
    }
}

void x_means::init(std::vector<Landmark>& data, std::vector<Landmark>& result)
{
    struct Landmark lm;
    for (auto &b:data)
    {
        b.clusterID_ = random(0, cluster_n);
    }
    std::vector<int> num;
    while (num.size() < cluster_n)
    {
        num.push_back(random(0, data.size()));
        std::sort(num.begin(), num.end());
        num.erase(std::unique(num.begin(), num.end()), num.end());
    }
    for (const auto &n:num)
    {
        lm = data[n];
        result.push_back(lm);
    }
}

void x_means::calc_bic()
{
}

void x_means::calc_centroid(std::vector<Landmark>& data, std::vector<Landmark>& result)
{
    struct Landmark lm;
    for (size_t i = 0; i < cluster_n; i++)
    {
        float sum_x = 0, sum_y = 0;
        int num = 0;
        for (const auto &b:data)
        {
            if (b.clusterID_ == i)
            {
                sum_x += b.pos_.x;
                sum_y += b.pos_.y;
                num++;
            }
        }
        // ROS_INFO("cluster ID: %ld, centroid_x: %lf, centroid_y: %lf, num: %d", i, sum_x/num, sum_y/num, num);
        // ROS_INFO("cluster ID: %ld, sum_x: %lf, sum_y: %lf, num: %d", i, sum_x, sum_y, num);
        lm.clusterID_ = i;
        lm.pos_.x = sum_x / num;
        lm.pos_.y = sum_y / num;
        result.push_back(lm);
    }
}

void x_means::allocate_id(std::vector<Landmark>& data, std::vector<Landmark>& result, bool& finish)
{
    finish = true;
    for (auto &b:data)
    {
        float dist, min_dist = 1000000;
        int old_ID = b.clusterID_;
        for (const auto &a:result)
        {
            dist = std::hypot(a.pos_.x - b.pos_.x, a.pos_.y - b.pos_.y);
            if (dist < min_dist)
            {
                min_dist = dist;
                b.clusterID_ = a.clusterID_;
            }
        }
        if (old_ID != b.clusterID_)
        {
            finish = false;
        }
    }
    result.clear();
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