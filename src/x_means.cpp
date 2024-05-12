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
        if (!data[i].empty())
        {
            if (i == 0)
            {
                // cluster_n = 10;
                cluster_n = 1;
            }else if (i == 1)
            {
                // cluster_n = 6;
                cluster_n = 5;
            }else
            {
                cluster_n = 2;
            }
            k_means(data[i], result[i]);
            ROS_INFO("====================");
        }
    }
}

void x_means::k_means(std::vector<Landmark>& data, std::vector<Landmark>& result)
{
    bool finish = false;
    init(data, result);
    while (!finish)
    {
        allocate_id(data, result, finish);
        calc_centroid(data, result);
    }
    for (const auto &r:result)
    {
        ROS_INFO("cluster ID: %d, centroid_x: %lf, centroid_y: %lf", r.clusterID_, r.pos_.x, r.pos_.y);
    }
}

void x_means::init(std::vector<Landmark>& data, std::vector<Landmark>& result)
{
    for (auto &d:data)
    {
        d.clusterID_ = random(0, cluster_n);
    }
    result.clear();
    select_centroid(data, result);
}

void x_means::select_centroid(std::vector<Landmark>& data, std::vector<Landmark>& result)
{
    //k-meansの初期化(重複しないようにサンプリング)
    // std::vector<int> num;
    // while (num.size() < cluster_n)
    // {
    //     num.push_back(random(0, data.size()));
    //     std::sort(num.begin(), num.end());
    //     num.erase(std::unique(num.begin(), num.end()), num.end());
    // }
    // for (const auto &n:num)
    // {
    //     result.push_back(data[n]);
    // }

    //k-means++の初期化(セントロイドが離れるようにサンプリング)
    static std::random_device rd;
    static std::mt19937 gen(rd());
    result.push_back(data[random(0, data.size())]);
    for (size_t i = 0; i < cluster_n - 1; i++)
    {
        std::vector<float> dist_list;
        std::vector<float> prob_list;
        float sum = 0;
        for (const auto &d:data)
        {
            float min_dist = std::numeric_limits<float>::max();
            for (const auto &r:result)
            {
                float dist = std::hypot(r.pos_.x - d.pos_.x, r.pos_.y - d.pos_.y);
                if (dist < min_dist)
                {
                    min_dist = dist;
                }
            }
            sum += min_dist;
            dist_list.push_back(min_dist);
        }
        float sum_pow = std::pow(sum, 2.0);
        float dist_pow = 0;
        for (const auto &d:dist_list)
        {
            dist_pow = std::pow(d, 2.0);
            prob_list.push_back(dist_pow / sum_pow);
        }
        std::discrete_distribution<std::size_t> dist(prob_list.begin(), prob_list.end());
        result.push_back(data[dist(gen)]);
    }
}

void x_means::calc_bic()
{
}

void x_means::allocate_id(std::vector<Landmark>& data, std::vector<Landmark>& result, bool& finish)
{
    finish = true;
    for (auto &d:data)
    {
        float min_dist = std::numeric_limits<float>::max();
        int old_ID = d.clusterID_;
        for (const auto &r:result)
        {
            float dist = std::hypot(r.pos_.x - d.pos_.x, r.pos_.y - d.pos_.y);
            if (dist < min_dist)
            {
                min_dist = dist;
                d.clusterID_ = r.clusterID_;
            }
        }
        if (old_ID != d.clusterID_)
        {
            finish = false;
        }
    }
    result.clear();
}

void x_means::calc_centroid(std::vector<Landmark>& data, std::vector<Landmark>& result)
{
    struct Landmark lm;
    for (size_t i = 0; i < cluster_n; i++)
    {
        float sum_x = 0, sum_y = 0;
        int num = 0;
        for (const auto &d:data)
        {
            if (d.clusterID_ == i)
            {
                sum_x += d.pos_.x;
                sum_y += d.pos_.y;
                num++;
            }
        }
        lm.clusterID_ = i;
        lm.pos_.x = sum_x / num;
        lm.pos_.y = sum_y / num;
        result.push_back(lm);
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