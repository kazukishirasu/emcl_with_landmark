#include "emcl/gmm.h"

namespace emcl {

GMM::GMM()
{
}

GMM::~GMM()
{
}

void GMM::main(std::vector<std::vector<Landmark>>& input, std::vector<std::vector<Landmark>>& result)
{
    init_list(result);
    for (size_t i = 0; i < input.size(); i++)
    {
        if (!input[i].empty())
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
            clustering(input[i], result[i], distr_list[i]);
            ROS_INFO("====================");
        }
    }
}

void GMM::init_list(std::vector<std::vector<Landmark>>& result)
{
    for (size_t i = 0; i < result.size(); i++)
    {
        std::vector<Distribution> list;
        distr_list.push_back(list);
    }
}

void GMM::clustering(std::vector<Landmark>& input, std::vector<Landmark>& result, std::vector<Distribution>& distr)
{
    initialization(input, distr);
}

void GMM::initialization(std::vector<Landmark>& input, std::vector<Distribution>& distr)
{
    init_mean(input, distr);
    allocate_id(input, distr);
    init_cov(input, distr);
}

void GMM::init_mean(std::vector<Landmark>& input, std::vector<Distribution>& distr)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    struct Distribution gd;
    int index = random(0, input.size());
    gd.mean_.x = input[index].pos_.x;
    gd.mean_.y = input[index].pos_.y;
    distr.push_back(gd);
    for (size_t i = 0; i < cluster_n - 1; i++)
    {
        std::vector<float> dist_list;
        std::vector<float> prob_list;
        float sum = 0;
        for (const auto &in:input)
        {
            float min_dist = std::numeric_limits<float>::max();
            for (const auto &di:distr)
            {
                float dist = std::hypot(di.mean_.x - in.pos_.x, di.mean_.y - in.pos_.y);
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
        std::discrete_distribution<std::size_t> rand(prob_list.begin(), prob_list.end());
        index = rand(gen);
        gd.mean_.x = input[index].pos_.x;
        gd.mean_.y = input[index].pos_.y;
        distr.push_back(gd);
    }
}

void GMM::init_cov(std::vector<Landmark>& input, std::vector<Distribution>& distr)
{
    std::vector<std::array<float, 2>> mean_list;
    float mean_x = 0, mean_y = 0;
    for (auto &di:distr)
    {
        int count = 0, id = 0;
        for (const auto &in:input)
        {
            if (in.clusterID_ == id)
            {
                mean_x += in.pos_.x;
                mean_y += in.pos_.y;
                count++;
            }
        }
        std::array<float, 2> m{mean_x / count, mean_y / count};
        mean_list.push_back(m);
        id++;
    }
}

void GMM::allocate_id(std::vector<Landmark>& input, std::vector<Distribution>& distr)
{
    for (auto &in:input)
    {
        float min_dist = std::numeric_limits<float>::max();
        for (size_t id = 0; const auto &di:distr)
        {
            float dist = std::hypot(di.mean_.x - in.pos_.x, di.mean_.y - in.pos_.y);
            if (dist < min_dist)
            {
                min_dist = dist;
                in.clusterID_ = id;
            }
            id++;
        }
    }
}

int GMM::random(int min, int max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> rand(min, max - 1);
    return rand(gen);
}

}