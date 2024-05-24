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
    for (size_t i = 0; i < input.size(); i++)
    {
        if (!input[i].empty())
        {
            if (i == 0)
            {
                cluster_n = 1;
            }else if (i == 1)
            {
                cluster_n = 5;
            }else
            {
                cluster_n = 2;
            }
            clustering(input[i], result[i]);
            ROS_INFO("====================");
        }
    }
}

void GMM::clustering(std::vector<Landmark>& input, std::vector<Landmark>& result)
{
    Data_point dp;
    Distribution distr;
    initialization(input, result, dp, distr);
}

void GMM::initialization(std::vector<Landmark>& input, std::vector<Landmark>& result, Data_point& dp, Distribution& distr)
{
    dp.point_.resize(input.size(), 2);
    dp.responsibility_.resize(input.size(), cluster_n);
    for (size_t i = 0; const auto &in:input)
    {
        dp.point_(i, 0) = in.pos_.x;
        dp.point_(i, 1) = in.pos_.y;
        for (size_t j = 0; j < cluster_n; j++)
        {
            dp.responsibility_(i, j) = 1.0 / cluster_n;
        }        
        i++;
    }
    distr.mean_.resize(result.size(), 2);
    // init_mean(input, distr);
    // allocate_id(input, distr);
    // init_cov(input, distr);
}

void GMM::init_mean(Data_point& dp, Distribution& distr)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    int index = random(0, dp.point_.rows());
    distr.mean_(0, 0) = dp.point_(index, 0);
    distr.mean_(0, 1) = dp.point_(index, 1);
    for (size_t i = 0; i < cluster_n; i++)
    {
        dp.point_.rowwise().sum();
        // for (auto &pos:dp.point_.rowwise().sum())
        // {
        //     /* code */
        // }
    }
}

// void GMM::init_cov(std::vector<Landmark>& input, std::vector<Distribution>& distr)
// {
//     for (size_t i = 0; i < distr.size(); i++)
//     {
//         Eigen::Matrix2f mat(0, 0);
//         for (const auto &&in:input)
//         {
//             if (in.clusterID_ == i)
//             {
//                 Eigen::RowVector2f newRow(2);
//                 newRow << in.pos_.x, in.pos_.y;
//                 Eigen::Matrix2f newMat(mat.rows() + 1, mat.cols());
//                 newMat << mat, newRow;
//             }
//         }
//     }
// }

// void GMM::allocate_id(std::vector<Landmark>& input, std::vector<Distribution>& distr)
// {
//     for (auto &in:input)
//     {
//         float min_dist = std::numeric_limits<float>::max();
//         for (size_t id = 0; const auto &di:distr)
//         {
//             float dist = std::hypot(di.mean_.x - in.pos_.x, di.mean_.y - in.pos_.y);
//             if (dist < min_dist)
//             {
//                 min_dist = dist;
//                 in.clusterID_ = id;
//             }
//             id++;
//         }
//     }
// }

int GMM::random(int min, int max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> rand(min, max - 1);
    return rand(gen);
}

}