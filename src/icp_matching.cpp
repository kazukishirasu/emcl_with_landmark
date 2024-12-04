#include "emcl/icp_matching.h"

namespace emcl{

ICP_Matching::ICP_Matching()
{
}

ICP_Matching::~ICP_Matching()
{
}

bool ICP_Matching::matching(const std::vector<Tree>& tree_list, Data& data, size_t size)
{
    Pair pair;
    pair.fix.resize(size, 2);
    pair.move.resize(size, 2);
    for (size_t i = 0; i < 10; i++){
        int index = 0;
        double dist = 0;
        for (const auto& landmark : data.landmarks){
            for (const auto& tree : tree_list){
                if (landmark.name == tree.name){
                    for (const auto& point : landmark.points){
                        KD_Tree::Point best_point;
                        double best_dist = std::numeric_limits<double>::max();
                        kdt.search(tree.node, point, best_point, best_dist, 0);
                        pair.move(index, 0) = point.x;
                        pair.move(index, 1) = point.y;
                        pair.fix(index, 0) = best_point.x;
                        pair.fix(index, 1) = best_point.y;
                        dist += squared_dist(point, best_point);
                        index++;
                    }
                }
            }
        }
        if (dist / size < 1.0)
            return true;
        Eigen::RowVector2d fix_mean = pair.fix.colwise().mean();
        Eigen::RowVector2d move_mean = pair.move.colwise().mean();
        pair.fix = pair.fix.rowwise() - fix_mean;
        pair.move = pair.move.rowwise() - move_mean;
        Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < pair.move.rows(); i++){
            W +=  pair.move.row(i).transpose() * pair.fix.row(i);
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        // 回転行列 R
        Eigen::Matrix2d R = svd.matrixU() * svd.matrixV();
        // 並進ベクトル t
        Eigen::Vector2d t = fix_mean.transpose() - R * move_mean.transpose();
        Eigen::Vector2d current_pose, update_pose;
        current_pose(0) = data.robot_pose.x;
        current_pose(1) = data.robot_pose.y;
        update_pose = R * current_pose + t;
        data.robot_pose.x = update_pose(0);
        data.robot_pose.y = update_pose(1);
        if (R.determinant() < 0)
            R.col(1) *= -1;
        data.robot_pose.t += std::acos(R(0, 0));
        if (data.robot_pose.t > M_PI * 2)
            data.robot_pose.t -= (M_PI * 2);
        for (auto& lm : data.landmarks){
            for (auto& point : lm.points){
                current_pose(0) = point.x;
                current_pose(1) = point.y;
                update_pose = R * current_pose + t;
                point.x = update_pose(0);
                point.y = update_pose(1);
            }
        }
    }
    return false;
}

double ICP_Matching::squared_dist(const Eigen::Vector2d& current_pose, const Eigen::Vector2d& update_pose)
{
    return (current_pose - update_pose).squaredNorm();
}

double ICP_Matching::squared_dist(const KD_Tree::Point& point, const KD_Tree::Point& best_point)
{
    return (point.x - best_point.x) * (point.x - best_point.x) + (point.y - best_point.y) * (point.y - best_point.y);
}

}