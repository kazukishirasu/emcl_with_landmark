#include "emcl/kd_tree.h"

namespace emcl{   
    KD_Tree::KD_Tree()
    {
    }

    KD_Tree::~KD_Tree()
    {
    }

    std::shared_ptr<KD_Tree::KdNode> KD_Tree::build_kd_tree(std::vector<Point> &points, int depth = 0)
    {
        if (points.empty())
            return nullptr;

        auto compare = [depth](const Point& a, const Point& b){
            return (depth % 2 == 0) ? (a.x < b.x) : (a.y < b.y);
        };

        std::sort(points.begin(), points.end(), compare);
        int median = points.size() / 2;

        auto node = std::make_shared<KdNode>(points[median]);
        std::vector<Point> left_points(points.begin(), points.begin() + median);
        std::vector<Point> right_points(points.begin() + median + 1, points.end());
        node->left = build_kd_tree(left_points, depth + 1);
        node->right = build_kd_tree(right_points, depth + 1);
        return node;
    }

    void KD_Tree::search(const std::shared_ptr<KdNode>& node, const Point& target, Point& best_point, double& best_dist, int depth = 0)
    {
        if (node == nullptr)
            return;
        
        auto squared_dist = [&](const Point a, const Point b){
            return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
        };

        double dist = squared_dist(node->point, target);
        if (dist < best_dist){
            best_dist = dist;
            best_point = node->point;
        }

        bool useX = (depth % 2 == 0);
        double diff = useX ? (target.x - node->point.x) : (target.y - node->point.y);
        std::shared_ptr<KdNode> near = (diff < 0) ? node->left : node->right;
        std::shared_ptr<KdNode> far = (diff < 0) ? node->right : node->left;

        search(near, target, best_point, best_dist, depth + 1);
        if (diff * diff < best_dist)
            search(far, target, best_point, best_dist, depth + 1);
        return;
    }
}