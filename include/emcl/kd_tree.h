#ifndef KDTREE_H__
#define KDTREE_H__

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>

namespace emcl{

class KD_Tree
{
public:
    struct Point {
        double x, y, t;
        Point(double x = 0, double y = 0, double t = 0) : x(x), y(y), t(t) {}
    };
    struct KdNode {
        Point point;
        std::shared_ptr<KdNode> left, right;
        KdNode(const Point &point) : point(point), left(nullptr), right(nullptr) {}
    };

    KD_Tree();
    ~KD_Tree();
    std::shared_ptr<KdNode> build_kd_tree(std::vector<Point>&, int);
    void search(const std::shared_ptr<KdNode>&, const Point&, Point&, double&, int);
private:
};

}

#endif