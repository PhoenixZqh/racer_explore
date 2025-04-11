#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

namespace fast_planner
{
// Static data
double ViewNode::vm_;
double ViewNode::am_;
double ViewNode::yd_;
double ViewNode::ydd_;
double ViewNode::w_dir_;
shared_ptr<Astar> ViewNode::astar_;
shared_ptr<RayCaster> ViewNode::caster_;
shared_ptr<SDFMap> ViewNode::map_;

// Graph node for viewpoints planning
ViewNode::ViewNode(const Vector3d &p, const double &y)
{
    pos_ = p;
    yaw_ = y;
    parent_ = nullptr;
    vel_.setZero(); // vel is zero by default, should be set explicitly
}

double ViewNode::costTo(const ViewNode::Ptr &node)
{
    vector<Vector3d> path;
    double c = ViewNode::computeCost(pos_, node->pos_, yaw_, node->yaw_, vel_, yaw_dot_, path);
    // std::cout << "cost from " << id_ << " to " << node->id_ << " is: " << c << std::endl;
    return c;
}

double ViewNode::searchPath(const Vector3d &p1, const Vector3d &p2, vector<Vector3d> &path)
{
    // Try connect two points with straight line
    bool safe = true;
    Vector3i idx;
    caster_->input(p1, p2);
    while (caster_->nextId(idx))
    {
        if (map_->getInflateOccupancy(idx) == 1 || !map_->isInBox(idx))
        {
            // map_->getOccupancy(idx) == SDFMap::UNKNOWN
            safe = false;
            break;
        }
    }
    if (safe)
    {
        path = {p1, p2};
        return (p1 - p2).norm();
    }
    // Search a path using decreasing resolution
    vector<double> res = {0.4};
    for (int k = 0; k < res.size(); ++k)
    {
        astar_->reset();
        astar_->setResolution(res[k]);
        if (astar_->search(p1, p2) == Astar::REACH_END)
        {
            path = astar_->getPath();
            return astar_->pathLength(path);
        }
    }
    // Use Astar early termination cost as an estimate
    path = {p1, p2};
    return 100;
}

/**
 * @brief 计算当前状态到目标状态的综合代价
 * @param：
 *      输入：
 *      p1： 当前位置
 *      p2： 目标位置
 *      y1： 当前航向角（弧度）
 *      y2： 目标航向角（弧度）
 *      v1： 当前速度
 *      yd1： 当前航向角速度
 * 
 *      输出：综合代价、 p1 ～ p2的路径
 */
double ViewNode::computeCost(const Vector3d &p1, const Vector3d &p2, const double &y1, const double &y2, const Vector3d &v1, const double &yd1, vector<Vector3d> &path)
{
    // 1. 计算位置变化代价， /最大速度，用于规一化距离成本为时间
    double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_;

    // 2. 考虑速度方向变化的代价
    if (v1.norm() > 1e-3)
    {
        Vector3d dir = (p2 - p1).normalized(); //目标方向
        Vector3d vdir = v1.normalized();       //速度方向
        double diff = acos(vdir.dot(dir));     //计算速度方向和目标方向的夹角
        pos_cost += w_dir_ * diff;             // 乘一个权重 加到位置代价
        // double vc = v1.dot(dir);
        // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
        // if (vc < 0)
        //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
    }

    // 3. 航向变化的代价
    double diff = fabs(y2 - y1);       // 计算航向角度差的绝对值
    diff = min(diff, 2 * M_PI - diff); // 考虑2派周期， 得到的是最短旋转路径
    double yaw_cost = diff / yd_;      // 用差值 除 最大航向角速度
    return max(pos_cost, yaw_cost);

    // // Consider yaw rate change
    // if (fabs(yd1) > 1e-3)
    // {
    //   double diff1 = y2 - y1;
    //   while (diff1 < -M_PI)
    //     diff1 += 2 * M_PI;
    //   while (diff1 > M_PI)
    //     diff1 -= 2 * M_PI;
    //   double diff2 = diff1 > 0 ? diff1 - 2 * M_PI : 2 * M_PI + diff1;
    // }
    // else
    // {
    // }
}

//FIXME： 针对枪声定位的输入、这里以这个角度进行代价计算
// double ViewNode::computeCost(const Vector3d &p1, const Vector3d &p2, const double &y1, const double &y2, const Vector3d &v1, const double &yd1, vector<Vector3d> &path, double prior_yaw = -1000)
// {
//     // --- 计算位置代价 ---
//     // 路径长度除以最大速度，转换为时间（秒）
//     double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_;

//     // --- 计算速度方向代价 ---
//     // 如果当前速度非零，惩罚速度方向与目标方向的偏差
//     if (v1.norm() > 1e-3)
//     {
//         Vector3d dir = (p2 - p1).normalized();
//         Vector3d vdir = v1.normalized();
//         double diff = acos(vdir.dot(dir));
//         pos_cost += w_dir_ * diff;
//     }

//     // --- 计算当前航向代价 ---
//     // 从当前航向（y1）到目标航向（y2）的旋转时间
//     double current_diff = fabs(y2 - y1);
//     current_diff = min(current_diff, 2 * M_PI - current_diff);
//     double current_yaw_cost = w_current_ * (current_diff / yd_); // 时间（秒）

//     // --- 计算先验航向代价 ---
//     // 目标航向（y2）与先验航向（prior_yaw）的偏差
//     double prior_yaw_cost = 0.0;
//     if (prior_yaw > -999) // 检查 prior_yaw 是否有效
//     {
//         double prior_diff = fabs(y2 - prior_yaw);
//         prior_diff = min(prior_diff, 2 * M_PI - prior_diff);
//         prior_yaw_cost = w_prior_yaw_ * prior_diff; // 直接惩罚弧度偏差
//     }

//     // --- 综合代价 ---
//     // 位置、当前航向、先验航向加权组合
//     double total_cost = w_pos_ * pos_cost + current_yaw_cost + prior_yaw_cost;

//     return total_cost;
// }

} // namespace fast_planner