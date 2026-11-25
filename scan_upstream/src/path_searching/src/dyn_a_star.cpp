#include "path_searching/dyn_a_star.h"
#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::findEndpoint(const Vector3d &point, const Vector3d &outward, double yaw,
                         Vector3i &index) const
{
    auto clear = [&](const Vector3d &candidate) {
        return Coord2Index(candidate, index) && checkOccupancy(Index2Coord(index), yaw) == 0;
    };
    if (clear(point))
        return true;

    // These are rebound guide endpoints, not physical body poses. Search a
    // short distance near each end instead of extending into unknown space.
    const Vector3d left(-std::sin(yaw), std::cos(yaw), 0.0);
    constexpr double max_adjustment = 0.45;
    for (double distance = step_size_; distance <= max_adjustment + 1e-6; distance += step_size_)
    {
        if (clear(point + distance * outward) ||
            clear(point + distance * left) || clear(point - distance * left))
            return true;
    }
    return false;
}

ASTAR_RET AStar::AstarSearch(double step_size, Vector3d start_pt, Vector3d end_pt,
                          double start_yaw, double end_yaw)
{
    const ros::SteadyTime deadline = ros::SteadyTime::now() + ros::WallDuration(0.20);
    ++rounds_;
    gridPath_.clear();

    const Vector3d chord = end_pt - start_pt;
    if (!std::isfinite(step_size) || step_size <= 0.0 || !chord.allFinite() ||
        chord.head<2>().squaredNorm() < 1e-12)
        return ASTAR_RET::INIT_ERR;
    const double chord_yaw = std::atan2(chord.y(), chord.x());
    const Vector3d direction = chord.normalized();
    step_size_ = step_size;
    inv_step_size_ = 1.0 / step_size;
    center_ = (start_pt + end_pt) / 2.0;

    Vector3i start_idx, end_idx;
    if (!findEndpoint(start_pt, -direction, std::isfinite(start_yaw) ? start_yaw : chord_yaw, start_idx))
    {
        ROS_WARN_THROTTLE(1., "SCAN A*: no clear start guide within 0.45 m (%.3f, %.3f, %.3f)",
                         start_pt.x(), start_pt.y(), start_pt.z());
        return ASTAR_RET::INIT_ERR;
    }
    if (!findEndpoint(end_pt, direction, std::isfinite(end_yaw) ? end_yaw : chord_yaw, end_idx))
    {
        ROS_WARN_THROTTLE(1., "SCAN A*: no clear end guide within 0.45 m (%.3f, %.3f, %.3f)",
                         end_pt.x(), end_pt.y(), end_pt.z());
        return ASTAR_RET::INIT_ERR;
    }

    const Eigen::Vector3d search_start = Index2Coord(start_idx);
    const Eigen::Vector3d search_end = Index2Coord(end_idx);
    const Eigen::Vector2d search_start_xy = search_start.head<2>();
    const Eigen::Vector2d search_xy_delta = search_end.head<2>() - search_start_xy;
    const double search_xy_len2 = search_xy_delta.squaredNorm();

    auto interpolateZIndexOnSearchPlane = [&](const int x_idx, const int y_idx) -> int {
        if (search_xy_len2 < 1e-8)
            return start_idx(2);

        Eigen::Vector3i sample_idx(x_idx, y_idx, start_idx(2));
        const Eigen::Vector2d sample_xy = Index2Coord(sample_idx).head<2>();
        double ratio = (sample_xy - search_start_xy).dot(search_xy_delta) / search_xy_len2;
        ratio = std::max(0.0, std::min(1.0, ratio));

        return static_cast<int>(std::lround(start_idx.z() + ratio * (end_idx.z() - start_idx.z())));
    };

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    decltype(openSet_) empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push({startPtr->fScore, startPtr});

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        if (ros::SteadyTime::now() >= deadline)
        {
            ROS_WARN_THROTTLE(1., "SCAN A*: 200 ms search budget exhausted");
            return ASTAR_RET::SEARCH_ERR;
        }
        const QueueEntry entry = openSet_.top();
        openSet_.pop();
        current = entry.node;
        if (current->state == GridNode::CLOSEDSET || entry.cost > current->fScore)
            continue;

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            return ASTAR_RET::SUCCESS;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                Vector3i neighborIdx;
                neighborIdx(0) = (current->index)(0) + dx;
                neighborIdx(1) = (current->index)(1) + dy;
                neighborIdx(2) = interpolateZIndexOnSearchPlane(neighborIdx(0), neighborIdx(1));

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    continue;
                }

                neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                neighborPtr->index = neighborIdx;

                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    continue; //in closed set.
                }

                const double neighbor_yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
                if (checkOccupancy(Index2Coord(neighborPtr->index), neighbor_yaw))
                {
                    continue;
                }

                const int dz = neighborIdx(2) - current->index(2);
                double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                tentative_gScore = current->gScore + static_cost;

                if (!flag_explored || tentative_gScore < neighborPtr->gScore)
                {
                    neighborPtr->rounds = rounds_;
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push({neighborPtr->fScore, neighborPtr});
                }
            }
    }

    ROS_WARN_THROTTLE(1., "SCAN A*: no connected guide path, expanded %d nodes", num_iter);
    return ASTAR_RET::SEARCH_ERR;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
