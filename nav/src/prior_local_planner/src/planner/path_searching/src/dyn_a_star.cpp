#include "path_searching/dyn_a_star.h"
#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

AStar::~AStar() = default;
void AStar::initGridMap(GridMap::Ptr occ_map,const Eigen::Vector3i pool_size) {
    grid_map_=occ_map;POOL_SIZE_=pool_size;CENTER_IDX_=pool_size/2;
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

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    auto recover = [&](const Vector3d& original,double yaw,Vector3i& index,const char* name) {
        Vector3i raw;
        if (!Coord2Index(original,raw)) {last_failure_=std::string(name)+" outside bounded query";return false;}
        const bool raw_free=checkOccupancy(original,yaw)==0;
        // This is optimizer guidance, never a relocation of measured odometry.
        // Search a bounded XY neighborhood instead of marching outside FREE
        // along a single chord. Preserve reference height/floor.
        const int radius=std::max(1,int(std::ceil(.4/step_size_)));
        std::vector<Eigen::Vector3i> candidates;
        for(int dx=-radius;dx<=radius;++dx)for(int dy=-radius;dy<=radius;++dy) {
            if((dx*dx+dy*dy)*step_size_*step_size_>.4*.4+1e-9)continue;
            candidates.push_back(raw+Vector3i(dx,dy,0));
        }
        std::stable_sort(candidates.begin(),candidates.end(),[&](const Vector3i& a,const Vector3i& b){
            return (Index2Coord(a)-original).squaredNorm()<(Index2Coord(b)-original).squaredNorm();
        });
        for(const auto& candidate:candidates) {
            Eigen::Vector3d point=Index2Coord(candidate);Vector3i bounded;
            if (grid_map_->terrainEnabled()) {
                double z;
                if (!grid_map_->terrainBodyHeight(point,z)) continue;
                point.z()=z;
                if (!Coord2Index(point,bounded)) continue;
                point=Index2Coord(bounded);
            }
            if((point-original).norm()>.4+1e-9||!Coord2Index(point,bounded)||checkOccupancy(point,yaw)!=0)continue;
            if(raw_free) {
                bool clear=true;const int count=std::max(1,int(std::ceil((point-original).norm()/(.5*grid_map_->getResolution()))));
                for(int i=1;i<count;++i)if(checkOccupancy(original+(point-original)*(double(i)/count),yaw)!=0){clear=false;break;}
                if(!clear)continue;
            }
            index=bounded;return true;
        }
        last_failure_=std::string(name)+" has no collision-free XY anchor within 0.4 m (prior="+
            std::to_string(int(grid_map_->isPlanFree(original,yaw)))+", obstacle="+
            std::to_string(grid_map_->getObstacleOccupancy(original,yaw))+")";
        return false;
    };
    if (!recover(start_pt,start_yaw_,start_idx,"start"))return false;
    if (!recover(end_pt,end_yaw_,end_idx,"end"))return false;
    return true;
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt,double start_yaw,double end_yaw)
{
    ros::SteadyTime time_1 = ros::SteadyTime::now();
    gridPath_.clear();
    decltype(openSet_) empty_open;openSet_.swap(empty_open);nodes_.clear();last_failure_.clear();
    ++rounds_;

    if (!start_pt.allFinite() || !end_pt.allFinite() || !std::isfinite(step_size) || step_size<=0.) return ASTAR_RET::INIT_ERR;
    // A 100-cell fixed pool truncated detours. Keep native grid resolution,
    // but bound the query around both endpoints and allocate visited nodes only.
    for (int axis=0;axis<3;++axis) {
        const double margin=axis<2?2.5:.75;
        const int cells=int(std::ceil((std::abs(end_pt[axis]-start_pt[axis])+2.*margin)/step_size))+5;
        if (cells>512) return ASTAR_RET::INIT_ERR;
        POOL_SIZE_[axis]=std::max(100,cells);
    }
    CENTER_IDX_=POOL_SIZE_/2;
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;
    const auto delta=end_pt-start_pt;const double chord_yaw=std::atan2(delta.y(),delta.x());
    start_yaw_=std::isfinite(start_yaw)?start_yaw:chord_yaw;
    end_yaw_=std::isfinite(end_yaw)?end_yaw:chord_yaw;

    original_start_=start_pt;original_end_=end_pt;
    original_start_free_=checkOccupancy(start_pt,start_yaw_)==0;
    original_end_free_=checkOccupancy(end_pt,end_yaw_)==0;
    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_WARN_THROTTLE(1.,"A-star endpoint rejected: %s; start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f).",last_failure_.c_str(),start_pt.x(),start_pt.y(),start_pt.z(),end_pt.x(),end_pt.y(),end_pt.z());
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

        const double z = search_start(2) + ratio * (search_end(2) - search_start(2));
        return static_cast<int>(std::floor((z - center_(2)) * inv_step_size_ + 0.5)) + CENTER_IDX_(2);
    };

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = nodeAt(start_idx);
    GridNodePtr endPtr = nodeAt(end_idx);

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, NodeComparator> empty;
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
    openSet_.push({startPtr,startPtr->gScore,startPtr->fScore}); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        const auto entry=openSet_.top();openSet_.pop();current=entry.node;
        if (entry.g!=current->gScore || current->state==GridNode::CLOSEDSET) continue;

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
                if (grid_map_->terrainEnabled()) {
                    Eigen::Vector3d query=Index2Coord(neighborIdx);
                    query.z()=Index2Coord(current->index).z();
                    double z;
                    if (!grid_map_->terrainBodyHeight(query,z) ||
                        std::abs(z-query.z())>grid_map_->terrain_max_step_+.5*step_size_+1e-6) continue;
                    query.z()=z;
                    if (!Coord2Index(query,neighborIdx)) continue;
                }

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    continue;
                }

                neighborPtr = nodeAt(neighborIdx);
                neighborPtr->index = neighborIdx;

                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    continue; //in closed set.
                }

                neighborPtr->rounds = rounds_;

                // An already reached node with no improved cost cannot be
                // relaxed. Skip its expensive footprint/edge queries first;
                // keep exactly the same native search states and g-scores.
                const int dz = neighborIdx(2) - current->index(2);
                tentative_gScore = current->gScore + std::sqrt(dx*dx+dy*dy+dz*dz);
                if (flag_explored && tentative_gScore >= neighborPtr->gScore)
                    continue;

                const double neighbor_yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
                if (checkOccupancy(Index2Coord(neighborPtr->index), neighbor_yaw))
                {
                    continue;
                }

                // A prior is a volume: checking only neighbor centers permits
                // a diagonal edge to jump over a forbidden voxel. Validate the
                // same native collision query at half-voxel spacing.
                const Eigen::Vector3d a=Index2Coord(current->index),b=Index2Coord(neighborIdx);
                const int samples=std::max(1,int(std::ceil((b-a).norm()/(.5*grid_map_->getResolution()))));
                bool edge_free=true;
                for (int i=1;i<samples;++i) if (checkOccupancy(a+(b-a)*(double(i)/samples),neighbor_yaw)!=0) { edge_free=false;break; }
                if (!edge_free) continue;

                if (!flag_explored)
                {
                    //discover a new node
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push({neighborPtr,neighborPtr->gScore,neighborPtr->fScore}); //put neighbor in open set and record it.
                }
                else if (tentative_gScore < neighborPtr->gScore)
                { //in open set and need update
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push({neighborPtr,neighborPtr->gScore,neighborPtr->fScore});
                }
            }
        ros::SteadyTime time_2 = ros::SteadyTime::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            last_failure_="0.2 second search budget exceeded";
            ROS_WARN_THROTTLE(1.,"A-star search budget exceeded.");
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    ros::SteadyTime time_2 = ros::SteadyTime::now();

    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    last_failure_="OPEN exhausted under the current prior/live occupancy";
    return ASTAR_RET::SEARCH_ERR;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    // Quantized control-point boundaries may share one grid node. Retain
    // their checked continuous connectors instead of returning one point.
    if(!path.empty()){
        if(original_start_free_&&(path.front()-original_start_).norm()>1e-8)path.insert(path.begin(),original_start_);
        if(original_end_free_&&(path.back()-original_end_).norm()>1e-8)path.push_back(original_end_);
    }
    return path;
}
