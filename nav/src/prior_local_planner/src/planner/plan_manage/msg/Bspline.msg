int32 order
int64 traj_id
time start_time
# A stationary obstacle hold keeps the current reference goal for replanning.
# Other stationary/emergency trajectories still invalidate the goal.
bool obstacle_hold

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt

