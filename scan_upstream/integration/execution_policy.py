"""Transport/goal ownership only; no map, collision, crawl or recovery policy."""
import math
import struct

class ExecutionPolicy:
    def __init__(self, command_timeout=.3, odom_timeout=.5, cloud_timeout=1.):
        self.timeouts = (command_timeout, odom_timeout, cloud_timeout)
        self.active = False
        self.generation = 0
        self.route_received = float('inf')
        self.curve_stamp = float('-inf')
        self.command_at = self.odom_at = self.cloud_at = float('-inf')
        self.odom_stamp = float('-inf')
        self.command = (0., 0., 0.)
        self.tracking = False

    def route(self, now):
        self.generation += 1
        self.active = True
        self.route_received = now
        self.curve_stamp = float('-inf')
        self.command_at = float('-inf')

    def cancel(self):
        self.active = False
        self.generation += 1
        self.command_at = float('-inf')

    def curve(self, stamp):
        if self.active and math.isfinite(stamp) and stamp >= self.route_received:
            self.curve_stamp = stamp
            self.command_at = float('-inf')
            return True
        return False

    def submit(self, command, now):
        if len(command) == 3 and all(math.isfinite(v) for v in command):
            self.command = tuple(command)
            self.command_at = now
        else:
            self.command_at = float('-inf')

    def reason(self, now, ros_now):
        if not self.active:return 'idle_or_canceled'
        if not self.tracking:return 'localization_unavailable'
        if self.curve_stamp < self.route_received:return 'waiting_for_new_upstream_curve'
        for name,stamp,limit in zip(('command','odometry','cloud'),
                (self.command_at,self.odom_at,self.cloud_at),self.timeouts):
            if not 0. <= now-stamp <= limit:return name+'_unavailable'
        if not -.1 <= ros_now-self.odom_stamp <= self.timeouts[1]:return 'odometry_source_stale'
        return 'forwarding_upstream'

    def output(self, now, ros_now):
        return self.command if self.reason(now,ros_now)=='forwarding_upstream' else (0.,0.,0.)


def limit_near_cabin(command, ground_xyz, cabins, cabin_speed, lobbies=()):
    """Limit planar velocity near a cabin on the robot's current floor."""
    x, y, z = ground_xyz
    distance = min(
        (math.hypot(x - cx, y - cy) for cx, cy, cz in cabins if abs(z - cz) <= .75),
        default=math.inf,
    )
    lobby_distance = min(
        (math.hypot(x - lx, y - ly) for lx, ly, lz in lobbies if abs(z - lz) <= .75),
        default=math.inf,
    )
    # Keep the last part of the approach slow even when the lobby is set back.
    buffer = max(0.0, min(distance - 1.60, lobby_distance - .55))
    planar_limit = cabin_speed + .5 * buffer
    lateral_limit = .10 + .5 * buffer
    yaw_limit = .30 + 1.4 * buffer
    vx, vy, wz = command
    vy = math.copysign(min(abs(vy), lateral_limit), vy)
    wz = math.copysign(min(abs(wz), yaw_limit), wz)
    speed = math.hypot(vx, vy)
    scale = min(1.0, planar_limit / speed) if speed else 1.0
    return vx * scale, vy * scale, wz


def limit_elevator_exit(command, ground_xyz, lobby_planes, cabin_speed, corridor_speed, yaw_limit):
    """Apply the cabin limits only until the body clears the door plane."""
    x, y, z = ground_xyz
    planes = [(abs(z - floor[2]), plane) for floor, plane in lobby_planes.items()
              if abs(z - floor[2]) <= .75]
    if not planes:
        return None
    _, (cx, cy, nx, ny) = min(planes, key=lambda item: item[0])
    clearance = (x - cx) * nx + (y - cy) * ny
    clear_distance = .65
    if clearance >= clear_distance:
        return command
    progress = max(0., clearance) / clear_distance
    speed_limit = cabin_speed + (corridor_speed - cabin_speed) * progress
    vx, vy, wz = command
    vy = math.copysign(min(abs(vy), .10), vy)
    wz = math.copysign(min(abs(wz), yaw_limit), wz)
    speed = math.hypot(vx, vy)
    scale = min(1., speed_limit / speed) if speed else 1.
    return vx * scale, vy * scale, wz


def cabin_turn_clear(points, body_xyz, body_height, swept_radius):
    """Conservatively check the full in-place sweep of SCAN's two cylinders."""
    x, y, z = body_xyz
    lower = z - body_height + .10
    upper = z + .30
    radius_squared = swept_radius * swept_radius
    seen = False
    for px, py, pz in points:
        if not all(map(math.isfinite, (px, py, pz))):
            continue
        seen = True
        if lower <= pz <= upper and (px - x) ** 2 + (py - y) ** 2 <= radius_squared:
            return False
    return seen


def world_velocity(quaternion, velocity):
    """Original SCAN expects world twist; localization exports body twist."""
    x,y,z,w = quaternion
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('invalid orientation')
    x,y,z,w = (v/norm for v in (x,y,z,w))
    vx,vy,vz = velocity
    return ((1-2*(y*y+z*z))*vx+2*(x*y-z*w)*vy+2*(x*z+y*w)*vz,
            2*(x*y+z*w)*vx+(1-2*(x*x+z*z))*vy+2*(y*z-x*w)*vz,
            2*(x*z-y*w)*vx+2*(y*z+x*w)*vy+(1-2*(x*x+y*y))*vz)


def hit_point_bytes(data, height, width, row_step, point_step, flag_offset, bigendian=False):
    if (point_step <= 0 or flag_offset < 0 or flag_offset+4 > point_step or
            height < 0 or width < 0 or row_step < width*point_step or len(data) < height*row_step):
        raise ValueError('malformed point cloud layout')
    result=bytearray()
    for row in range(height):
        for col in range(width):
            offset=row*row_step+col*point_step
            flag=struct.unpack_from(('>' if bigendian else '<')+'f',data,offset+flag_offset)[0]
            if math.isfinite(flag) and flag <= .5:
                result.extend(data[offset:offset+point_step])
    return bytes(result)
