"""Validate correlated requests before touching the existing planner."""
import math
import re


def validate_request(value, context):
    if not isinstance(value,dict):raise ValueError('Expected request object')
    if value.get('owner')!='web':raise ValueError('Wrong task owner')
    for key in ('instance_id','asset_id','map_request_id'):
        if value.get(key)!=context.get(key) or not context.get(key):raise ValueError('Stale '+key)
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,100}',value.get('request_id','')):
        raise ValueError('Invalid request_id')
    if value.get('coordinate_frame')!=context['frame_id']:raise ValueError('Wrong frame')
    if 'map_label' in value and (not isinstance(value['map_label'],str) or not re.fullmatch(r'floor[1-9][0-9]*',value['map_label'])):
        raise ValueError('Invalid map_label')
    pose=value.get('pose',{})
    for key in ('x','y','z','qx','qy','qz','qw'):
        x=pose.get(key)
        if type(x) not in (int,float) or not math.isfinite(x):raise ValueError('Invalid pose')
    if abs(sum(pose[k]**2 for k in ('qx','qy','qz','qw'))-1)>.001:
        raise ValueError('Nonunit quaternion')
    return value
