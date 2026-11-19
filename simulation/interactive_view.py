"""Loopback-only controls and real MuJoCo snapshots for interactive tests."""
import io
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import queue
import threading


def validate_command(value, boxes, case):
    if not isinstance(value, dict):
        raise ValueError('请提交一个操作')
    action = value.get('action')
    if action in {'stop', 'reset', 'close'}:
        return {'action': action}
    if action != 'goal':
        raise ValueError('未知操作')
    try:
        x, y, yaw = (float(value[name]) for name in ('x', 'y', 'yaw'))
    except (KeyError, TypeError, ValueError):
        raise ValueError('目标需要有效的位置和朝向') from None
    if not all(math.isfinite(n) for n in (x, y, yaw)):
        raise ValueError('目标坐标必须是有限数值')
    if not (-5.4 <= x <= 5.4 and -2.5 <= y <= 2.5):
        raise ValueError('请在地图中的可通行范围选点')
    if case == 'stairs' and x >= 0 and abs(y) > .65:
        raise ValueError('请在楼梯或平台内选点')
    for name, center, half in boxes:
        if name.startswith(('stair_', 'landing')):
            continue
        if abs(x-center[0]) <= half[0]+.03 and abs(y-center[1]) <= half[1]+.03:
            raise ValueError('目标在障碍物内，请重新选空地')
    return {'action': 'goal', 'x': x, 'y': y, 'yaw': math.atan2(math.sin(yaw), math.cos(yaw))}


class InteractiveView:
    def __init__(self, port, boxes, case, goal_validator=None):
        self.commands = queue.Queue()
        self.lock = threading.Lock()
        self.state = {'ready': False, 'status': '正在初始化仿真', 'case': case}
        self.frame = b''
        self.closed = False
        page = Path(__file__).with_name('interactive.html')
        view = self

        class Handler(BaseHTTPRequestHandler):
            def send(self, status, content, mime):
                self.send_response(status)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(content)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    self.wfile.write(content)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_GET(self):
                path = self.path.split('?', 1)[0]
                if path == '/':
                    self.send(200, page.read_bytes(), 'text/html; charset=utf-8')
                elif path == '/api/state':
                    with view.lock:
                        state = json.dumps(view.state, ensure_ascii=False).encode()
                    self.send(200, state, 'application/json; charset=utf-8')
                elif path == '/api/frame':
                    with view.lock:
                        frame = view.frame
                    self.send(200 if frame else 503, frame, 'image/jpeg')
                else:
                    self.send(404, b'Not found', 'text/plain')

            def do_POST(self):
                if self.path != '/api/command':
                    self.send(404, b'Not found', 'text/plain')
                    return
                origin = self.headers.get('Origin')
                if origin and origin != f'http://127.0.0.1:{view.port}':
                    self.send(403, b'Local interface only', 'text/plain')
                    return
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 4096:
                        raise ValueError('操作内容过长或为空')
                    command = validate_command(json.loads(self.rfile.read(size)), boxes, case)
                    with view.lock:
                        ready = view.state.get('ready', False)
                        fallen = view.state.get('fallen', False)
                    if command['action'] == 'goal' and (not ready or fallen):
                        raise ValueError('请先等待站稳；失稳后请点击重置')
                    if command['action']=='goal' and goal_validator is not None:
                        goal_validator(command)
                    view.commands.put(command)
                    self.send(200, b'{"accepted":true}', 'application/json')
                except (ValueError, TypeError) as exc:
                    self.send(400, json.dumps({'error': str(exc)}, ensure_ascii=False).encode(), 'application/json; charset=utf-8')

            def log_message(self, *_args):
                pass

        self.server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def update(self, state, frame):
        with self.lock:
            self.state, self.frame = state, frame

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.closed = True


class LiveRenderer:
    """A separate render model keeps visual cutaways out of LiDAR/physics."""
    def __init__(self, xml):
        import mujoco
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        self.model.vis.global_.offwidth, self.model.vis.global_.offheight = 960, 600
        for i in range(self.model.ngeom):
            if self.model.geom_group[i] == 3 and self.model.geom_contype[i] == self.model.geom_conaffinity[i] == 0:
                self.model.geom_group[i] = 1
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if name in {'wall_n', 'wall_s', 'wall_e', 'wall_w', 'wall_stair_n', 'wall_stair_s'}:
                self.model.geom_rgba[i, 3] = 0.
        self.model.vis.headlight.ambient[:] = .4
        self.model.vis.headlight.diffuse[:] = .7
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0
        self.camera = mujoco.MjvCamera()
        self.camera.distance, self.camera.azimuth, self.camera.elevation = 4.5, 105., -27.
        self.renderer = mujoco.Renderer(self.model, height=600, width=960)

    def frame(self, source, target=None, curve=()):
        from PIL import Image
        import numpy as np
        mj = self.mujoco
        self.data.qpos[:] = source.qpos
        self.data.mocap_pos[:] = source.mocap_pos
        dynamic=self.model.geom('dynamic_box_geom').id
        self.model.geom_rgba[dynamic,3]=0. if abs(source.mocap_pos[0,1])>3.5 else 1.
        mj.mj_forward(self.model, self.data)
        body = self.data.body('base').xpos
        self.camera.lookat[:] = body+[.6, 0., .15]
        self.renderer.update_scene(self.data, camera=self.camera, scene_option=self.option)
        scene = self.renderer.scene
        if target is not None and scene.ngeom < scene.maxgeom:
            geom = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(geom, mj.mjtGeom.mjGEOM_SPHERE, np.array([.09]*3), np.asarray(target)+[0., 0., .13], np.eye(3).ravel(), np.array([.95, .55, .08, 1.], dtype=np.float32))
            scene.ngeom += 1
        for a, b in zip(curve[:-1:2], curve[1::2]):
            if scene.ngeom >= scene.maxgeom:
                break
            geom = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(geom, mj.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.array([1., .6, .1, 1.], dtype=np.float32))
            mj.mjv_connector(geom, mj.mjtGeom.mjGEOM_LINE, 3., np.asarray(a), np.asarray(b))
            scene.ngeom += 1
        stream = io.BytesIO()
        Image.fromarray(self.renderer.render()).save(stream, format='JPEG', quality=78)
        return stream.getvalue()

    def close(self):
        self.renderer.close()
