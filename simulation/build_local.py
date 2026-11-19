#!/usr/bin/env python3
"""Build the actual SCAN sources for this computer, without robot access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BUILD = ROOT/'simulation/artifacts/local_build'
VISION_COMMIT = '08b012c038e575d7fe1d538f11235a994159dc93'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sources():
    paths = []
    for directory in [ROOT/'nav/src/prior_local_planner']:
        paths += [p for p in directory.rglob('*') if p.is_file() and not any(
            part in {'.git', '__pycache__', '.pytest_cache', 'datasets', 'checkpoints', 'models'} for part in p.relative_to(directory).parts)]
    paths += [ROOT/p for p in ['simulation/local_build/CMakeLists.txt', 'simulation/build_local.py',
        'nav/src/dog_nav_bringup/scripts/scan_command_gate.py','nav/src/dog_nav_bringup/scripts/start_alignment_policy.py', 'nav/src/dog_nav_bringup/src/reference_path_transform.cpp',
        'nav/src/dog_nav_bringup/include/dog_nav_bringup/planar_reference.hpp',
        'system/tests/scan_spatial_guard_test.cpp','system/tests/body_reference_collision_test.cpp',
        'system/tests/narrow_prior_replan_test.cpp','system/tests/astar_endpoint_regression_test.cpp','system/tests/map_publication_test.cpp']]
    paths += [ROOT/'nav/src/dog_nav_bringup/src/fastlio_map_localizer.cpp',
              ROOT/'nav/src/dog_nav_bringup/include/dog_nav_bringup/registration_checks.hpp']
    paths += [p for p in (ROOT/'nav/src/fast_gicp').rglob('*') if p.is_file()]
    return {str(p.relative_to(ROOT)): digest(p) for p in sorted(paths)}


def verify(build):
    record = json.loads((build/'manifest.json').read_text())
    if record['architecture'] != platform.machine() or record['sources'] != sources():
        raise RuntimeError('本地源码已变化，请重新运行 python3 simulation/build_local.py')
    for name, value in record['artifacts'].items():
        if digest(build/name) != value:
            raise RuntimeError('本地编译文件发生变化：'+name)
    return record


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build', type=Path, default=DEFAULT_BUILD)
    p.add_argument('--jobs', type=int, default=2)
    p.add_argument('--verify', action='store_true')
    args = p.parse_args()
    build = args.build.resolve()
    if args.verify:
        verify(build)
        print('Local SCAN source and binary hashes verified.')
        return
    if args.jobs < 1:
        p.error('--jobs must be positive')
    build.mkdir(parents=True, exist_ok=True)
    dependencies = ROOT/'simulation/artifacts/local_dependencies'
    dependencies.mkdir(parents=True, exist_ok=True)
    vision = dependencies/'vision_opencv'
    if not vision.exists():
        subprocess.run(['git', 'clone', '--depth', '1', '--branch', '1.16.2',
                        'https://github.com/ros-perception/vision_opencv.git', str(vision)], check=True)
    if subprocess.check_output(['git', '-C', str(vision), 'rev-parse', 'HEAD'], text=True).strip() != VISION_COMMIT:
        raise RuntimeError('cv_bridge dependency revision differs from the pinned version')
    if subprocess.check_output(['git', '-C', str(vision), 'status', '--porcelain'], text=True).strip():
        raise RuntimeError('cv_bridge dependency has local modifications')
    tf_candidates = [Path('/opt/ros/noetic'), Path.home()/'ros/noetic_overlay/install']
    tf = next((x for x in tf_candidates if (x/'include/tf/tf.h').exists() and (x/'lib/libtf.so').exists()), None)
    if tf is None:
        raise RuntimeError('缺少本机 ROS tf 开发库；未连接任何远端机器')
    shutil.copytree(tf/'include/tf', dependencies/'tf/include/tf', dirs_exist_ok=True)
    (dependencies/'tf/lib').mkdir(parents=True, exist_ok=True)
    shutil.copy2(tf/'lib/libtf.so', dependencies/'tf/lib/libtf.so')
    headers = [Path('/opt/ros/noetic/include')]
    headers += sorted((Path.home()/'.cache/rattler/cache/pkgs').glob('ros-noetic-pcl-conversions-*/include'))
    conversion = next((x for x in headers if (x/'pcl_conversions/pcl_conversions.h').exists()), None)
    if conversion is None:
        raise RuntimeError('缺少本机 pcl_conversions 头文件')
    shutil.copytree(conversion/'pcl_conversions', dependencies/'pcl_conversions/include/pcl_conversions', dirs_exist_ok=True)
    prefix = build/'devel'
    package = ROOT/'nav/src/prior_local_planner/src/planner/plan_manage'
    cpp, py = prefix/'include/scan_planner', prefix/'lib/python3/dist-packages/scan_planner/msg'
    cpp.mkdir(parents=True, exist_ok=True)
    py.mkdir(parents=True, exist_ok=True)
    (py.parent/'__init__.py').touch()
    includes = ['-Istd_msgs:/opt/ros/noetic/share/std_msgs/msg', '-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/msg', '-Iscan_planner:'+str(package/'msg')]
    env = os.environ.copy()
    env['PYTHONPATH'] = '/opt/ros/noetic/lib/python3/dist-packages'
    for name in ['Bspline', 'DataDisp']:
        msg = str(package/'msg'/f'{name}.msg')
        subprocess.run(['/usr/bin/python3', '/opt/ros/noetic/lib/gencpp/gen_cpp.py', msg,
                        '-p', 'scan_planner', '-o', str(cpp), '-e', '/opt/ros/noetic/share/gencpp', *includes], env=env, check=True)
        subprocess.run(['/usr/bin/python3', '/opt/ros/noetic/lib/genpy/genmsg_py.py', msg,
                        '-p', 'scan_planner', '-o', str(py), *includes], env=env, check=True)
    (py/'__init__.py').write_text('from ._Bspline import *\nfrom ._DataDisp import *\n')
    before = sources()
    subprocess.run(['cmake', '-S', str(ROOT/'simulation/local_build'), '-B', str(build/'cmake'),
                    '-DREPOSITORY='+str(ROOT), '-DOUTPUT_PREFIX='+str(prefix), '-DROS_PREFIX=/opt/ros/noetic',
                    '-DDEPENDENCIES='+str(dependencies)], check=True)
    subprocess.run(['cmake', '--build', str(build/'cmake'), '--parallel', str(args.jobs)], check=True)
    if before != sources():
        raise RuntimeError('源码在编译中改变，没有生成验证记录')
    packages = {'scan_planner': package, 'dog_nav_bringup': ROOT/'nav/src/dog_nav_bringup'}
    for name, source in packages.items():
        link = prefix/'share'/name
        link.parent.mkdir(parents=True, exist_ok=True)
        if link.is_symlink() and link.resolve()!=source.resolve():
            link.unlink()
        if not link.exists():
            link.symlink_to(source, target_is_directory=True)
    (prefix/'.catkin').write_text(';'.join(str(x) for x in packages.values()))
    gate = prefix/'lib/dog_nav_bringup/scan_command_gate.py'
    if not gate.exists():
        gate.symlink_to(ROOT/'nav/src/dog_nav_bringup/scripts/scan_command_gate.py')
    artifacts = [x for x in (prefix/'lib').rglob('*') if x.is_file() and '__pycache__' not in x.parts]
    artifacts += [dependencies/'tf/lib/libtf.so', dependencies/'pcl_conversions/include/pcl_conversions/pcl_conversions.h']
    record = {'architecture': platform.machine(), 'sources': before, 'cv_bridge_commit': VISION_COMMIT,
              'artifacts': {os.path.relpath(x, build): digest(x) for x in sorted(artifacts)}}
    (build/'manifest.json').write_text(json.dumps(record, indent=2)+'\n')
    verify(build)
    print('Local SCAN build verified:', prefix)


if __name__ == '__main__':
    main()
