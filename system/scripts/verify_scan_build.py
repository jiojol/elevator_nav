#!/usr/bin/env python3
"""Bind the deployed SCAN binaries to a clean build of the current source tree."""
import argparse
import fcntl
import hashlib
import json
import os
import platform
from pathlib import Path
import subprocess
import shutil
import sys
import tempfile


ARTIFACTS = (
    'nav/devel/lib/dog_nav_bringup/fastlio_map_localizer',
    'nav/devel/lib/libfast_gicp.so',
    'nav/devel/lib/dog_nav_bringup/reference_path_transform',
    'nav/devel/lib/scan_planner/scan_planner_node',
    'nav/devel/lib/scan_planner/closed_loop_controller',
    'nav/devel/lib/libplan_env.so',
    'nav/devel/lib/libpath_searching.so',
    'nav/devel/lib/libbspline_opt.so',
    'nav/devel/lib/libtraj_utils.so',
    'nav/devel/include/scan_planner/Bspline.h',
    'nav/devel/lib/python3/dist-packages/scan_planner/msg/_Bspline.py',
)
MANIFEST = 'nav/devel/scan_build_manifest.json'
EXECUTION_SOURCES = (
    'nav/src/dog_nav_bringup/scripts/scan_command_gate.py',
    'nav/src/dog_nav_bringup/scripts/start_alignment_policy.py',
    'nav/src/dog_nav_bringup/src/fastlio_map_localizer.cpp',
    'nav/src/dog_nav_bringup/include/dog_nav_bringup/registration_checks.hpp',
    'nav/src/dog_nav_bringup/config/fastlio_localization.yaml',
    'nav/src/dog_nav_bringup/src/reference_path_transform.cpp',
    'nav/src/dog_nav_bringup/include/dog_nav_bringup/planar_reference.hpp',
    'nav/src/dog_nav_bringup/CMakeLists.txt',
    'nav/src/dog_nav_bringup/launch/prior_local.launch',
    'nav/src/dog_nav_bringup/launch/navigation.launch',
    'nav/src/map_for_plan/scripts/global_planner.py',
    'nav/src/map_for_plan/scripts/path_smoothing.py',
    'nav/src/map_for_plan/scripts/plan_graph.py',
    'nav/src/map_for_plan/CMakeLists.txt',
)


def digest(path):
    with path.open('rb') as handle:
        value = hashlib.sha256()
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            value.update(block)
        return value.hexdigest()


def sources(root):
    base = root / 'nav/src/prior_local_planner'
    if not base.is_dir():
        raise ValueError('SCAN source tree is missing')
    result = {}
    for path in sorted(base.rglob('*')):
        if '.git' in path.relative_to(base).parts:
            continue
        key = str(path.relative_to(root))
        if path.is_symlink():
            result[key] = 'link:' + os.readlink(path)
        elif path.is_file():
            result[key] = digest(path)
    for name in ('nav/src/CMakeLists.txt', 'nav/cmake_overlay/cxx17-g++.sh', *EXECUTION_SOURCES):
        result[name] = digest(root / name)
    for path in sorted((root / 'nav/src/fast_gicp').rglob('*')):
        if path.is_file() and '.git' not in path.parts:
            result[str(path.relative_to(root))]=digest(path)
    return result


def artifacts(root):
    return {name: digest(root / name) for name in ARTIFACTS}


def verify(root):
    with (root / MANIFEST).open() as handle:
        record = json.load(handle)
    if record.get('schema') != 1 or record.get('clean_objects') is not True:
        raise ValueError('missing clean-build provenance')
    current = sources(root)
    previous = record.get('sources', {})
    # These files run directly from the source tree; rebuilding C++ cannot
    # validate or deploy their changes. Keep native source/artifact checks exact.
    runtime_only = {
        'nav/src/map_for_plan/scripts/global_planner.py',
        'nav/src/dog_nav_bringup/launch/navigation.launch',
    }
    changed = {name for name in set(previous) | set(current)
               if previous.get(name) != current.get(name)}
    if changed - runtime_only:
        raise ValueError('SCAN source/build configuration changed since the clean build')
    if record.get('artifacts') != artifacts(root):
        raise ValueError('SCAN executable or shared library changed since the clean build')


def running_nodes(root):
    directory = root / 'nav/devel/lib/scan_planner'
    for exe in Path('/proc').glob('[0-9]*/exe'):
        try:
            target = Path(os.readlink(exe).split(' (deleted)', 1)[0])
            if target.parent == directory:
                return True
            if target in {root/'nav/devel/lib/dog_nav_bringup/fastlio_map_localizer',
                          root/'nav/devel/lib/dog_nav_bringup/reference_path_transform'}:
                return True
            arguments = (exe.parent / 'cmdline').read_bytes().split(b'\0')
            if any(Path(os.fsdecode(arg)).name == 'scan_command_gate.py' for arg in arguments if arg):
                return True
        except OSError:
            continue
    return False


def rebuild(root):
    if running_nodes(root):
        raise ValueError('navigation nodes are running; close navigation before rebuilding')
    build = root / 'nav/build'
    if not (build / 'CMakeCache.txt').is_file():
        raise ValueError('configure the nav workspace with catkin_make first')
    before = sources(root)
    manifest = root / MANIFEST
    manifest.unlink(missing_ok=True)
    # git archive preserves old timestamps. Remove every SCAN object rather
    # than trusting make's timestamp-based decision after a source restore.
    objects = list((build / 'prior_local_planner').rglob('*.o'))
    objects += list((build / 'dog_nav_bringup/CMakeFiles/reference_path_transform.dir').rglob('*.o'))
    objects += list((build / 'dog_nav_bringup/CMakeFiles/fastlio_map_localizer.dir').rglob('*.o'))
    objects += list((build / 'fast_gicp').rglob('*.o'))
    for path in objects:
        path.unlink()
    # A stale CMake cache must not certify leftover binaries if it links its
    # outputs into a different devel directory.
    for name in ARTIFACTS:
        (root / name).unlink(missing_ok=True)
    log = build / 'scan_clean_build.log'
    print('Removed %d cached SCAN objects. Build log: %s' % (len(objects), log), flush=True)
    command = ['cmake', '--build', str(build), '--target',
               'scan_planner_node', 'closed_loop_controller', 'reference_path_transform',
               'fastlio_map_localizer', '--', '-j2']
    with log.open('w') as handle:
        subprocess.run(command, cwd=root, stdout=handle, stderr=subprocess.STDOUT, check=True)
    if before != sources(root):
        raise ValueError('source changed during build; no deployment manifest was written')
    record = {'schema': 1, 'clean_objects': True, 'sources': before, 'artifacts': artifacts(root)}
    temporary = manifest.with_suffix('.tmp')
    temporary.write_text(json.dumps(record, indent=2, sort_keys=True) + '\n')
    temporary.replace(manifest)
    verify(root)


def install_candidate(root, candidate):
    """Install a verified isolated build after navigation has stopped."""
    if running_nodes(root):
        raise ValueError('navigation nodes are running; close navigation before installing')
    candidate = candidate.resolve()
    record = json.loads((candidate / 'manifest.json').read_text())
    if (record.get('schema') != 1 or record.get('clean_objects') is not True
            or record.get('architecture') != platform.machine()):
        raise ValueError('candidate build provenance or architecture is invalid')
    current = sources(root)
    if record.get('sources') != current or set(record.get('artifacts', {})) != set(ARTIFACTS):
        raise ValueError('candidate was not built from the current SCAN source')
    for name in ARTIFACTS:
        if digest(candidate / name) != record['artifacts'][name]:
            raise ValueError('candidate artifact changed: ' + name)
    backup = Path(tempfile.mkdtemp(prefix='scan-before-reference-', dir=root / 'nav/build'))
    manifest = root / MANIFEST
    if manifest.exists():
        shutil.copy2(manifest, backup / 'manifest.json')
    staged = []
    try:
        for name in ARTIFACTS:
            target = root / name
            saved = backup / name
            saved.parent.mkdir(parents=True, exist_ok=True)
            if target.exists():
                shutil.copy2(target, saved)
            target.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(prefix=target.name + '.new.', dir=target.parent, delete=False) as handle:
                temporary = Path(handle.name)
            staged.append((temporary, target))
            shutil.copy2(candidate / name, temporary)
            if digest(temporary) != record['artifacts'][name]:
                raise ValueError('candidate changed during installation: ' + name)
        if current != sources(root) or running_nodes(root):
            raise ValueError('source or running navigation changed during installation')
        manifest.unlink(missing_ok=True)
        for temporary, target in staged:
            temporary.replace(target)
        installed = {'schema': 1, 'clean_objects': True, 'sources': current, 'artifacts': artifacts(root)}
        temporary_manifest = manifest.with_suffix('.tmp')
        temporary_manifest.write_text(json.dumps(installed, indent=2, sort_keys=True) + '\n')
        temporary_manifest.replace(manifest)
        verify(root)
    finally:
        for temporary, _ in staged:
            temporary.unlink(missing_ok=True)
    print('Previous SCAN artifacts saved to: %s' % backup)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument('--rebuild', action='store_true')
    actions.add_argument('--install-candidate', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    try:
        with (root / 'nav/.scan_build.lock').open('a') as lock:
            mode = fcntl.LOCK_EX if args.rebuild or args.install_candidate else fcntl.LOCK_SH
            fcntl.flock(lock, mode | fcntl.LOCK_NB)
            if args.rebuild:
                rebuild(root)
            elif args.install_candidate:
                install_candidate(root, args.install_candidate)
            else:
                verify(root)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('SCAN build verification failed: %s' % error, file=sys.stderr)
        print('Run: bash system/scripts/rebuild_scan.sh', file=sys.stderr)
        return 1
    print('SCAN native build sources, executable and library hashes verified.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
