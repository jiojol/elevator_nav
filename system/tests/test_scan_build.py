#!/usr/bin/env python3
"""Regression tests for source restores with old mtimes and cached objects."""
import os
import json
import platform
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import verify_scan_build as build


class BuildTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / 'nav/src/prior_local_planner/src/controller.cpp'
        self.source.parent.mkdir(parents=True)
        self.source.write_text('restored original code')
        for name in (*build.ARTIFACTS, 'nav/src/CMakeLists.txt',
                     *build.EXECUTION_SOURCES,
                     'nav/cmake_overlay/cxx17-g++.sh', 'nav/build/CMakeCache.txt'):
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('fixture')
        self.obj = self.root / 'nav/build/prior_local_planner/core/CMakeFiles/controller.cpp.o'
        self.obj.parent.mkdir(parents=True)
        self.obj.write_text('cached modified code')

    def rebuild(self, callback=None):
        def compile(*args, **kwargs):
            self.assertTrue(all(not (self.root / name).exists() for name in build.ARTIFACTS))
            if callback:
                callback(*args, **kwargs)
            for name in build.ARTIFACTS:
                (self.root / name).write_text('freshly linked fixture')
        with patch.object(build, 'running_nodes', return_value=False), \
                patch.object(build.subprocess, 'run', side_effect=compile) as command:
            build.rebuild(self.root)
        return command

    def test_clean_rebuild_discards_newer_object_even_when_source_timestamp_is_old(self):
        os.utime(self.source, (1., 1.))
        def compile(*args, **kwargs):
            self.assertFalse(self.obj.exists())
            self.obj.write_text(self.source.read_text())
        self.rebuild(compile)
        self.assertEqual(self.obj.read_text(), 'restored original code')
        build.verify(self.root)

    def test_missing_manifest_and_changed_source_are_rejected_even_with_preserved_mtime(self):
        with self.assertRaises(OSError):
            build.verify(self.root)
        self.rebuild()
        old = self.source.stat()
        self.source.write_text('different code with old timestamp')
        os.utime(self.source, ns=(old.st_atime_ns, old.st_mtime_ns))
        with self.assertRaisesRegex(ValueError, 'source'):
            build.verify(self.root)

    def test_changed_shared_library_or_executable_is_rejected(self):
        self.rebuild()
        for name in build.ARTIFACTS:
            with self.subTest(name=name):
                path = self.root / name
                original = path.read_bytes()
                path.write_bytes(b'old binary copied back')
                with self.assertRaisesRegex(ValueError, 'executable or shared library'):
                    build.verify(self.root)
                path.write_bytes(original)

    def test_failed_build_invalidates_previous_manifest(self):
        self.rebuild()
        with self.assertRaises(subprocess.CalledProcessError):
            self.rebuild(lambda *args, **kwargs: (_ for _ in ()).throw(subprocess.CalledProcessError(2, 'make')))
        self.assertFalse((self.root / build.MANIFEST).exists())

    def test_source_changed_during_build_cannot_be_certified(self):
        with self.assertRaisesRegex(ValueError, 'source changed during build'):
            self.rebuild(lambda *args, **kwargs: self.source.write_text('edited while compiling'))
        self.assertFalse((self.root / build.MANIFEST).exists())

    def test_live_navigation_is_not_rebuilt(self):
        with patch.object(build, 'running_nodes', return_value=True):
            with self.assertRaisesRegex(ValueError, 'navigation nodes are running'):
                build.rebuild(self.root)
        self.assertTrue(self.obj.exists())

    def test_linking_into_wrong_output_directory_cannot_certify_leftover_binaries(self):
        with patch.object(build, 'running_nodes', return_value=False), \
                patch.object(build.subprocess, 'run'):
            with self.assertRaises(OSError):
                build.rebuild(self.root)
        self.assertFalse((self.root / build.MANIFEST).exists())

    def candidate(self):
        directory = self.root / 'candidate'
        for name in build.ARTIFACTS:
            path = directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('candidate ' + name)
        record = {'schema': 1, 'clean_objects': True, 'architecture': platform.machine(),
                  'sources': build.sources(self.root), 'artifacts': build.artifacts(directory)}
        (directory / 'manifest.json').write_text(json.dumps(record))
        return directory

    def test_verified_candidate_installs_and_preserves_previous_artifacts(self):
        candidate = self.candidate()
        with patch.object(build, 'running_nodes', return_value=False):
            build.install_candidate(self.root, candidate)
        build.verify(self.root)
        for name in build.ARTIFACTS:
            self.assertEqual((self.root / name).read_text(), 'candidate ' + name)
        backups = list((self.root / 'nav/build').glob('scan-before-reference-*'))
        self.assertEqual(len(backups), 1)
        self.assertEqual((backups[0] / build.ARTIFACTS[0]).read_text(), 'fixture')

    def test_candidate_source_or_artifact_mismatch_does_not_replace_active_files(self):
        candidate = self.candidate()
        with patch.object(build, 'running_nodes', return_value=False):
            self.source.write_text('changed after compilation')
            with self.assertRaisesRegex(ValueError, 'current SCAN source'):
                build.install_candidate(self.root, candidate)
            candidate = self.candidate()
            (candidate / build.ARTIFACTS[0]).write_text('unverified replacement')
            with self.assertRaisesRegex(ValueError, 'artifact changed'):
                build.install_candidate(self.root, candidate)
        self.assertEqual((self.root / build.ARTIFACTS[0]).read_text(), 'fixture')

    def test_live_navigation_cannot_install_a_candidate(self):
        with patch.object(build, 'running_nodes', return_value=True):
            with self.assertRaisesRegex(ValueError, 'navigation nodes are running'):
                build.install_candidate(self.root, self.candidate())


if __name__ == '__main__':
    unittest.main()
