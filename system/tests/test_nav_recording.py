#!/usr/bin/env python3
"""Verify recorder shutdown, bag contents, diagnostic snapshots and MP4 export."""

import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client


def main():
    root = Path(__file__).resolve().parents[2]
    logs = Path(tempfile.mkdtemp(prefix="dog-nav-record-test-"))
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
    os.environ["ROS_HOSTNAME"] = "127.0.0.1"
    os.environ.pop("ROS_IP", None)
    processes = []
    handles = []

    def launch(name, args):
        handle = (logs / (name + ".log")).open("w")
        handles.append(handle)
        proc = subprocess.Popen(args, cwd=root, stdout=handle, stderr=subprocess.STDOUT,
                                start_new_session=True)
        processes.append(proc)
        return proc

    try:
        launch("master", ["roscore", "-p", str(port)])
        master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
        deadline = time.monotonic() + 15
        while True:
            try:
                if master.getPid("/record_test")[0] == 1:
                    break
            except OSError:
                pass
            assert time.monotonic() < deadline, "master startup timeout"
            time.sleep(.1)
        import cv2
        import numpy as np
        import rosbag
        import rospy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String

        rospy.init_node("recording_test", disable_signals=True)
        rospy.set_param("/recording_test/value", 42)
        camera = rospy.Publisher("/camera/image_raw/compressed", CompressedImage, queue_size=1)
        commands = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        status = rospy.Publisher("/dog_nav/localization_status", String, queue_size=1, latch=True)
        safety = rospy.Publisher('/dog_nav/safety_stop', String, queue_size=1, latch=True)
        recording = logs / "recording"
        recorder = launch("recorder", ["bash", "system/scripts/record_nav.sh", "--video",
                                       "--output", str(recording)])
        deadline = time.monotonic() + 20
        while not (camera.get_num_connections() and commands.get_num_connections() and safety.get_num_connections()):
            assert recorder.poll() is None, "recorder exited: " + str(logs)
            assert time.monotonic() < deadline, "recorder startup timeout: " + str(logs)
            time.sleep(.1)
        safety.publish(String(data='{"reason":"test safety snapshot"}'))
        for i in range(30):
            frame = np.full((120, 160, 3), (i * 7) % 255, np.uint8)
            ok, data = cv2.imencode(".jpg", frame)
            assert ok
            msg = CompressedImage(format="jpeg", data=data.tobytes())
            msg.header.stamp = rospy.Time.now()
            camera.publish(msg)
            commands.publish(Twist())
            status.publish(String(data="TRACKING"))
            time.sleep(.05)
        os.kill(recorder.pid, signal.SIGINT)
        assert recorder.wait(timeout=30) in (0, 130), "recorder shutdown failed: " + str(logs)
        bags = sorted(recording.glob("navigation*.bag"))
        assert bags and not list(recording.glob("*.active")), "bag was not finalized"
        with rosbag.Bag(str(bags[0])) as bag:
            topics = bag.get_type_and_topic_info().topics
            assert "/cmd_vel" in topics and "/dog_nav/localization_status" in topics
            assert '/dog_nav/safety_stop' in topics
            assert topics["/camera/image_raw/compressed"].message_count >= 20
        for name in ("params_start.yaml", "params_end.yaml", "topics_start.txt",
                     "topics_end.txt", "scan.yaml", "navigation.launch"):
            assert (recording / name).is_file(), "snapshot missing: " + name
        assert (recording / "ros_logs_end/rosout.log").is_file()
        assert '/planning/reference_refresh' in (recording / 'recorded_topics.txt').read_text().splitlines()
        for name in ('reference', 'bspline', 'odometry', 'obstacle_points'):
            assert '/dog_nav/safety/' + name in (recording / 'recorded_topics.txt').read_text().splitlines()
        capture = cv2.VideoCapture(str(recording / "camera.mp4"))
        assert capture.isOpened() and capture.get(cv2.CAP_PROP_FRAME_COUNT) >= 15
        ok, frame = capture.read()
        capture.release()
        assert ok and frame.shape[:2] == (120, 160)
        print("PASS: Ctrl+C finalized bag, captured topics/parameters/logs and exported playable MP4")
        print("Artifacts: " + str(recording))
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=3)
        for handle in handles:
            handle.close()


if __name__ == "__main__":
    main()
