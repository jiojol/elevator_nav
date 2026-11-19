#!/usr/bin/env python3
"""Export a compressed camera stream from split navigation bags to MP4."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import rosbag


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--topic", default="/camera/image_raw/compressed")
    parser.add_argument("--fps", type=float, default=15.0)
    args = parser.parse_args()
    if not np.isfinite(args.fps) or args.fps <= 0:
        parser.error("--fps must be positive")
    bags = sorted(args.recording.glob("navigation*.bag"),
                  key=lambda path: int(path.stem.rsplit("_", 1)[-1])
                  if path.stem.rsplit("_", 1)[-1].isdigit() else -1)
    if not bags:
        raise SystemExit("No finalized navigation bags found: " + str(args.recording))
    output = args.recording / "camera.mp4"
    if output.exists():
        raise SystemExit("Video already exists: " + str(output))
    writer = None
    count = 0
    first_time = None
    previous = None
    try:
        for path in bags:
            with rosbag.Bag(str(path)) as bag:
                for _, msg, stamp in bag.read_messages(topics=[args.topic]):
                    if getattr(msg, "_type", "") != "sensor_msgs/CompressedImage":
                        raise ValueError("Expected sensor_msgs/CompressedImage: " + args.topic)
                    frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        continue
                    if writer is None:
                        size = (frame.shape[1], frame.shape[0])
                        writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, size)
                        if not writer.isOpened():
                            raise RuntimeError("Cannot open MP4 encoder")
                        first_time = stamp.to_sec()
                        previous = frame
                    if (frame.shape[1], frame.shape[0]) != size:
                        frame = cv2.resize(frame, size)
                    frame_index = max(0, round((stamp.to_sec() - first_time) * args.fps))
                    while count < frame_index:
                        writer.write(previous)
                        count += 1
                    if count == frame_index:
                        writer.write(frame)
                        count += 1
                    previous = frame
    finally:
        if writer is not None:
            writer.release()
    if count == 0:
        raise SystemExit("No camera frames on " + args.topic + "; bags remain available for RViz replay")
    print("Camera video: %s (%d frames, %.1f fps)" % (output, count, args.fps))


if __name__ == "__main__":
    main()
