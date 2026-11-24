import pyrealsense2 as rs
import time

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.depth, 640, 480, rs.format.z16, 6
)
config.enable_stream(
    rs.stream.color, 640, 480, rs.format.bgr8, 6
)

try:
    print("Starting D415...", flush=True)
    pipeline.start(config)
    print("Started. Waiting for frames...", flush=True)

    for i in range(10):
        frames = pipeline.wait_for_frames(15000)
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        print(
            f"Frame {i+1}: depth={bool(depth)}, "
            f"color={bool(color)}",
            flush=True
        )

finally:
    pipeline.stop()