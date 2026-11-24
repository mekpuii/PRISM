"""Grab 1 RealSense colour frame (same 640x480 @ 6 fps as vision) and draw the saved ROI on it.
Camera only: no MQTT, no serial."""
import json, time, sys
import numpy as np, cv2, pyrealsense2 as rs
OUT = sys.argv[1] if len(sys.argv) > 1 else "/home/mekkhram/Desktop/PRISM/vision/snap"
cal = json.load(open("/home/mekkhram/Desktop/PRISM/vision/vision_calibration.json"))
p = rs.pipeline(); c = rs.config()
c.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
p.start(c)
try:
    for _ in range(15):              # let auto-exposure settle (~2.5 s)
        f = p.wait_for_frames()
    img = np.asanyarray(f.get_color_frame().get_data()).copy()
finally:
    p.stop()
cv2.imwrite(OUT + "_raw.png", img)
roi = np.array(cal["roi"], np.int32)
cv2.polylines(img, [roi], True, (0, 0, 255), 2)
cv2.putText(img, "ROI saved 2026-10-22 " + time.strftime("now %m-%d %H:%M"), (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
cv2.imwrite(OUT + "_roi.png", img)
print("saved", OUT + "_raw.png", OUT + "_roi.png")
