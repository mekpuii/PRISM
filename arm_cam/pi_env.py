"""pi_env.py - what is different on the Raspberry Pi (Linux): find the Mega port, find the wrist camera,
see who already holds a device, and read Enter from the terminal without blocking.

Nothing here opens the Mega port or a camera stream: it only looks at /dev, /sys and /proc.

Overrides (environment variables, read by config.py):
    SO101_PORT=/dev/ttyUSB0      use this serial port for the Mega
    SO101_CAM=/dev/video2        use this camera (a path or an index number)

Run it to see what the Pi has right now:   python pi_env.py
"""
import glob
import os
import sys

IS_LINUX = sys.platform.startswith("linux")

# The wrist camera is an OV9281 global-shutter module. Its USB name differs between sellers, so several hints are tried.
WRIST_CAM_HINTS = ("9281", "global shutter", "arducam", "innomaker", "usb camera", "usb2.0 camera", "uvc camera")
# PRISM's own overhead camera (vision_usb.py) and the Pi's internal video devices: never pick these as the wrist camera.
NOT_WRIST_CAM = ("astro", "bcm2835", "rpi-hevc", "pispbe", "unicam", "realsense")

# Same search order as PRISM's autoen_node.py, so both programs agree on which device is the Mega.
MEGA_PATTERNS = ("/dev/prism-mega", "/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*")


def find_mega_port():
    """Path of the Mega's serial port, or None. Genuine Mega = ttyACM*, CH340 clone = ttyUSB*."""
    for pattern in MEGA_PATTERNS:
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return None


def list_cameras():
    """Every V4L2 node that can capture pictures: [(device, name)], e.g. [("/dev/video0", "Astro HD Camera: ...")].
    A USB camera makes two nodes (pictures + metadata); only the picture node (index 0) is listed."""
    out = []
    for sysdir in sorted(glob.glob("/sys/class/video4linux/video*"),
                         key=lambda p: int(p.rsplit("video", 1)[1])):
        try:
            with open(os.path.join(sysdir, "name")) as f:
                name = f.read().strip()
            with open(os.path.join(sysdir, "index")) as f:
                index = int(f.read().strip())
        except (OSError, ValueError):
            continue
        if index == 0:
            out.append(("/dev/" + os.path.basename(sysdir), name))
    return out


def find_wrist_camera():
    """Device path of the OV9281 wrist camera, or None when it cannot be told apart for sure.
    Rule: a name hint wins; otherwise the only camera left after removing PRISM's and the Pi's own devices."""
    cams = [(dev, name) for dev, name in list_cameras()
            if not any(bad in name.lower() for bad in NOT_WRIST_CAM)]
    for dev, name in cams:
        if any(hint in name.lower() for hint in WRIST_CAM_HINTS):
            return dev
    if len(cams) == 1:
        return cams[0][0]
    return None


def camera_help():
    """Text for the error shown when the wrist camera is not found."""
    cams = list_cameras()
    usb = [f"{dev} = {name}" for dev, name in cams if not any(b in name.lower() for b in NOT_WRIST_CAM[1:])]
    lines = ["the OV9281 wrist camera was not found on this Pi."]
    lines.append("cameras seen: " + ("; ".join(usb) if usb else "none"))
    lines.append("plug the wrist camera in, or name it yourself:  SO101_CAM=/dev/videoN ./pi.sh <script>")
    return "\n  ".join(lines)


def holders(device):
    """Programs that have this device open right now: [(pid, command line)]. Our own process is left out.
    Looks at /proc/<pid>/fd, which works for programs of the same user (PRISM runs as the same user)."""
    try:
        real = os.path.realpath(device)
    except OSError:
        return []
    out = []
    me = os.getpid()
    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        pid = int(fd_dir.split("/")[2])
        if pid == me:
            continue
        try:
            names = os.listdir(fd_dir)
        except OSError:
            continue
        for n in names:
            try:
                if os.path.realpath(os.path.join(fd_dir, n)) == real:
                    with open(f"/proc/{pid}/cmdline", "rb") as f:
                        cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
                    out.append((pid, cmd))
                    break
            except OSError:
                continue
    return out


def check_port_free(port):
    """Raise RuntimeError when another program holds the Mega port.
    Why this matters: Linux lets two programs open the same serial port, and opening it RESETS the Mega.
    Starting an arm_cam script while autoen runs would reboot the arm firmware in the middle of a PRISM cycle."""
    if not IS_LINUX or not port:
        return
    busy = holders(port)
    if busy:
        who = "; ".join(f"pid {pid}: {cmd[:90]}" for pid, cmd in busy)
        raise RuntimeError(f"the Mega port {port} is in use by another program ({who}). "
                           "Stop PRISM first (~/Desktop/PRISM/stop_cell.sh), then run this again.")


def enter_pressed():
    """True when the user has pressed Enter in the terminal (the line is read and thrown away). Never waits.
    Windows: msvcrt. Linux/Pi: select() on stdin. Lets a script keep the camera window live while it waits."""
    try:
        import msvcrt
    except ImportError:
        import select
        try:
            ready, _, _ = select.select([sys.stdin], [], [], 0)
        except (OSError, ValueError):
            return False
        if ready:
            return sys.stdin.readline() != ""       # "" = end of input (no terminal): that is not an Enter
        return False
    hit = False
    while msvcrt.kbhit():
        hit = msvcrt.getwch() in ("\r", "\n") or hit
    return hit


if __name__ == "__main__":
    port = find_mega_port()
    print("Mega port   :", port or "NOT FOUND (is the USB cable plugged in?)")
    if port:
        busy = holders(port)
        print("  in use by :", "; ".join(f"pid {p}: {c[:80]}" for p, c in busy) if busy else "nobody (free)")
    print("cameras     :")
    for dev, name in list_cameras():
        if not any(b in name.lower() for b in NOT_WRIST_CAM[1:]):
            print(f"  {dev}  {name}")
    cam = find_wrist_camera()
    print("wrist camera:", cam or "NOT FOUND")
    if cam:
        busy = holders(cam)
        print("  in use by :", "; ".join(f"pid {p}: {c[:80]}" for p, c in busy) if busy else "nobody (free)")
