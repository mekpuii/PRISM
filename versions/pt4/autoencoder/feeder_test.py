# feeder_test.py - push ONLY the feeder, a few times, and measure each push (2026-11-12).
#
# Moves the feeder blade and nothing else: no arm command, no belt command (the belt is stopped at the end, to be sure).
# For every push it prints: did the Mega confirm it (FEED_DONE), how long it took, and the 12 V rail dip during the push
# (a real push draws current: ~200 mV dip; no dip = the servo did not move).
#
# autoen must NOT be running (it holds the Mega's port):  cd ~/Desktop/PRISM && ./stop_cell.sh
# Run (autoen-env, from autoencoder/):
#   ~/autoen-env/bin/python feeder_test.py                 5 pushes, the current settings (back 170, end 10)
#   ~/autoen-env/bin/python feeder_test.py --pushes 3 --home 170 --push 10 --wait-ms 1000 --hold-ms 800
#   ~/autoen-env/bin/python feeder_test.py --manual        YOU drive the blade, one key + Enter each:
#        p = push (to --push, 10 deg)   l = pull back (to --home, 170 deg)   Enter alone = the opposite of the last move
#        f = one full feed (wait back, jump, hold, return)   q = quit (blade back home first)
# Hand away from the blade; Ctrl+C stops it (the blade goes back home).
import argparse
import statistics
import time

import auto_en_new as A


def main():
    ap = argparse.ArgumentParser(description="push only the feeder and measure each push")
    ap.add_argument("--pushes", type=int, default=5)
    ap.add_argument("--home", type=int, default=170, help="blade back (degrees)")
    ap.add_argument("--push", type=int, default=10, help="push end (degrees)")
    ap.add_argument("--wait-ms", type=int, default=1000, help="wait fully back before each push")
    ap.add_argument("--hold-ms", type=int, default=800, help="hold at the push end")
    ap.add_argument("--return-ms", type=int, default=300)
    ap.add_argument("--pause-s", type=float, default=2.0, help="pause between two pushes")
    ap.add_argument("--port", default=None)
    ap.add_argument("--manual", action="store_true", help="p = push, l = pull, Enter = toggle, f = full feed, q = quit")
    a = ap.parse_args()

    port = a.port or A.find_serial_port()
    print(f"Mega on {port} - opening (the Mega restarts, ~3 s) ...")
    link = A.Link(port)
    try:
        link.wait_ready()
        link.belt_cmd("STOP")
        link.belt_cmd(f"FEEDCFG,{a.home},{a.push},{a.hold_ms},{a.return_ms}")
        try:
            link.belt_cmd(f"FEEDPRE,{a.home},{a.wait_ms},0")
            pre = f"wait {a.wait_ms} ms at {a.home} deg, jump"
        except RuntimeError:
            pre = "this sketch has no ready wait: plain jump"
        print(f"feeder: back {a.home} deg -> push {a.push} deg ({pre}), hold {a.hold_ms} ms, return {a.return_ms} ms\n")
        if a.manual:
            manual(link, a)
            return
        dips = []
        for n in range(1, a.pushes + 1):
            base = []
            for _ in range(10):                             # the rail before the push (~1 s)
                b = link.belt
                if b and b.get("mv") is not None:
                    base.append(b["mv"])
                time.sleep(0.1)
            while not link.events.empty():
                link.events.get_nowait()
            t0 = time.monotonic()
            link.belt_cmd("FEED")
            low, done, end = None, None, t0 + (a.wait_ms + a.hold_ms + a.return_ms) / 1000.0 + 3.0
            while time.monotonic() < end and done is None:
                b = link.belt
                if b and b.get("mv") is not None and b.get("feed") in (1, 2):
                    low = b["mv"] if low is None else min(low, b["mv"])
                try:
                    name, _args = link.events.get(timeout=0.05)
                    if name == "FEED_DONE":
                        done = time.monotonic() - t0
                except Exception:                           # noqa: BLE001 - queue.Empty
                    pass
            dip = (statistics.median(base) - low) if base and low is not None else None
            if dip is not None:
                dips.append(dip)
            verdict = ("no telemetry" if dip is None else "MOVED" if dip >= 30 else
                       "did NOT move (no current)" if dip < 15 else "unclear")
            print(f"push {n}: {'FEED_DONE after %.2f s' % done if done else 'NO FEED_DONE'}, "
                  f"12 V {statistics.median(base) / 1000 if base else float('nan'):.2f} V, "
                  f"dip {dip if dip is not None else float('nan'):.0f} mV -> {verdict}")
            time.sleep(a.pause_s)
        if dips:
            print(f"\n{sum(d >= 30 for d in dips)} of {len(dips)} pushes moved the servo; dips {[round(d) for d in dips]} mV")
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        link.close()


def rail_mv(link, n=6):
    vals = []
    for _ in range(n):
        b = link.belt
        if b and b.get("mv") is not None:
            vals.append(b["mv"])
        time.sleep(0.05)
    return statistics.median(vals) if vals else None


def move(link, angle, watch_s=1.0):
    """Raw blade move (X,SERVO) and the lowest 12 V while it moves. Returns the dip in mV (None: no telemetry)."""
    base = rail_mv(link)
    link.belt_cmd(f"SERVO,{angle}")
    low, end = None, time.monotonic() + watch_s
    while time.monotonic() < end:
        b = link.belt
        if b and b.get("mv") is not None:
            low = b["mv"] if low is None else min(low, b["mv"])
        time.sleep(0.02)
    return None if base is None or low is None else base - low


def manual(link, a):
    print("MANUAL: p = push (%d deg), l = pull (%d deg), Enter = the opposite, f = full feed, q = quit" % (a.push, a.home))
    at = "home"
    move(link, a.home)
    while True:
        try:
            k = input(f"[blade {'PUSHED' if at == 'push' else 'BACK'}] > ").strip().lower()
        except EOFError:
            k = "q"
        if k == "q":
            break
        if k == "":
            k = "l" if at == "push" else "p"
        if k == "f":
            while not link.events.empty():
                link.events.get_nowait()
            t0 = time.monotonic()
            link.belt_cmd("FEED")
            done = None
            while time.monotonic() - t0 < (a.wait_ms + a.hold_ms + a.return_ms) / 1000 + 3 and done is None:
                try:
                    name, _x = link.events.get(timeout=0.05)
                    done = time.monotonic() - t0 if name == "FEED_DONE" else None
                except Exception:                           # noqa: BLE001 - queue.Empty
                    pass
            print(f"  full feed: {'FEED_DONE after %.2f s' % done if done else 'NO FEED_DONE'}")
            at = "home"
            continue
        if k not in ("p", "l"):
            print("  p = push, l = pull, Enter = the opposite, f = full feed, q = quit")
            continue
        angle = a.push if k == "p" else a.home
        dip = move(link, angle)
        at = "push" if k == "p" else "home"
        word = ("no telemetry" if dip is None else "the servo worked" if dip >= 30 else
                "NO current - did it move?" if dip < 15 else "small current")
        print(f"  {'push' if k == 'p' else 'pull'} to {angle} deg: 12 V dip {dip if dip is not None else float('nan'):.0f} mV -> {word}")
    if at != "home":
        move(link, a.home)
        print("blade back home")


if __name__ == "__main__":
    main()
