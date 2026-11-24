#!/usr/bin/env python3
"""color_train.py - labelled colour-sorting session for training the camera's colour order.

Run it (orch-env) while the camera node is running:
    python3 cal/color_train.py

Steps it walks you through:
  1. Colours: the colours the camera is set up for (vision/vision_calibration.json "enabled").
  2. Bins:    for each colour you choose L (left bin) or R (right bin). Saved to orchestrator/sort_bins.json,
              which optimize.py reads at start (restart the orchestrator after changing it).
  3. Order:   you type the exact order you will feed, first letters, e.g. OWOWWO. Cubes cannot overtake on
              the belt, so this is the true colour of every cube = the training label.
  4. Record:  every camera/arm/belt MQTT message goes to cal/runs/train_<time>.jsonl (same format as
              record_run.py), with your labels and bins in the header.
              While it runs, type  l  or  r  + Enter each time a cube lands in the left / right bin
              (optional second check), or  q  + Enter to stop.
Nothing here moves the robot.
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ORCH = os.path.dirname(HERE)
PRISM = os.path.dirname(ORCH)
sys.path.insert(0, HERE)
from record_run import Recorder  # noqa: E402

CALIB = os.path.join(PRISM, "vision", "vision_calibration.json")
BINS_FILE = os.path.join(ORCH, "sort_bins.json")
DEFAULT_BINS = {"orange": "left", "white": "right"}


def load_colours():
    try:
        with open(CALIB) as f:
            c = json.load(f)
        return list(c.get("enabled") or c.get("colors", {}).keys())
    except Exception as e:
        print("could not read", CALIB, e)
        return list(DEFAULT_BINS)


def load_bins():
    try:
        with open(BINS_FILE) as f:
            return {k: v for k, v in json.load(f).items() if v in ("left", "right")}
    except Exception:
        return dict(DEFAULT_BINS)


def ask_bins(colours, bins):
    print("\n== Bins: choose L (left) or R (right) for each colour, Enter keeps the current one ==")
    out = {}
    for col in colours:
        cur = bins.get(col, "left")
        while True:
            a = input(f"  {col:<8} [{cur}] L/R: ").strip().lower()
            if a == "":
                out[col] = cur
                break
            if a in ("l", "left", "r", "right"):
                out[col] = "left" if a.startswith("l") else "right"
                break
            print("    type L or R")
    with open(BINS_FILE, "w") as f:
        json.dump(out, f, indent=2)
    print("  saved", BINS_FILE, "->", out)
    return out


def ask_order(colours):
    letters = {}
    for col in colours:
        k = col[0].upper()
        if k in letters:
            sys.exit(f"two colours start with {k}: {letters[k]} and {col}; rename one")
        letters[k] = col
    keys = "/".join(f"{k}={v}" for k, v in letters.items())
    print(f"\n== Feed order: type the cubes in the order you will feed them ({keys}) ==")
    while True:
        s = input("  order: ").strip().upper().replace(" ", "").replace(",", "")
        bad = [ch for ch in s if ch not in letters]
        if s and not bad:
            return s, [letters[ch] for ch in s]
        print("    unknown letters:", "".join(bad) or "(empty)")


def main():
    colours = load_colours()
    print("Colours the camera detects:", ", ".join(colours))
    bins = ask_bins(colours, load_bins())
    order_str, order = ask_order(colours)
    expect = "".join(bins[c][0].upper() for c in order)
    print(f"\n  {len(order)} cubes: {order_str}")
    print(f"  expected bins:  {expect}   (L = left, R = right)")
    note = input("  note (optional, Enter to skip): ").strip()

    os.makedirs(os.path.join(HERE, "runs"), exist_ok=True)
    out = os.path.join(HERE, "runs", time.strftime("train_%Y%m%d_%H%M%S.jsonl"))
    header = {"rt": time.time(), "kind": "color_train", "colours": colours, "bins": bins,
              "order": order_str, "labels": order, "expected_bins": expect, "note": note}
    rec = Recorder(out, header, os.environ.get("MQTT_HOST", "localhost"), 1883, mode="x")
    rec.start()

    time.sleep(3)
    if not rec.counts.get("camera/perception"):
        print("\n  !! no camera/perception messages yet: is the camera plugged in and the vision node running?")
    print("\nRecording. Feed the cubes now in that order.")
    print("Type  l / r + Enter when a cube lands in a bin,  q + Enter to stop.\n")
    landed = []
    try:
        while True:
            a = input().strip().lower()
            if a in ("q", "quit", "stop"):
                break
            if a in ("l", "r"):
                landed.append(a.upper())
                i = len(landed) - 1
                ok = "ok" if i < len(expect) and expect[i] == a.upper() else "WRONG"
                rec.write_event("train/landed", {"n": i + 1, "bin": "left" if a == "l" else "right",
                                                  "expected": expect[i] if i < len(expect) else None})
                print(f"  cube {i+1}: {a.upper()} ({ok})   frames: {rec.counts.get('camera/perception', 0)}")
            elif a:
                print("  l, r or q")
    except (KeyboardInterrupt, EOFError):
        pass
    rec.write_event("train/end", {"landed": "".join(landed)})
    rec.stop()

    print("\nsaved", out)
    print("  expected:", expect)
    if landed:
        right = sum(1 for i, b in enumerate(landed) if i < len(expect) and expect[i] == b)
        print(f"  landed:   {''.join(landed)}   -> {right}/{len(expect)} in the right bin")
    print("  messages:", ", ".join(f"{k} {v}" for k, v in sorted(rec.counts.items())))


if __name__ == "__main__":
    main()
