# PRISM night report - 2026-11-12

You left the Pi on and asked me to make the optimizer better. I did **not move the robot** (arm, belt, feeder) while
nobody was there - that is your rule, and an arm moving alone is the one thing that can hurt someone. Everything else
below was done on the computer: code, simulator tests, and measurements from your recordings and the live camera.

## 1. The short version

| | |
|---|---|
| **pt1** | saved in `versions/pt1/` (flow mode as of 01:00). One command goes back to it (see its README). |
| **pt2** | every push of the feeder is **checked before the belt moves** (servo current + camera + feeder IR). Ready to run. |
| **Last run (03:55)** | the optimizer said "feeder empty", but the real problem was the **feeder servo not moving**. Fixed: it now says so and gives the ALARM (arm shake 3 times). |
| **New camera** | measured its geometry (feeder exit, mm per pixel, IR position) and wrote a **proposal** - not active until you OK it. |
| **Hardware** | the **feeder servo** moves only sometimes (and weakly). The **12 V rail is off now** (2.2 V). |
| **Tests** | see section 7. |

## 2. Hardware - please look at these first

1. **Feeder servo (most important).** In the 03:55 run most pushes drew **no current at all** (12 V dip 1-2 mV) while
   the feeder IR saw cubes. The one push that worked drew **53 mV** - a good push draws ~200 mV. The 12 V rail was
   healthy during the run (12.05-12.16 V), so the problem is the servo's own **5 V supply or its D12 signal plug**.
   Feeder and arm share one supply: a **separate 5 V supply for the feeder servo** is the real fix.
2. **12 V rail now reads 2.2 V** (since ~04:00; during all runs it was fine). Probably switched off when you left -
   switch it on before the next run.
3. **Pick IR reads "blocked"** with no cube visible on the camera. Maybe a cube sits at the IR (the arm hides that
   spot from the camera), or the IR reads wrong while the 12 V is off. Check it is clear before starting.

## 3. What pt2 does (your flow chart)

After **every** push the optimizer looks at three things before the belt may move the cube:

| servo current (12 V dip) | camera: new block at the feeder exit (left edge) | decision |
|---|---|---|
| any | **yes** | **FED** - the belt runs for it |
| moved | no | **nudge** the belt 1 s, look again; still nothing -> **EMPTY**, push again |
| **did not move** | no | **EMPTY**, push again at once; **3 in a row -> ALARM**: arm shakes 3 times, red banner |
| no usable camera | - | like pt1: servo moved = fed (a late cube is caught later) |

- The belt does **not** run for pushes that brought nothing (only 1 s nudges). In the simulator: 7 empty pushes cost
  6.5 s of belt in total; before, each one was a full ride (~9 s).
- The next push waits until the last cube has **left the exit** (camera), or has travelled a third of the ride
  (= your "3 s of belt" at 20 rpm).
- The feeder exit position is **learned** from the confirmed cubes. A blob that never moves (glare) never counts.
- The **feeder IR is only recorded** next to each verdict, as we agreed - later we can see if it is worth trusting.
- **No fixed 1 s pause** between picks any more.
- The dashboard's **Optimizer block now lists the last pushes** with their evidence, e.g.
  `FED  feed 12 during GRIP: servo moved (205 mV), camera: new block at 70 px after 0.6 s, feeder IR cubes->empty->cubes`.

## 4. What went wrong in the 03:55 run, and the fixes

| What you saw | Why | Fixed |
|---|---|---|
| "feeder empty" | most pushes drew no current, but in a fresh run the current check is not "proven" yet, so they counted as "unknown" | **no current + no new block on the camera = the servo did not move** -> ALARM + arm shake, even early in a run (an empty feeder still moves the servo, ~200 mV) |
| it kept pushing while pausing | autoen's own fallback feed pushed every 4 s | autoen feeds by itself **only when no optimizer has spoken for 30 s**, never while a pause is coming |
| "hands off" with no reason | a cube arrived just as its own pause landed; it took its own pause for yours | it **takes its own pause back**; and when someone else pauses, it says so and why it only watches |

## 5. The new camera (Astro HD) - geometry proposal, needs your OK

Until now the optimizer used the **old RealSense picture**: belt right -> left, IR at 245 px, feeder at 600 px,
0.95 mm/px. With the new camera (left -> right) the scene rules (backlog, stuck cube) and the **colour queue for
sorting** saw **no cube at all** - so sorting (white -> right bin, orange -> left bin) could not work.

- **Code (done, behaviour unchanged):** `orchestrator/camgeo.py` - all positions are now "pixels before the IR", any
  camera direction. Without a geometry file it uses the old numbers exactly (all old tests pass).
- **Numbers (proposal, NOT active):** `orchestrator/camera_geometry.proposed.json`

| | old (RealSense) | proposed (Astro) | measured from |
|---|---|---|---|
| belt direction | right -> left | **left -> right** | your calibration |
| feeder exit | 600 px | **65 px** | new cubes first seen at 60-85 px (01:07 run), 63-74 px (03:55 run) |
| mm per pixel | 0.95 | **0.64** | cube width 62 px for 40 mm (0.645) and steady belt speed 88 px/s at 56.2 mm/s (0.639) - two methods agree |
| pick IR | 245 px | **805 px** | one sighting 812 px, and ride 8.37 s x 56.2 mm/s from the exit = 799 px. The arm at Standby hides this spot, so **confirm it** (below) |

`MM_PER_PX` is on your do-not-touch list - so nothing changes until you say OK.
**To confirm the IR (1 minute, nothing moves):** put one cube at the pick IR (IR reads blocked), then
`cd ~/Desktop/PRISM/orchestrator && ~/orch-env/bin/python cal/check_camera_geometry.py`.
**To switch it on after your OK:** rename `camera_geometry.proposed.json` to `camera_geometry.json` and restart the
optimizer (`./stop_cell.sh && ./start_cell.sh`). Then sorting, stuck-cube, backlog and slip work with the new camera.

## 6. Numbers from your flow run at 01:07 (pt1)

- **32 picks at ~11 s each** (serial was 18-20 s). It stopped only when the feeder was empty.
- 40 pushes, 31 delivered; almost all pushes were during **GRIP** (26 of 30 delivered there) - that was the
  "repeating pattern": it found the moment that works and kept it.
- One pick: grip 1.7 + lift 1.0 + carry 2.4 + release 1.4 + home 2.3 + standby 0.8 = **9.6 s of arm work**, plus
  1.4 s waiting for the cube and the 1.0 s fixed pause (now removed). **The arm itself is the limit**: flow cannot go
  below ~9.6 s per cube; faster needs a faster arm path or the feeder on its own supply (pushes at any moment).

## 7. Tests

**All 27 test suites pass** on the final code (simulator: the real autoen against a fake Mega, the real optimizer,
fake camera; nothing on the robot). Highlights:

| test | result |
|---|---|
| pt2 normal | 8 of 8 pushes confirmed by the camera before the belt moved them; exit learned; a still glare blob never counted |
| pt2 empty feeder | servo moved, nothing came out -> nudge, look again, EMPTY; 3 in a row -> pause; belt ran only 6.3 s for them |
| pt2 dead servo (the 03:55 case) | no current + nothing on the camera -> pushed again, 3rd -> **ALARM** (not "feeder empty") |
| flow speed | 3.0 s per cube vs 5.8 s serial on the same fake hardware (48 % less) |
| safety | no cube ever reached the IR before the arm was parked |
| camera direction | with the new camera's numbers: cubes listed in feeder order, backlog seen; old numbers: nothing seen (the bug) |
| everything older | serial, pipeline, rest/wake, sorting, stability, experiments: unchanged |

New tests tonight: `test_verify_sim` (pt2: normal / empty / dead servo, including the 03:55 case),
`test_camgeo` (new camera direction), `test_flow_sim`, `test_late_sim`.

## 8. When you are back - in this order

1. **Power:** switch the 12 V on. Check the feeder servo's **D12 plug and 5 V supply**. Belt empty, pick IR clear.
2. **Start pt2:** `cd ~/Desktop/PRISM && ./stop_cell.sh --all && ./start_cell.sh` - it says "Cycle: FLOW".
   (or start from the autoen page: flow is its default now)
3. **Watch:** after each push the cube waits at the left edge until confirmed (< 1 s), then rides. The dashboard's
   Optimizer block lists every push with its evidence.
4. **Camera geometry:** run the check in section 5 with one cube at the IR, tell me the numbers or just "OK" ->
   I switch the proposal on and colour sorting works.
5. Not happy? **Back to pt1:** see `versions/pt1/README.md`.

## 9. Files changed tonight (old copies in `backup/<path>.before-<reason>`)

- `autoencoder/auto_en_new.py` - pt2 verdicts (`/api/feed_verdict`), belt only for confirmed pushes, no fixed pause in
  flow, own feeds only without an optimizer (`before-pt2`, `before-pt2fix`)
- `orchestrator/optimize.py` - pt2 verifier, servo-dead diagnosis, takes its own pause back, clear "hands off",
  push evidence for the dashboard (`before-pt2`, `before-pt2fast`, `before-pt2fix`)
- `orchestrator/actuate.py` - sends verdicts at once (`before-pt2`)
- `orchestrator/camgeo.py` (new), `scene.py`, `sortq.py`, `predict.py` - camera-direction-free geometry (`before-camgeo`)
- `orchestrator/camera_geometry.proposed.json` (new, not active), `orchestrator/cal/check_camera_geometry.py` (new)
- `dashboard/overview.py`, `dashboard/web/p_overview.js`, `app.css` - last pushes in the Optimizer block
- `vision/vision_usb.py` - takes the Astro HD camera first (`before-astrofind`)
- `docs/HOW_TO_USE_PRISM.md` - sections 4d (flow) and 4e (pt2)
