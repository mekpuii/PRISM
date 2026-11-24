# automation - the plain cycle, nothing smart

FEED -> belt runs to the IR -> arm picks -> carries and places (alternating left / down) -> HOME -> next FEED.
One cube at a time. This is the original "riff-real" cycle, run through `autoencoder/auto_en_new.py` (so it uses today's
arm poses, calibration and firmware protocol) with everything smart switched off:
no optimizer, no vision, no dashboard, no MQTT, no colour sorting, no grip check or recovery, no emergency grip, no feeder
rest, no arrival guard, no sleep. Only autoen's own safety limits (servo temperature, belt watchdog) stay.

    cd ~/Desktop/PRISM
    ./stop_cell.sh --all                # if PRISM is running
    ./automation/run.sh                 # 100 cubes, belt 20 rpm, feeder fd1
    ./automation/run.sh 50 fd2          # 50 cubes, feeder fd2
    ./automation/stop.sh                # stop

Belt empty, cubes in the feeder, hand near the E-stop. Press Enter when asked. autoen's page: http://127.0.0.1:8781.
Data goes to `automation/riff_logs/` (raw 5 Hz samples, cycles, commands); autoen's console log to `automation/logs/`.
Tested in the simulator (4 cubes, 4 x OK, phase order as above); not yet run on the real cell.

## Feed after place (default since 2026-11-13)
`run.sh` now feeds the next cube the moment the arm has released the placed one; the belt brings it while the arm goes
Home -> Standby (autoen `--feed-after-place 1`, which uses the pipelined cycle; the arrival guard stays on so a cube
cannot reach the IR before the arm is parked). `FAP=0 ./automation/run.sh` gives the old plain cycle (feed only after
HOME and CLEAR). Simulator: 4 cubes, 4 x OK, every FEED after the release. Not run on the real cell.

## Feeder status line
`run.sh` ends in a live line, refreshed every second: `FEEDER OK | link 0.1 s old | blade idle | cube at exit | pushes
answered 7 | running TRANSPORT cube 7/100 | rail 12.1 V`. It turns red with a beep (FEEDER LOST) when the Mega's telemetry is
older than 1.5 s, autoen does not answer, or the belt board reports a fault; yellow (FEEDER ??) when no push was answered for
90 s during a run. Open it any time in another terminal: `python3 ~/Desktop/PRISM/automation/status.py`.
Limit: the feeder servo has no feedback wire. "Pushes answered" means the Mega ran the push; the proof that the blade really
moved is the cube at the exit and the next pick. autoen's `/api/state` now carries `feeder_link` for this.
