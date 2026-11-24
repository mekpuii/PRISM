# limits.py - every limit that makes PRISM rest, in ONE place (2026-11-12).
# The optimizer (optimize.py) acts on these numbers and the dashboard's Overview ("Limits" zone) shows the same ones,
# so the page can never show a limit the machine does not use. Pure constants: nothing is imported, nothing runs.
#
#   warm  = gentler settings (the machine keeps working)      hot / limit = REST (COOL mode) until it is cool again
SERVO_WARM_C, SERVO_HOT_C = 55.0, 65.0   # each arm servo J1-J6 (STS3215): temperature in the servo (A1 peak: 39 C)
BELT_WARM_C, BELT_HOT_C = 50.0, 57.0     # belt motor (DS18B20); the firmware itself faults at TEMPMAX 60 C
PI_WARM_C, PI_HOT_C = 70.0, 80.0         # Raspberry Pi SoC (the Pi slows itself down above ~80 C)
COOL_HYST_C = 5.0                        # back to normal only this far below the WARM limit
COOL_MIN_S = 30.0                        # a rest lasts at least this long
SERVO_STRAIN_MA = 1500.0                 # a servo drawing more than this ...
SERVO_STRAIN_S = 2.0                     # ... for this long = straining (blocked / overloaded): rest
RAIL_OK_V = 11.5                         # 12 V rail: below this the feeder does not push (it waits)
RAIL_BAD_V = 10.5                        # below this the supply is collapsing
FEEDER_MAX_PER_MIN = 12                  # feeder servo (no temperature sensor): more pushes a minute -> it rests
FEEDER_REST_S = 45.0                     # ... this long (autoen --feeder-max-per-min / --feeder-rest-s mirror these)
SLEEP_AFTER_S = 300.0                    # empty feeder this long -> SLEEP (arm rests, power off)
WAKE_EVERY_S = 600.0                     # asleep: wake and check this often

JOINT_NAME = {"1": "J1 base", "2": "J2 shoulder", "3": "J3 elbow", "4": "J4 wrist flex", "5": "J5 wrist roll",
              "6": "J6 gripper"}

# ---- FAULT system (guardian.py, 2026-11-13): when something is broken the cell PAUSES at a checkpoint, never shuts down.
# The Raspberry Pi, the dashboard and the logging keep running whatever happens.
FAULT_SHAKE_SETS = 3                     # warning: 3 sets of ...
FAULT_SHAKE_TIMES = 3                    # ... 3 wrist shakes ...
FAULT_SHAKE_GAP_S = 60.0                 # ... one minute apart
FAULT_RETRY_FAST_S = 30.0                # after the checkpoint: try to open / send the signal again every 30 s ...
FAULT_RETRY_FAST_FOR_S = 300.0           # ... for the first 5 minutes ...
FAULT_RETRY_SLOW_S = 120.0               # ... then every 2 minutes
FAULT_REPEAT_MAX = 3                     # the same fault 3 times in ...
FAULT_REPEAT_WINDOW_S = 600.0            # ... 10 minutes: stop resuming by itself, wait for the operator
FAULT_BELT_ROLL_RPM = 8.0                # after a feeder retry push the belt rolls a little ...
FAULT_BELT_ROLL_S = 1.5                  # ... so cubes do not stack or stick at the exit
CAMERA_LOST_S = 5.0                      # no camera frames this long (after it was seen) = camera lost
COVER_V_MEAN = 18.0                      # belt brightness (0-255) below this ...
COVER_DARK_PCT = 92.0                    # ... or this percent of too-dark belt pixels ...
COVER_S = 2.0                            # ... for this long = camera covered / lens blocked
OBJECT_PCT = 8.0                         # something bigger than this percent of the belt area, different from the empty-
OBJECT_S = 1.0                           # workspace reference photo, for this long = a hand / an object in the workspace
OBJECT_CLEAR_S = 3.0                     # ... and it must be gone this long before the cell carries on
LINK_LOST_S = 3.0                        # no telemetry from the Mega (belt/feeder) or the arm servos this long = link lost
CLEAR_S = 10.0                           # a fault must be gone this long before the cell carries on by itself
