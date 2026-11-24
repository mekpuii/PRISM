PRISM Python environment
========================
Environment : ~/prism-env   (Python 3.11)
Packages    : see requirements.txt  (numpy, opencv-python, paho-mqtt)
RealSense   : pyrealsense2 is linked from ~/librealsense/build/Release
              via prism-env/lib/python3.11/site-packages/realsense-build.pth

Quick start (in any new terminal):
    prism                 # activates env + cd into ~/Desktop/PRISM
    cd DCAM && python 3mins.py

Install a new package (ALWAYS inside the env, never --break-system-packages):
    prism
    pip install <package>
    pip freeze > ~/Desktop/PRISM/requirements.txt

Rebuild env from scratch:
    /usr/local/bin/python3.11 -m venv ~/prism-env
    ~/prism-env/bin/pip install -r ~/Desktop/PRISM/requirements.txt
    echo /home/mekkhram/librealsense/build/Release > ~/prism-env/lib/python3.11/site-packages/realsense-build.pth

Autoencoder / arm node environment
==================================
Environment : ~/autoen-env   (Python 3.11)
Packages    : see autoencoder/requirements.txt  (pyserial, evdev, paho-mqtt)
Quick start :
    autoen                # activates autoen-env + cd into ~/Desktop/PRISM/autoencoder
    python arm_node.py    # needs the Mega on /dev/ttyACM0 and a keyboard on the Pi
Rebuild     :
    /usr/local/bin/python3.11 -m venv ~/autoen-env
    ~/autoen-env/bin/pip install -r ~/Desktop/PRISM/autoencoder/requirements.txt

Logger environment
==================
Environment : ~/logger-env   (Python 3.11)
Packages    : see logger/requirements.txt
Quick start :
    logger-env            # activates logger-env + cd into ~/Desktop/PRISM/logger
    python arm_logger.py
Rebuild     :
    /usr/local/bin/python3.11 -m venv ~/logger-env
    ~/logger-env/bin/pip install -r ~/Desktop/PRISM/logger/requirements.txt

Health environment
==================
Environment : ~/health-env   (Python 3.11)
Packages    : see health/requirements.txt
Quick start :
    health-env            # activates health-env + cd into ~/Desktop/PRISM/health
    python health.py
Rebuild     :
    /usr/local/bin/python3.11 -m venv ~/health-env
    ~/health-env/bin/pip install -r ~/Desktop/PRISM/health/requirements.txt

Analysis environment
====================
Environment : ~/ana-env   (Python 3.11)
Packages    : see analysis/requirements.txt
Quick start :
    ana-env               # activates ana-env + cd into ~/Desktop/PRISM/analysis
    python conv_ana.py
Rebuild     :
    /usr/local/bin/python3.11 -m venv ~/ana-env
    ~/ana-env/bin/pip install -r ~/Desktop/PRISM/analysis/requirements.txt

Dashboard environment
=====================
Environment : ~/db-env   (Python 3.11)
Packages    : see dashboard/requirements.txt - ALL-IN-ONE: has every package of prism-env, autoen-env,
              logger-env, health-env and ana-env (numpy, opencv, pyrealsense2 link, evdev, pyserial, paho-mqtt),
              so every PRISM script runs in this one env. The other envs still work on their own.
autoen_node : sim_cell.py imports the real autoencoder/autoen_node.py through
              db-env/lib/python3.11/site-packages/prism-autoencoder.pth
Quick start :
    db-env                # activates db-env + cd into ~/Desktop/PRISM/dashboard
    python dashboard_node.py --data-dir data --log-dir ~/Desktop/PRISM/autoencoder/riff_logs   # page :8080
              (dashboard/data/ holds symlinks to logger/*.db and analysis/*_analyzer.db)
    python sim_cell.py --fast 2                             # real autoen_node + fake Mega, page :8781
    python fake_camera.py                                   # fake camera/* messages
Rebuild     :
    /usr/local/bin/python3.11 -m venv ~/db-env
    ~/db-env/bin/pip install -r ~/Desktop/PRISM/dashboard/requirements.txt
    echo /home/mekkhram/librealsense/build/Release > ~/db-env/lib/python3.11/site-packages/realsense-build.pth
    echo /home/mekkhram/Desktop/PRISM/autoencoder > ~/db-env/lib/python3.11/site-packages/prism-autoencoder.pth

Orchestrator environment
=========================
Environment : ~/orch-env   (Python 3.11)
Packages    : see orchestrator/requirements.txt (paho-mqtt only, same as observe.py's imports)
Quick start :
    orch-env              # activates orch-env + cd into ~/Desktop/PRISM/orchestrator
    python main.py
Rebuild     :
    /usr/local/bin/python3.11 -m venv ~/orch-env
    ~/orch-env/bin/pip install -r ~/Desktop/PRISM/orchestrator/requirements.txt
