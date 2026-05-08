# Robot Follower Final Code

This folder is the final working project.

## What is here

- `robot_follower_final.ino`: Arduino/Pico robot follower firmware.
- `analyze.py`: Serial capture and plotting script.
- `data/`: CSV logs from test runs.
- `plots/`: Generated PNG plots from those CSV logs.

## Current status

This version is newer than the archived `../archive/robot_follower_older_version` folder and the `Ki4` pre-final milestone copy. It includes the later PID tuning and the analysis fix that normalizes plot time to the start of each capture.

## Typical workflow

1. Flash the `.ino` file to the robot.
2. Capture a run:

   ```powershell
   python analyze.py --port COM3 --test steady_state --dur 20
   ```

3. Replay an existing CSV:

   ```powershell
   python analyze.py --csv data\steady_state_20260427_083855.csv --test steady_state_replot
   ```
