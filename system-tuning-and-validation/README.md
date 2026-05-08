# Milestone Robot Follower Files

This directory is organized as:

- `robot_follower_final/`: final firmware, analyzer, CSV data, and plots. Use this for demos/report work.
- `archive/firmware_versions/code_milestone_debug_prints_no_csv/`: original `code_milestone` sketch. It uses human-readable serial debug output and does not produce CSV metrics.
- `archive/firmware_versions/code_milestone_csv_metrics_Ki4_prefinal/`: `code_milestone_copy_20260427082409`. It adds CSV metrics and is almost the final firmware, but uses `Ki_dist = 4.0`.
- `archive/robot_follower_older_version/`: earlier `robot_follower` project folder with its own firmware, analyzer, data, and plots.
- `archive/empty_top_level_data/` and `archive/empty_top_level_plots/`: old empty folders moved out of the way.
- `test_cases.txt`: test plan for collecting report data.

Use `robot_follower_final/robot_follower_final.ino` as the Arduino/Pico sketch.
Run `analyze.py` from inside `robot_follower_final/` so new CSVs go into its `data/` folder and plots go into its `plots/` folder.