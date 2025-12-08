# Analysis Tools and Notebooks

Use this directory for offline analysis, calibration, and data quality checks. Python utilities can live alongside Jupyter notebooks under `notebooks/`.

Recommended workflow:
1. Copy or mount rosbags into `../bags/`.
2. Use `../tools/fix_timestamps.py` or other utilities to clean data.
3. Explore datasets in the notebooks, keeping large outputs out of version control.
