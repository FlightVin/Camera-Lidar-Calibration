# 3D Lidar Camera Calibration

| S. No. | File/Directory Names | Description |
| :----- | :--------- | :------ |
| 1 | cam_lidar_annotate.py | Script to visualize the 2D image and 3D pointcloud
| 2 | cam_lidar_calib.py | Script that calibrates and generates extrinsics based on annotated data
| 3 | save_3d_points.ipynb | Saves annotations in `./src/lidar_camera_calibration/calibration_data/lidar_camera_calibration`
| 4 | src | Contains bagfiles, images, annotation and extrinsics (see below)

## Contents of `./src/lidar_camera_calibration/`
 
| S. No. | File/Directory Names | Description |
| :----- | :--------- | :------ |
| 1 | bagfiles | folder containing collected data from simulations
| 2 | calibration_data | contains camera images, annotations and extrinsics
| 3 | cam_calib.yaml | YAML file containing intrinsics from Zhang's method
| 4 | Everything else | Deprecated
