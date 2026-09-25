# FRA371_Kinematics_Workshop_0

![Course](https://img.shields.io/badge/Course-FRA371-blue)
![Workshop](https://img.shields.io/badge/Workshop-0-green)
![Jupyter](https://img.shields.io/badge/Jupyter-Notebook-F37626?logo=jupyter&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)

## Robot Setup

Add your own robot to this project and confirm it loads.

### 1. Move your robot into `my_robot/`

Your robot comes from a **URDF exporter** (your CAD tool): one `.urdf` file and a set
of `.stl` meshes.

1. Put your mesh files in `my_robot/meshes/`.
2. Save your URDF as `my_robot/robot.urdf`.

### 2. Re-path the meshes

In `my_robot/robot.urdf`, every mesh path **must** look like this:

```xml
<mesh filename="package://my_robot/meshes/YOUR_FILE.stl"/>
```

## Files

- **`check_robot.py`** — `python check_robot.py`, no args. Opens your URDF in Swift with joint sliders.
- **`practice.ipynb`** — RTB practice notebook, forward kinematics basics.
