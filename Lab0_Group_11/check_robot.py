import math
import sys
import threading
from pathlib import Path

try:
    import numpy as np
    from roboticstoolbox import Robot
    import swift
    import spatialgeometry as sg
except Exception as e:
    print("[FAIL] import error:", e)
    print('Fix:  pip install "numpy<2" "roboticstoolbox-python[swift]"')
    sys.exit(1)

HERE = Path(__file__).resolve().parent
LAUNCH_TIMEOUT_S = 20

if not (HERE / "my_robot" / "robot.urdf").exists():
    print("[FAIL] my_robot/robot.urdf not found - put your exported URDF there")
    sys.exit(1)

links, name, _, _ = Robot.URDF_read("my_robot/robot.urdf", tld=HERE.as_posix())
robot = Robot(links, name=name)
print(f"[ok] loaded '{name}'  ({robot.n} joints)")

missing = [g.filename for link in robot.links for g in link.geometry
           if getattr(g, "filename", None) and not Path(g.filename).exists()]
if missing:
    print("[FAIL] these mesh files were not found - fix the paths or add the files:")
    for m in missing:
        print("   ", m)
    sys.exit(1)
print("[ok] all mesh files found")


def build_sliders(robot):
    q = np.zeros(robot.n)
    sliders = []

    def make_cb(idx, is_revolute):
        def cb(value):
            v = float(value)
            q[idx] = math.radians(v) if is_revolute else v
            robot.q = q
        return cb

    idx = 0
    for link in robot.links:
        if not link.isjoint:
            continue
        is_rev = link.v.axis.startswith("R")
        lo, hi = link.qlim if link.qlim is not None else (-math.pi, math.pi)
        if is_rev:
            lo, hi = math.degrees(lo), math.degrees(hi)
        lo, hi = round(lo, 2), round(hi, 2)

        unit = "deg" if is_rev else "m"
        rp = "R" if is_rev else "P"
        desc = f"{link.name} ({rp}) [{lo:.2f}..{hi:.2f} {unit}]"

        slider = swift.Slider(make_cb(idx, is_rev), min=lo, max=hi, step=0.01, value=0.0, desc=desc, unit=unit)
        sliders.append(slider)
        idx += 1

    return q, sliders


env = swift.Swift()
launch_error = {}


def do_launch():
    try:
        env.launch(realtime=True)
    except Exception as e:
        launch_error["e"] = e


t = threading.Thread(target=do_launch, daemon=True)
t.start()
t.join(timeout=LAUNCH_TIMEOUT_S)

if t.is_alive():
    print(f"[FAIL] Swift did not connect to a browser within {LAUNCH_TIMEOUT_S}s")
    sys.exit(1)
if "e" in launch_error:
    print(f"[FAIL] Swift failed to launch: {launch_error['e']}")
    sys.exit(1)

env.add(robot, robot_alpha=0.75)

frame_axes = [sg.Axes(length=0.15) for _ in robot.fkine_all(np.zeros(robot.n))]
for ax in frame_axes:
    env.add(ax)

q, sliders = build_sliders(robot)
for slider in sliders:
    env.add(slider)


def reset_cb(_):
    q[:] = 0.0
    robot.q = q
    for slider in sliders:
        slider.value = 0.0


env.add(swift.Button(reset_cb, desc="Reset to home (q=0)"))

print("[ok] your robot is shown in the browser with joint sliders - press Ctrl+C here to close")

try:
    while True:
        T_all = robot.fkine_all(q)
        for ax, T in zip(frame_axes, T_all):
            ax.T = T.A
        env.step(0.05)
except KeyboardInterrupt:
    pass
finally:
    try:
        env.close()
    except Exception:
        pass
    print("closed.")
