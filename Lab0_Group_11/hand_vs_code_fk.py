# -*- coding: utf-8 -*-
"""
ทดสอบ Forward Kinematics: คำนวณมือ (closed-form) VS โค้ดจริง (roboticstoolbox โหลดจาก URDF)
ใช้หุ่นของกลุ่มเรา (my_robot/robot.urdf) — รันไฟล์นี้เพื่อฝึกก่อนสอบ

โครงหุ่น: q1 (revolute, Z) -> q2 (prismatic, Z, offset 0.235) -> q3 (prismatic, X ของโลกที่หมุนตาม q1, offset 0.09+0.475)

หมายเหตุ Windows: ถ้าเจอ UnicodeEncodeError ให้รันด้วย:
    $env:PYTHONIOENCODING="utf-8"; python hand_vs_code_fk.py
"""
import sys
import numpy as np
from pathlib import Path
from roboticstoolbox import Robot

if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8")

HERE = Path(__file__).resolve().parent


def hand_fk(q1, q2, q3):
    """คำนวณมือแบบ closed-form (DH table ที่ derive ไว้แล้ว)"""
    L = 0.09 + q3 + 0.475          # ระยะรวมตามแนวแกนที่ q3 เลื่อน
    x = L * np.cos(q1)
    y = L * np.sin(q1)
    z = 0.235 + q2
    return np.array([x, y, z])


def hand_fk_dh_matrix(q1, q2, q3):
    """เวอร์ชันคูณ DH matrix เต็ม (Tx(a)Rx(alpha)Tz(d)Rz(theta) ทีละ joint) เพื่อดูขั้นตอนจริง"""
    def Rx(t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

    def Rz(t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def Tx(a):
        M = np.eye(4)
        M[0, 3] = a
        return M

    def Tz(d):
        M = np.eye(4)
        M[2, 3] = d
        return M

    def DH(a, alpha, d, theta):
        return Tx(a) @ Rx(alpha) @ Tz(d) @ Rz(theta)

    # DH table ของหุ่นเรา:
    #  i-1->i |  a              | alpha | d          | theta
    #  0 -> 1 |  0              | 0     | 0          | q1
    #  1 -> 2 |  0              | 0     | 0.235+q2   | 0
    #  2 -> 3 |  0              | pi/2  | 0          | 0
    #  3 -> e |  0.09+q3+0.475  | 0     | 0          | 0
    T01 = DH(a=0, alpha=0, d=0, theta=q1)
    T12 = DH(a=0, alpha=0, d=0.235 + q2, theta=0)
    T23 = DH(a=0, alpha=np.pi / 2, d=0, theta=0)
    T3e = DH(a=0.09 + q3 + 0.475, alpha=0, d=0, theta=0)
    return T01 @ T12 @ T23 @ T3e


def main():
    links, name, _, _ = Robot.URDF_read("my_robot/robot.urdf", tld=HERE.as_posix())
    robot = Robot(links, name=name)
    print(f"[ok] loaded '{name}' ({robot.n} joints)")
    print()

    test_cases = [
        (0.0, 0.0, 0.0),
        (np.deg2rad(30), 0.1, 0.05),
        (np.deg2rad(90), 0.3, 0.2),
        (np.deg2rad(-45), 0.5, 0.4),
        (np.deg2rad(180), 0.2, 0.1),
    ]

    print(f"{'q1(deg)':>8} {'q2(m)':>7} {'q3(m)':>7} | {'hand x,y,z':^28} | {'code x,y,z':^28} | diff")
    print("-" * 95)
    for q1, q2, q3 in test_cases:
        pos_hand = hand_fk(q1, q2, q3)
        pos_hand_matrix = hand_fk_dh_matrix(q1, q2, q3)[:3, 3]
        T_code = robot.fkine([q1, q2, q3])
        pos_code = T_code.t

        diff = np.linalg.norm(pos_hand - pos_code)
        print(f"{np.rad2deg(q1):8.1f} {q2:7.3f} {q3:7.3f} | "
              f"{np.round(pos_hand, 4)!s:^28} | {np.round(pos_code, 4)!s:^28} | {diff:.6f}")

        assert np.allclose(pos_hand, pos_hand_matrix, atol=1e-9), "closed-form กับ DH-matrix ไม่ตรงกัน!"
        assert np.allclose(pos_hand, pos_code, atol=1e-6), "คำนวณมือกับโค้ดไม่ตรงกัน!"

    print()
    print("[PASS] คำนวณมือ (closed-form), คำนวณมือ (DH matrix), และโค้ดจริง (URDF) ตรงกันทุกกรณี")


if __name__ == "__main__":
    main()
