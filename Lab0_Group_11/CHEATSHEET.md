# FRA371 Kinematics — Cheat Sheet ฉบับเต็ม (อ่านคนเดียวได้ ไม่ต้องมีเน็ต)

สารบัญ:
1. คำศัพท์พื้นฐาน (ต้องรู้ก่อนอ่านต่อ)
2. แปลงหน่วยองศา ↔ เรเดียน
3. Import ที่ต้องใช้ทุกครั้ง
4. สร้างหุ่นยนต์จาก DH Table (RevoluteMDH / PrismaticMDH)
5. Forward Kinematics — คำนวณตำแหน่งปลายมือ
6. อ่านผลลัพธ์ T (SE3 object) ยังไง
7. โหลดหุ่นยนต์จากไฟล์ URDF (CAD export)
8. แสดงผลหุ่นยนต์ใน browser ด้วย Swift
9. URDF convention: origin/rpy ทำงานยังไง (สำคัญมาก ถ้าโจทย์ให้แก้แกน joint)
10. รายการ Error ที่เจอบ่อย + วิธีแก้ทันที
11. เช็คลิสต์ก่อนส่งงาน

---

## 1. คำศัพท์พื้นฐาน

| คำ | ความหมาย |
|---|---|
| **Joint** | ข้อต่อของหุ่นยนต์ 1 จุด เช่น ข้อศอก, เพลาเลื่อน |
| **Link** | ท่อนแขนแข็ง 1 ท่อนที่เชื่อมระหว่าง joint สองจุด |
| **Revolute joint** | ข้อต่อแบบ**หมุน** (เหมือนบานพับ) ตัวแปรคือ "มุม" (หน่วยเรเดียน) |
| **Prismatic joint** | ข้อต่อแบบ**เลื่อนตรง** (เหมือนลูกสูบ) ตัวแปรคือ "ระยะทาง" (หน่วยเมตร) |
| **q** | ตัวแปรของ joint ทั้งหมด รวมกันเป็น array เช่น `q = [q1, q2, q3]` |
| **DH Parameters** | ชุดตัวเลข 4 ค่า (`a`, `alpha`, `d`, `theta`) ที่ใช้อธิบายว่า link แต่ละท่อนวางตัวยังไงเทียบกับท่อนก่อนหน้า |
| **Forward Kinematics (FK)** | คำนวณ "ตำแหน่ง + ทิศทาง ของปลายมือ" จากค่า joint (q) ที่รู้อยู่แล้ว — เป็นทิศทางเดียวที่ lab นี้เน้น |
| **Inverse Kinematics (IK)** | ทิศทางกลับกัน (รู้ตำแหน่งปลายมือ ต้องการหาค่า q) — lab นี้ไม่เน้นส่วนนี้ |
| **End-effector** | จุดปลายสุดของหุ่นยนต์ (มือจับ/เครื่องมือ) ที่เราสนใจตำแหน่งของมัน |
| **SE3** | ชนิดข้อมูลที่เก็บ "ตำแหน่ง + การหมุน" พร้อมกันในรูป matrix ขนาด 4x4 |
| **URDF** | ไฟล์ `.urdf` (XML) ที่อธิบายโครงหุ่นยนต์ทั้งหมด ปกติ export มาจาก SolidWorks/CAD |

---

## 2. แปลงหน่วยองศา ↔ เรเดียน (ต้องทำทุกครั้งที่ใส่มุม)

**กฎเหล็ก: โค้ดทุกอย่างในนี้ต้องการมุมเป็น "เรเดียน" ไม่ใช่ "องศา"**
ถ้าใส่องศาตรง ๆ โค้ดจะไม่ error แต่ตอบผิดแบบเงียบ ๆ (อันตรายที่สุด)

```python
import numpy as np

# องศา -> เรเดียน (ใช้บ่อยที่สุด)
rad = np.deg2rad(90)      # 90 องศา -> 1.5708 rad

# เรเดียน -> องศา (ใช้ตอนอยากอ่านผลลัพธ์เป็นองศา)
deg = np.rad2deg(np.pi/2) # 1.5708 rad -> 90 องศา

# สูตรมือ ถ้าจำ np.deg2rad ไม่ได้:
rad = 90 * np.pi / 180
deg = rad_value * 180 / np.pi
```

ค่ามุมที่ใช้บ่อย (จำไว้เผื่อคำนวณมือ):
| องศา | เรเดียน (ประมาณ) | เรเดียน (แบบ pi) |
|---|---|---|
| 0°   | 0        | 0 |
| 30°  | 0.5236   | π/6 |
| 45°  | 0.7854   | π/4 |
| 60°  | 1.0472   | π/3 |
| 90°  | 1.5708   | π/2 |
| 180° | 3.1416   | π |

---

## 3. Import ที่ต้องใช้ทุกครั้ง

```python
import numpy as np                                  # คำนวณเลข/array
from roboticstoolbox import DHRobot, RevoluteMDH, PrismaticMDH, Robot   # สร้าง/โหลดหุ่นยนต์
from spatialmath import SE3                          # ชนิดข้อมูล transform
import swift                                         # แสดงผล 3D ใน browser
import spatialgeometry as sg                         # วาดแกน/รูปทรงเสริมใน swift
```

ถ้า import แล้วขึ้น `ModuleNotFoundError` → แปลว่าเครื่องนี้ไม่มี library ติดตั้งไว้ (ในห้องสอบไม่มีเน็ตจะลง pip ไม่ได้ ต้องเช็คก่อนเข้าห้องสอบว่าเครื่องพร้อมหรือยัง)

---

## 4. สร้างหุ่นยนต์จาก DH Table

### 4.1 วิธีคิด DH parameter ต่อ 1 ข้อต่อ (Modified DH / Craig convention)

แต่ละ joint ใช้เลข 4 ตัว: `a`, `alpha`, `d`, `theta`

| พารามิเตอร์ | ความหมาย | หน่วย |
|---|---|---|
| `a`     | ความยาวของท่อนแขน**ก่อนหน้า** วัดตามแกน x | เมตร |
| `alpha` | มุมบิดของแกน z ระหว่างท่อนก่อนหน้ากับท่อนนี้ วัดรอบแกน x | เรเดียน |
| `d`     | ระยะเลื่อนตามแกน z ของ joint นี้ | เมตร |
| `theta` | มุมหมุนรอบแกน z ของ joint นี้ | เรเดียน |

**กฎจำง่าย:**
- ถ้า joint เป็น **revolute** (หมุน) → ตัวแปรที่เปลี่ยนคือ `theta` (ไม่ต้องใส่ตอนสร้าง object เพราะโค้ดจะถือว่ามันคือ q) → ใส่แค่ `a, alpha, d` คงที่
- ถ้า joint เป็น **prismatic** (เลื่อน) → ตัวแปรที่เปลี่ยนคือ `d` → ใส่แค่ `a, alpha, theta` คงที่

### 4.2 Syntax จริง

```python
robot = DHRobot([
    RevoluteMDH(a=0.0,  alpha=0.0,  d=0.0),     # joint 1: หมุน, ท่อนก่อนหน้ายาว 0
    RevoluteMDH(a=0.15, alpha=0.0,  d=0.0),     # joint 2: หมุน, ท่อนก่อนหน้ายาว 0.15 m
    PrismaticMDH(a=0.0, alpha=np.pi/2, theta=0.0),  # joint 3: เลื่อน, บิดแกน 90 องศา
], name='my_robot')

# offset คงที่หลัง joint สุดท้าย (เช่น ความยาว gripper/มือจับ)
robot.tool = SE3.Tx(0.20)    # เลื่อนตามแกน x อีก 0.20 m
# SE3.Ty(x), SE3.Tz(x) ก็มี ถ้า offset ไปทาง y หรือ z แทน

print(robot)   # แสดงตาราง DH ทั้งหมด เอาไว้ตรวจว่าใส่ค่าถูกไหม
```

**ตัวอย่างจริงจาก lab (แขน 2 ข้อต่อ แบบระนาบเดียว):**
```python
toy = DHRobot([
    RevoluteMDH(a=0.0,  alpha=0.0, d=0.0),   # ข้อต่อที่ 1 (ฐานหมุน)
    RevoluteMDH(a=0.15, alpha=0.0, d=0.0),   # ข้อต่อที่ 2 (ท่อนแขนยาว 0.15 m)
], name='toy_2link')
toy.tool = SE3.Tx(0.20)   # ปลายมือยื่นอีก 0.20 m
```
ความยาวรวมของแขนเมื่อเหยียดตรง = 0.15 + 0.20 = 0.35 m

---

## 5. Forward Kinematics — คำนวณตำแหน่งปลายมือ

```python
q = np.array([0.0, 0.0])       # ค่า joint ทุกตัว (rad สำหรับ revolute, m สำหรับ prismatic) เรียงตามลำดับที่สร้าง robot
T = robot.fkine(q)             # คำนวณ FK -> ได้ SE3 object

print(T)            # แสดง matrix 4x4 สวยงาม
print(T.t)          # ตำแหน่ง (x, y, z) -> numpy array ขนาด 3
print(np.round(T.t, 4))   # ปัดเศษ 4 ตำแหน่ง จะได้อ่านง่าย
```

**ตัวอย่าง**: แขน 2 ข้อต่อ ที่ q = [0, 0] (เหยียดตรงตามแกน x):
```python
q_zero = np.array([0.0, 0.0])
T = toy.fkine(q_zero)
print('tip :', np.round(T.t, 4))
# ผลลัพธ์: tip : [0.35 0.   0.  ]   <- ตรงกับ 0.15+0.20 = 0.35 ตามที่คำนวณมือ
```

**ตัวอย่าง**: งอข้อต่อทั้งคู่ 90 องศา:
```python
q_bent = np.array([np.deg2rad(90), np.deg2rad(90)])   # ห้ามลืม deg2rad!
T = toy.fkine(q_bent)
print('tip :', np.round(T.t, 4))
```

**เทียบกับคำนวณมือ (สำคัญมากในข้อสอบ):**
```python
hand_calc = np.array([x, y, z])     # ใส่ค่าที่คำนวณมือได้เอง
diff = np.linalg.norm(T.t - hand_calc)
print('diff:', round(float(diff), 4), 'm')   # ถ้า diff ใกล้ 0 แปลว่าคำนวณมือถูก
```

**Loop สวีปค่า joint (ดูว่าปลายมือขยับยังไงเมื่อ joint หมุน):**
```python
for deg in range(0, 91, 30):              # ลอง 0, 30, 60, 90 องศา
    q = np.array([np.deg2rad(deg), np.deg2rad(30)])   # joint1 เปลี่ยน, joint2 คงที่ 30 องศา
    tip = toy.fkine(q).t
    print(f'q1={deg:3d} deg -> tip {np.round(tip, 4)}')
```

---

## 6. อ่านผลลัพธ์ T (SE3 object) ยังไง

`T` ที่ได้จาก `.fkine(q)` คือ matrix ขนาด 4x4:
```
T = [ R11 R12 R13 | x ]
    [ R21 R22 R23 | y ]
    [ R31 R32 R33 | z ]
    [  0   0   0  | 1 ]
```
ส่วนซ้ายบน 3x3 คือการ**หมุน** (rotation), คอลัมน์ขวาสุด 3 แถวคือ**ตำแหน่ง** (x,y,z)

```python
T.t          # -> array([x, y, z])              ตำแหน่งปลายมือ
T.R          # -> matrix 3x3                     ทิศทาง/การหมุนของปลายมือ
T.A          # -> numpy array 4x4 เต็ม (ไว้เอาไปคำนวณต่อเอง)
T.eul()      # -> มุม Euler (ZYZ) หน่วยเรเดียน
T.rpy()      # -> มุม Roll-Pitch-Yaw หน่วยเรเดียน
```

รวมหลาย transform ต่อกัน (คูณ matrix ซ้อนกัน = เชื่อม frame ต่อกัน):
```python
T_total = T1 * T2 * T3      # ใน spatialmath ใช้ * แทนการคูณ matrix (ไม่ใช่ @)
```

---

## 7. โหลดหุ่นยนต์จากไฟล์ URDF (กรณี export จาก SolidWorks/CAD)

### 7.1 โครงสร้างโฟลเดอร์ที่ต้องมี

```
my_robot/
├── robot.urdf              <- ไฟล์หลัก
└── meshes/
    ├── base_link.STL
    ├── Link1.STL
    └── ...
```

ใน `robot.urdf` ทุกจุดที่อ้าง mesh ต้องเขียนแบบนี้เป๊ะ (ผิด case ก็หาไม่เจอ):
```xml
<mesh filename="package://my_robot/meshes/YOUR_FILE.stl"/>
```

### 7.2 โค้ดโหลด

```python
from roboticstoolbox import Robot
from pathlib import Path

HERE = Path('.').resolve()     # ถ้าอยู่ใน .py ใช้ Path(__file__).resolve().parent แทน
links, name, _, _ = Robot.URDF_read("my_robot/robot.urdf", tld=HERE.as_posix())
robot = Robot(links, name=name)

print(f"loaded '{name}' ({robot.n} joints)")
```

### 7.3 เช็คว่า mesh ไฟล์ครบไหม

```python
missing = [g.filename for link in robot.links for g in link.geometry
           if getattr(g, "filename", None) and not Path(g.filename).exists()]
if missing:
    print("หา mesh ไม่เจอ:", missing)
else:
    print("mesh ครบ")
```

### 7.4 ดูว่าแต่ละ joint เป็นแบบไหน แกนอะไร

```python
for link in robot.links:
    if link.isjoint:
        print(link.name, link.v.axis, link.qlim)
```
ผลลัพธ์ที่จะเห็น:
- `'Rz'` = revolute หมุนรอบแกน z (มีได้ `Rx`, `Ry`, `Rz`)
- `'tz'` = prismatic เลื่อนตามแกน z (มีได้ `tx`, `ty`, `tz`)

### 7.5 ทำ FK กับหุ่นที่โหลดจาก URDF — เหมือนกับ DHRobot ทุกอย่าง

```python
q = np.zeros(robot.n)          # ตั้งทุก joint = 0 ก่อน
T = robot.fkine(q)
print(np.round(T.t, 4))
```

---

## 8. แสดงผลหุ่นยนต์ใน browser ด้วย Swift

```python
env = swift.Swift()
env.launch(realtime=True)      # เปิด browser tab ใหม่ แสดงพื้นที่ 3D ว่าง ๆ

env.add(robot, robot_alpha=0.75)   # เอาหุ่นยนต์เข้าไปแสดง (โปร่งใส 75%)

# วาดแกน xyz ที่ทุก frame ของหุ่น (แดง=x, เขียว=y, น้ำเงิน=z)
frame_axes = [sg.Axes(length=0.15) for _ in robot.fkine_all(np.zeros(robot.n))]
for ax in frame_axes:
    env.add(ax)

# ทำ loop อัพเดตภาพเรื่อย ๆ (ต้องมี ไม่งั้นภาพค้าง)
q = np.zeros(robot.n)
while True:
    T_all = robot.fkine_all(q)
    for ax, T in zip(frame_axes, T_all):
        ax.T = T.A
    env.step(0.05)     # เดินหน้าซิมูเลชัน 0.05 วินาที
```

**Slider ปรับค่า joint แบบลากเมาส์:**
```python
def my_callback(value):
    q[0] = np.deg2rad(float(value))   # ถ้าเป็น revolute ต้องแปลง deg->rad เอง
    robot.q = q

slider = swift.Slider(my_callback, min=-180, max=180, step=1, value=0, desc="joint1", unit="deg")
env.add(slider)
```

**ปุ่มกด:**
```python
def reset(_):
    q[:] = 0.0
    robot.q = q

env.add(swift.Button(reset, desc="Reset"))
```

**ปิดโปรแกรม:** กด `Ctrl+C` ใน terminal แล้วเรียก `env.close()`

---

## 9. URDF convention: origin/rpy ทำงานยังไง (สำคัญถ้าโจทย์ให้ปรับแกน joint)

### 9.1 กฎเหล็กที่ต้องจำ

ใน URDF ทุก `<origin xyz="..." rpy="...">` หมายความว่า:

```
T = Translate(xyz)  ·  Rotate(rpy)
```

**ย้ายตำแหน่งก่อน แล้วค่อยหมุนทับทีหลัง** (ไม่ใช่หมุนก่อนย้าย) — สลับลำดับผิดจุดเดียว คำนวณจะเพี้ยนทั้ง chain

`rpy` คือมุมหมุน Roll(x)-Pitch(y)-Yaw(z) หน่วยเรเดียน เช่น หมุนรอบแกน Y +90° เขียนเป็น:
```xml
<origin xyz="0 0 0" rpy="0 1.5707963267948966 0" />
```
(1.5707963267948966 คือ π/2 พิมพ์เต็มเพื่อความแม่นยำ ใช้ `np.pi/2` แทนได้ถ้าคำนวณด้วยโค้ด)

### 9.2 axis ของ joint

```xml
<axis xyz="0 0 1" />
```
บอกว่า joint นี้หมุน/เลื่อนไปตามแกนไหน **ของ local frame หลังจาก apply origin แล้ว** ไม่ใช่แกนของ parent link ตรง ๆ — ถ้า origin มี rpy หมุนไปแล้ว axis ก็อ้างอิงตามแกนที่หมุนไปแล้วนั้น

### 9.3 ถ้าโจทย์ให้เปลี่ยนแกน joint (เช่น จากเลื่อนตาม X เป็นเลื่อนตาม Z) แต่ห้ามให้ชิ้นงานขยับ

ต้องแก้ **4 จุดพร้อมกัน** ไม่งั้นหุ่นจะเพี้ยนหรือ mesh หมุนไปจากเดิม:

1. **origin ของ joint ที่จะแก้** — เพิ่ม `rpy` ให้หมุนแกนเป้าหมายไปตรงกับแกนที่ต้องการ (เช่น `Ry(+90°)` ทำให้แกน X เดิมกลายเป็นแกน Z ใหม่) แล้วเปลี่ยน `axis` ให้ตรงกับแกนใหม่
2. **origin ของ visual/collision ของ child link นั้น** — ต้องหมุนกลับด้วย rotation ผกผัน ไม่งั้น mesh จะหมุนตามไปด้วย:
   ```
   mesh_origin_ใหม่ = Rotate(-θ) · mesh_origin_เดิม
   ```
3. **origin ของ inertial (mass center) ของ child link** — หมุนกลับแบบเดียวกับ mesh:
   ```
   inertial_origin_ใหม่ = Rotate(-θ) · inertial_origin_เดิม
   ```
4. **Inertia tensor** (`ixx, iyy, izz, ixy, ixz, iyz`) — ต้องแปลงเป็น matrix 3x3 แล้วคูณ:
   ```
   I_ใหม่ = M · I_เดิม · Mᵀ
   ```
   โดย `M` คือ rotation matrix ที่แปลงพิกัดเก่า→ใหม่ (ถ้า origin ของ joint หมุนด้วย `Rotate(+θ)`, ตัว `M` ที่ใช้กับ tensor คือ `Rotate(-θ)`)

5. ถ้ามี **joint ถัดไปที่ต่อจาก child link นั้น** (เช่น fixed joint ที่ปลายมือ) ก็ต้อง compensate เหมือนข้อ 2 ไม่งั้นตำแหน่งปลายแขนสุดท้ายจะขยับออกจากตำแหน่งเดิม

### 9.4 วิธี verify ว่าแก้ถูก (ทำทุกครั้งหลังแก้ URDF)

```python
# 1. เทียบ FK ปลายมือ ก่อน/หลังแก้ ต้องตรงกันทุกค่า q ทั้งตำแหน่งและการหมุน
np.allclose(T_new.t, T_old.t, atol=1e-6)   # ตำแหน่งต้องเหมือนเดิม
np.allclose(T_new.R, T_old.R, atol=1e-6)   # ทิศทางต้องเหมือนเดิม

# 2. เทียบ eigenvalues ของ inertia tensor (ค่านี้ไม่เปลี่ยนไม่ว่าจะหมุนแกนยังไง)
np.linalg.eigvalsh(I_matrix)   # ต้อง sort แล้วเทียบ ต้องเท่าเดิมทุกตัว

# 3. เช็คว่า inertia tensor ยังถูกต้องทางฟิสิกส์
eigvals > 0   # ทุกค่าต้องเป็นบวก (positive definite)
ixx + iyy >= izz  and  iyy + izz >= ixx  and  izz + ixx >= iyy   # อสมการสามเหลี่ยม
```

---

## 10. รายการ Error ที่เจอบ่อย + วิธีแก้ทันที

### Error 1: `UnicodeEncodeError: 'charmap' codec can't encode characters...`
**เกิดตอน:** `print(robot)` บน Windows (ตาราง DH มีตัวอักษรพิเศษ เช่น `⍺`, `ⱼ`, `θ`)
**วิธีแก้:**
```powershell
$env:PYTHONIOENCODING="utf-8"     # PowerShell — รันก่อน python
```
```cmd
set PYTHONIOENCODING=utf-8        # Command Prompt (cmd.exe)
```
หรือถ้าไม่อยากยุ่งกับ env var แค่หลีกเลี่ยงไม่ print(robot) ตรง ๆ — ใช้ `robot.fkine(q)` ดูผลลัพธ์แทนก็พอ

### Error 2: `ValueError: cannot reshape array of size N into shape (M)`
**เกิดตอน:** เรียก `robot.fkine(q)` แต่ `q` มีจำนวนตัวเลขไม่ตรงกับจำนวน joint ของหุ่น
**วิธีแก้:** เช็คว่า `len(q) == robot.n` ก่อนเรียก fkine
```python
print(robot.n)         # ดูว่าหุ่นมีกี่ joint
q = np.zeros(robot.n)  # สร้าง q ให้จำนวนตรงกันเสมอ แล้วค่อยแก้ค่าทีละตัว
```

### Error 3: `FileNotFoundError: [Errno 2] No such file or directory: 'my_robot/robot.urdf'`
**เกิดตอน:** เรียก `Robot.URDF_read(...)` แต่ path ผิด หรือรันจาก working directory คนละที่กับที่คิดไว้
**วิธีแก้:**
```python
from pathlib import Path
print(Path('.').resolve())          # เช็คว่า working directory ตอนนี้อยู่ที่ไหน
print((Path('.') / 'my_robot' / 'robot.urdf').exists())   # เช็คว่าไฟล์มีจริงไหม
```
แก้ path ให้ตรง หรือ `cd` ไปที่โฟลเดอร์ที่ถูกต้องก่อนรัน

### Error 4: ผลลัพธ์ผิดแบบไม่มี error ใด ๆ เลย (อันตรายที่สุด — เช็คทุกครั้ง!)
**สาเหตุที่พบบ่อยที่สุด: ลืม `np.deg2rad()`** — ใส่องศาตรง ๆ แทนเรเดียน โค้ดไม่ error แต่คำตอบผิด
```python
# ผิด (คำตอบเพี้ยนแต่ไม่ error):
q = np.array([90.0, 90.0])

# ถูก:
q = np.array([np.deg2rad(90), np.deg2rad(90)])
```
**วิธีเช็ค:** ถ้าค่า q ที่ใส่มากกว่า ~6.28 (2π) แสดงว่าน่าจะลืมแปลงหน่วย (เพราะเรเดียนไม่ควรเกิน 2π = 360°)

### Error 5: `ModuleNotFoundError: No module named 'roboticstoolbox'` (หรือ swift/spatialmath)
**เกิดตอน:** เครื่องไม่มี library ติดตั้ง หรือรันผิด python environment
**วิธีแก้ (ต้องมีเน็ต ทำก่อนเข้าห้องสอบเท่านั้น):**
```
pip install "numpy<2" "roboticstoolbox-python[swift]"
```
**ในห้องสอบไม่มีเน็ต** — เช็ค environment ให้พร้อมตั้งแต่ก่อนเข้าห้อง

### Error 6: mesh ไฟล์หาไม่เจอ (`[FAIL] these mesh files were not found`)
**สาเหตุ:** path ใน `<mesh filename="...">` เขียนผิด หรือ case ตัวอักษรไม่ตรง (`.stl` vs `.STL`) หรือไฟล์ยังไม่ได้ก็อปมาไว้ใน `meshes/`
**วิธีแก้:** เปิด `robot.urdf` เช็คทุกบรรทัดที่มี `<mesh filename=`  ให้ path ตรงกับชื่อไฟล์จริงเป๊ะ (Windows ไม่สนตัวพิมพ์เล็ก-ใหญ่ แต่บาง parser สน)

### Error 7: Swift เปิด browser ไม่ขึ้น / ค้าง
**สาเหตุ:** ไม่มี default browser ตั้งไว้ หรือ firewall บล็อก localhost
**วิธีแก้:** เช็คว่ามี browser ติดตั้งและตั้งเป็น default แล้ว, ลองรันใหม่, หรือเปิด URL ที่ terminal print ออกมาด้วยมือ

### Error 8: ตั้งค่า joint เกิน `qlim` แต่โค้ดไม่ error / ไม่เตือน
**ความเข้าใจผิดที่พบบ่อย:** คิดว่า `qlim` จะ block ค่าที่เกินขอบเขตอัตโนมัติ — **ไม่จริง**, `fkine()` ไม่ enforce qlim ให้ ต้องเช็คเองถ้าโจทย์ต้องการ
```python
lo, hi = robot.qlim[0][i], robot.qlim[1][i]
if not (lo <= q[i] <= hi):
    print(f"joint {i} ค่าเกินขอบเขต! ({q[i]} ไม่อยู่ใน [{lo}, {hi}])")
```

---

## 11. เช็คลิสต์ก่อนส่งงาน

- [ ] ทุกมุมแปลงเป็นเรเดียนด้วย `np.deg2rad()` แล้วหรือยัง (เช็คว่าไม่มีค่า q ไหนเกิน ~6.28)
- [ ] `len(q) == robot.n` ก่อนเรียก `fkine(q)`
- [ ] path ของ mesh ใน urdf ตรงกับไฟล์จริงในโฟลเดอร์ `meshes/`
- [ ] เทียบ `T.t` (และ `T.R` ถ้าจำเป็น) กับค่าที่คำนวณมือ ก่อนเชื่อผลจากโค้ด
- [ ] ถ้าแก้ origin/axis ใน URDF แล้วภาพหุ่นเพี้ยนหรือ mesh หมุนไปเอง → ลืม compensate ตามข้อ 9.3
- [ ] ถ้า `print(robot)` error encoding → ตั้ง `PYTHONIOENCODING=utf-8` ก่อนรัน
- [ ] working directory ตอนรันตรงกับที่ path ใน URDF อ้างอิงไหม (`Path('.').resolve()` เช็คได้)
