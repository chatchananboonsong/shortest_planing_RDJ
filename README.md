## สมาชิกกลุ่ม
1. 6810110066 นาย ซัซวาลย์ บินสะอิ
2. 6810110055 นาย ชัชนันท์ บุญส่ง
3. 6810110324 นาย วิญญู สิงห์สาธร
4. 6810110448 นาย จิระธาดา พัดบุรี

## 📋 ข้อกำหนดของระบบ (Requirements)

1. **สนามขนาด $4 \times 4$ ช่อง**:
   - แต่ละช่องมีขนาด $60\text{ cm} \times 60\text{ cm}$ ($0.60\text{ m} \times 0.60\text{ m}$)
   - พิกัดเริ่มต้น (Start): `(1, 4)`
   - พิกัดเป้าหมาย (Goal): `(4, 1)`
   - สิ่งกีดขวาง (Obstacles): `(4, 4)`, `(2, 3)`, `(3, 2)`
2. **ขนาดหุ่นยนต์**: ความยาวประมาณ $33\text{ cm}$, ความกว้างประมาณ $25\text{ cm}$ (มีระยะ Clearance ปลอดภัยในช่อง $60 \times 60\text{ cm}$)
3. **เซนเซอร์ที่ใช้งาน**:
   - **ToF Sensor**: วัดระยะสิ่งกีดขวางด้านหน้าหน่วยมิลลิเมตร (mm) พร้อมระบบความปลอดภัยฉุกเฉินหยุดทันทีหากระยะน้อยกว่า $420\text{ mm}$
   - **IMU Sensor (Attitude)**: ติดตามมุมหัน Yaw ($\pm 180^\circ$) เพื่อยืนยันการเลี้ยว $90^\circ$ และตรวจจับการเบี่ยงเบน (Drift)
   - **Gimbal Lock**: ล็อก Gimbal ให้อยู่กึ่งกลางตรงแนวตัวรถ (Pitch = $0^\circ$, Yaw = $0^\circ$, โหมด `CHASSIS_LEAD`) เพื่อให้ ToF ส่องตรงไปข้างหน้าตลอดเวลา
4. **การบันทึกข้อมูล (CSV Logging)**: บันทึกข้อมูล Telemetry และสถานะเซนเซอร์ในแต่ละก้าวลงไฟล์ CSV **เฉพาะเมื่อรันกับหุ่นจริงเท่านั้น** (โหมดจำลองจะไม่บันทึก เว้นแต่จะระบุ `--log-sim`)

---

## 🗺️ แผนที่สนาม ค่าใช้จ่าย และเส้นทาง (Grid Map & Costs)

พิกัดระบบ $(x, y)$ โดยแกน $x$ คือคอลัมน์แนวนอน ($1$ ถึง $4$) และแกน $y$ คือแถวแนวตั้ง ($1$ ถึง $4$):

```text
  y
  4 | [START c=2]  [  c=3   ]  [  c=1   ]  [ OBSTACLE ]
  3 | [   c=2   ]  [OBSTACLE]  [  c=4   ]  [   c=2    ]
  2 | [   c=3   ]  [  c=1   ]  [OBSTACLE]  [   c=4    ]
  1 | [   c=2   ]  [  c=3   ]  [  c=2   ]  [ GOAL c=1 ]
    ---------------------------------------------------
        x = 1        x = 2        x = 3        x = 4
```

### สรุปเส้นทางของแต่ละอัลกอริทึม:

| อัลกอริทึม | เส้นทางเดิน | จำนวนก้าว (Steps) | Cost รวม (ไม่รวมจุดเริ่ม) | คุณสมบัติเด่น |
|:---|:---|:---:|:---:|:---|
| **BFS** | `(1,4) -> (1,3) -> (1,2) -> (2,2) -> (2,1) -> (3,1) -> (4,1)` | 6 ก้าว | 12 หน่วย | การันตีจำนวนก้าวสั้นที่สุด (Shortest Hops) |
| **A\*** | `(1,4) -> (1,3) -> (1,2) -> (2,2) -> (2,1) -> (3,1) -> (4,1)` | 6 ก้าว | 12 หน่วย | คำนวณรวดเร็วด้วย Heuristic และต้นทุนช่อง |
| **Dijkstra** | `(1,4) -> (1,3) -> (1,2) -> (2,2) -> (2,1) -> (3,1) -> (4,1)` | 6 ก้าว | 12 หน่วย | การันตีต้นทุนต่ำที่สุด (Optimal Cost) |
| **DFS** | เดินสำรวจลึกตามลำดับกิ่ง (ตัวอย่าง: ไปขวาก่อนหรือลงก่อน) | 6-12 ก้าว | แตกต่างตามลำดับกิ่ง | ใช้ทดสอบการค้นหาเชิงลึกและวัดเวลาประมวลผล |

---

## 📁 โครงสร้างโปรเจกต์ (Project Structure)

```text
.
├── src/                             # ซอร์สโค้ดและโมดูลหลัก
│   ├── __init__.py
│   ├── bfs_planner.py               # ตัวคำนวณและสร้างแผนที่ BFS 4x4
│   ├── astar_planner.py             # ตัวคำนวณ A* Search (f = g + h)
│   ├── dfs_planner.py               # ตัวคำนวณ Depth-First Search
│   ├── dijkstra_planner.py          # ตัวคำนวณ Dijkstra's Algorithm
│   ├── robot_navigator.py           # ตัวควบคุมหุ่น RoboMaster EP, ToF, IMU, Gimbal Lock
│   ├── csv_logger.py                # บันทึก Telemetry และสถิติเซนเซอร์ลง CSV
│   └── plot_path.py                 # เอนจินวาดแผนที่เส้นทาง, ตาราง Telemetry และ Dashboard
├── docs/                            # เอกสารและภาพอ้างอิง
│   ├── RQM.md                       # ข้อกำหนดเบื้องต้น
│   └── Picture2.png                 # ภาพสนามต้นฉบับ 4x4
├── logs/                            # โฟลเดอร์เก็บไฟล์บันทึก CSV จากเซนเซอร์จริง
│   ├── navigation_log.csv           # ข้อมูล Telemetry จากการเดิน BFS
│   └── navigation_log_astar.csv     # ข้อมูล Telemetry จากการเดิน A*
├── output/                          # โฟลเดอร์ผลลัพธ์ภาพแผนที่และ Dashboard
│   ├── path_map.png                 # แผนที่จำลอง BFS
│   ├── path_map_astar.png           # แผนที่จำลอง A*
│   ├── dfs_path.png                 # แผนที่จำลอง DFS
│   ├── dijkstra_path.png            # แผนที่จำลอง Dijkstra
│   ├── path_map_from_csv.png        # แผนที่จริงจาก CSV พร้อม Sensor Tags & Telemetry Table
│   └── path_map_multi_round_comparison.png # Dashboard เปรียบเทียบผลหลายรอบ
├── main.py                          # จุดสั่งการหลัก (รองรับทั้ง BFS และ --algo astar)
├── main_astar.py                    # จุดสั่งการสำหรับอัลกอริทึม A* โดยเฉพาะ
├── main_dfs.py                      # จุดสั่งการสำหรับอัลกอริทึม DFS (Simulation / Benchmark)
├── main_dijkstra.py                 # จุดสั่งการสำหรับอัลกอริทึม Dijkstra (Simulation / Optimal Cost)
├── plot_path.py                     # สคริปต์สั่งพล็อตแผนที่และ Dashboard จาก CSV
├── test_bfs.py                      # ชุดทดสอบ Unit Test สำหรับ BFS และตรรกะนำทาง
└── README.md                        # คู่มือการทำงานและวิธีใช้งานระบบ
```

---

## ⚙️ หลักการทำงานของระบบ (System Architecture & Core Logic)

### 4.1 อัลกอริทึมค้นหาเส้นทาง (Path Planning Algorithms)
1. **BFS (`src/bfs_planner.py`)**:
   - ใช้ Queue (`collections.deque`) ในการแผ่ขยายโหนดทีละชั้น (Breadth-First)
   - การันตีเส้นทางที่มีจำนวนก้าวสั้นที่สุดเสมอ ($6$ ก้าว)
   - หากมีเส้นทางที่จำนวนก้าวเท่ากัน ระบบจะเลือกเส้นทางที่มี Total Cost ต่ำที่สุด
2. **A\* (`src/astar_planner.py`)**:
   - ใช้ฟังก์ชันประเมินราคา $f(n) = g(n) + h(n)$
   - $g(n)$: ต้นทุนสะสมจริงจากจุดเริ่มต้นถึงช่องปัจจุบัน
   - $h(n)$: ค่าประมาณระยะห่างแบบ Manhattan Distance สู่จุดหมาย $(|x - x_{goal}| + |y - y_{goal}|)$
3. **Dijkstra (`src/dijkstra_planner.py`)**:
   - ใช้ Priority Queue (Min-Heap) คัดเลือกโหนดที่มีต้นทุนสะสม $g(n)$ ต่ำที่สุดในแต่ละก้าว
   - การันตีการหาเส้นทางที่ Total Cost ต่ำที่สุดเสมอ
4. **DFS (`src/dfs_planner.py`)**:
   - ใช้ Stack สำรวจลึกไปตามกิ่งจนสุดทางก่อนย้อนกลับ (Backtrack)
   - เหมาะสำหรับการเปรียบเทียบประสิทธิภาพเวลาประมวลผล (Computation Time)

---

### 4.2 การควบคุมหุ่นยนต์และเซนเซอร์ (`src/robot_navigator.py`)
- **การเชื่อมต่อ**: รองรับทั้ง Direct Wi-Fi (`ap`) และ Wi-Fi Router (`sta`) ผ่าน RoboMaster SDK
- **Gimbal Lock**: เรียกฟังก์ชัน `lock_and_center_gimbal()` สั่ง `recenter(pitch=0, yaw=0)` และเปลี่ยนโหมดหุ่นเป็น `CHASSIS_LEAD` เพื่อล็อกหัวเล็งและเซนเซอร์ ToF ให้ชี้ตรงไปตามแนวตัวรถเสมอ
- **การตรวจสอบความปลอดภัยด้วย ToF**:
  - ก่อนเคลื่อนที่เข้าสู่แต่ละช่อง ระบบจะอ่านระยะจาก ToF Sensor
  - หากระยะที่วัดได้ **น้อยกว่า $420\text{ mm}$** ระบบจะส่งสัญญาณเตือน `ToF WARNING` และสั่งหยุดเดินทันที (`status = OBSTACLE_BLOCKED`)
- **การหมุนตัวและการเดินหน้าด้วย IMU**:
  - เมื่อเปลี่ยนทิศทาง หุ่นยนต์จะหมุนตัวด้วยความเร็วเชิงมุมที่กำหนด พร้อมตรวจสอบมุม Yaw จริงจาก IMU ว่าหมุนได้ครบ $90^\circ$ หรือไม่
  - ขณะเดินหน้า $60\text{ cm}$ ระบบจะคำนวณค่าการเบี่ยงเบนของมุม (IMU Drift) และบันทึกลงในระบบ

---

### 4.3 ระบบบันทึกข้อมูล Telemetry (`src/csv_logger.py`)
- บันทึกข้อมูลอย่างละเอียดในทุก Step:
  - **Run ID**: หมายเลขรอบ (ตรวจจับและนับต่อให้อัตโนมัติในโหมด Append)
  - **Step**: ลำดับก้าว (#0 สำหรับจุดเริ่ม START)
  - **Coordinate**: พิกัด `(x, y)`
  - **Action**: การกระทำ (`START`, `MOVE_TO_CELL`, `REACH_GOAL`)
  - **Heading & IMU**: ทิศทางเป้าหมาย, Yaw จริง, และค่า Drift
  - **ToF Distance**: ระยะสิ่งกีดขวางด้านหน้าหน่วยมิลลิเมตร
  - **Gimbal Angles**: องศา Pitch และ Yaw ของ Gimbal
  - **Cost & Distance**: Cost ประจำช่อง, Cost สะสม, และระยะทางสะสมหน่วยเมตร
  - **Status**: สถานะ (`SUCCESS`, `GOAL_REACHED`, `OBSTACLE_BLOCKED`)
  - **Raw IMU**: ค่าความเร่งและ Gyroscope $(a_x, a_y, a_z, g_x, g_y, g_z)$
- **เงื่อนไขสำคัญ**: ระบบจะบันทึกลง CSV เมื่อรันกับหุ่นจริงเท่านั้น เพื่อป้องกันข้อมูลจำลองปะปนกับข้อมูลจริง

---

## 🚀 วิธีรันโปรแกรม (Execution Guide)


### 1) ตั้งค่า Python environment

เปิด PowerShell หรือ Command Prompt ในโฟลเดอร์โปรเจคแล้วทำตามขั้นตอนนี้

```powershell
py -3.8 -m venv .venv
.\.venv\Scripts\Activate.ps1
```

หากใช้ Git Bash หรือ bash อื่น อาจใช้คำสั่ง:

```bash
python3.8 -m venv .venv
source .venv/bin/activate
```
### 2) ติดตั้ง dependency

```powershell
cd .\shortest_planing_RDJ\
```

```powershell
pip install -r requirements.txt
```
### 3) การรันสามารถกดรันที่ไฟล์ชื่อขึ้นต้นด้วย main ได้เลยแต่ต้องต่อหุ่นก่อน (main.py คือ bfs)
