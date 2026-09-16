# RoboMaster EP - Grid Navigation & Telemetry System

ระบบนำทางอัตโนมัติสำหรับหุ่นยนต์ **RoboMaster EP** บนสนามจำลองขนาด $4 \times 4$ ช่อง พร้อมระบบตรวจสอบเซนเซอร์ (ToF & IMU), ระบบความปลอดภัยฉุกเฉิน, ระบบบันทึกข้อมูล Telemetry ลงไฟล์ CSV และการสร้าง Dashboard แผนที่เส้นทาง
อ้างอิงข้อกำหนดตาม [docs/RQM.md](file:///C:/Users/LENOVO_CT/Downloads/New%20folder%20%282%29/docs/RQM.md) และรูปภาพแผนที่ [docs/Picture2.png](file:///C:/Users/LENOVO_CT/Downloads/New%20folder%20%282%29/docs/Picture2.png)

---

## 📌 สารบัญ (Table of Contents)
1. [ข้อกำหนดของระบบ (Requirements)](#-ข้อกำหนดของระบบ-requirements)
2. [แผนที่สนาม ค่าใช้จ่าย และเส้นทาง (Grid Map & Costs)](#-แผนที่สนาม-ค่าใช้จ่าย-และเส้นทาง-grid-map--costs)
3. [โครงสร้างโปรเจกต์ (Project Structure)](#-โครงสร้างโปรเจกต์-project-structure)
4. [หลักการทำงานของระบบ (System Architecture & Core Logic)](#-หลักการทำงานของระบบ-system-architecture--core-logic)
   - [4.1 อัลกอริทึมค้นหาเส้นทาง (Path Planning Algorithms)](#41-อัลกอริทึมค้นหาเส้นทาง-path-planning-algorithms)
   - [4.2 การควบคุมหุ่นยนต์และเซนเซอร์ (Robot & Sensor Control)](#42-การควบคุมหุ่นยนต์และเซนเซอร์-robot--sensor-control)
   - [4.3 ระบบบันทึกข้อมูล Telemetry (CSV Logging)](#43-ระบบบันทึกข้อมูล-telemetry-csv-logging)
   - [4.4 การพล็อตแผนที่และ Dashboard (Visualization)](#44-การพล็อตแผนที่และ-dashboard-visualization)
5. [การเตรียม Virtual Environment และติดตั้ง Requirements (Setup & Install)](#-การเตรียม-virtual-environment-และติดตั้ง-requirements-setup--install)
   - [ขั้นตอนที่ 1: สร้าง venv ด้วย Python 3.8](#ขั้นตอนที่-1-สร้าง-virtual-environment-ด้วย-python-38)
   - [ขั้นตอนที่ 2: เปิดใช้งาน venv และติดตั้งไลบรารีจาก requirements.txt](#ขั้นตอนที่-2-เปิดใช้งาน-activate-และติดตั้งจาก-requirementstxt)
   - [ขั้นตอนที่ 3: ตรวจสอบความถูกต้องของสภาพแวดล้อม](#ขั้นตอนที่-3-ตรวจสอบความถูกต้องของสภาพแวดล้อม)
6. [วิธีรันโปรแกรม (Execution Guide)](#-วิธีรันโปรแกรม-execution-guide)
   - [6.1 การรัน BFS (ค่าเริ่มต้น)](#61-การรัน-bfs-breadth-first-search)
   - [6.2 การรัน A* (A-Star Search)](#62-การรัน-a-a-star-search)
   - [6.3 การรัน DFS (Depth-First Search)](#63-การรัน-dfs-depth-first-search)
   - [6.4 การรัน Dijkstra's Algorithm](#64-การรัน-dijkstras-algorithm)
   - [6.5 การสร้างรูปภาพแผนที่และ Dashboard จาก CSV](#65-การสร้างรูปภาพแผนที่และ-dashboard-จาก-csv)
   - [6.6 การรันชุดทดสอบ (Unit Tests)](#66-การรันชุดทดสอบ-unit-tests)
7. [ตารางสรุปพารามิเตอร์คำสั่ง (CLI Arguments Reference)](#-ตารางสรุปพารามิเตอร์คำสั่ง-cli-arguments-reference)

---

## 📋 ข้อกำหนดของระบบ (Requirements)

อ้างอิงตาม [docs/RQM.md](file:///C:/Users/LENOVO_CT/Downloads/New%20folder%20%282%29/docs/RQM.md):
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
5. **สภาพแวดล้อม Python**: **ต้องใช้ Python 3.8 เท่านั้น** (โดยในโฟลเดอร์ `.venv` ใช้เวอร์ชัน 3.8.10 เพื่อความเข้ากันได้กับ RoboMaster SDK)

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

### 4.4 การพล็อตแผนที่และ Dashboard (`src/plot_path.py`)
- **Single Run Dashboard**: แสดงตารางสนาม $4 \times 4$, จุด Start/Goal/Obstacle, แท็กเซนเซอร์ ToF/Yaw ในแต่ละก้าว, ตาราง Telemetry สรุปผล และกราฟแนวโน้ม ToF / IMU Drift
- **Multi-Round Comparison Dashboard**: เมื่อรันหุ่นยนต์สะสมตั้งแต่ 2 รอบขึ้นไป ระบบจะสร้างแดชบอร์ดเปรียบเทียบผลลัพธ์ระหว่างรอบ (เปรียบเทียบระยะ ToF, การเลี้ยวของ IMU และความเสถียรของแต่ละรอบ)

---

## 🛠️ การเตรียม Virtual Environment และติดตั้ง Requirements (Setup & Install)

> [!IMPORTANT]
> **ระบบนี้ต้องใช้งานบน Python 3.8 เท่านั้น**
> ตัวไลบรารี **RoboMaster SDK** มีการคอมไพล์ C-Extensions และระบบจัดการสตรีมเสียง/วิดีโอ (libopus / PyAudio) ที่รองรับได้อย่างเสถียรบน **Python 3.8 (เช่น 3.8.10)** เท่านั้น หากใช้ Python 3.9 หรือใหม่กว่าจะไม่สามารถติดตั้งหรือเชื่อมต่อหุ่นยนต์ได้

โปรดทำตามขั้นตอนตามลำดับดังต่อไปนี้:

### ขั้นตอนที่ 1: สร้าง Virtual Environment ด้วย Python 3.8
เปิด PowerShell หรือ Command Prompt ในโฟลเดอร์โปรเจกต์ แล้วสร้าง `.venv` โดยชี้ไปยัง Python 3.8:
```powershell
# สำหรับ Windows (ใช้ Python Launcher ระบุเวอร์ชัน 3.8)
py -3.8 -m venv .venv
```
*(หากติดตั้ง Python 3.8 ไว้ที่พาธเฉพาะ สามารถระบุเต็มได้ เช่น `C:\Python38\python.exe -m venv .venv`)*

---

### ขั้นตอนที่ 2: เปิดใช้งาน (Activate) และติดตั้งจาก `requirements.txt`
ทำการเปิดใช้งาน Virtual Environment จากนั้นอัปเกรด pip และติดตั้งไลบรารีทั้งหมดผ่าน `requirements.txt`:

```powershell
# 1. เปิดใช้งาน Virtual Environment บน PowerShell
.\.venv\Scripts\Activate.ps1

# (หากใช้ Command Prompt ให้ใช้: .venv\Scripts\activate.bat)

# 2. อัปเกรดเครื่องมือ pip
python -m pip install --upgrade pip

# 3. ติดตั้งไลบรารีที่จำเป็นทั้งหมดจาก requirements.txt
pip install -r requirements.txt
```

> [!NOTE]
> ภายใน [`requirements.txt`](file:///C:/Users/LENOVO_CT/Downloads/New%20folder%20%282%29/requirements.txt) ได้รวบรวมไลบรารีสำคัญไว้ครบถ้วน ได้แก่:
> - `numpy` & `matplotlib`: คำนวณกริดและสร้างกราฟ Dashboard
> - `robomaster`: SDK สำหรับควบคุมหุ่นยนต์ EP, แชสซี, กิมบอล, ToF และ IMU
> - `opencv-python`, `netaddr`, `netifaces`: การเชื่อมต่อเครือข่ายและประมวลผลเซนเซอร์

---

### ขั้นตอนที่ 3: ตรวจสอบความถูกต้องของสภาพแวดล้อม
ตรวจสอบว่า Python ที่เรียกใช้งานเป็นเวอร์ชัน 3.8 ใน `.venv`:
```powershell
python --version
# หรือเรียกตรงโดยไม่ต้อง activate:
.\.venv\Scripts\python.exe --version
```
> ผลลัพธ์ควรแสดงเป็น: `Python 3.8.x` (เช่น `Python 3.8.10`)

---

## 🚀 วิธีรันโปรแกรม (Execution Guide)

> [!TIP]
> เมื่อทำการเปิดใช้งาน Virtual Environment (`Activate.ps1`) เรียบร้อยแล้ว สามารถใช้คำสั่งย่อเป็น `python <script>.py` แทน `.\.venv\Scripts\python.exe <script>.py` ได้ทันที

### 6.1 การรัน BFS (Breadth-First Search)

#### ก. โหมดจำลอง (Simulation Mode - ไม่ต้องต่อหุ่นจริง, ไม่สร้าง CSV):
```powershell
.\.venv\Scripts\python.exe main.py --sim
```
> ระบบจะคำนวณเส้นทาง BFS แสดงแผนที่ ASCII ในเทอร์มินัล และบันทึกภาพแผนที่ไปที่ `output/path_map.png`

#### ข. รันบนหุ่นยนต์จริงผ่าน Direct Wi-Fi (AP Mode):
1. เปิดเครื่อง RoboMaster EP และสับสวิตช์การเชื่อมต่อที่ตัวหุ่นไปที่โหมด **Direct (AP)**
2. ใช้คอมพิวเตอร์เชื่อมต่อกับสัญญาณ Wi-Fi ของหุ่น (เช่น `RM_EP_...`)
3. รันคำสั่ง:
```powershell
.\.venv\Scripts\python.exe main.py --conn ap
```
> ข้อมูลเซนเซอร์จริงจะถูกบันทึกสะสมลงใน `logs/navigation_log.csv` และสร้างแผนที่จริงที่ `output/path_map_from_csv.png`

#### ค. รันบนหุ่นยนต์จริงผ่าน Wi-Fi Router (STA Mode):
1. สับสวิตช์ที่หุ่นไปที่โหมด **Router (STA)** และสแกน QR Code เพื่อให้หุ่นเกาะ Wi-Fi เดียวกับคอมพิวเตอร์
2. รันคำสั่ง:
```powershell
.\.venv\Scripts\python.exe main.py --conn sta
```

#### ง. รันสะสมหลายรอบ (Multi-Round):
รันคำสั่งเดิมซ้ำในรอบถัดไป ระบบจะตรวจจับและระบุรอบที่ 2, 3 ให้อัตโนมัติ:
```powershell
.\.venv\Scripts\python.exe main.py --conn ap
```

---

### 6.2 การรัน A* (A-Star Search)

#### ก. โหมดจำลอง (Simulation Mode):
```powershell
.\.venv\Scripts\python.exe main_astar.py --sim
```
หรือใช้ผ่าน `main.py`:
```powershell
.\.venv\Scripts\python.exe main.py --algo astar --sim
```

#### ข. รันบนหุ่นยนต์จริง:
```powershell
# ผ่าน Direct Wi-Fi
.\.venv\Scripts\python.exe main_astar.py --conn ap

# ผ่าน Router
.\.venv\Scripts\python.exe main_astar.py --conn sta
```
> ข้อมูลจะถูกบันทึกแยกใน `logs/navigation_log_astar.csv` และสร้างภาพที่ `output/path_map_astar_from_csv.png`

---

### 6.3 การรัน DFS (Depth-First Search)

รันเพื่อทดสอบการคำนวณและดูสถิติเวลาประมวลผล (Computation Time):
```powershell
.\.venv\Scripts\python.exe main_dfs.py
```
> แสดงลำดับการค้นหา, ตารางโหนดที่สำรวจ, ค่า Cost รวม และบันทึกแผนที่ไปที่ `output/dfs_path.png`

---

### 6.4 การรัน Dijkstra's Algorithm

รันเพื่อตรวจสอบเส้นทางต้นทุนต่ำสุด (Optimal Cost Path):
```powershell
.\.venv\Scripts\python.exe main_dijkstra.py
```
> แสดงตารางค่าใช้จ่ายสะสมของแต่ละโหนดและบันทึกรูปภาพไปที่ `output/dijkstra_path.png`

---

### 6.5 การสร้างรูปภาพแผนที่และ Dashboard จาก CSV

คุณสามารถสั่งให้ `plot_path.py` อ่านข้อมูลเซนเซอร์จากไฟล์ CSV มาวาดกราฟใหม่ได้ตลอดเวลา:

#### ก. สร้างแผนที่รอบล่าสุดจากไฟล์ CSV:
```powershell
.\.venv\Scripts\python.exe plot_path.py --csv logs/navigation_log.csv
```

#### ข. สร้างแผนที่เจาะจงเฉพาะรอบ (เช่น รอบที่ 1):
```powershell
.\.venv\Scripts\python.exe plot_path.py --csv logs/navigation_log.csv --run 1 --output output/path_map_run1.png
```

#### ค. สร้าง Dashboard เปรียบเทียบผลทุกรอบ (Multi-Round Comparison):
```powershell
.\.venv\Scripts\python.exe plot_path.py --csv logs/navigation_log.csv --compare --output output/path_map_multi_round_comparison.png
```

#### ง. สร้าง Dashboard เปรียบเทียบของอัลกอริทึม A*:
```powershell
.\.venv\Scripts\python.exe plot_path.py --csv logs/navigation_log_astar.csv --compare --output output/path_map_astar_multi_round_comparison.png
```

---

### 6.6 การรันชุดทดสอบ (Unit Tests)

ทดสอบความถูกต้องของตรรกะการเดิน, ขนาดกริด, สิ่งกีดขวาง, การคำนวณ Cost และระบบ Logging:
```powershell
.\.venv\Scripts\python.exe -m unittest test_bfs.py
```

---

## 📊 ตัวอย่างข้อมูลในไฟล์ Telemetry CSV

ตัวอย่างข้อมูลที่บันทึกจากเซนเซอร์ของหุ่นยนต์จริงในแต่ละสเต็ป (`logs/navigation_log.csv`):

| run | step | pos_x | pos_y | action | target_heading | imu_yaw | imu_drift | tof_distance_mm | cell_cost | cumulative_cost | distance_m | status |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 1 | 0 | 1 | 4 | START | 0.0 | -7.40 | 0.00 | 1568.0 | 2 | 0 | 0.00 | SUCCESS |
| 1 | 1 | 1 | 3 | MOVE_TO_CELL | -90.0 | 81.30 | 0.80 | 1964.0 | 2 | 2 | 0.60 | SUCCESS |
| 1 | 2 | 1 | 2 | MOVE_TO_CELL | -90.0 | 81.20 | 0.70 | 1457.0 | 3 | 5 | 1.20 | SUCCESS |
| 1 | 3 | 2 | 2 | MOVE_TO_CELL | 0.0 | -8.30 | -0.90 | 889.0 | 1 | 6 | 1.80 | SUCCESS |
| 1 | 4 | 2 | 1 | MOVE_TO_CELL | -90.0 | 80.50 | 0.00 | 842.0 | 3 | 9 | 2.40 | SUCCESS |
| 1 | 5 | 3 | 1 | MOVE_TO_CELL | 0.0 | -8.90 | -1.50 | 1301.0 | 2 | 11 | 3.00 | SUCCESS |
| 1 | 6 | 4 | 1 | REACH_GOAL | 0.0 | -9.00 | -1.60 | 719.0 | 1 | 12 | 3.60 | GOAL_REACHED |

---

## 📑 ตารางสรุปพารามิเตอร์คำสั่ง (CLI Arguments Reference)

### พารามิเตอร์ของ `main.py` / `main_astar.py`:

| อาร์กิวเมนต์ | ชนิด | ค่าเริ่มต้น | คำอธิบาย |
|:---|:---:|:---:|:---|
| `--sim` | Flag | `False` | รันในโหมดจำลอง ไม่ต้องเชื่อมต่อหุ่นยนต์จริง และไม่บันทึก CSV |
| `--conn` | String | `ap` | รูปแบบการเชื่อมต่อหุ่นจริง: `ap` (ต่อ Wi-Fi ตรง) หรือ `sta` (ผ่าน Router) |
| `--algo` | String | `bfs` | เลือกอัลกอริทึมสำหรับ `main.py`: `bfs` หรือ `astar` |
| `--csv` | String | Auto | กำหนดพาธไฟล์ CSV สำหรับบันทึกข้อมูล Telemetry เอง |
| `--clear-csv` | Flag | `False` | ล้างข้อมูลเดิมในไฟล์ CSV และเริ่มนับรอบที่ 1 ใหม่ |
| `--run` | Int | Auto | ระบุหมายเลขรอบ (Run ID) เอง หากไม่ระบุจะนับต่อจากรอบเดิมในไฟล์ |
| `--log-sim` | Flag | `False` | บังคับบันทึกข้อมูลลงไฟล์ CSV แม้รันอยู่ในโหมดจำลอง (`--sim`) |
| `--all-paths` | Flag | `False` | ค้นหาและแสดงผลเส้นทาง BFS ที่เป็นไปได้ทั้งหมด |

### พารามิเตอร์ของ `plot_path.py`:

| อาร์กิวเมนต์ | ชนิด | ค่าเริ่มต้น | คำอธิบาย |
|:---|:---:|:---:|:---|
| `--csv` | String | Auto | ระบุพาธไฟล์ CSV ที่ต้องการนำมาพล็อต |
| `--output` | String | Auto | ระบุพาธและชื่อไฟล์รูปภาพปลายทาง (.png) |
| `--run` | Int | ล่าสุด | ระบุรอบที่ต้องการพล็อตเฉพาะเจาะจง |
| `--compare` | Flag | `False` | สร้างภาพ Dashboard เปรียบเทียบผลลัพธ์ทุกรอบ |
| `--bfs` | Flag | `False` | บังคับพล็อตเฉพาะแผนที่ทางทฤษฎี BFS โดยไม่อ่านไฟล์ CSV |
| `--show` | Flag | `False` | เปิดหน้าต่างแสดงรูปภาพบนหน้าจอทันทีเมื่อสร้างเสร็จ |
