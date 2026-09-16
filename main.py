"""
RoboMaster EP BFS Navigation Program
อ้างอิงข้อกำหนดตาม docs/RQM.md และ docs/Picture2.png:
1. เดินตาม Picture2.png โดยใช้ BFS (Breadth-First Search)
2. ใช้ ToF sensor ตรวจจับสิ่งกีดขวาง และ IMU ตรวจสอบมุมหัน (Attitude/Yaw)
3. ขนาดหุ่น: ความยาว ~33 cm, ความกว้าง ~25 cm
4. แต่ละช่องมีขนาด 60 cm * 60 cm (0.60 m * 0.60 m)
5. ToF sensor ติดตั้งอยู่บน Gimbal ให้ล็อก Gimbal ให้อยู่ตรงกลางตามแนวตัวรถ (CHASSIS_LEAD)
6. บันทึกประวัติการเดินลงไฟล์ CSV เฉพาะเมื่อรันกับหุ่นจริงเท่านั้น (Real Robot Only)

วิธีใช้งาน:
- รันบนหุ่นยนต์จริง (WiFi Router / STA) พร้อมบันทึก CSV:
    python main.py --conn sta
- รันบนหุ่นยนต์จริง (ต่อ WiFi ตรงกับหุ่น / AP) พร้อมบันทึก CSV:
    python main.py --conn ap
- รันโหมดทดสอบจำลอง (Simulation - ไม่ต่อหุ่นจริง, ไม่สร้าง CSV):
    python main.py --sim
- สร้างแมพการเดินจาก CSV:
    python plot_path.py --csv logs/navigation_log.csv
"""

import os
import sys
import argparse

# ตั้งค่า encoding ของ stdout ให้รองรับภาษาไทยบน Windows
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

# โหลดโมดูลจากโฟลเดอร์ src
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from bfs_planner import GridMap
from astar_planner import AStarPlanner
from robot_navigator import RobotNavigator
from csv_logger import NavigationLogger
from plot_path import create_path_map, plot_navigation_map_from_csv, plot_multi_round_comparison


def print_banner():
    print("=" * 60)
    print("      [RoboMaster EP - BFS Grid Navigation System]")
    print("      อ้างอิง: docs/RQM.md & docs/Picture2.png (สนาม 4x4)")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="RoboMaster EP BFS Navigation based on Picture2.png")
    parser.add_argument(
        "--conn",
        type=str,
        default="ap",
        choices=["sta", "ap"],
        help="รูปแบบการเชื่อมต่อ RoboMaster: 'sta' (Router) หรือ 'ap' (Direct Wi-Fi)"
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="รันในโหมดจำลอง (Simulation Mode) โดยไม่ต้องเชื่อมต่อหุ่นยนต์จริง (ไม่บันทึก CSV)"
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="พาธไฟล์ CSV สำหรับบันทึกข้อมูล (ค่าเริ่มต้น: logs/navigation_log.csv สำหรับ BFS, logs/navigation_log_astar.csv สำหรับ A*)"
    )
    parser.add_argument(
        "--clear-csv",
        action="store_true",
        help="ล้างไฟล์ CSV เดิมและเริ่มนับรอบ 1 ใหม่ (ค่าเริ่มต้น: False คือเก็บสะสมข้อมูลต่อท้าย ไม่ลบของเดิม)"
    )
    parser.add_argument(
        "--run",
        type=int,
        default=None,
        help="ระบุหมายเลขรอบ (Run ID) เช่น 1, 2, 3 (ค่าเริ่มต้น: ตรวจจับอัตโนมัติต่อจากรอบเดิมใน CSV)"
    )
    parser.add_argument(
        "--log-sim",
        action="store_true",
        help="บันทึกข้อมูลลงไฟล์ CSV แม้อยู่ในโหมดจำลอง (ค่าเริ่มต้น: False)"
    )
    parser.add_argument(
        "--algo",
        type=str,
        default="bfs",
        choices=["bfs", "astar"],
        help="เลือกอัลกอริทึมการวางแผนเส้นทาง: 'bfs' หรือ 'astar' (ค่าเริ่มต้น: bfs)"
    )
    parser.add_argument(
        "--all-paths",
        action="store_true",
        help="แสดงทุกเส้นทาง BFS ที่เป็นไปได้"
    )
    args = parser.parse_args()

    print_banner()

    # 1. โหลดข้อมูลแผนที่ 4x4 จาก Picture2.png
    if args.algo == "astar":
        grid = AStarPlanner()
        algo_title = "A* (A-Star)"
    else:
        grid = GridMap()
        algo_title = "BFS (Breadth-First Search)"

    print(f"[MAP CONFIG] อัลกอริทึม: {algo_title}")
    print(f"[MAP CONFIG] ขนาดสนาม: {grid.width}x{grid.height} ช่อง")
    print(f"[MAP CONFIG] ขนาดแต่ละช่อง: {grid.cell_size_m * 100:.0f} cm x {grid.cell_size_m * 100:.0f} cm")
    print(f"[MAP CONFIG] ขนาดหุ่นยนต์: ยาว {grid.robot_length_m * 100:.0f} cm, กว้าง {grid.robot_width_m * 100:.0f} cm")
    print(f"[MAP CONFIG] จุดเริ่มต้น (Start): {grid.start}")
    print(f"[MAP CONFIG] เป้าหมาย (Goal): {grid.goal}")
    print(f"[MAP CONFIG] สิ่งกีดขวาง (Obstacles): {sorted(list(grid.obstacles))}")

    # 2. ค้นหาเส้นทาง
    print(f"\n[PLANNING] กำลังค้นหาเส้นทางด้วย {algo_title}...")
    if args.algo == "astar":
        search_result = grid.astar_search()
        path = search_result.get("path")
    else:
        path = grid.bfs_shortest_path(optimize_cost_among_shortest=True)

    if not path:
        print(f"[ERROR] ไม่พบเส้นทางจากจุดเริ่มต้นไปยังเป้าหมายด้วย {algo_title}!")
        sys.exit(1)

    steps = len(path) - 1
    total_cost_excl = grid.calculate_path_cost(path, include_start=False)
    total_cost_incl = grid.calculate_path_cost(path, include_start=True)

    print(f"[PLANNING SUCCESS] พบเส้นทาง {algo_title} ({steps} ก้าว):")
    print(" -> ".join([f"({x},{y})" for x, y in path]))
    print(f"[COST] ค่าใช้จ่ายรวม (ไม่รวมจุดเริ่ม): {total_cost_excl} | (รวมจุดเริ่ม): {total_cost_incl}")

    # แสดงแผนที่ ASCII พร้อมเส้นทาง
    grid.print_grid(path)
    if args.algo == "astar":
        grid.print_search_summary(search_result)

    # สร้างและบันทึกรูปภาพแผนที่แสดงเส้นทางเริ่มต้น (output/path_map.png)
    # กำหนดพาธไฟล์ CSV แยกตามอัลกอริทึม
    if args.csv:
        csv_file = args.csv
    else:
        csv_file = "logs/navigation_log_astar.csv" if args.algo == "astar" else "logs/navigation_log.csv"

    # กำหนด prefix สำหรับไฟล์ภาพแผนที่
    prefix = "path_map_astar" if args.algo == "astar" else "path_map"

    # สร้างและบันทึกรูปภาพแผนที่แสดงเส้นทางเริ่มต้น (output/path_map.png หรือ output/path_map_astar.png)
    try:
        os.makedirs("output", exist_ok=True)
        initial_map_path = f"output/{prefix}.png"
        map_file = create_path_map(path=path, output_path=initial_map_path)
        print(f"[IMAGE] บันทึกรูปแผนที่เส้นทางสำเร็จ: {map_file}\n")
    except Exception as e:
        print(f"[WARNING] ไม่สามารถสร้างไฟล์รูปภาพแผนที่ได้: {e}\n")

    # 3. เตรียมระบบบันทึก CSV แยกเฉพาะของอัลกอริทึม
    is_real_run = not args.sim
    enable_logging = is_real_run or args.log_sim

    os.makedirs(os.path.dirname(csv_file) if os.path.dirname(csv_file) else ".", exist_ok=True)
    logger = NavigationLogger(
        filename=csv_file,
        enabled=enable_logging,
        append=(not args.clear_csv),
        run_id=args.run
    )
    if args.log_sim:
        logger.allow_sim_logging = True

    if is_real_run:
        print(f"[CSV CONFIG - {args.algo.upper()}] บันทึกข้อมูลหุ่นจริงลงในไฟล์ CSV แยก: {logger.filename} (รอบที่ #{logger.current_run})")
    elif args.log_sim:
        print(f"[CSV CONFIG - {args.algo.upper()}] โหมดจำลองเปิดการบันทึก (Sim Logging): {logger.filename} (รอบที่ #{logger.current_run})")
    else:
        print(f"[CSV CONFIG - {args.algo.upper()}] โหมดจำลอง (Simulation) - ไม่บันทึก CSV (เมื่อรันหุ่นจริงจะบันทึกลง: {logger.filename})")

    navigator = RobotNavigator(
        conn_type=args.conn,
        sim_mode=args.sim,
        cell_size_m=grid.cell_size_m,
        obstacle_threshold_mm=420.0,
        logger=logger
    )

    try:
        connected = navigator.connect()
        if not connected:
            print("[ERROR] ไม่สามารถเริ่มต้นระบบหุ่นยนต์ได้")
            sys.exit(1)

        def step_callback(step_num, pos):
            cost = grid.costs.get(pos, 0)
            print(f"  -> อยู่ที่ช่อง: {pos} | Cost ช่องนี้ = {cost}")

        # 4. ดำเนินการนำทางหุ่นยนต์ตามเส้นทาง
        success = navigator.navigate_path(
            path,
            cell_costs=grid.costs,
            on_step_callback=step_callback
        )

        if success:
            print("\n" + "=" * 60)
            print(f"  ภารกิจสำเร็จ! หุ่นยนต์เดินตามเส้นทาง {algo_title} ครบถ้วน (รอบที่ #{logger.current_run})")
            print(f"  ระยะทางเคลื่อนที่รวม: {steps * grid.cell_size_m:.2f} เมตร ({steps * grid.cell_size_m * 100:.0f} cm)")
            print(f"  ต้นทุนเส้นทาง (Path Cost): {total_cost_excl}")
            if logger.enabled:
                print(f"  บันทึกประวัติการเดินลงไฟล์ CSV แยก: {logger.filename}")
            print("=" * 60)
            if logger.enabled:
                logger.print_summary()
                # สร้างแมพจากไฟล์ CSV จริงโดยอัตโนมัติ
                try:
                    # 1. แผนที่สำหรับรอบปัจจุบัน
                    run_map_file = f"output/{prefix}_run{logger.current_run}.png"
                    plot_navigation_map_from_csv(
                        logger.filename,
                        output_path=run_map_file,
                        run_id=logger.current_run
                    )
                    # 2. อัปเดตไฟล์ภาพล่าสุด
                    csv_map_file = plot_navigation_map_from_csv(
                        logger.filename,
                        output_path=f"output/{prefix}_from_csv.png",
                        run_id=logger.current_run
                    )
                    print(f"[CSV MAP] อัปเดตแผนที่จริงรอบ #{logger.current_run} จาก CSV: {csv_map_file}")

                    # 3. หากมีข้อมูลสะสมตั้งแต่ 2 รอบขึ้นไป ให้สร้าง Dashboard เปรียบเทียบผล
                    total_runs = logger.get_total_runs()
                    if total_runs >= 2:
                        comp_map_file = plot_multi_round_comparison(
                            logger.filename,
                            output_path=f"output/{prefix}_multi_round_comparison.png"
                        )
                        print(f"[MULTI-ROUND MAP] สร้างแผนที่เปรียบเทียบผล {total_runs} รอบ: {comp_map_file}")
                except Exception as e:
                    print(f"[WARNING] ไม่สามารถสร้างแผนที่จาก CSV: {e}")
        else:
            print("\n[WARNING] การเดินทางหยุดชะงักก่อนถึงเป้าหมาย")
            if logger.enabled:
                logger.print_summary()

    except KeyboardInterrupt:
        print("\n[USER STOP] ผู้ใช้สั่งหยุดการทำงาน (Ctrl+C)")
    finally:
        navigator.disconnect()


if __name__ == "__main__":
    main()
