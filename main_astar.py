"""
RoboMaster EP - A* (A-Star) Grid Navigation System
ไฟล์รันหลักสำหรับอัลกอริทึม A* (A-Star) แยกออกมาเป็นอิสระ
อ้างอิง: docs/RQM.md & docs/Picture2.png (สนามขนาด 4 x 4 ช่อง)

การทำงานหลัก:
1. คำนวณเส้นทางต้นทุนต่ำสุด (Optimal Cost Path) ด้วย A* Algorithm (f = g + h)
2. วางแผนเส้นทางโดยพิจารณาค่าน้ำหนัก (Cell Costs) แต่ละช่อง
3. แสดงค่าการประเมิน f(n) = g(n) + h(n) ในแต่ละก้าว
4. นำทางหุ่นยนต์ RoboMaster EP (หรือโหมดจำลอง --sim)
5. ตรวจสอบสิ่งกีดขวางด้วย ToF Sensor (< 420 mm หยุดฉุกเฉิน)
6. ติดตามทิศทางด้วย IMU และล็อก Gimbal ให้อยู่กึ่งกลาง (CHASSIS_LEAD)
7. บันทึกผล Telemetry ลง CSV และสร้างแผนที่เปรียบเทียบ
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

from astar_planner import AStarPlanner
from robot_navigator import RobotNavigator
from csv_logger import NavigationLogger
from plot_path import create_path_map, plot_navigation_map_from_csv, plot_multi_round_comparison


def print_banner():
    print("=" * 65)
    print("     [RoboMaster EP - A* (A-Star) Grid Navigation System]")
    print("      อ้างอิง: docs/RQM.md & docs/Picture2.png (สนาม 4x4)")
    print("      อัลกอริทึม: A* Search (f = g + h, Weighted Cell Costs)")
    print("=" * 65)


def main():
    parser = argparse.ArgumentParser(description="RoboMaster EP A* (A-Star) Navigation based on Picture2.png")
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
        default="logs/navigation_log_astar.csv",
        help="พาธไฟล์ CSV สำหรับบันทึกข้อมูลเมื่อรันกับหุ่นจริง (ค่าเริ่มต้น: logs/navigation_log_astar.csv)"
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
        help="บันทึกข้อมูลลงไฟล์ CSV แยกของ A* แม้อยู่ในโหมดจำลอง (ค่าเริ่มต้น: False คือบันทึกเฉพาะหุ่นจริงตาม RQM.md)"
    )
    args = parser.parse_args()

    print_banner()

    # 1. โหลดข้อมูลแผนที่ 4x4 และสร้างตัววางแผน A*
    planner = AStarPlanner()
    print(f"[MAP CONFIG] ขนาดสนาม: {planner.width}x{planner.height} ช่อง")
    print(f"[MAP CONFIG] ขนาดแต่ละช่อง: {planner.cell_size_m * 100:.0f} cm x {planner.cell_size_m * 100:.0f} cm")
    print(f"[MAP CONFIG] ขนาดหุ่นยนต์: ยาว {planner.robot_length_m * 100:.0f} cm, กว้าง {planner.robot_width_m * 100:.0f} cm")
    print(f"[MAP CONFIG] จุดเริ่มต้น (Start): {planner.start}")
    print(f"[MAP CONFIG] เป้าหมาย (Goal): {planner.goal}")
    print(f"[MAP CONFIG] สิ่งกีดขวาง (Obstacles): {sorted(list(planner.obstacles))}")
    print(f"[A* HEURISTIC] ฮิวริสติก: Manhattan Distance (min_cost = {planner.min_step_cost})")

    # 2. ค้นหาเส้นทางด้วย A* (A-Star)
    print("\n[PLANNING] กำลังค้นหาเส้นทางต้นทุนต่ำสุดด้วย A* (A-Star)...")
    search_result = planner.astar_search()
    path = search_result.get("path")

    if not path:
        print("[ERROR] ไม่พบเส้นทาง A* จากจุดเริ่มต้นไปยังเป้าหมาย!")
        sys.exit(1)

    steps = search_result["steps"]
    total_cost_excl = search_result["total_cost_excl"]
    total_cost_incl = search_result["total_cost_incl"]
    explored_nodes = search_result["explored_nodes"]

    print(f"[PLANNING SUCCESS] พบเส้นทาง A* ที่ดีที่สุด ({steps} ก้าว, สำรวจ {explored_nodes} โหนด):")
    print(" -> ".join([f"({x},{y})" for x, y in path]))
    print(f"[COST] ค่าใช้จ่ายรวม (ไม่รวมจุดเริ่ม): {total_cost_excl} | (รวมจุดเริ่ม): {total_cost_incl}")

    # แสดงแผนที่ ASCII และตารางการประเมิน f = g + h
    planner.print_grid(path)
    planner.print_search_summary(search_result)

    # สร้างและบันทึกรูปภาพแผนที่แสดงเส้นทางเริ่มต้นของ A* (output/path_map_astar.png)
    try:
        os.makedirs("output", exist_ok=True)
        map_file = create_path_map(path=path, output_path="output/path_map_astar.png")
        print(f"[IMAGE] บันทึกรูปแผนที่เส้นทาง A* สำเร็จ: {map_file}\n")
    except Exception as e:
        print(f"[WARNING] ไม่สามารถสร้างไฟล์รูปภาพแผนที่ได้: {e}\n")

    # 3. เตรียมระบบบันทึก CSV แยกเฉพาะของ A* และระบบควบคุมหุ่นยนต์
    is_real_run = not args.sim
    enable_logging = is_real_run or args.log_sim

    os.makedirs(os.path.dirname(args.csv) if os.path.dirname(args.csv) else ".", exist_ok=True)
    logger = NavigationLogger(
        filename=args.csv,
        enabled=enable_logging,
        append=(not args.clear_csv),
        run_id=args.run
    )
    if args.log_sim:
        logger.allow_sim_logging = True

    if is_real_run:
        print(f"[CSV CONFIG - A*] กำลังบันทึกข้อมูลหุ่นจริงลงในไฟล์ CSV แยก: {logger.filename} (รอบที่ #{logger.current_run})")
    elif args.log_sim:
        print(f"[CSV CONFIG - A*] โหมดจำลองเปิดการบันทึก (Sim Logging): {logger.filename} (รอบที่ #{logger.current_run})")
    else:
        print(f"[CSV CONFIG - A*] โหมดจำลอง (Simulation) - ไม่บันทึก CSV (เมื่อรันหุ่นจริงจะบันทึกลง: {logger.filename})")

    navigator = RobotNavigator(
        conn_type=args.conn,
        sim_mode=args.sim,
        cell_size_m=planner.cell_size_m,
        obstacle_threshold_mm=420.0,
        logger=logger
    )

    try:
        connected = navigator.connect()
        if not connected:
            print("[ERROR] ไม่สามารถเริ่มต้นระบบหุ่นยนต์ได้")
            sys.exit(1)

        def step_callback(step_num, pos):
            cost = planner.costs.get(pos, 0)
            print(f"  -> อยู่ที่ช่อง: {pos} | Cost ช่องนี้ = {cost}")

        # 4. ดำเนินการนำทางหุ่นยนต์ตามเส้นทาง A*
        success = navigator.navigate_path(
            path,
            cell_costs=planner.costs,
            on_step_callback=step_callback
        )

        if success:
            print("\n" + "=" * 65)
            print(f"  ภารกิจสำเร็จ! หุ่นยนต์เดินตามเส้นทาง A* ครบถ้วน (รอบที่ #{logger.current_run})")
            print(f"  ระยะทางเคลื่อนที่รวม: {steps * planner.cell_size_m:.2f} เมตร ({steps * planner.cell_size_m * 100:.0f} cm)")
            print(f"  ต้นทุนเส้นทาง A* (Path Cost): {total_cost_excl}")
            if logger.enabled:
                print(f"  บันทึกประวัติการเดินลงไฟล์ CSV: {logger.filename}")
            print("=" * 65)

            if logger.enabled:
                logger.print_summary()
                # สร้างแมพจากไฟล์ CSV จริงโดยอัตโนมัติ
                try:
                    run_map_file = f"output/path_map_astar_run{logger.current_run}.png"
                    plot_navigation_map_from_csv(
                        logger.filename,
                        output_path=run_map_file,
                        run_id=logger.current_run
                    )
                    csv_map_file = plot_navigation_map_from_csv(
                        logger.filename,
                        output_path="output/path_map_astar_from_csv.png",
                        run_id=logger.current_run
                    )
                    print(f"[CSV MAP] อัปเดตแผนที่จริงรอบ #{logger.current_run} จาก CSV: {csv_map_file}")

                    total_runs = logger.get_total_runs()
                    if total_runs >= 2:
                        comp_map_file = plot_multi_round_comparison(
                            logger.filename,
                            output_path="output/path_map_astar_multi_round_comparison.png"
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
