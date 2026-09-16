"""
Script สำหรับสร้างแผนที่แสดงเส้นทางการเดินของหุ่นยนต์ (Path & Telemetry Visualization)
รองรับทั้ง:
1. สร้างแผนที่จริงจากไฟล์ CSV (Actual Navigation Log from Robot Sensors) ทั้งแบบรอบเดี่ยวและเปรียบเทียบ 3 รอบ
2. สร้างแผนที่จำลองจากการคำนวณ BFS (Theoretical BFS Path)
อ้างอิง: RQM.md และ Picture2.png (สนาม 4x4, ช่องละ 60x60 cm)
"""

import os
import sys
import csv
import argparse
from typing import List, Tuple, Optional, Dict, Any

# ตั้งค่า stdout ให้รองรับ UTF-8 บน Windows
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.font_manager as fm

try:
    from bfs_planner import GridMap
except ImportError:
    try:
        from src.bfs_planner import GridMap
    except ImportError:
        sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
        from bfs_planner import GridMap


def setup_font() -> str:
    """ตั้งค่าฟอนต์ที่รองรับภาษาไทยบน Windows"""
    thai_fonts = ['Tahoma', 'Leelawadee UI', 'Leelawadee', 'Segoe UI', 'Arial']
    available_fonts = {f.name for f in fm.fontManager.ttflist}
    for font in thai_fonts:
        if font in available_fonts:
            plt.rcParams['font.sans-serif'] = [font] + plt.rcParams['font.sans-serif']
            plt.rcParams['axes.unicode_minus'] = False
            return font
    return 'sans-serif'


def load_navigation_csv(csv_path: str) -> List[Dict[str, Any]]:
    """
    โหลดข้อมูล Telemetry จากไฟล์ CSV รองรับทั้งไฟล์แบบมีคอลัมน์ run และไม่มี (ตรวจจับรอบอัตโนมัติ)
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"ไม่พบไฟล์ CSV ที่ระบุ: {csv_path}")

    records = []
    current_inferred_run = 1

    with open(csv_path, mode='r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
            if not row or not any(row.values()):
                continue

            raw_run = row.get("run")
            if raw_run is not None and raw_run.strip():
                try:
                    run_val = int(raw_run.strip())
                except ValueError:
                    run_val = 1
            else:
                step_val = int(row.get("step_round", 0))
                action_val = row.get("action", "").strip()
                if i > 0 and (step_val == 0 or action_val == "START"):
                    current_inferred_run += 1
                run_val = current_inferred_run

            parsed = {
                "run": run_val,
                "timestamp": row.get("timestamp", "").strip(),
                "step_round": int(row.get("step_round", 0)),
                "pos_x": int(row.get("pos_x", 0)),
                "pos_y": int(row.get("pos_y", 0)),
                "action": row.get("action", "").strip(),
                "target_heading_deg": float(row.get("target_heading_deg", 0.0)),
                "imu_yaw_deg": float(row.get("imu_yaw_deg", 0.0)),
                "imu_drift_deg": float(row.get("imu_drift_deg", 0.0)),
                "tof_distance_mm": float(row.get("tof_distance_mm", 0.0)),
                "gimbal_pitch_deg": float(row.get("gimbal_pitch_deg", 0.0)),
                "gimbal_yaw_deg": float(row.get("gimbal_yaw_deg", 0.0)),
                "cell_cost": int(row.get("cell_cost", 0)),
                "cumulative_cost": int(row.get("cumulative_cost", 0)),
                "distance_m": float(row.get("distance_m", 0.0)),
                "status": row.get("status", "").strip(),
                "acc_x": float(row.get("acc_x", 0.0)),
                "acc_y": float(row.get("acc_y", 0.0)),
                "acc_z": float(row.get("acc_z", 0.0)),
                "gyro_x": float(row.get("gyro_x", 0.0)),
                "gyro_y": float(row.get("gyro_y", 0.0)),
                "gyro_z": float(row.get("gyro_z", 0.0)),
            }
            records.append(parsed)

    return records


def group_records_by_run(records: List[Dict[str, Any]]) -> Dict[int, List[Dict[str, Any]]]:
    """จัดกลุ่มระเบียนข้อมูลตามหมายเลขรอบ (run)"""
    runs: Dict[int, List[Dict[str, Any]]] = {}
    for r in records:
        r_id = r.get("run", 1)
        if r_id not in runs:
            runs[r_id] = []
        runs[r_id].append(r)
    return runs


def draw_grid_map(
    ax: plt.Axes,
    grid: GridMap,
    path: List[Tuple[int, int]],
    records: Optional[List[Dict[str, Any]]] = None,
    custom_labels: Optional[Dict[int, str]] = None
):
    """
    วาดตารางสนาม 4x4 สิ่งกีดขวาง จุดเริ่ม เป้าหมาย และเส้นทางเดิน
    """
    ax.set_facecolor('#F8F9FA')
    ax.set_xlim(0.5, 4.5)
    ax.set_ylim(0.5, 4.5)
    ax.set_aspect('equal')

    # 1. วาดแต่ละช่อง (Cells)
    for y in range(1, 5):
        for x in range(1, 5):
            pos = (x, y)
            is_start = (pos == grid.start)
            is_goal = (pos == grid.goal)
            is_obstacle = (pos in grid.obstacles)
            cost = grid.costs.get(pos, 0)

            # กำหนดสีพื้นหลัง
            if is_start:
                facecolor = '#D4EDDA'
                edgecolor = '#28A745'
            elif is_goal:
                facecolor = '#F8D7DA'
                edgecolor = '#DC3545'
            elif is_obstacle:
                facecolor = '#E9ECEF'
                edgecolor = '#6C757D'
            else:
                facecolor = '#FFFFFF'
                edgecolor = '#DEE2E6'

            rect = patches.Rectangle(
                (x - 0.48, y - 0.48),
                0.96, 0.96,
                linewidth=1.8,
                edgecolor=edgecolor,
                facecolor=facecolor,
                zorder=1
            )
            ax.add_patch(rect)

            # แสดงพิกัดมุมบนซ้าย (x, y)
            ax.text(
                x - 0.42, y + 0.36,
                f"({x}, {y})",
                fontsize=10,
                fontweight='bold',
                color='#6C757D',
                ha='left',
                va='center',
                zorder=2
            )

            # รายละเอียดในช่อง
            if is_start:
                ax.text(x, y + 0.22, "START", fontsize=11, fontweight='bold', color='#155724', ha='center', va='center', zorder=3)
                ax.text(x, y - 0.33, f"Cost = {cost}", fontsize=10, fontweight='bold', color='#155724', ha='center', va='center', zorder=3)
            elif is_goal:
                ax.text(x, y + 0.22, "GOAL", fontsize=11, fontweight='bold', color='#721C24', ha='center', va='center', zorder=3)
                ax.text(x, y - 0.33, f"Cost = {cost}", fontsize=10, fontweight='bold', color='#721C24', ha='center', va='center', zorder=3)
            elif is_obstacle:
                obs_box = patches.Rectangle((x - 0.24, y - 0.10), 0.48, 0.36, facecolor='#343A40', edgecolor='#212529', linewidth=1.5, zorder=2)
                ax.add_patch(obs_box)
                ax.text(x, y - 0.32, "OBSTACLE", fontsize=9, fontweight='bold', color='#DC3545', ha='center', va='center', zorder=3)
            else:
                ax.text(x, y - 0.33, f"Cost = {cost}", fontsize=10, color='#6C757D', ha='center', va='center', zorder=3)

    # 2. วาดเส้นทางการเดิน
    if path and len(path) > 1:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]

        # เส้นเชื่อมทางเดินหลัก
        ax.plot(xs, ys, color='#0D6EFD', linewidth=4.5, linestyle='-', alpha=0.9, zorder=4)

        # ลูกศรบอกทิศทางในแต่ละช่วง
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            dx = (x2 - x1) * 0.48
            dy = (y2 - y1) * 0.48
            ax.annotate(
                '',
                xy=(x1 + dx * 1.45, y1 + dy * 1.45),
                xytext=(x1 + dx * 0.55, y1 + dy * 0.55),
                arrowprops=dict(
                    arrowstyle='-|>',
                    color='#0A58CA',
                    lw=3.0,
                    mutation_scale=18
                ),
                zorder=5
            )

        # วาดวงกลมสัญลักษณ์ Step แต่ละจุด
        for step_idx, (px, py) in enumerate(path):
            if step_idx == 0:
                badge_col = '#198754'
                step_txt = 'S'
            elif step_idx == len(path) - 1:
                badge_col = '#DC3545'
                step_txt = 'G'
            else:
                badge_col = '#0D6EFD'
                step_txt = f"#{step_idx}"

            circle = patches.Circle((px, py - 0.02), 0.14, color=badge_col, ec='white', lw=2.0, zorder=6)
            ax.add_patch(circle)
            ax.text(px, py - 0.02, step_txt, color='white', fontsize=9, fontweight='bold', ha='center', va='center', zorder=7)

            # ตำแหน่ง offset สำหรับ callout tag
            ox, oy = 0.0, 0.18
            if step_idx == 0 or step_idx == 3 or step_idx == 6:
                ox, oy = 0.0, -0.18

            # ป้ายข้อความเซนเซอร์
            sensor_str = None
            if custom_labels and step_idx in custom_labels:
                sensor_str = custom_labels[step_idx]
            elif records and step_idx < len(records):
                rec = records[step_idx]
                tof_val = rec["tof_distance_mm"]
                yaw_val = rec["imu_yaw_deg"]
                sensor_str = f"#{step_idx}: ToF {tof_val:.0f}mm\nYaw {yaw_val:.1f}°"

            if sensor_str:
                ax.text(
                    px + ox, py + oy,
                    sensor_str,
                    fontsize=7.5,
                    fontweight='bold',
                    ha='center',
                    va='center',
                    color='#0F5132' if step_idx == 0 else ('#842029' if step_idx == len(path)-1 else '#084298'),
                    bbox=dict(
                        boxstyle='round,pad=0.25',
                        facecolor='#D1E7DD' if step_idx == 0 else ('#F8D7DA' if step_idx == len(path)-1 else '#CFE2FF'),
                        edgecolor='white',
                        alpha=0.92,
                        lw=1.0
                    ),
                    zorder=8
                )

    # ปรับแกน
    ax.set_xticks([1, 2, 3, 4])
    ax.set_yticks([1, 2, 3, 4])
    ax.set_xticklabels(['x = 1', 'x = 2', 'x = 3', 'x = 4'], fontsize=11, fontweight='bold')
    ax.set_yticklabels(['y = 1', 'y = 2', 'y = 3', 'y = 4'], fontsize=11, fontweight='bold')
    ax.grid(False)

    outer_box = patches.Rectangle((0.5, 0.5), 4.0, 4.0, linewidth=2.5, edgecolor='#212529', facecolor='none', zorder=9)
    ax.add_patch(outer_box)


def plot_navigation_map_from_csv(
    csv_path: str,
    output_path: str = "output/path_map_from_csv.png",
    run_id: Optional[int] = None,
    show_plot: bool = False
) -> str:
    """
    อ่านประวัติการเดินจริงจากไฟล์ CSV สำหรับรอบที่กำหนด (หรือรอบล่าสุด) และสร้างแผนที่พร้อม Dashboard
    """
    setup_font()
    grid = GridMap()
    all_records = load_navigation_csv(csv_path)

    if not all_records:
        raise ValueError(f"ไฟล์ CSV ไม่มีข้อมูล: {csv_path}")

    runs_dict = group_records_by_run(all_records)
    available_runs = sorted(list(runs_dict.keys()))

    if run_id is not None:
        if run_id not in runs_dict:
            raise ValueError(f"ไม่พบข้อมูลรอบที่ {run_id} ในไฟล์ CSV (รอบที่มี: {available_runs})")
        target_run = run_id
    else:
        target_run = available_runs[-1]  # ค่าเริ่มต้น: รอบล่าสุด

    records = runs_dict[target_run]
    total_runs_in_csv = len(available_runs)

    # ดึงเส้นทางพิกัดของรอบนี้
    path = [(r["pos_x"], r["pos_y"]) for r in records]

    # คำนวณสรุปสถิติ
    total_steps = len(records) - 1
    total_distance_m = records[-1]["distance_m"] if records else 0.0
    total_cost = records[-1]["cumulative_cost"] if records else 0
    final_status = records[-1]["status"] if records else "UNKNOWN"
    start_time = records[0]["timestamp"] if records else ""
    end_time = records[-1]["timestamp"] if records else ""

    fig = plt.figure(figsize=(16, 10), dpi=160)
    fig.patch.set_facecolor('#F4F6F9')

    gs = fig.add_gridspec(3, 2, width_ratios=[1.2, 1.05], height_ratios=[0.95, 0.95, 1.1], hspace=0.35, wspace=0.25)

    # 1. แผนที่สนาม 4x4
    ax_map = fig.add_subplot(gs[:, 0])
    draw_grid_map(ax_map, grid, path, records=records)
    run_info_str = f"รอบที่ {target_run} (จากสะสมทั้งหมด {total_runs_in_csv} รอบ)" if total_runs_in_csv > 1 else f"รอบที่ {target_run}"
    ax_map.set_title(
        f"แผนที่การเดินของหุ่นยนต์จากเซนเซอร์จริง - {run_info_str}\n"
        f"ระยะทาง: {total_distance_m:.2f} m | ก้าว: {total_steps} ก้าว | ต้นทุน (Cost): {total_cost} | สถานะ: {final_status}",
        fontsize=12.5,
        fontweight='bold',
        pad=12,
        color='#212529'
    )

    # 2. การ์ดสรุป KPI ด้านขวาบน
    ax_kpi = fig.add_subplot(gs[0, 1])
    ax_kpi.axis('off')
    ax_kpi.set_facecolor('#FFFFFF')

    kpi_box = patches.FancyBboxPatch(
        (0.01, 0.05), 0.98, 0.90,
        boxstyle="round,pad=0.03,rounding_size=0.05",
        edgecolor='#B6D4FE',
        facecolor='#E7F1FF',
        linewidth=1.5,
        transform=ax_kpi.transAxes
    )
    ax_kpi.add_patch(kpi_box)

    kpi_title = f"[LOG REPORT] ข้อมูลการเดินจริง: {os.path.basename(csv_path)} (รอบที่ #{target_run})"
    ax_kpi.text(0.05, 0.80, kpi_title, transform=ax_kpi.transAxes, fontsize=12, fontweight='bold', color='#084298')
    ax_kpi.text(0.05, 0.61, f"• จุดเริ่ม: {grid.start}  ->  เป้าหมาย: {grid.goal}", transform=ax_kpi.transAxes, fontsize=10.5, color='#212529')
    ax_kpi.text(0.05, 0.44, f"• จำนวนก้าวเดิน: {total_steps} ก้าว ({total_distance_m:.2f} เมตร)", transform=ax_kpi.transAxes, fontsize=10.5, color='#212529')
    ax_kpi.text(0.05, 0.27, f"• ต้นทุนรวม (Cumulative Cost): {total_cost} หน่วย", transform=ax_kpi.transAxes, fontsize=10.5, color='#212529')
    start_t_clean = start_time.split()[1] if " " in start_time else start_time
    end_t_clean = end_time.split()[1] if " " in end_time else end_time
    ax_kpi.text(0.05, 0.10, f"• เวลาที่บันทึก: {start_t_clean} -> {end_t_clean} ({final_status})", transform=ax_kpi.transAxes, fontsize=10.5, color='#155724', fontweight='bold')

    # 3. กราฟ Telemetry เซนเซอร์ ToF & IMU Yaw
    ax_chart = fig.add_subplot(gs[1, 1])
    steps_list = [r["step_round"] for r in records]
    tof_list = [r["tof_distance_mm"] for r in records]
    yaw_list = [r["imu_yaw_deg"] for r in records]

    color_tof = '#0D6EFD'
    color_yaw = '#D63384'

    ax_chart.set_facecolor('#FFFFFF')
    line1 = ax_chart.plot(steps_list, tof_list, color=color_tof, marker='o', linewidth=2.0, label='ToF Distance (mm)')
    line_thresh = ax_chart.axhline(420.0, color='#DC3545', linestyle='--', linewidth=1.5, label='เกณฑ์สิ่งกีดขวาง (420 mm)')
    ax_chart.set_xlabel('รอบการเดิน (Step Round)', fontsize=9.5, fontweight='bold')
    ax_chart.set_ylabel('ระยะ ToF (mm)', fontsize=9.5, color=color_tof, fontweight='bold')
    ax_chart.tick_params(axis='y', labelcolor=color_tof)
    ax_chart.set_xticks(steps_list)
    ax_chart.grid(True, linestyle=':', alpha=0.6)

    ax_yaw = ax_chart.twinx()
    line2 = ax_yaw.plot(steps_list, yaw_list, color=color_yaw, marker='s', linestyle='-.', linewidth=1.8, label='IMU Yaw (°)')
    ax_yaw.set_ylabel('มุมหัน IMU Yaw (°)', fontsize=9.5, color=color_yaw, fontweight='bold')
    ax_yaw.tick_params(axis='y', labelcolor=color_yaw)

    lines = line1 + [line_thresh] + line2
    labels = [l.get_label() for l in lines]
    ax_chart.legend(lines, labels, loc='upper right', fontsize=8.5, framealpha=0.85)
    ax_chart.set_title(f"[SENSOR TELEMETRY - รอบ #{target_run}] ToF Distance & IMU Yaw ในแต่ละก้าว", fontsize=11, fontweight='bold', pad=8)

    # 4. ตารางบันทึก Telemetry ในแต่ละรอบ
    ax_tbl = fig.add_subplot(gs[2, 1])
    ax_tbl.axis('off')

    col_labels = ["Step", "พิกัด", "Action", "Cost", "Cost รวม", "ToF (mm)", "Yaw (°)", "Status"]
    table_data = []
    for r in records:
        step_label = f"#{r['step_round']}"
        pos_str = f"({r['pos_x']},{r['pos_y']})"
        action_short = "START" if r['step_round'] == 0 else ("GOAL" if r['step_round'] == total_steps else "MOVE")
        status_clean = "SUCCESS" if r["status"] in ("SUCCESS", "GOAL_REACHED") else r["status"]
        table_data.append([
            step_label,
            pos_str,
            action_short,
            str(r["cell_cost"]),
            str(r["cumulative_cost"]),
            f"{r['tof_distance_mm']:.0f}",
            f"{r['imu_yaw_deg']:.1f}",
            status_clean
        ])

    table = ax_tbl.table(
        cellText=table_data,
        colLabels=col_labels,
        loc='center',
        cellLoc='center',
        colColours=['#E2E3E5'] * len(col_labels)
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8.5)
    table.scale(1.0, 1.45)

    for (row_idx, col_idx), cell in table.get_celld().items():
        if row_idx == 0:
            cell.set_text_props(fontweight='bold', color='#212529')
            cell.set_facecolor('#CED4DA')
        elif row_idx == 1:
            cell.set_facecolor('#D1E7DD')
        elif row_idx == len(records):
            cell.set_facecolor('#F8D7DA')
        elif row_idx % 2 == 0:
            cell.set_facecolor('#F8F9FA')

    ax_tbl.set_title(f"[LOG TABLE - รอบ #{target_run}] ตารางข้อมูล Telemetry จากไฟล์ CSV", fontsize=11, fontweight='bold', pad=10)

    footer_text = (
        f"ไฟล์: {csv_path} (รอบที่ {target_run}/{total_runs_in_csv}) | ช่องละ 60x60 cm | ToF ล็อกกึ่งกลางตัวรถ | ตรวจสอบ IMU ทุกการก้าว"
    )
    fig.text(0.5, 0.015, footer_text, ha='center', va='bottom', fontsize=9.5, color='#495057')

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    abs_output = os.path.abspath(output_path)
    plt.savefig(abs_output, dpi=200, bbox_inches='tight')
    if show_plot:
        plt.show()
    plt.close(fig)

    print(f"[MAP FROM CSV] บันทึกแผนที่รอบที่ #{target_run}: {abs_output}")
    return abs_output


def plot_multi_round_comparison(
    csv_path: str,
    output_path: str = "output/path_map_multi_round_comparison.png",
    show_plot: bool = False
) -> str:
    """
    สร้างภาพเปรียบเทียบผลการทดสอบหลายรอบ (Multi-Round Comparison Dashboard)
    เช่น เปรียบเทียบผลการเดิน 3 รอบ: ความแปรปรวนเซนเซอร์ ToF, มุม IMU Yaw, และเสถียรภาพ
    """
    setup_font()
    grid = GridMap()
    all_records = load_navigation_csv(csv_path)

    if not all_records:
        raise ValueError(f"ไฟล์ CSV ไม่มีข้อมูล: {csv_path}")

    runs_dict = group_records_by_run(all_records)
    run_ids = sorted(list(runs_dict.keys()))
    num_runs = len(run_ids)

    # ใช้เส้นทางอ้างอิงจากรอบที่ 1
    ref_records = runs_dict[run_ids[0]]
    path = [(r["pos_x"], r["pos_y"]) for r in ref_records]
    num_steps = len(path)

    # คำนวณค่าเฉลี่ยเซนเซอร์ของแต่ละ Step เพื่อนำไปแสดงบนแผนที่
    custom_labels = {}
    for s in range(num_steps):
        tofs = [runs_dict[r_id][s]["tof_distance_mm"] for r_id in run_ids if s < len(runs_dict[r_id])]
        yaws = [runs_dict[r_id][s]["imu_yaw_deg"] for r_id in run_ids if s < len(runs_dict[r_id])]
        avg_tof = sum(tofs) / len(tofs) if tofs else 0.0
        avg_yaw = sum(yaws) / len(yaws) if yaws else 0.0
        custom_labels[s] = f"#{s}: ToF เฉลี่ย {avg_tof:.0f}mm\nYaw เฉลี่ย {avg_yaw:.1f}°"

    # สร้างรูป Dashboard ขนาดใหญ่
    fig = plt.figure(figsize=(18, 11), dpi=180)
    fig.patch.set_facecolor('#F4F6F9')

    gs = fig.add_gridspec(3, 2, width_ratios=[1.15, 1.15], height_ratios=[0.85, 1.05, 1.1], hspace=0.32, wspace=0.22)

    # 1. แผนที่สนาม 4x4 (คอลัมน์ซ้าย)
    ax_map = fig.add_subplot(gs[:, 0])
    draw_grid_map(ax_map, grid, path, custom_labels=custom_labels)
    ax_map.set_title(
        f"แผนที่เปรียบเทียบการเดินหุ่นยนต์ {num_runs} รอบ (RoboMaster EP Multi-Round BFS)\n"
        f"แสดงค่าเฉลี่ย ToF & Yaw จากการทดสอบจริงสะสม {num_runs} รอบ | ช่องละ 60x60 cm",
        fontsize=12.5,
        fontweight='bold',
        pad=12,
        color='#212529'
    )

    # 2. การ์ดสรุป KPI (ขวาบน)
    ax_kpi = fig.add_subplot(gs[0, 1])
    ax_kpi.axis('off')
    kpi_box = patches.FancyBboxPatch(
        (0.01, 0.05), 0.98, 0.90,
        boxstyle="round,pad=0.03,rounding_size=0.05",
        edgecolor='#198754',
        facecolor='#D1E7DD',
        linewidth=1.5,
        transform=ax_kpi.transAxes
    )
    ax_kpi.add_patch(kpi_box)

    kpi_title = f"[MULTI-ROUND REPORT] รายงานผลการทดสอบการเดินสะสม {num_runs} รอบ"
    ax_kpi.text(0.04, 0.78, kpi_title, transform=ax_kpi.transAxes, fontsize=12, fontweight='bold', color='#0F5132')
    ax_kpi.text(0.04, 0.58, f"• จำนวนรอบที่บันทึก: {num_runs} รอบ (รอบที่: {', '.join([str(r) for r in run_ids])}) | เส้นทาง BFS 6 ก้าว (3.60 m)", transform=ax_kpi.transAxes, fontsize=10, color='#212529')
    
    # คำนวณความสม่ำเสมอ
    status_all = [runs_dict[r_id][-1]["status"] for r_id in run_ids]
    success_count = sum(1 for st in status_all if st in ("SUCCESS", "GOAL_REACHED"))
    ax_kpi.text(0.04, 0.38, f"• อัตราความสำเร็จ (Success Rate): {success_count}/{num_runs} รอบ ({(success_count/num_runs)*100:.0f}%) บรรลุเป้าหมายครบถ้วน", transform=ax_kpi.transAxes, fontsize=10, color='#212529')

    last_step_tofs = [runs_dict[r_id][-1]["tof_distance_mm"] for r_id in run_ids]
    tof_min, tof_max = min(last_step_tofs), max(last_step_tofs)
    ax_kpi.text(0.04, 0.16, f"• ช่วงระยะ ToF ที่ Goal (4,1): {tof_min:.0f} - {tof_max:.0f} mm (ความแปรปรวน {(tof_max - tof_min):.0f} mm)", transform=ax_kpi.transAxes, fontsize=10, color='#155724', fontweight='bold')

    # 3. กราฟเปรียบเทียบ ToF และ IMU Yaw ข้ามรอบ (ขวากลาง)
    ax_chart = fig.add_subplot(gs[1, 1])
    ax_chart.set_facecolor('#FFFFFF')

    # พาเลตต์สีสำหรับรอบต่างๆ
    run_colors = ['#0D6EFD', '#198754', '#FD7E14', '#6F42C1', '#20C997']
    steps_list = list(range(num_steps))

    for idx, r_id in enumerate(run_ids):
        col = run_colors[idx % len(run_colors)]
        tofs = [runs_dict[r_id][s]["tof_distance_mm"] for s in range(num_steps) if s < len(runs_dict[r_id])]
        ax_chart.plot(steps_list[:len(tofs)], tofs, marker='o', linewidth=2.0, color=col, label=f"ToF รอบ #{r_id}")

    ax_chart.axhline(420.0, color='#DC3545', linestyle='--', linewidth=1.5, label='เกณฑ์สิ่งกีดขวาง (420 mm)')
    ax_chart.set_xlabel('รอบการเดิน (Step Round)', fontsize=9.5, fontweight='bold')
    ax_chart.set_ylabel('ระยะ ToF (mm)', fontsize=9.5, color='#0D6EFD', fontweight='bold')
    ax_chart.tick_params(axis='y', labelcolor='#0D6EFD')
    ax_chart.set_xticks(steps_list)
    ax_chart.grid(True, linestyle=':', alpha=0.6)

    # แกนคู่ Yaw
    ax_yaw = ax_chart.twinx()
    for idx, r_id in enumerate(run_ids):
        col = run_colors[idx % len(run_colors)]
        yaws = [runs_dict[r_id][s]["imu_yaw_deg"] for s in range(num_steps) if s < len(runs_dict[r_id])]
        ax_yaw.plot(steps_list[:len(yaws)], yaws, marker='s', linestyle='-.', linewidth=1.8, color=col, alpha=0.75, label=f"Yaw รอบ #{r_id}")

    ax_yaw.set_ylabel('มุมหัน IMU Yaw (°)', fontsize=9.5, color='#495057', fontweight='bold')
    ax_yaw.tick_params(axis='y', labelcolor='#495057')

    h1, l1 = ax_chart.get_legend_handles_labels()
    h2, l2 = ax_yaw.get_legend_handles_labels()
    ax_chart.legend(h1 + h2, l1 + l2, loc='upper right', fontsize=8, framealpha=0.9, ncol=2)
    ax_chart.set_title(f"[COMPARISON] เปรียบเทียบเซนเซอร์ ToF และ Yaw ของทั้ง {num_runs} รอบ", fontsize=11, fontweight='bold', pad=8)

    # 4. ตารางเปรียบเทียบข้อมูลละเอียด (ขวาล่าง)
    ax_tbl = fig.add_subplot(gs[2, 1])
    ax_tbl.axis('off')

    col_labels = ["Step", "พิกัด"]
    for r_id in run_ids:
        col_labels.append(f"ToF R{r_id}")
    col_labels.append("เฉลี่ย ToF")
    for r_id in run_ids:
        col_labels.append(f"Yaw R{r_id}")
    col_labels.append("เฉลี่ย Yaw")

    table_data = []
    for s in range(num_steps):
        pos_str = f"({path[s][0]},{path[s][1]})"
        tofs = [runs_dict[r_id][s]["tof_distance_mm"] for r_id in run_ids if s < len(runs_dict[r_id])]
        yaws = [runs_dict[r_id][s]["imu_yaw_deg"] for r_id in run_ids if s < len(runs_dict[r_id])]
        avg_tof = sum(tofs) / len(tofs) if tofs else 0.0
        avg_yaw = sum(yaws) / len(yaws) if yaws else 0.0

        row = [f"#{s}", pos_str]
        for tf in tofs:
            row.append(f"{tf:.0f}")
        row.append(f"{avg_tof:.0f}")
        for yw in yaws:
            row.append(f"{yw:.1f}")
        row.append(f"{avg_yaw:.1f}")
        table_data.append(row)

    table = ax_tbl.table(
        cellText=table_data,
        colLabels=col_labels,
        loc='center',
        cellLoc='center',
        colColours=['#E2E3E5'] * len(col_labels)
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8.0)
    table.scale(1.0, 1.45)

    for (row_idx, col_idx), cell in table.get_celld().items():
        if row_idx == 0:
            cell.set_text_props(fontweight='bold', color='#212529')
            cell.set_facecolor('#CED4DA')
        elif row_idx == 1:
            cell.set_facecolor('#D1E7DD')
        elif row_idx == num_steps:
            cell.set_facecolor('#F8D7DA')
        elif row_idx % 2 == 0:
            cell.set_facecolor('#F8F9FA')

    ax_tbl.set_title(f"[COMPARISON TABLE] ตารางเปรียบเทียบค่าเซนเซอร์จริงทั้ง {num_runs} รอบ", fontsize=11, fontweight='bold', pad=10)

    footer_text = (
        f"ไฟล์ข้อมูล: {csv_path} | จำนวนรอบทดสอบ: {num_runs} รอบ | หุ่นยนต์ RoboMaster EP (CHASSIS_LEAD Gimbal Lock) | BFS Shortest Path"
    )
    fig.text(0.5, 0.015, footer_text, ha='center', va='bottom', fontsize=9.5, color='#495057')

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    abs_output = os.path.abspath(output_path)
    plt.savefig(abs_output, dpi=200, bbox_inches='tight')
    if show_plot:
        plt.show()
    plt.close(fig)

    print(f"[MULTI-ROUND MAP] บันทึกแผนที่เปรียบเทียบ {num_runs} รอบ: {abs_output}")
    return abs_output


def create_path_map(
    path: Optional[List[Tuple[int, int]]] = None,
    output_path: str = "output/path_map.png",
    show_plot: bool = False
) -> str:
    """
    สร้างรูปภาพแผนที่แสดงสนาม 4x4 พร้อมเส้นทาง BFS (Simulation/Theoretical)
    """
    setup_font()
    grid = GridMap()

    if path is None:
        path = grid.bfs_shortest_path(optimize_cost_among_shortest=True)

    fig, ax = plt.subplots(figsize=(10, 10), dpi=160)
    fig.patch.set_facecolor('#F8F9FA')

    draw_grid_map(ax, grid, path, records=None)

    steps = len(path) - 1 if path else 0
    total_cost_excl = grid.calculate_path_cost(path, include_start=False) if path else 0
    total_distance_m = steps * grid.cell_size_m

    plt.title(
        "สนามขนาด 4 x 4 พร้อมเส้นทางเดินหุ่นยนต์ BFS (RoboMaster EP)\n"
        f"ระยะก้าว: {steps} ก้าว ({total_distance_m:.2f} m) | Cost รวม: {total_cost_excl} | ช่องละ 60x60 cm",
        fontsize=13,
        fontweight='bold',
        pad=16,
        color='#212529'
    )

    info_text = (
        f"จุดเริ่ม (Start): (1, 4)  |  เป้าหมาย (Goal): (4, 1)  |  สิ่งกีดขวาง: (4,4), (2,3), (3,2)\n"
        f"เซนเซอร์: ToF ตรวจสิ่งกีดขวาง (< 420 mm หยุด) | IMU ยืนยันมุมหัน | Gimbal Lock กึ่งกลาง (CHASSIS_LEAD)"
    )
    fig.text(
        0.5, 0.02,
        info_text,
        ha='center',
        va='bottom',
        fontsize=9.5,
        bbox=dict(boxstyle='round,pad=0.5', facecolor='#E9ECEF', edgecolor='#CED4DA', lw=1.2)
    )

    plt.tight_layout(rect=[0.02, 0.06, 0.98, 0.98])
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    abs_output = os.path.abspath(output_path)
    plt.savefig(abs_output, dpi=200, bbox_inches='tight')
    if show_plot:
        plt.show()
    plt.close(fig)

    print(f"[MAP GENERATED] สร้างแผนที่แสดงเส้นทางสำเร็จ: {abs_output}")
    return abs_output


def main():
    parser = argparse.ArgumentParser(description="สร้างแผนที่แสดงเส้นทางเดินของหุ่นยนต์ (จาก CSV หรือ BFS)")
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="พาธไฟล์ CSV บันทึกการเดิน (เช่น logs/navigation_log.csv)"
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default=None,
        help="ชื่อไฟล์รูปภาพปลายทาง (ค่าเริ่มต้น: output/path_map_from_csv.png หรือ output/path_map.png)"
    )
    parser.add_argument(
        "--run",
        type=int,
        default=None,
        help="ระบุหมายเลขรอบที่ต้องการพล็อต (หากไม่ระบุจะพล็อตภาพรอบล่าสุดและภาพเปรียบเทียบทุกรอบ)"
    )
    parser.add_argument(
        "--compare",
        action="store_true",
        help="สร้างแผนที่เปรียบเทียบผลทุกรอบ (Multi-Round Comparison Dashboard)"
    )
    parser.add_argument(
        "--bfs",
        action="store_true",
        help="บังคับสร้างแผนที่จากอัลกอริทึม BFS เท่านั้น (ไม่ใช้ CSV)"
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="เปิดแสดงหน้าต่างรูปภาพทันที"
    )
    args = parser.parse_args()

    # ตรวจหาไฟล์ CSV อัตโนมัติหากไม่ได้ระบุ --bfs
    target_csv = args.csv
    if not args.bfs and not target_csv:
        candidate_paths = [
            "logs/navigation_log.csv",
            "navigation_log.csv",
            os.path.join(os.path.dirname(__file__), "..", "logs", "navigation_log.csv")
        ]
        for p in candidate_paths:
            if os.path.exists(p) and os.path.getsize(p) > 0:
                target_csv = p
                break

    if target_csv and not args.bfs:
        records = load_navigation_csv(target_csv)
        runs_dict = group_records_by_run(records)
        total_runs = len(runs_dict)

        if args.compare or (args.run is None and total_runs >= 2):
            # สร้าง Dashboard เปรียบเทียบหลายรอบ
            comp_file = args.output or "output/path_map_multi_round_comparison.png"
            saved_comp = plot_multi_round_comparison(target_csv, output_path=comp_file, show_plot=args.show)
            print(f"บันทึกแผนที่เปรียบเทียบผล {total_runs} รอบเรียบร้อย: {saved_comp}")

        # สร้างแผนที่รอบเดี่ยว (รอบที่เลือก หรือรอบล่าสุด)
        out_file = args.output or "output/path_map_from_csv.png"
        saved = plot_navigation_map_from_csv(target_csv, output_path=out_file, run_id=args.run, show_plot=args.show)
        print(f"บันทึกแผนที่จาก CSV เรียบร้อย: {saved}")
    else:
        out_file = args.output or "output/path_map.png"
        saved = create_path_map(output_path=out_file, show_plot=args.show)
        print(f"บันทึกแผนที่ BFS เรียบร้อย: {saved}")


if __name__ == "__main__":
    main()
