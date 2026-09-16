"""
DFS (Depth-First Search) Path Planning for 4x4 Grid
สร้างทางเดินด้วย Depth-First Search (DFS) โดยคำนวณ Total Cost และเวลาประมวลผล (Computation Time)
อ้างอิง: docs/RQM.md & docs/Picture2.png (สนามขนาด 4 x 4 ช่อง)
โหมด: คำนวณเส้นทางเพียวๆ โดยไม่ต้องเชื่อมต่อหรือรันหุ่นจริง (Offline / Pure Simulation)
"""

import os
import sys
import time
import argparse
from typing import List, Tuple, Dict, Optional, Set, Any

# ตั้งค่า encoding ของ stdout ให้รองรับภาษาไทยบน Windows
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass


class DFSPlanner:
    """
    คลาสสำหรับค้นหาเส้นทางด้วย Depth-First Search (DFS)
    บนตารางขนาด 4x4 ช่อง ตาม Picture2.png
    """
    def __init__(
        self,
        start: Tuple[int, int] = (1, 4),
        goal: Tuple[int, int] = (4, 1),
        width: int = 4,
        height: int = 4,
        cell_size_m: float = 0.60
    ):
        self.width = width
        self.height = height
        self.cell_size_m = cell_size_m
        self.start = start
        self.goal = goal

        # สิ่งกีดขวางตาม Picture2.png
        self.obstacles: Set[Tuple[int, int]] = {
            (4, 4),
            (2, 3),
            (3, 2)
        }

        # ต้นทุน (Cost) ของแต่ละช่องตาม Picture2.png
        self.costs: Dict[Tuple[int, int], int] = {
            # y = 4
            (1, 4): 2, (2, 4): 3, (3, 4): 1, (4, 4): -1,
            # y = 3
            (1, 3): 2, (2, 3): -1, (3, 3): 4, (4, 3): 2,
            # y = 2
            (1, 2): 3, (2, 2): 1, (3, 2): -1, (4, 2): 4,
            # y = 1
            (1, 1): 2, (2, 1): 3, (3, 1): 2, (4, 1): 1,
        }

    def is_valid(self, pos: Tuple[int, int]) -> bool:
        """ตรวจสอบว่าตำแหน่งอยู่ในสนามและไม่ชนสิ่งกีดขวาง"""
        x, y = pos
        if not (1 <= x <= self.width and 1 <= y <= self.height):
            return False
        if pos in self.obstacles:
            return False
        return True

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """คืนค่าช่องข้างเคียง 4 ทิศทาง (ขวา, ลง, ซ้าย, ขึ้น)"""
        x, y = pos
        candidates = [
            (x + 1, y),  # ขวา (+x)
            (x, y - 1),  # ลง (-y)
            (x - 1, y),  # ซ้าย (-x)
            (x, y + 1),  # ขึ้น (+y)
        ]
        return [c for c in candidates if self.is_valid(c)]

    def compute_cost(self, path: List[Tuple[int, int]]) -> Tuple[int, int]:
        """
        คำนวณ Total Cost ของเส้นทาง:
        - total_cost_excl: ผลรวมต้นทุนไม่รวมช่องเริ่มต้น (ต้นทุนการย้ายเข้าแต่ละช่อง)
        - total_cost_incl: ผลรวมต้นทุนรวมช่องเริ่มต้น
        """
        if not path:
            return 0, 0
        cost_excl = sum(self.costs.get(p, 0) for p in path[1:])
        cost_incl = sum(self.costs.get(p, 0) for p in path)
        return cost_excl, cost_incl

    def dfs_first_path(self) -> Dict[str, Any]:
        """
        ค้นหาเส้นทางแบบ DFS มาตรฐาน (First Found)
        สำรวจแบบเจาะลึก (Depth-First) ตามลำดับกิ่งแรกที่พบจนถึงเป้าหมาย
        จับเวลาประมวลผล (Computation Time) ด้วย time.perf_counter()
        """
        explored_nodes = 0
        explored_order = []

        start_time = time.perf_counter()

        stack = [(self.start, [self.start], {self.start})]
        found_path: Optional[List[Tuple[int, int]]] = None

        while stack:
            current, path, visited = stack.pop()
            explored_nodes += 1
            explored_order.append(current)

            if current == self.goal:
                found_path = path
                break

            # กลับลำดับ neighbors เพื่อให้ pop ออกมาสำรวจตามลำดับ ขวา -> ลง -> ซ้าย -> ขึ้น
            neighbors = self.get_neighbors(current)
            for neighbor in reversed(neighbors):
                if neighbor not in visited:
                    new_visited = visited | {neighbor}
                    stack.append((neighbor, path + [neighbor], new_visited))

        end_time = time.perf_counter()
        computation_time_sec = end_time - start_time

        cost_excl, cost_incl = self.compute_cost(found_path) if found_path else (0, 0)

        return {
            "algorithm": "DFS (Depth-First Search - First Found)",
            "path": found_path,
            "steps": len(found_path) - 1 if found_path else 0,
            "total_cost_excl": cost_excl,
            "total_cost_incl": cost_incl,
            "explored_nodes": explored_nodes,
            "computation_time_sec": computation_time_sec,
            "computation_time_ms": computation_time_sec * 1000.0,
            "computation_time_us": computation_time_sec * 1_000_000.0,
        }

    def dfs_all_paths(self) -> Dict[str, Any]:
        """
        ค้นหาเส้นทางที่เป็นไปได้ทั้งหมดด้วย DFS Backtracking
        เพื่อเปรียบเทียบ Total Cost ของทุกเส้นทางและหาเส้นทาง DFS ที่ต้นทุนต่ำที่สุด
        """
        all_paths = []
        explored_nodes = 0

        start_time = time.perf_counter()

        def backtrack(current: Tuple[int, int], path: List[Tuple[int, int]], visited: Set[Tuple[int, int]]):
            nonlocal explored_nodes
            explored_nodes += 1

            if current == self.goal:
                cost_excl, cost_incl = self.compute_cost(path)
                all_paths.append({
                    "path": path.copy(),
                    "steps": len(path) - 1,
                    "total_cost_excl": cost_excl,
                    "total_cost_incl": cost_incl,
                })
                return

            for neighbor in self.get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    path.append(neighbor)
                    backtrack(neighbor, path, visited)
                    path.pop()
                    visited.remove(neighbor)

        backtrack(self.start, [self.start], {self.start})

        end_time = time.perf_counter()
        computation_time_sec = end_time - start_time

        # เรียงลำดับตาม Total Cost ต่ำสุด
        all_paths.sort(key=lambda p: (p["total_cost_excl"], p["steps"]))
        best_path_info = all_paths[0] if all_paths else None

        return {
            "algorithm": "DFS (Depth-First Search - All Paths Exhaustive)",
            "all_paths": all_paths,
            "best_path": best_path_info["path"] if best_path_info else None,
            "best_cost_excl": best_path_info["total_cost_excl"] if best_path_info else 0,
            "best_cost_incl": best_path_info["total_cost_incl"] if best_path_info else 0,
            "total_paths_found": len(all_paths),
            "explored_nodes": explored_nodes,
            "computation_time_sec": computation_time_sec,
            "computation_time_ms": computation_time_sec * 1000.0,
            "computation_time_us": computation_time_sec * 1_000_000.0,
        }

    def print_ascii_grid(self, path: Optional[List[Tuple[int, int]]] = None):
        """แสดงแผนที่สนาม 4x4 เป็นตาราง ASCII พร้อมเส้นทางเดิน"""
        path_set = set(path) if path else set()
        path_indices = {pos: idx for idx, pos in enumerate(path)} if path else {}

        print("\n+-------- แผนที่สนาม 4x4 และเส้นทางเดิน (ASCII Grid) --------+")
        print("  y\\x   [1]          [2]          [3]          [4]")
        print("     +------------+------------+------------+------------+")

        for y in range(self.height, 0, -1):
            row_top = f" [{y}] |"
            row_mid = "     |"
            row_bot = "     |"

            for x in range(1, self.width + 1):
                pos = (x, y)
                cost = self.costs.get(pos, 0)
                is_obs = pos in self.obstacles
                is_start = (pos == self.start)
                is_goal = (pos == self.goal)
                in_path = pos in path_set

                if is_obs:
                    tag = "  [X] OBS  "
                    c_tag = "  Cost: --  "
                elif is_start:
                    tag = "  (S) START "
                    c_tag = f"  Cost: {cost:<2}  "
                elif is_goal:
                    tag = "  (G) GOAL  "
                    c_tag = f"  Cost: {cost:<2}  "
                elif in_path:
                    step_num = path_indices[pos]
                    tag = f"  Step #{step_num:<2}  "
                    c_tag = f"  Cost: {cost:<2}  "
                else:
                    tag = f"  Cell({x},{y}) "
                    c_tag = f"  Cost: {cost:<2}  "

                path_symbol = "  ● PATH ●  " if in_path and not (is_start or is_goal) else ("  ▲ START ▲ " if is_start else ("  ★ GOAL ★  " if is_goal else "            "))

                row_top += f"{tag}|"
                row_mid += f"{path_symbol}|"
                row_bot += f"{c_tag}|"

            print(row_top)
            print(row_mid)
            print(row_bot)
            print("     +------------+------------+------------+------------+")
        print("+------------------------------------------------------------+\n")


def print_banner():
    print("=" * 68)
    print("      [DFS PATH PLANNER - DEPTH-FIRST SEARCH ON 4x4 GRID]")
    print("      อ้างอิง: docs/RQM.md & docs/Picture2.png")
    print("      โหมด: คำนวณเส้นทางเพียวๆ ไม่ต้องรันหุ่นยนต์จริง (Simulation)")
    print("=" * 68)


def main():
    parser = argparse.ArgumentParser(description="สร้างเส้นทางด้วย DFS คำนวณ Total Cost และเวลาประมวลผล (ไม่ต้องรันหุ่นจริง)")
    parser.add_argument(
        "--all-paths",
        action="store_true",
        help="ค้นหาทุกเส้นทางที่เป็นไปได้ด้วย DFS Backtracking เพื่อเปรียบเทียบ Total Cost"
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="วาดและบันทึกรูปภาพเส้นทางลงโฟลเดอร์ output/ (ใช้ matplotlib)"
    )
    args = parser.parse_args()

    print_banner()

    planner = DFSPlanner()
    print(f"[*] ขนาดสนาม: {planner.width} x {planner.height} ช่อง (ช่องละ {planner.cell_size_m * 100:.0f} x {planner.cell_size_m * 100:.0f} cm)")
    print(f"[*] จุดเริ่มต้น (Start): {planner.start}")
    print(f"[*] จุดเป้าหมาย (Goal):  {planner.goal}")
    print(f"[*] สิ่งกีดขวาง (Obstacles): {sorted(list(planner.obstacles))}")

    # 1. รัน DFS Standard (First Path Found)
    print("\n>>> กำลังประมวลผลค้นหาเส้นทางด้วย DFS (Depth-First Search)...")
    res = planner.dfs_first_path()

    path = res["path"]
    if not path:
        print("[!] ไม่พบเส้นทางไปยังเป้าหมาย!")
        return

    # แสดงผลลัพธ์
    print("\n" + "=" * 68)
    print("                   [ผลการค้นหาเส้นทางด้วย DFS]")
    print("=" * 68)
    print(f"  อัลกอริทึม:              {res['algorithm']}")
    print(f"  สถานะ:                  สำเร็จ (Reachable)")
    print(f"  จำนวนก้าว (Steps):       {res['steps']} ก้าว ({len(path)} พิกัด)")
    print(f"  Total Cost (ไม่รวมจุดเริ่ม): {res['total_cost_excl']} หน่วย (ต้นทุนการย้ายเข้าแต่ละช่อง)")
    print(f"  Total Cost (รวมจุดเริ่ม):   {res['total_cost_incl']} หน่วย")
    print(f"  จำนวนโหนดที่สำรวจ (Nodes): {res['explored_nodes']} โหนด")
    print(f"  เวลาคำนวณ (Computation): {res['computation_time_ms']:.4f} ms ({res['computation_time_us']:.2f} µs | {res['computation_time_sec']:.6f} วินาที)")
    print("=" * 68)

    # ตารางแจกแจงรายละเอียดแต่ละก้าว
    print("\n[รายละเอียดการเดินแต่ละก้าว (Step Breakdown)]:")
    print(f"  {'Step':<6} | {'พิกัด (x, y)':<14} | {'Cell Cost':<10} | {'Cumulative Cost (excl)':<22} | {'หมายเหตุ'}")
    print("  " + "-" * 75)

    accum = 0
    for idx, pos in enumerate(path):
        c = planner.costs.get(pos, 0)
        if idx > 0:
            accum += c
        note = "จุดเริ่มต้น (Start)" if idx == 0 else ("จุดเป้าหมาย (Goal)" if idx == len(path) - 1 else "เดินปกติ")
        print(f"  #{idx:<5} | ({pos[0]}, {pos[1]}){'':<8} | {c:<10} | {accum:<22} | {note}")

    print("  " + "-" * 75)
    print(f"  สรุปเส้นทาง: {' -> '.join(str(p) for p in path)}")

    # แสดงแผนที่แบบ ASCII
    planner.print_ascii_grid(path)

    # 2. หากมี flag --all-paths ให้แสดงการสำรวจทุกเส้นทาง
    if args.all_paths:
        print("=" * 68)
        print("          [การสำรวจทุกเส้นทางที่เป็นไปได้ด้วย DFS Backtracking]")
        print("=" * 68)
        all_res = planner.dfs_all_paths()
        print(f"  พบทั้งหมด: {all_res['total_paths_found']} เส้นทาง | โหนดสำรวจ: {all_res['explored_nodes']} โหนด")
        print(f"  เวลาคำนวณทั้งหมด: {all_res['computation_time_ms']:.4f} ms ({all_res['computation_time_us']:.2f} µs)\n")

        for rk, p_info in enumerate(all_res["all_paths"], 1):
            p_str = ' -> '.join(str(p) for p in p_info["path"])
            print(f"  อันดับ {rk}: Total Cost = {p_info['total_cost_excl']:<2} (รวมเริ่ม: {p_info['total_cost_incl']:<2}) | {p_info['steps']} ก้าว")
            print(f"           เส้นทาง: {p_str}\n")

    # 3. หากมี flag --plot ให้บันทึกรูปกราฟิก
    if args.plot:
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches

            os.makedirs("output", exist_ok=True)
            output_png = "output/dfs_path.png"

            fig, ax = plt.subplots(figsize=(8, 8), dpi=150)
            ax.set_facecolor('#F8F9FA')
            ax.set_xlim(0.5, 4.5)
            ax.set_ylim(0.5, 4.5)
            ax.set_aspect('equal')

            # วาดกริด
            for y in range(1, 5):
                for x in range(1, 5):
                    pos = (x, y)
                    is_start = (pos == planner.start)
                    is_goal = (pos == planner.goal)
                    is_obs = (pos in planner.obstacles)
                    cost = planner.costs.get(pos, 0)

                    fc = '#D4EDDA' if is_start else ('#F8D7DA' if is_goal else ('#6C757D' if is_obs else '#FFFFFF'))
                    rect = patches.Rectangle((x - 0.48, y - 0.48), 0.96, 0.96, lw=1.5, ec='#CED4DA', fc=fc, zorder=1)
                    ax.add_patch(rect)

                    ax.text(x - 0.4, y + 0.35, f"({x},{y})", fontsize=9, color='#6C757D', fontweight='bold', zorder=2)
                    if is_start:
                        ax.text(x, y + 0.15, "START", fontsize=11, fontweight='bold', color='#155724', ha='center', zorder=3)
                        ax.text(x, y - 0.3, f"Cost={cost}", fontsize=9, color='#155724', ha='center', zorder=3)
                    elif is_goal:
                        ax.text(x, y + 0.15, "GOAL", fontsize=11, fontweight='bold', color='#721C24', ha='center', zorder=3)
                        ax.text(x, y - 0.3, f"Cost={cost}", fontsize=9, color='#721C24', ha='center', zorder=3)
                    elif is_obs:
                        ax.text(x, y, "OBSTACLE", fontsize=9, fontweight='bold', color='white', ha='center', va='center', zorder=3)
                    else:
                        ax.text(x, y - 0.3, f"Cost={cost}", fontsize=9, color='#495057', ha='center', zorder=3)

            # วาดเส้นทาง
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            ax.plot(xs, ys, color='#FD7E14', lw=4, alpha=0.85, zorder=4, label='DFS Path')

            for s_idx, (px, py) in enumerate(path):
                circ = patches.Circle((px, py), 0.13, color='#FD7E14', ec='white', lw=1.5, zorder=5)
                ax.add_patch(circ)
                ax.text(px, py, str(s_idx), color='white', fontsize=8, fontweight='bold', ha='center', va='center', zorder=6)

            ax.set_title(
                f"DFS Path Planning (Simulation)\n"
                f"Total Cost: {res['total_cost_excl']} | Steps: {res['steps']} | Time: {res['computation_time_ms']:.4f} ms",
                fontsize=11, fontweight='bold', pad=10
            )
            ax.set_xticks(range(1, 5))
            ax.set_yticks(range(1, 5))
            ax.grid(False)

            plt.savefig(output_png, bbox_inches='tight')
            plt.close(fig)
            print(f"[PLOT] บันทึกรูปภาพเส้นทาง DFS เรียบร้อย: {os.path.abspath(output_png)}")
        except Exception as e:
            print(f"[!] ไม่สามารถสร้างรูปภาพได้: {e}")


if __name__ == "__main__":
    main()
