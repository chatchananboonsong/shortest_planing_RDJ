"""
A* (A-Star) Path Planner for 4x4 Grid
อ้างอิงจาก Picture2.png และ RQM.md
- ขนาดสนาม: 4 x 4 ช่อง
- ขนาดแต่ละช่อง: 60 cm x 60 cm (0.60 m x 0.60 m)
- จุดเริ่มต้น (Start): (1, 4)
- เป้าหมาย (Goal): (4, 1)
- สิ่งกีดขวาง (Obstacles): (4, 4), (2, 3), (3, 2)
- ค่าใช้จ่ายแต่ละช่อง (Cell Costs): 1 - 4 หน่วยตาม Picture2.png

อัลกอริทึม A* (A-Star):
- ฟังก์ชันการประเมิน: f(n) = g(n) + h(n)
  - g(n): ต้นทุนจริงสะสมจากจุดเริ่มต้นจนถึงโหนด n (ตามค่าน้ำหนัก Cell Cost ของแต่ละช่อง)
  - h(n): ค่าประเมินแบบฮิวริสติก (Heuristic) ถึงเป้าหมาย โดยใช้ Manhattan Distance
          คูณด้วยต้นทุนขั้นต่ำต่อช่อง (min_cost = 1) เพื่อรับประกัน Admissibility และ Consistency
  - f(n): ค่าใช้จ่ายรวมที่คาดการณ์เพื่อเลือกขยายโหนดที่มีโอกาสดีที่สุดก่อนเสมอ (Priority Queue / Min-Heap)
"""

import sys
import heapq
from typing import List, Tuple, Dict, Optional, Set, Any

# ตั้งค่า encoding ของ stdout ให้รองรับภาษาไทยบน Windows
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass


class AStarPlanner:
    def __init__(
        self,
        start: Tuple[int, int] = (1, 4),
        goal: Tuple[int, int] = (4, 1),
        width: int = 4,
        height: int = 4,
        cell_size_m: float = 0.60,
        robot_length_m: float = 0.33,
        robot_width_m: float = 0.25
    ):
        self.width = width
        self.height = height
        self.cell_size_m = cell_size_m          # 60 cm = 0.60 m
        self.robot_length_m = robot_length_m    # 33 cm
        self.robot_width_m = robot_width_m      # 25 cm

        self.start = start
        self.goal = goal

        # ตำแหน่งสิ่งกีดขวางตาม Picture2.png
        self.obstacles: Set[Tuple[int, int]] = {
            (4, 4),
            (2, 3),
            (3, 2)
        }

        # ค่าใช้จ่าย (Cost) แต่ละช่องตาม Picture2.png
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

        # หาต้นทุนขั้นต่ำของช่องที่เดินได้ เพื่อใช้กับ Heuristic ให้ Admissible
        walkable_costs = [c for pos, c in self.costs.items() if pos not in self.obstacles and c > 0]
        self.min_step_cost = min(walkable_costs) if walkable_costs else 1

    def is_valid(self, pos: Tuple[int, int]) -> bool:
        """ตรวจสอบว่าตำแหน่งอยู่ในสนามและไม่ใช่สิ่งกีดขวาง"""
        x, y = pos
        if not (1 <= x <= self.width and 1 <= y <= self.height):
            return False
        if pos in self.obstacles:
            return False
        return True

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """คืนค่าช่องข้างเคียง 4 ทิศทาง (Right, Down, Left, Up)"""
        x, y = pos
        # ลำดับทิศ: ขวา (+x), ลง (-y), ซ้าย (-x), ขึ้น (+y)
        candidates = [
            (x + 1, y),  # ขวา (East)
            (x, y - 1),  # ลง (South)
            (x - 1, y),  # ซ้าย (West)
            (x, y + 1),  # ขึ้น (North)
        ]
        return [c for c in candidates if self.is_valid(c)]

    def heuristic(self, pos: Tuple[int, int], goal: Optional[Tuple[int, int]] = None) -> float:
        """
        ฮิวริสติกฟังก์ชัน h(n): Manhattan Distance
        h(n) = (|x - goal_x| + |y - goal_y|) * min_step_cost
        เนื่องจาก min_step_cost = 1 จึงเป็น Admissible (h(n) <= h*(n)) และ Consistent แน่นอน
        """
        target = goal if goal is not None else self.goal
        dx = abs(pos[0] - target[0])
        dy = abs(pos[1] - target[1])
        return (dx + dy) * self.min_step_cost

    def astar_shortest_path(self) -> Optional[List[Tuple[int, int]]]:
        """
        ค้นหาเส้นทางต้นทุนต่ำสุด (Optimal Cost Path) ด้วย A* Algorithm
        คืนค่า: List ของพิกัด (x, y) จาก Start ถึง Goal
        """
        result = self.astar_search()
        return result.get("path") if result else None

    def astar_search(self) -> Dict[str, Any]:
        """
        ดำเนินการค้นหาด้วยอัลกอริทึม A* พร้อมบันทึกรายละเอียดสถิติการสำรวจ
        คืนค่า: Dictionary ประกอบด้วย
          - path: เส้นทางที่ดีที่สุด
          - total_cost_excl: ค่าใช้จ่ายรวมไม่รวมช่องเริ่มต้น
          - total_cost_incl: ค่าใช้จ่ายรวมรวมช่องเริ่มต้น
          - steps: จำนวนก้าว
          - explored_nodes: จำนวนโหนดที่เปิดสำรวจ
          - step_breakdown: ข้อมูลการคำนวณ g, h, f แต่ละสเต็ป
        """
        if self.start == self.goal:
            return {
                "path": [self.start],
                "total_cost_excl": 0,
                "total_cost_incl": self.costs.get(self.start, 0),
                "steps": 0,
                "explored_nodes": 1,
                "step_breakdown": []
            }

        # Min-Heap Priority Queue: (f_score, tie_breaker, current_pos, path, g_score)
        tie_counter = 0
        h_start = self.heuristic(self.start)
        open_set = []
        heapq.heappush(open_set, (h_start, 0, tie_counter, self.start, [self.start], 0))

        # บันทึกค่า g_score ที่ดีที่สุดสำหรับแต่ละโหนด
        g_scores: Dict[Tuple[int, int], int] = {self.start: 0}
        closed_set: Set[Tuple[int, int]] = set()

        explored_count = 0
        best_path: Optional[List[Tuple[int, int]]] = None
        best_g: Optional[int] = None

        while open_set:
            f, h, _, current, path, g = heapq.heappop(open_set)

            # หากมีเส้นทางที่เข้าถึง current ด้วย g_score ต่ำกว่านี้แล้ว ให้ข้าม
            if current in closed_set and g > g_scores.get(current, float('inf')):
                continue

            closed_set.add(current)
            explored_count += 1

            if current == self.goal:
                best_path = path
                best_g = g
                break

            for neighbor in self.get_neighbors(current):
                step_cost = self.costs.get(neighbor, 1)
                tentative_g = g + step_cost

                if tentative_g < g_scores.get(neighbor, float('inf')):
                    g_scores[neighbor] = tentative_g
                    h_neighbor = self.heuristic(neighbor)
                    f_neighbor = tentative_g + h_neighbor
                    tie_counter += 1
                    heapq.heappush(
                        open_set,
                        (f_neighbor, h_neighbor, tie_counter, neighbor, path + [neighbor], tentative_g)
                    )

        if not best_path:
            return {
                "path": None,
                "total_cost_excl": 0,
                "total_cost_incl": 0,
                "steps": 0,
                "explored_nodes": explored_count,
                "step_breakdown": []
            }

        # สรุปตาราง step breakdown
        breakdown = []
        accum_g = 0
        for i, pos in enumerate(best_path):
            cell_cost = self.costs.get(pos, 0)
            if i > 0:
                accum_g += cell_cost
            h_val = self.heuristic(pos)
            breakdown.append({
                "step": i,
                "pos": pos,
                "cell_cost": cell_cost,
                "g": accum_g,
                "h": h_val,
                "f": accum_g + h_val
            })

        return {
            "path": best_path,
            "total_cost_excl": self.calculate_path_cost(best_path, include_start=False),
            "total_cost_incl": self.calculate_path_cost(best_path, include_start=True),
            "steps": len(best_path) - 1,
            "explored_nodes": explored_count,
            "step_breakdown": breakdown
        }

    def calculate_path_cost(self, path: List[Tuple[int, int]], include_start: bool = False) -> int:
        """คำนวณต้นทุนรวมของเส้นทาง"""
        if not path:
            return 0
        nodes = path if include_start else path[1:]
        return sum(self.costs.get(node, 0) for node in nodes)

    def print_grid(self, path: Optional[List[Tuple[int, int]]] = None):
        """แสดงแผนที่แบบ ASCII พร้อม Cost และเส้นทาง A*"""
        path_set = set(path) if path else set()
        path_order = {pos: idx for idx, pos in enumerate(path)} if path else {}

        print("\n" + "=" * 54)
        print("    สนามขนาด 4 x 4 (A* Search ตาม Picture2.png)")
        print("=" * 54)
        print("  y")
        for y in range(self.height, 0, -1):
            row_str = f"  {y} | "
            for x in range(1, self.width + 1):
                pos = (x, y)
                if pos in self.obstacles:
                    cell_repr = "[ OBST ]"
                elif pos == self.start and pos in path_set:
                    cell_repr = f"[S*{path_order[pos]}]"
                elif pos == self.goal and pos in path_set:
                    cell_repr = f"[G*{path_order[pos]}]"
                elif pos in path_set:
                    cell_repr = f"[ *{path_order[pos]:2d} ]"
                elif pos == self.start:
                    cell_repr = "[START ]"
                elif pos == self.goal:
                    cell_repr = "[ GOAL ]"
                else:
                    cost = self.costs.get(pos, 0)
                    cell_repr = f"[  c={cost} ]"
                row_str += f"{cell_repr:<9} "
            print(row_str)
        print("    " + "-" * 42)
        print("      " + "    ".join([f"x={x}" for x in range(1, self.width + 1)]))
        print("=" * 54 + "\n")

    def print_search_summary(self, result: Dict[str, Any]):
        """พิมพ์รายละเอียดผลการค้นหาและค่า f(n) = g(n) + h(n) ในแต่ละก้าว"""
        path = result.get("path")
        if not path:
            print("[ERROR] ไม่พบเส้นทาง A* ไปยังเป้าหมาย!")
            return

        print("-" * 65)
        print(f"  [A* ALGORITHM SUMMARY]")
        print(f"  เส้นทางที่พบ ({result['steps']} ก้าว):")
        print("  " + " -> ".join([f"({x},{y})" for x, y in path]))
        print(f"  ต้นทุนรวม (ไม่รวมจุดเริ่ม): {result['total_cost_excl']} หน่วย")
        print(f"  ต้นทุนรวม (รวมจุดเริ่ม):   {result['total_cost_incl']} หน่วย")
        print(f"  จำนวนโหนดที่สำรวจ (Explored Nodes): {result['explored_nodes']} โหนด")
        print("-" * 65)
        print("  รายละเอียดการประเมินในแต่ละก้าว (f = g + h):")
        print(f"  {'Step':<5} | {'พิกัด':<8} | {'Cell Cost':<10} | {'g(n)':<8} | {'h(n)':<8} | {'f(n)':<8}")
        print("  " + "-" * 57)
        for item in result["step_breakdown"]:
            step_str = f"#{item['step']}"
            pos_str = f"({item['pos'][0]},{item['pos'][1]})"
            c_str = f"{item['cell_cost']}" + (" (Start)" if item['step'] == 0 else "")
            print(f"  {step_str:<5} | {pos_str:<8} | {c_str:<10} | {item['g']:<8.1f} | {item['h']:<8.1f} | {item['f']:<8.1f}")
        print("-" * 65)


# Alias สำหรับให้เรียกใช้งานสอดคล้องกับ GridMap
GridMap = AStarPlanner


if __name__ == "__main__":
    planner = AStarPlanner()
    print("[RUNNING] ทดสอบ A* Path Planner บนสนาม 4x4...")
    result = planner.astar_search()
    planner.print_grid(result["path"])
    planner.print_search_summary(result)
