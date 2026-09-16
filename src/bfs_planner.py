"""
BFS Path Planner for 4x4 Grid
อ้างอิงจาก Picture2.png และ RQM.md
- ขนาดสนาม: 4 x 4 ช่อง
- ขนาดแต่ละช่อง: 60 cm x 60 cm (0.60 m x 0.60 m)
- จุดเริ่มต้น (Start): (1, 4)
- เป้าหมาย (Goal): (4, 1)
- สิ่งกีดขวาง (Obstacles): (4, 4), (2, 3), (3, 2)
"""

from collections import deque
from typing import List, Tuple, Dict, Optional, Set


class GridMap:
    def __init__(self):
        self.width = 4
        self.height = 4
        self.cell_size_m = 0.60  # 60 cm = 0.60 m
        self.robot_length_m = 0.33  # 33 cm
        self.robot_width_m = 0.25   # 25 cm

        self.start = (1, 4)
        self.goal = (4, 1)

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

    def bfs_shortest_path(self, optimize_cost_among_shortest: bool = True) -> Optional[List[Tuple[int, int]]]:
        """
        ค้นหาเส้นทางสั้นที่สุดด้วย Breadth-First Search (BFS)
        - หาก optimize_cost_among_shortest เป็น True จะเลือกเส้นทาง BFS ที่มี Cost ต่ำที่สุดในบรรดาเส้นทางที่สั้นที่สุด
        """
        if self.start == self.goal:
            return [self.start]

        if not optimize_cost_among_shortest:
            # BFS แบบมาตรฐาน (First found)
            queue = deque([(self.start, [self.start])])
            visited = {self.start}

            while queue:
                current, path = queue.popleft()

                if current == self.goal:
                    return path

                for neighbor in self.get_neighbors(current):
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append((neighbor, path + [neighbor]))
            return None

        # ค้นหาทุกเส้นทาง BFS ที่สั้นที่สุด (ระยะก้าวเท่ากัน) แล้วเลือกเส้นทางที่ Cost ต่ำสุด
        queue = deque([(self.start, [self.start], 0)])  # (pos, path, cost_excl_start)
        min_steps_to_goal = None
        shortest_paths = []
        best_cost_at_step: Dict[Tuple[Tuple[int, int], int], int] = {}

        while queue:
            current, path, cost = queue.popleft()
            step_count = len(path) - 1

            if min_steps_to_goal is not None and step_count > min_steps_to_goal:
                break

            if current == self.goal:
                if min_steps_to_goal is None:
                    min_steps_to_goal = step_count
                shortest_paths.append((path, cost))
                continue

            for neighbor in self.get_neighbors(current):
                if neighbor in path:
                    continue
                next_step = step_count + 1
                next_cost = cost + self.costs.get(neighbor, 0)
                state = (neighbor, next_step)

                # Prune paths that reach the same node in the same number of steps with a higher cost
                if state in best_cost_at_step and best_cost_at_step[state] <= next_cost:
                    continue
                best_cost_at_step[state] = next_cost
                queue.append((neighbor, path + [neighbor], next_cost))

        if not shortest_paths:
            return None

        # เลือกเส้นทางที่มี cost รวมต่ำที่สุด
        shortest_paths.sort(key=lambda item: item[1])
        return shortest_paths[0][0]

    def calculate_path_cost(self, path: List[Tuple[int, int]], include_start: bool = False) -> int:
        """คำนวณต้นทุนรวมของเส้นทาง"""
        if not path:
            return 0
        nodes = path if include_start else path[1:]
        return sum(self.costs.get(node, 0) for node in nodes)

    def print_grid(self, path: Optional[List[Tuple[int, int]]] = None):
        """แสดงแผนที่แบบ ASCII พร้อม Cost และเส้นทาง"""
        path_set = set(path) if path else set()
        path_order = {pos: idx for idx, pos in enumerate(path)} if path else {}

        print("\n" + "=" * 50)
        print("  สนามขนาด 4 x 4 (ตาม Picture2.png)")
        print("=" * 50)
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
        print("=" * 50 + "\n")
