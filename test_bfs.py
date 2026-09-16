import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from bfs_planner import GridMap
from robot_navigator import RobotNavigator


class TestBFSPlanner(unittest.TestCase):
    def setUp(self):
        self.grid = GridMap()

    def test_grid_boundaries_and_obstacles(self):
        # จุดเริ่มต้นและเป้าหมายต้อง valid
        self.assertTrue(self.grid.is_valid(self.grid.start))
        self.assertTrue(self.grid.is_valid(self.grid.goal))

        # สิ่งกีดขวางต้อง invalid
        for obs in self.grid.obstacles:
            self.assertFalse(self.grid.is_valid
            (obs))

        # นอกกรอบต้อง invalid
        self.assertFalse(self.grid.is_valid((0, 1)))
        self.assertFalse(self.grid.is_valid((5, 1)))
        self.assertFalse(self.grid.is_valid((1, 0)))
        self.assertFalse(self.grid.is_valid((1, 5)))

    def test_bfs_shortest_path(self):
        path = self.grid.bfs_shortest_path()
        self.assertIsNotNone(path)
        self.assertEqual(path[0], self.grid.start)
        self.assertEqual(path[-1], self.grid.goal)

        # ตรวจสอบว่าแต่ละจุดที่ติดกันห่างกัน 1 ช่องพอดี (Manhattan distance = 1)
        for i in range(len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            dist = abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
            self.assertEqual(dist, 1, f"Step from {p1} to {p2} is not adjacent")

        # ตรวจสอบว่าไม่มีสิ่งกีดขวางในเส้นทาง
        for p in path:
            self.assertNotIn(p, self.grid.obstacles)

        # จำนวนก้าวต้องเป็น shortest step (6 steps = 7 nodes)
        self.assertEqual(len(path) - 1, 6)

    def test_cost_calculation(self):
        path = [(1, 4), (1, 3), (1, 2), (2, 2), (2, 1), (3, 1), (4, 1)]
        cost_excl = self.grid.calculate_path_cost(path, include_start=False)
        # (1,3)=2, (1,2)=3, (2,2)=1, (2,1)=3, (3,1)=2, (4,1)=1 => 2+3+1+3+2+1 = 12
        self.assertEqual(cost_excl, 12)

    def test_sim_navigation(self):
        navigator = RobotNavigator(sim_mode=True, cell_size_m=0.60)
        path = self.grid.bfs_shortest_path()
        visited = []

        def callback(idx, pos):
            visited.append(pos)

        success = navigator.navigate_path(path, cell_costs=self.grid.costs, on_step_callback=callback)
        self.assertTrue(success)
        self.assertEqual(visited, path)
        self.assertEqual(navigator.current_pos, self.grid.goal)

    def test_csv_logging_only_for_real_robot(self):
        import os
        from csv_logger import NavigationLogger
        test_csv = "test_log.csv"

        # 1. ในโหมดจำลอง (sim_mode=True) ต้องไม่บันทึก CSV
        sim_logger = NavigationLogger(filename=test_csv, enabled=False)
        sim_navigator = RobotNavigator(sim_mode=True, cell_size_m=0.60, logger=sim_logger)
        path = self.grid.bfs_shortest_path()
        sim_navigator.navigate_path(path, cell_costs=self.grid.costs)
        self.assertFalse(os.path.exists(test_csv), "Simulation should NOT generate CSV file")

        # 2. เมื่อเปิดใช้งานสำหรับหุ่นจริง (enabled=True) ต้องบันทึก CSV ครบ 8 บรรทัด (Header + Start + 6 Steps)
        real_logger = NavigationLogger(filename=test_csv, enabled=True)
        real_logger.log_step(0, 1, 4, "START", 0.0, 0.0, 0.0, 1200.0, 0.0, 0.0, 2, 0, 0.0, "SUCCESS")
        for i in range(1, 7):
            real_logger.log_step(i, i, i, "MOVE_TO_CELL", 0.0, 0.0, 0.0, 1200.0, 0.0, 0.0, 1, i, i * 0.6, "SUCCESS")

        self.assertTrue(os.path.exists(test_csv))
        with open(test_csv, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        self.assertEqual(len(lines), 8)
        os.remove(test_csv)


if __name__ == "__main__":
    unittest.main()
