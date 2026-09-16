"""
Dijkstra Path Planner Module for 4x4 Grid
"""
import time
import heapq
from typing import List, Tuple, Dict, Optional, Set, Any


class DijkstraPlanner:
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

        self.obstacles: Set[Tuple[int, int]] = {
            (4, 4),
            (2, 3),
            (3, 2)
        }

        self.costs: Dict[Tuple[int, int], int] = {
            (1, 4): 2, (2, 4): 3, (3, 4): 1, (4, 4): -1,
            (1, 3): 2, (2, 3): -1, (3, 3): 4, (4, 3): 2,
            (1, 2): 3, (2, 2): 1, (3, 2): -1, (4, 2): 4,
            (1, 1): 2, (2, 1): 3, (3, 1): 2, (4, 1): 1,
        }

    def is_valid(self, pos: Tuple[int, int]) -> bool:
        x, y = pos
        if not (1 <= x <= self.width and 1 <= y <= self.height):
            return False
        if pos in self.obstacles:
            return False
        return True

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        x, y = pos
        candidates = [
            (x + 1, y),  # Right
            (x, y - 1),  # Down
            (x - 1, y),  # Left
            (x, y + 1),  # Up
        ]
        return [c for c in candidates if self.is_valid(c)]

    def compute_cost(self, path: List[Tuple[int, int]]) -> Tuple[int, int]:
        if not path:
            return 0, 0
        cost_excl = sum(self.costs.get(p, 0) for p in path[1:])
        cost_incl = sum(self.costs.get(p, 0) for p in path)
        return cost_excl, cost_incl

    def dijkstra_search(self) -> Dict[str, Any]:
        start_time = time.perf_counter()

        tie_counter = 0
        pq = []
        heapq.heappush(pq, (0, tie_counter, self.start, [self.start]))

        distances: Dict[Tuple[int, int], int] = {self.start: 0}
        visited_nodes: Set[Tuple[int, int]] = set()

        explored_count = 0
        optimal_path: Optional[List[Tuple[int, int]]] = None

        while pq:
            current_cost, _, current, path = heapq.heappop(pq)

            if current in visited_nodes:
                continue

            visited_nodes.add(current)
            explored_count += 1

            if current == self.goal:
                optimal_path = path
                break

            for neighbor in self.get_neighbors(current):
                step_cost = self.costs.get(neighbor, 1)
                new_cost = current_cost + step_cost

                if neighbor not in distances or new_cost < distances[neighbor]:
                    distances[neighbor] = new_cost
                    tie_counter += 1
                    heapq.heappush(pq, (new_cost, tie_counter, neighbor, path + [neighbor]))

        end_time = time.perf_counter()
        elapsed = end_time - start_time
        cost_excl, cost_incl = self.compute_cost(optimal_path) if optimal_path else (0, 0)

        return {
            "algorithm": "Dijkstra",
            "path": optimal_path,
            "steps": len(optimal_path) - 1 if optimal_path else 0,
            "total_cost_excl": cost_excl,
            "total_cost_incl": cost_incl,
            "explored_nodes": explored_count,
            "computation_time_sec": elapsed,
            "computation_time_ms": elapsed * 1000.0,
            "distances": distances,
        }
