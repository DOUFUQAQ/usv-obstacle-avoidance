"""A* 全局路径规划：可解性、避障、边界情况。"""
import unittest

import numpy as np

from src.planning.global_planner import AStarPlanner


def empty_map(size=50):
    return np.zeros((size, size), dtype=np.uint8)


class TestAStarPlanner(unittest.TestCase):
    def setUp(self):
        self.planner = AStarPlanner()

    def assertClose(self, actual, expected, tol=1.0):
        # grid_to_world() 返回的是栅格中心，允许半个格子多一点误差
        self.assertLessEqual(abs(actual[0] - expected[0]), tol)
        self.assertLessEqual(abs(actual[1] - expected[1]), tol)

    def test_plans_path_on_empty_grid(self):
        path = self.planner.plan((5.0, 5.0), (45.0, 45.0), empty_map())

        self.assertIsNotNone(path)
        self.assertGreaterEqual(len(path), 2)
        self.assertClose(path[0], (5.0, 5.0))
        self.assertClose(path[-1], (45.0, 45.0))

    def test_path_stays_out_of_obstacles(self):
        grid = empty_map()
        grid[20:30, 0:35] = 1          # 一堵墙，右侧留通道

        path = self.planner.plan((5.0, 5.0), (45.0, 25.0), grid)

        self.assertIsNotNone(path, "存在绕行方案，规划器必须找到它")
        for x, y in path:
            gx, gy = self.planner.world_to_grid(x, y)
            self.assertEqual(grid[gy, gx], 0, "路径穿过了障碍格 %r" % ((x, y),))

    def test_returns_none_when_goal_is_inside_obstacle(self):
        grid = empty_map()
        grid[45, 45] = 1
        self.assertIsNone(self.planner.plan((5.0, 5.0), (45.0, 45.0), grid))

    def test_returns_none_when_start_is_inside_obstacle(self):
        grid = empty_map()
        grid[5, 5] = 1
        self.assertIsNone(self.planner.plan((5.0, 5.0), (45.0, 45.0), grid))

    def test_smoothing_keeps_endpoints_and_shrinks_path(self):
        grid = empty_map()
        raw = self.planner.plan((5.0, 5.0), (45.0, 45.0), grid)
        smoothed = self.planner.smooth_path(raw, grid)

        self.assertGreaterEqual(len(smoothed), 2)
        self.assertLessEqual(len(smoothed), len(raw))
        self.assertEqual(smoothed[0], raw[0])
        self.assertEqual(smoothed[-1], raw[-1])

    def test_world_grid_roundtrip(self):
        gx, gy = self.planner.world_to_grid(12.3, 7.8)
        x, y = self.planner.grid_to_world(gx, gy)

        self.assertLess(abs(x - 12.3), self.planner.grid_size)
        self.assertLess(abs(y - 7.8), self.planner.grid_size)


if __name__ == "__main__":
    unittest.main()
