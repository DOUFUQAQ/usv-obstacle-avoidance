"""DWA 局部规划器：动态窗口、避障约束、代价函数。"""
import math
import unittest

from src.planning.local_planner import DWALocalPlanner, DynamicState
from src.utils.config import Config


class TestDWALocalPlanner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # 关闭 VLM：这些用例必须离线、可复现
        cls.planner = DWALocalPlanner(enable_vlm=False)

    def setUp(self):
        self.planner.planning_times = []

    def test_dynamic_window_respects_acceleration_limits(self):
        state = DynamicState(x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0.0)
        v_min, v_max, w_min, w_max = self.planner.calc_dynamic_window(state)

        self.assertAlmostEqual(
            v_min, max(self.planner.min_speed, -self.planner.max_accel * self.planner.dt))
        self.assertAlmostEqual(
            v_max, min(self.planner.max_speed, self.planner.max_accel * self.planner.dt))
        self.assertGreaterEqual(w_min, self.planner.min_angular_speed - 1e-9)
        self.assertLessEqual(w_max, self.planner.max_angular_speed + 1e-9)

    def test_drives_towards_goal_when_path_is_clear(self):
        state = DynamicState(x=5.0, y=25.0, yaw=0.0, v=0.5)
        v, omega = self.planner.plan(state, goal=(45.0, 25.0), static_obstacles=[])

        self.assertGreater(v, 0.0, "空旷地图上应该向前运动")
        self.assertLess(abs(omega), 0.3, "目标在正前方，不需要大幅转向")

    def test_never_returns_a_colliding_command(self):
        """规划器可以停下，但绝不该返回一条违反安全清距的指令。"""
        cases = [
            (7.5, 25.0, 1.0),      # 很近，正前方
            (9.0, 25.0, 1.0),
            (12.0, 25.0, 1.5),
        ]
        for obstacle in cases:
            with self.subTest(obstacle=obstacle):
                state = DynamicState(x=5.0, y=25.0, yaw=0.0, v=1.0)
                v, omega = self.planner.plan(
                    state, goal=(45.0, 25.0), static_obstacles=[obstacle])

                trajectory = self.planner.predict_trajectory(state, v, omega)
                cost = self.planner.calc_obstacle_cost(trajectory, [obstacle])

                self.assertTrue(
                    cost < float("inf") or (v == 0.0 and omega == 0.0),
                    "返回了会碰撞的指令 (v=%.2f, omega=%.2f)" % (v, omega),
                )

    def test_keeps_clearance_from_blocking_obstacle(self):
        obstacle = (14.0, 25.0, 1.0)          # 正前方，但留有绕行空间
        state = DynamicState(x=5.0, y=25.0, yaw=0.0, v=1.0)
        v, omega = self.planner.plan(state, goal=(45.0, 25.0), static_obstacles=[obstacle])

        trajectory = self.planner.predict_trajectory(state, v, omega)
        required = self.planner.robot_radius + Config.SAFETY_MARGIN
        for s in trajectory:
            clearance = math.hypot(s.x - obstacle[0], s.y - obstacle[1]) - obstacle[2]
            self.assertGreater(
                clearance, required - 1e-6,
                "选中轨迹违反安全清距：%.2f m" % clearance)

    def test_rejects_trajectory_through_obstacle(self):
        obstacle = (6.0, 25.0, 1.0)            # 前方 1 m，不可能安全通过
        state = DynamicState(x=5.0, y=25.0, yaw=0.0, v=1.0)

        straight = self.planner.predict_trajectory(state, 1.5, 0.0)
        self.assertEqual(self.planner.calc_obstacle_cost(straight, [obstacle]), float("inf"))

    def test_obstacle_cost_decays_with_clearance(self):
        near = [(5.5, 25.0, 0.5)]      # 清距 0 m，必然判为碰撞
        far = [(5.5, 35.0, 0.5)]       # 清距约 9.5 m
        state = DynamicState(x=5.0, y=25.0, yaw=0.0, v=0.0)
        trajectory = self.planner.predict_trajectory(state, 0.0, 0.0)

        self.assertEqual(self.planner.calc_obstacle_cost(trajectory, near), float("inf"))
        self.assertLess(self.planner.calc_obstacle_cost(trajectory, far), float("inf"))

    def test_plan_records_planning_time(self):
        state = DynamicState(x=5.0, y=5.0, yaw=0.0, v=0.0)
        self.planner.plan(state, goal=(40.0, 40.0), static_obstacles=[])

        stats = self.planner.get_planning_statistics()
        self.assertGreaterEqual(stats["total_calls"], 1)
        self.assertGreaterEqual(stats["average_time_ms"], 0.0)
        self.assertGreaterEqual(stats["max_time_ms"], stats["average_time_ms"])

    def test_motion_model_integrates_velocity(self):
        state = DynamicState(x=0.0, y=0.0, yaw=0.0, v=0.0)
        nxt = self.planner.motion_model(state, v=1.0, omega=0.0)

        self.assertAlmostEqual(nxt.x, self.planner.dt)
        self.assertAlmostEqual(nxt.y, 0.0)
        self.assertAlmostEqual(nxt.yaw, 0.0)

    def test_safety_configuration_is_conservative(self):
        """防止有人顺手把 VLM 的安全裕度调没了。"""
        self.assertGreater(Config.VLM_OBS_INFLATION_FACTOR, 1.0)
        self.assertGreater(Config.VLM_OBS_EXTRA_MARGIN, 0.0)
        self.assertGreater(Config.SAFETY_MARGIN, 0.0)
        self.assertGreater(Config.DWA_OBSTACLE_COST_GAIN, Config.DWA_TO_GOAL_COST_GAIN)


if __name__ == "__main__":
    unittest.main()
