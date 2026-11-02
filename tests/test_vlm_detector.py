"""VLM 响应解析与图像->世界坐标反投影。

全部离线：不会发起任何 HTTP 请求。
"""
import math
import unittest

from src.perception.vlm_detector import VLMObstacleDetector
from src.utils.config import Config


class TestVLMResponseParsing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.detector = VLMObstacleDetector()

    def test_extracts_plain_json(self):
        payload = '{"obstacles": [{"class": "boat"}]}'
        self.assertEqual(self.detector._extract_json_content(payload), payload)

    def test_extracts_json_wrapped_in_prose(self):
        payload = 'Here is the result:\n{"obstacles": []}\nHope this helps!'
        self.assertEqual(self.detector._extract_json_content(payload), '{"obstacles": []}')

    def test_extracts_json_inside_markdown_fence(self):
        payload = '```json\n{"obstacles": [{"class": "buoy"}]}\n```'
        extracted = self.detector._extract_json_content(payload)

        self.assertIsNotNone(extracted)
        self.assertTrue(extracted.startswith("{"))
        self.assertTrue(extracted.endswith("}"))

    def test_extracts_nested_json(self):
        payload = 'prefix {"a": {"b": 1}, "c": [1, 2]} suffix'
        self.assertEqual(
            self.detector._extract_json_content(payload),
            '{"a": {"b": 1}, "c": [1, 2]}')

    def test_returns_none_without_json(self):
        self.assertIsNone(self.detector._extract_json_content("没有发现任何障碍物。"))


class TestDetectionPostProcessing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.detector = VLMObstacleDetector()

    def test_bbox_is_normalised_to_unit_range(self):
        result = {"obstacles": [
            {"class": "boat", "bbox": [400, 200, 600, 400], "risk_level": "中"},
        ]}
        obstacles, _ = self.detector.process_detection_result(result, image_shape=(768, 1024))

        self.assertEqual(len(obstacles), 1)
        norm_x, norm_y, radius = obstacles[0]
        self.assertGreaterEqual(norm_x, 0.0)
        self.assertLessEqual(norm_x, 1.0)
        self.assertGreaterEqual(norm_y, 0.0)
        self.assertLessEqual(norm_y, 1.0)
        self.assertGreater(radius, 0.0)
        self.assertLess(radius, 1.0)
        self.assertAlmostEqual(norm_x, 500 / 1024)

    def test_high_risk_inflates_radius(self):
        base = {"class": "rock", "bbox": [400, 200, 600, 400]}
        low, _ = self.detector.process_detection_result(
            {"obstacles": [dict(base, risk_level="低")]}, (768, 1024))
        high, _ = self.detector.process_detection_result(
            {"obstacles": [dict(base, risk_level="高")]}, (768, 1024))

        self.assertGreater(high[0][2], low[0][2])

    def test_missing_obstacles_key_is_handled(self):
        obstacles, safe = self.detector.process_detection_result({}, (768, 1024))
        self.assertEqual(obstacles, [])
        self.assertEqual(safe, [])

    def test_safe_areas_are_collected(self):
        result = {"obstacles": [], "safe_areas": [{"direction": "left"},
                                                  {"direction": "right"}]}
        _, safe = self.detector.process_detection_result(result, (768, 1024))
        self.assertEqual(safe, ["left", "right"])


class TestWorldProjection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.detector = VLMObstacleDetector()

    def test_centre_of_image_projects_onto_vehicle_position(self):
        (x, y, _), = self.detector.convert_to_world_coordinates(
            [(0.5, 0.5, 0.02)], robot_pose=(10.0, 20.0, 0.0), camera_range=12.0)

        self.assertAlmostEqual(x, 10.0)
        self.assertAlmostEqual(y, 20.0)

    def test_object_above_centre_is_ahead(self):
        (x, y, _), = self.detector.convert_to_world_coordinates(
            [(0.5, 0.75, 0.02)], robot_pose=(0.0, 0.0, 0.0), camera_range=10.0)

        self.assertAlmostEqual(x, 2.5)      # (0.75 - 0.5) * 10
        self.assertAlmostEqual(y, 0.0)

    def test_object_right_of_centre_has_negative_lateral_offset(self):
        (x, y, _), = self.detector.convert_to_world_coordinates(
            [(0.75, 0.5, 0.02)], robot_pose=(0.0, 0.0, 0.0), camera_range=10.0)

        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, -2.5)     # (0.5 - 0.75) * 10

    def test_projection_rotates_with_heading(self):
        straight, = self.detector.convert_to_world_coordinates(
            [(0.5, 0.75, 0.02)], robot_pose=(0.0, 0.0, 0.0), camera_range=10.0)
        turned, = self.detector.convert_to_world_coordinates(
            [(0.5, 0.75, 0.02)], robot_pose=(0.0, 0.0, math.pi / 2), camera_range=10.0)

        self.assertGreater(straight[0], 0.0)
        self.assertAlmostEqual(straight[1], 0.0)
        self.assertAlmostEqual(turned[0], 0.0, places=9)
        self.assertAlmostEqual(turned[1], 2.5)

    def test_radius_is_inflated_by_config(self):
        norm_radius = 0.02
        camera_range = 10.0
        (_, _, radius), = self.detector.convert_to_world_coordinates(
            [(0.5, 0.6, norm_radius)], robot_pose=(0.0, 0.0, 0.0), camera_range=camera_range)

        expected = (norm_radius * camera_range * Config.VLM_OBS_INFLATION_FACTOR
                    + Config.VLM_OBS_EXTRA_MARGIN)
        self.assertAlmostEqual(radius, expected)


if __name__ == "__main__":
    unittest.main()
