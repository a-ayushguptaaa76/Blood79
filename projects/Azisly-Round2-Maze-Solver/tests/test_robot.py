import importlib.util
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "robot.py"


def load_robot():
    spec = importlib.util.spec_from_file_location("robot_submission", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def sensor_packet(**overrides):
    packet = {
        "tick": 0,
        "dist_front": 1,
        "dist_left": 1,
        "dist_right": 0,
        "rpm_left": 0,
        "rpm_right": 0,
        "accel_fwd": 0.0,
        "accel_lat": 0.0,
        "at_goal": False,
    }
    packet.update(overrides)
    return packet


class RobotSmokeTests(unittest.TestCase):
    def test_module_imports_and_initializes_memory(self):
        robot = load_robot()
        memory = {}
        action = robot.decide(sensor_packet(), memory)
        self.assertIn(action, {"forward", "turn_left", "turn_right", "wait"})
        self.assertTrue(memory["initialized"])
        self.assertEqual(memory["heading"], "N")

    def test_goal_is_defensive_wait(self):
        robot = load_robot()
        memory = {}
        self.assertEqual(robot.decide(sensor_packet(at_goal=True), memory), "wait")

    def test_action_sequence_stays_json_protocol_safe(self):
        robot = load_robot()
        memory = {}
        for tick in range(8):
            action = robot.decide(sensor_packet(tick=tick), memory)
            payload = json.dumps({"action": action})
            round_trip = json.loads(payload)
            self.assertIn(
                round_trip["action"],
                {"forward", "turn_left", "turn_right", "wait"},
            )


if __name__ == "__main__":
    unittest.main()
