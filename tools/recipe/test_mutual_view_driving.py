from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT = Path(__file__).with_name("mutual_view_driving.py")
SPEC = importlib.util.spec_from_file_location("mutual_view_driving_recipe", SCRIPT)
assert SPEC and SPEC.loader
recipe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = recipe
SPEC.loader.exec_module(recipe)

APP = Path(__file__).with_name("assets") / "mutual_view_driving"
sys.path.insert(0, str(APP))
# The gateway and the smoke client import hakopy, hakoniwa_pdu, and websockets at module level;
# the pure logic is tested without them, so the tests also run on a plain Python (as in CI).
for name in ("hakopy", "websockets", "websockets.asyncio", "websockets.asyncio.server",
             "websockets.asyncio.client", "hakoniwa_pdu", "hakoniwa_pdu.pdu_msgs",
             "hakoniwa_pdu.pdu_msgs.geometry_msgs",
             "hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist",
             "hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Vector3",
             "hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist",
             "hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Vector3"):
    sys.modules.setdefault(name, mock.MagicMock())
import vehicles  # noqa: E402
import smoke_client  # noqa: E402
import driver_gateway  # noqa: E402

OVERTAKE = json.loads((APP / "world-overtake.json").read_text(encoding="utf-8"))


def pose(x: float, y: float, yaw: float = 0.0, speed: float = 0.0, profile: str = "sedan") -> dict:
    return {"x": x, "y": y, "yaw": yaw, "speed": speed, "profile": profile}


def drive(profile_id: str, seconds: float, throttle: float = 1.0, brake: float = 0.0, steer: float = 0.0,
          state: vehicles.VehicleState | None = None) -> vehicles.VehicleState:
    profile = vehicles.PROFILES[profile_id]
    state = state or vehicles.VehicleState(0.0, 0.0, 0.0)
    for _ in range(int(seconds / vehicles.DT_SEC)):
        state = vehicles.step(profile, state, throttle, brake, steer)
    return state


class VehicleModelTest(unittest.TestCase):
    def test_profiles_cover_bicycle_four_car_types_and_the_scenario_bus(self):
        self.assertEqual(vehicles.PROFILE_IDS, ["bicycle", "sedan", "suv", "minivan", "truck", "bus"])
        self.assertEqual(vehicles.CAR_PROFILE_IDS, ["sedan", "suv", "minivan", "truck"])

    def test_driver_eye_heights_are_ordered_by_vehicle_type(self):
        eye = {pid: p.eye[2] for pid, p in vehicles.PROFILES.items()}
        self.assertLess(eye["sedan"], eye["suv"])
        self.assertLess(eye["suv"], eye["minivan"])
        self.assertLess(eye["minivan"], eye["truck"])
        self.assertGreater(eye["bicycle"], eye["sedan"])

    def test_car_driver_sits_on_the_right_for_left_hand_traffic(self):
        for pid in vehicles.CAR_PROFILE_IDS:
            self.assertLess(vehicles.PROFILES[pid].eye[1], 0.0, pid)

    def test_full_throttle_moves_forward_and_respects_max_speed(self):
        for pid, profile in vehicles.PROFILES.items():
            state = drive(pid, 60.0)
            self.assertAlmostEqual(state.speed, profile.max_speed, places=6, msg=pid)
            self.assertGreater(state.x, 0.0)
            self.assertAlmostEqual(state.y, 0.0, places=6)

    def test_brake_stops_without_reversing(self):
        moving = drive("sedan", 5.0)
        stopped = drive("sedan", 10.0, throttle=0.0, brake=1.0, state=moving)
        self.assertEqual(stopped.speed, 0.0)

    def test_negative_throttle_reverses_up_to_the_reverse_limit(self):
        state = drive("suv", 20.0, throttle=-1.0)
        self.assertAlmostEqual(state.speed, -vehicles.PROFILES["suv"].max_reverse, places=6)
        self.assertLess(state.x, 0.0)

    def test_left_steer_turns_counter_clockwise_with_the_kinematic_radius(self):
        profile = vehicles.PROFILES["sedan"]
        state = vehicles.VehicleState(0.0, 0.0, 0.0, speed=5.0, steer=profile.max_steer)
        nxt = vehicles.step(profile, state, 0.0, 0.0, 1.0)
        expected = 5.0 / profile.wheelbase * math.tan(profile.max_steer) * vehicles.DT_SEC
        self.assertGreater(nxt.yaw, 0.0)
        self.assertAlmostEqual(nxt.yaw, expected * nxt.speed / 5.0, places=4)

    def test_steering_is_rate_limited(self):
        profile = vehicles.PROFILES["truck"]
        nxt = vehicles.step(profile, vehicles.VehicleState(0, 0, 0), 0.0, 0.0, 1.0)
        self.assertAlmostEqual(nxt.steer, profile.steer_rate * vehicles.DT_SEC)

    def test_profile_index_round_trip(self):
        for pid in vehicles.PROFILE_IDS:
            self.assertEqual(vehicles.profile_by_index(float(vehicles.profile_index(pid))).id, pid)
        with self.assertRaises(ValueError):
            vehicles.profile_by_index(99.0)


class ContactTest(unittest.TestCase):
    def test_overlap_and_separation(self):
        sedan = vehicles.PROFILES["sedan"]
        a = vehicles.footprint(sedan, 0.0, 0.0, 0.0)
        self.assertTrue(vehicles.polygons_overlap(a, vehicles.footprint(sedan, 4.0, 0.5, 0.3)))
        self.assertFalse(vehicles.polygons_overlap(a, vehicles.footprint(sedan, 6.0, 0.0, 0.0)))
        # Rotated rectangles whose bounding boxes overlap but shapes do not.
        self.assertFalse(vehicles.polygons_overlap(a, vehicles.footprint(sedan, 4.5, 3.0, math.pi / 4)))

    def test_spawn_points_are_clear_for_the_largest_car(self):
        states = {
            "bike": {**OVERTAKE["spawn"]["bike"], "profile": "bicycle"},
            "car": {**OVERTAKE["spawn"]["car"], "profile": "truck"},
        }
        self.assertEqual(driver_gateway.detect_contacts(OVERTAKE, states), {"bike": None, "car": None})

    def test_vehicle_contact_takes_priority_over_obstacles(self):
        states = {
            "bike": {"x": 0.0, "y": 0.0, "yaw": 0.0, "profile": "bicycle"},
            "car": {"x": 1.0, "y": 0.0, "yaw": 0.0, "profile": "sedan"},
        }
        self.assertEqual(driver_gateway.detect_contacts(OVERTAKE, states), {"bike": "vehicle", "car": "vehicle"})

    def test_building_contact(self):
        b = OVERTAKE["buildings"][0]
        states = {"bike": {"x": (b["x0"] + b["x1"]) / 2, "y": (b["y0"] + b["y1"]) / 2, "yaw": 0.0,
                           "profile": "bicycle"}}
        self.assertEqual(driver_gateway.detect_contacts(OVERTAKE, states), {"bike": "obstacle"})

    def test_parked_vehicle_contact(self):
        world = dict(OVERTAKE, parked=[{"x": 0.0, "y": 4.0, "yaw": 0.0, "profile": "minivan"}])
        states = {"bike": {"x": 0.0, "y": 4.0, "yaw": 0.0, "profile": "bicycle"}}
        self.assertEqual(driver_gateway.detect_contacts(world, states), {"bike": "obstacle"})


class GatewayCommandTest(unittest.TestCase):
    def setUp(self):
        self.shared = driver_gateway.Shared(OVERTAKE)

    def test_recent_input_is_forwarded(self):
        self.shared.apply_message("car", {"type": "input", "throttle": 0.5, "brake": 0.1, "steer": -0.2}, 10.0)
        self.assertEqual(self.shared.command("car", 10.1), (0.5, 0.1, -0.2, 0.0))

    def test_stale_input_becomes_a_light_brake(self):
        self.shared.apply_message("car", {"type": "input", "throttle": 1.0}, 10.0)
        self.assertEqual(self.shared.command("car", 12.0), (0.0, 0.3, 0.0, 0.0))

    def test_contact_and_respawn_override_input(self):
        self.shared.apply_message("bike", {"type": "input", "throttle": 1.0}, 10.0)
        self.shared.hold = "contact"
        self.assertEqual(self.shared.command("bike", 10.5), (0.0, 1.0, 0.0, 0.0))
        self.shared.apply_message("bike", {"type": "respawn"}, 10.6)
        self.assertEqual(self.shared.command("bike", 10.7)[3], 1.0)

    def test_start_hold_keeps_everyone_at_the_spawn_until_the_throttle(self):
        self.shared.hold = "start"
        self.shared.apply_message("car", {"type": "input", "brake": 1.0, "steer": 0.5}, 10.0)
        self.shared.release_start_hold(10.1)
        self.assertEqual(self.shared.command("car", 10.1), driver_gateway.HOLD)
        self.assertEqual(self.shared.command("bike", 10.1, fallback=(0.5, 0.0, 0.0)), driver_gateway.HOLD)
        self.shared.apply_message("bike", {"type": "input", "throttle": 0.6}, 10.2)
        self.shared.release_start_hold(10.3)
        self.assertIsNone(self.shared.hold)
        self.assertEqual(self.shared.command("bike", 10.3)[0], 0.6)

    def test_contact_hold_is_released_only_by_a_reset(self):
        self.shared.hold = "contact"
        self.shared.apply_message("car", {"type": "input", "throttle": 1.0}, 10.0)
        self.shared.release_start_hold(10.1)
        self.assertEqual(self.shared.hold, "contact")
        self.assertEqual(self.shared.command("car", 10.1), (0.0, 1.0, 0.0, 0.0))

    def test_only_the_car_can_select_a_car_profile(self):
        self.shared.apply_message("bike", {"type": "profile", "id": "truck"}, 1.0)
        self.assertEqual(self.shared.car_profile, "sedan")
        self.shared.apply_message("car", {"type": "profile", "id": "bicycle"}, 1.0)
        self.assertEqual(self.shared.car_profile, "sedan")
        self.shared.apply_message("car", {"type": "profile", "id": "truck"}, 1.0)
        self.assertEqual(self.shared.car_profile, "truck")

    def test_spectators_cannot_drive(self):
        self.shared.apply_message("spectator", {"type": "input", "throttle": 1.0}, 10.0)
        self.assertEqual(self.shared.command("bike", 10.1), (0.0, 0.3, 0.0, 0.0))

    def test_any_participant_can_switch_the_oncoming_vehicle_and_reset(self):
        self.shared.apply_message("spectator", {"type": "scenario", "oncoming": False}, 1.0)
        self.assertFalse(self.shared.oncoming_enabled)
        self.shared.apply_message("bike", {"type": "scenario", "oncoming": True}, 1.0)
        self.assertTrue(self.shared.oncoming_enabled)
        self.shared.apply_message(None, {"type": "reset"}, 1.0)
        self.assertTrue(self.shared.reset_requested)

    def test_fallback_drives_only_without_recent_input(self):
        fallback = (0.5, 0.0, 0.1)
        self.assertEqual(self.shared.command("bike", 10.0, fallback), (0.5, 0.0, 0.1, 0.0))
        self.shared.apply_message("bike", {"type": "input", "throttle": 1.0}, 10.0)
        self.assertEqual(self.shared.command("bike", 10.1, fallback)[0], 1.0)


class OvertakeScenarioTest(unittest.TestCase):
    def scenario(self):
        return driver_gateway.OncomingScenario(OVERTAKE["scenario"]["oncoming"], OVERTAKE["spawn"]["oncoming"],
                                               OVERTAKE["bounds"])

    def spawn_states(self, car_speed: float = 0.0) -> dict:
        s = OVERTAKE["spawn"]
        return {"car": pose(s["car"]["x"], s["car"]["y"], speed=car_speed),
                "bike": pose(s["bike"]["x"], s["bike"]["y"], speed=4.5, profile="bicycle"),
                "oncoming": pose(s["oncoming"]["x"], s["oncoming"]["y"], math.pi, 13.9, "bus")}

    def test_overtake_world_is_a_straight_two_lane_road_with_clear_spawns(self):
        self.assertEqual(len(OVERTAKE["roads"]), 1)
        states = {role: dict(spawn, profile=profile, speed=0.0) for role, profile, spawn in (
            ("bike", "bicycle", OVERTAKE["spawn"]["bike"]), ("car", "sedan", OVERTAKE["spawn"]["car"]),
            ("oncoming", "bus", OVERTAKE["spawn"]["oncoming"]))}
        self.assertEqual(driver_gateway.detect_contacts(OVERTAKE, states),
                         {"bike": None, "car": None, "oncoming": None})
        # The car starts behind the cyclist in the same lane; the bus starts beyond the fog.
        self.assertLess(OVERTAKE["spawn"]["car"]["x"], OVERTAKE["spawn"]["bike"]["x"])
        self.assertGreater(OVERTAKE["spawn"]["oncoming"]["x"] - OVERTAKE["spawn"]["car"]["x"], OVERTAKE["fog"][1])

    def test_bus_waits_until_the_car_moves_and_only_when_enabled(self):
        scenario = self.scenario()
        self.assertEqual(scenario.step(True, self.spawn_states(0.0), None), driver_gateway.HOLD)
        self.assertEqual(scenario.step(False, self.spawn_states(5.0), None), driver_gateway.HOLD)
        self.assertEqual(scenario.phase, "waiting")
        command = scenario.step(True, self.spawn_states(5.0), None)
        self.assertEqual(scenario.phase, "approaching")
        self.assertEqual(command[3], 0.0)

    def test_meeting_speed_brings_the_bus_to_the_predicted_overtaking_point(self):
        scenario = self.scenario()
        car = pose(-100.0, 1.6, speed=13.9)
        bike = pose(-40.0, 2.55, speed=4.5, profile="bicycle")
        bus = pose(60.0, -1.6, math.pi, 13.9, "bus")
        t_meet = 60.0 / (13.9 - 4.5)
        meet_x = -40.0 + 4.5 * t_meet
        expected = (60.0 - meet_x) / t_meet
        self.assertLess(expected, OVERTAKE["scenario"]["oncoming"]["max_speed"])
        # The spawn heading is 3.1416 (not exactly pi), hence 4 places.
        self.assertAlmostEqual(scenario.meeting_speed(car, bike, bus), expected, places=4)

    def test_meeting_speed_is_clamped_and_slow_when_the_car_hangs_back(self):
        scenario = self.scenario()
        cfg = OVERTAKE["scenario"]["oncoming"]
        bike = pose(-40.0, 2.55, speed=4.5, profile="bicycle")
        bus = pose(230.0, -1.6, math.pi, 13.9, "bus")
        self.assertEqual(scenario.meeting_speed(pose(-50.0, 1.6, speed=4.5), bike, bus), cfg["min_speed"])
        self.assertEqual(scenario.meeting_speed(pose(-41.0, 1.6, speed=30.0), bike, bus), cfg["max_speed"])

    def test_bus_leaves_after_passing_both_and_finishes_near_the_bounds(self):
        scenario = self.scenario()
        scenario.phase = "approaching"
        states = {"car": pose(0.0, 1.6, speed=13.9), "bike": pose(-5.0, 2.55, speed=4.5, profile="bicycle"),
                  "oncoming": pose(-30.0, -1.6, math.pi, 13.9, "bus")}
        scenario.step(True, states, None)
        self.assertEqual(scenario.phase, "leaving")
        states["oncoming"] = pose(-240.0, -1.6, math.pi, 13.9, "bus")
        self.assertEqual(scenario.step(True, states, None), driver_gateway.HOLD)
        self.assertEqual(scenario.phase, "finished")
        scenario.reset()
        self.assertEqual(scenario.phase, "waiting")

    def test_bus_brakes_on_contact(self):
        scenario = self.scenario()
        scenario.phase = "approaching"
        states = self.spawn_states(13.9)
        self.assertEqual(scenario.step(True, states, "vehicle"), (0.0, 1.0, 0.0, 0.0))

    def test_autopilot_keeps_the_lane_and_speed(self):
        spawn = OVERTAKE["spawn"]["bike"]
        throttle, brake, steer = driver_gateway.autopilot_command(
            pose(spawn["x"], spawn["y"] + 0.5, speed=2.0, profile="bicycle"), spawn, 4.5)
        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)
        self.assertLess(steer, 0.0)  # drifted left of the lane: steer right


class PassingMonitorTest(unittest.TestCase):
    def test_gap_is_measured_only_side_by_side(self):
        sedan, bike = vehicles.PROFILES["sedan"], vehicles.PROFILES["bicycle"]
        bike_pose = pose(0.0, 2.55, profile="bicycle")
        self.assertIsNone(vehicles.side_by_side_gap(sedan, pose(-10.0, 0.9), bike, bike_pose))
        gap = vehicles.side_by_side_gap(sedan, pose(0.5, 0.9), bike, bike_pose)
        self.assertAlmostEqual(gap, 1.65 - (1.8 + 0.6) / 2)

    def test_records_minimum_gap_speed_and_oncoming_after_the_car_is_ahead(self):
        monitor = driver_gateway.PassingMonitor()
        bike = pose(0.0, 2.55, speed=4.5, profile="bicycle")
        bus = pose(10.0, -1.6, math.pi, 13.9, "bus")
        monitor.update(pose(-2.0, 0.9, speed=10.0), bike, None)  # gap 0.45 m
        info = monitor.update(pose(0.0, 1.2, speed=12.0), bike, bus)  # gap 0.15 m, closest
        self.assertTrue(info["active"])
        info = monitor.update(pose(10.0, 1.2, speed=12.0), bike, None)
        self.assertFalse(info["active"])
        self.assertEqual(info["last"], {"min_gap": 0.15, "speed_kmh": 43.2, "oncoming_near": True})

    def test_warns_only_while_the_side_gap_is_under_one_metre(self):
        monitor = driver_gateway.PassingMonitor()
        bike = pose(0.0, 2.55, speed=4.5, profile="bicycle")
        self.assertTrue(monitor.update(pose(0.0, 0.9, speed=10.0), bike, None)["warning"])  # gap 0.45 m
        self.assertFalse(monitor.update(pose(0.0, 0.15, speed=10.0), bike, None)["warning"])  # gap 1.2 m
        info = monitor.update(pose(30.0, 0.15, speed=10.0), bike, None)
        self.assertFalse(info["warning"])  # no longer side by side
        self.assertEqual(info["min_clearance_m"], 1.0)

    def test_exactly_one_metre_is_not_a_warning(self):
        monitor = driver_gateway.PassingMonitor()
        bike = pose(0.0, 2.55, speed=4.5, profile="bicycle")
        with mock.patch.object(driver_gateway.vehicles, "side_by_side_gap", return_value=1.0):
            self.assertFalse(monitor.update(pose(0.0, 0.35, speed=10.0), bike, None)["warning"])

    def test_falling_back_is_not_an_overtake(self):
        monitor = driver_gateway.PassingMonitor()
        bike = pose(0.0, 2.55, speed=4.5, profile="bicycle")
        monitor.update(pose(-2.0, 1.2, speed=4.0), bike, None)
        info = monitor.update(pose(-10.0, 1.2, speed=4.0), bike, None)
        self.assertIsNone(info["last"])


def simulate_demo(profile_id: str, oncoming: bool, limit_sec: float = 90.0) -> dict:
    """Closed-loop kinematic run of the demo driver against the oncoming-bus scenario."""
    s = OVERTAKE["spawn"]
    car = vehicles.VehicleState(s["car"]["x"], s["car"]["y"], s["car"]["yaw"])
    bike = vehicles.VehicleState(s["bike"]["x"], s["bike"]["y"], s["bike"]["yaw"])
    bus_spawn = vehicles.VehicleState(s["oncoming"]["x"], s["oncoming"]["y"], s["oncoming"]["yaw"])
    bus = bus_spawn
    demo = driver_gateway.DemoDriver(OVERTAKE, oncoming)
    scenario = driver_gateway.OncomingScenario(OVERTAKE["scenario"]["oncoming"], s["oncoming"], OVERTAKE["bounds"])
    monitor = driver_gateway.PassingMonitor()
    phases, contacts, t = [], set(), 0.0
    as_dict = lambda state, profile: {"x": state.x, "y": state.y, "yaw": state.yaw, "speed": state.speed,
                                      "profile": profile}
    while demo.phase != "done" and t < limit_sec:
        states = {"car": as_dict(car, profile_id), "bike": as_dict(bike, "bicycle"), "oncoming": as_dict(bus, "bus")}
        found = driver_gateway.detect_contacts(OVERTAKE, states)
        contacts.update((role, kind) for role, kind in found.items() if kind)
        threat = oncoming and scenario.phase in ("waiting", "approaching")
        bike_command, car_command = demo.step(states["car"], states["bike"], states["oncoming"], threat)
        bus_command = scenario.step(oncoming, states, found.get("oncoming"))
        monitor.update(states["car"], states["bike"], states["oncoming"])
        if not phases or phases[-1] != demo.phase:
            phases.append(demo.phase)
        car = vehicles.step(vehicles.PROFILES[profile_id], car, *car_command)
        bike = vehicles.step(vehicles.PROFILES["bicycle"], bike, *bike_command)
        bus = bus_spawn if bus_command[3] else vehicles.step(vehicles.PROFILES["bus"], bus, *bus_command[:3])
        t += vehicles.DT_SEC
    return {"phases": phases, "last": monitor.last, "contacts": contacts, "car": car}


class DemoTest(unittest.TestCase):
    def test_every_car_overtakes_with_at_least_one_metre_and_no_contact(self):
        for profile_id in vehicles.CAR_PROFILE_IDS:
            for oncoming in (False, True):
                with self.subTest(profile=profile_id, oncoming=oncoming):
                    run = simulate_demo(profile_id, oncoming)
                    self.assertEqual(run["phases"][-1], "done")
                    self.assertIn("pass", run["phases"])
                    self.assertEqual(run["contacts"], set())
                    self.assertGreaterEqual(run["last"]["min_gap"], driver_gateway.PassingMonitor.MIN_CLEARANCE_M)
                    # Back in the car lane at the end.
                    self.assertAlmostEqual(run["car"].y, OVERTAKE["spawn"]["car"]["y"], delta=0.3)

    def test_with_the_bus_the_car_waits_and_passes_after_it(self):
        run = simulate_demo("sedan", True)
        self.assertEqual(run["phases"][:2], ["wait", "pass"])
        self.assertFalse(run["last"]["oncoming_near"])

    def test_pass_line_keeps_the_target_clearance_for_each_width(self):
        demo = driver_gateway.DemoDriver(OVERTAKE, False)
        bike = vehicles.PROFILES["bicycle"]
        for profile_id in vehicles.CAR_PROFILE_IDS:
            profile = vehicles.PROFILES[profile_id]
            lateral = demo.pass_lateral(profile, 0.95)
            self.assertAlmostEqual(0.95 - lateral - (bike.width + profile.width) / 2, demo.TARGET_CLEARANCE_M)

    def test_far_bus_or_no_threat_is_clear(self):
        demo = driver_gateway.DemoDriver(OVERTAKE, True)
        car, bike = pose(-60.0, 1.6, speed=11.1), pose(-40.0, 2.55, speed=4.5, profile="bicycle")
        near_bus = pose(40.0, -1.6, math.pi, 8.0, "bus")
        self.assertFalse(demo.oncoming_clear(car, bike, near_bus, True))
        self.assertTrue(demo.oncoming_clear(car, bike, near_bus, False))
        self.assertTrue(demo.oncoming_clear(car, bike, pose(-80.0, -1.6, math.pi, 8.0, "bus"), True))

    def test_demo_messages_are_open_to_everyone(self):
        shared = driver_gateway.Shared(OVERTAKE)
        shared.apply_message("spectator", {"type": "demo", "oncoming": True}, 1.0)
        self.assertEqual(shared.demo_request, {"oncoming": True})
        shared.apply_message("bike", {"type": "demo", "stop": True}, 1.0)
        self.assertEqual(shared.demo_request, {"stop": True})

    def test_held_controls_count_as_manual_only_while_fresh(self):
        shared = driver_gateway.Shared(OVERTAKE)
        shared.apply_message("car", {"type": "input", "brake": 1.0, "active": True}, 10.0)
        self.assertTrue(shared.manual("car", 10.2))
        self.assertFalse(shared.manual("car", 10.0 + driver_gateway.MANUAL_HOLD_SEC + 0.1))
        shared.apply_message("car", {"type": "input", "steer": 0.4, "active": False}, 11.0)
        self.assertFalse(shared.manual("car", 11.0))
        self.assertFalse(shared.manual("bike", 11.0))

    def test_manual_driver_takes_over_their_vehicle_and_the_demo_resumes_on_release(self):
        args = argparse.Namespace(world=str(APP / "world-overtake.json"), host="127.0.0.1", port=0, delta_usec=20000)
        gateway = driver_gateway.Gateway(args)
        gateway.demo = driver_gateway.DemoDriver(OVERTAKE, False)
        gateway.shared.hold = None  # start_demo leaves the start hold
        states = {"car": pose(-60.0, 1.6, speed=11.1), "bike": pose(-40.0, 2.55, speed=4.5, profile="bicycle")}
        gateway.shared.apply_message("car", {"type": "input", "brake": 1.0, "active": True}, 10.0)
        demo = gateway.demo_commands(states, 10.1, False)
        self.assertNotIn("car", demo)  # the driver's brake wins
        self.assertIn("bike", demo)
        self.assertEqual(gateway.shared.command("car", 10.1, override=demo.get("car")), (0.0, 1.0, 0.0, 0.0))
        demo = gateway.demo_commands(states, 11.0, False)  # released: no fresh active input
        self.assertIn("car", demo)

    def test_demo_override_ignores_driver_input_but_not_the_respawn_hold(self):
        shared = driver_gateway.Shared(OVERTAKE)
        shared.apply_message("car", {"type": "input", "throttle": 1.0}, 10.0)
        self.assertEqual(shared.command("car", 10.1, override=(0.2, 0.0, 0.3)), (0.2, 0.0, 0.3, 0.0))
        shared.respawn_until["car"] = 11.0
        self.assertEqual(shared.command("car", 10.5, override=(0.2, 0.0, 0.3)), driver_gateway.HOLD)


class HoldTest(unittest.TestCase):
    def tick(self, gateway, states, now):
        with mock.patch.object(gateway, "read_state", side_effect=lambda role: states.get(role)), \
                mock.patch.object(gateway, "pace"), \
                mock.patch.object(driver_gateway, "hakopy") as hakopy, \
                mock.patch.object(driver_gateway.time, "monotonic", return_value=now):
            hakopy.simulation_time.return_value = 0
            gateway.tick()
        return gateway.shared.snapshot

    def test_contact_stops_everything_until_the_reset_returns_to_the_start_hold(self):
        args = argparse.Namespace(world=str(APP / "world-overtake.json"), host="127.0.0.1", port=0, delta_usec=20000)
        gateway = driver_gateway.Gateway(args)
        self.assertEqual(gateway.shared.hold, "start")
        gateway.shared.apply_message("car", {"type": "input", "throttle": 1.0}, 10.0)
        apart = {"car": pose(-60.0, 1.6, speed=5.0), "bike": pose(-40.0, 2.55, speed=4.5, profile="bicycle")}
        self.assertIsNone(self.tick(gateway, apart, 10.1)["scenario"]["hold"])
        gateway.demo = driver_gateway.DemoDriver(OVERTAKE, False)
        touching = {"car": pose(-40.0, 2.0, speed=5.0), "bike": pose(-40.0, 2.55, speed=4.5, profile="bicycle")}
        self.assertEqual(self.tick(gateway, touching, 10.2)["scenario"]["hold"], "contact")
        self.assertIsNone(gateway.demo)
        self.assertEqual(gateway.shared.command("car", 10.2), (0.0, 1.0, 0.0, 0.0))
        self.assertEqual(self.tick(gateway, apart, 10.3)["scenario"]["hold"], "contact")
        gateway.shared.apply_message(None, {"type": "reset"}, 10.4)
        # Stale overlapping poses right after the reset must not stop the scenario again.
        gateway.shared.apply_message("car", {"type": "input", "throttle": 0.0}, 10.5)
        self.assertEqual(self.tick(gateway, touching, 10.5)["scenario"]["hold"], "start")
        self.assertEqual(gateway.shared.command("car", 11.0), driver_gateway.HOLD)


class PacingTest(unittest.TestCase):
    def gateway(self):
        args = argparse.Namespace(world=str(APP / "world-overtake.json"), host="127.0.0.1", port=0, delta_usec=20000)
        return driver_gateway.Gateway(args)

    def test_sleeps_when_simulation_runs_ahead(self):
        gateway = self.gateway()
        with mock.patch.object(driver_gateway.time, "monotonic", side_effect=[100.0, 100.0]), \
                mock.patch.object(driver_gateway.time, "sleep") as sleep:
            gateway.pace(0)
            gateway.pace(50_000)
        sleep.assert_called_once()
        self.assertAlmostEqual(sleep.call_args.args[0], 0.05)

    def test_large_lag_resets_the_reference_instead_of_fast_forwarding(self):
        gateway = self.gateway()
        with mock.patch.object(driver_gateway.time, "monotonic", side_effect=[100.0, 102.0, 102.0]), \
                mock.patch.object(driver_gateway.time, "sleep") as sleep:
            gateway.pace(0)
            gateway.pace(20_000)
        sleep.assert_not_called()
        self.assertEqual((gateway.wall_start, gateway.sim_start), (102.0, 20_000))


class SmokeEvaluationTest(unittest.TestCase):
    def entry(self, moved: float, profile: str) -> dict:
        return {"welcome": True, "profiles": 5, "first": {"x": 0.0, "y": 0.0},
                "last": {"x": moved, "y": 0.0, "profile": profile}, "occupied": {"bike": True, "car": True}}

    def test_pass(self):
        log = {"bike": self.entry(5.0, "bicycle"), "car": self.entry(5.0, "truck")}
        self.assertEqual(smoke_client.evaluate(log, "truck"), [])

    def test_fails_when_a_vehicle_does_not_move_or_profile_is_wrong(self):
        log = {"bike": self.entry(0.5, "bicycle"), "car": self.entry(5.0, "sedan")}
        errors = smoke_client.evaluate(log, "truck")
        self.assertEqual(len(errors), 2)

    def overtake_run(self, oncoming: bool, near: bool, bus_end_x: float) -> dict:
        return {"oncoming": oncoming, "phases": ["waiting", "approaching"] if oncoming else ["waiting"],
                "contacts": [], "autopilot": True, "bike_start": {"x": -40.0}, "bike_end": {"x": 10.0},
                "bus_start": {"x": 230.0}, "bus_end": {"x": bus_end_x},
                "passing": {"min_gap": 0.4, "speed_kmh": 50.0, "oncoming_near": near}}

    def test_overtake_pass_requires_the_bus_only_when_enabled(self):
        runs = [self.overtake_run(True, True, 20.0), self.overtake_run(False, False, 230.0)]
        self.assertEqual(smoke_client.evaluate_overtake(runs), [])

    def test_overtake_fails_when_the_switch_is_ignored(self):
        runs = [self.overtake_run(True, False, 20.0), self.overtake_run(False, False, 100.0)]
        self.assertEqual(len(smoke_client.evaluate_overtake(runs)), 2)

    def demo_run(self, oncoming: bool, min_gap: float, phases: list[str]) -> dict:
        return {"oncoming": oncoming, "phases": phases, "contacts": [], "finished": True,
                "passing": {"min_gap": min_gap, "speed_kmh": 40.0, "oncoming_near": False}, "min_clearance_m": 1.0}

    def test_demo_pass_requires_clearance_and_waiting_for_the_bus(self):
        runs = [self.demo_run(False, 1.4, ["approach", "pass", "return", "stopping"]),
                self.demo_run(True, 1.4, ["wait", "pass", "return", "stopping"])]
        self.assertEqual(smoke_client.evaluate_demo(runs), [])
        runs = [self.demo_run(False, 0.8, ["approach", "pass"]), self.demo_run(True, 1.4, ["approach", "pass"])]
        self.assertEqual(len(smoke_client.evaluate_demo(runs)), 2)

    def test_overtake_driver_moves_towards_the_centre_line_beside_the_bicycle(self):
        bike = pose(0.0, 2.55, profile="bicycle")
        self.assertLess(smoke_client.overtake_input(pose(-5.0, 1.6), bike)["steer"], 0.0)
        self.assertAlmostEqual(smoke_client.overtake_input(pose(-60.0, 1.6), bike)["steer"], 0.0)


class RunnerTest(unittest.TestCase):
    def setUp(self):
        # workspace.py run sets HAKONIWA_WORK_DIR; these tests expect the default work directory.
        environ = mock.patch.dict(os.environ)
        environ.start()
        self.addCleanup(environ.stop)
        os.environ.pop("HAKONIWA_WORK_DIR", None)

    def args(self, **overrides) -> argparse.Namespace:
        values = {"host": "127.0.0.1", "http_port": 8770, "ws_port": 8771, "car_profile": "sedan",
                  "oncoming": "on"}
        values.update(overrides)
        return argparse.Namespace(**values)

    def test_recipe_identity_is_applied_to_managed_helpers(self):
        self.assertEqual(recipe.base.RECIPE_ID, recipe.RECIPE_ID)
        self.assertTrue((Path(__file__).parents[2] / "recipes/examples" / f"{recipe.RECIPE_ID}.yaml").is_file())

    def test_launcher_runs_gateway_first_and_stays_in_the_recipe_workspace(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve() / "business"
            app = root / "work/recipes/x/assets/app"
            with mock.patch.object(recipe.base, "business_root", return_value=root):
                launch = recipe.write_launcher(self.args(car_profile="truck"), app)
            data = json.loads(launch.read_text(encoding="utf-8"))
            names = [asset["name"] for asset in data["assets"]]
            self.assertEqual(names, ["driver_gateway", "bike_plant", "car_plant", "oncoming_plant", "web_server"])
            assets = {asset["name"]: asset for asset in data["assets"]}
            self.assertIn("truck", assets["car_plant"]["args"])
            self.assertEqual(assets["driver_gateway"]["args"][-4:], ["--host", "127.0.0.1", "--port", "8771"])
            self.assertTrue(Path(launch).is_relative_to(root / "work/recipes"))
            self.assertTrue(Path(data["defaults"]["cwd"]).is_relative_to(app))
            self.assertTrue(assets["driver_gateway"]["args"][4].endswith("world-overtake.json"))

    def test_launcher_passes_the_oncoming_setting(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve() / "business"
            app = root / "work/recipes/x/assets/app"
            with mock.patch.object(recipe.base, "business_root", return_value=root):
                launch = recipe.write_launcher(self.args(oncoming="off"), app)
            data = json.loads(launch.read_text(encoding="utf-8"))
            assets = {asset["name"]: asset for asset in data["assets"]}
            self.assertIn("oncoming", assets["oncoming_plant"]["args"])
            gateway = assets["driver_gateway"]["args"]
            self.assertEqual(gateway[gateway.index("--oncoming") + 1], "off")

    def test_urls_carry_role_and_websocket(self):
        urls = recipe.urls(self.args(host="127.0.0.1", http_port=9000, ws_port=9001))
        self.assertEqual(urls["bike"], "http://127.0.0.1:9000/index.html?role=bike&ws=ws://127.0.0.1:9001")
        self.assertIn("role=car", urls["car"])
        self.assertEqual(urls["aerial"],
                         "http://127.0.0.1:9000/index.html?role=spectator&view=aerial&ws=ws://127.0.0.1:9001")

    def test_app_sources_exist(self):
        for name in recipe.APP_FILES:
            self.assertTrue((recipe.APP_SOURCE / name).is_file(), name)
        for name in ("index.html", "app.js", "vehicle_models.js"):
            self.assertTrue((recipe.APP_SOURCE / "web" / name).is_file(), name)


if __name__ == "__main__":
    unittest.main()
