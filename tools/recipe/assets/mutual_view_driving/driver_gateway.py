#!/usr/bin/env python3
"""Hakoniwa controller asset that connects driver browsers to the road-user plants.

It owns the single-host Conductor, paces simulation time to wall-clock time,
turns browser inputs into Bike/Car cmd PDUs, drives the scenario vehicles
(oncoming bus, bicycle autopilot), reads pose/vel PDUs, detects contacts, and
broadcasts one shared state to every WebSocket client.

Browser -> gateway (JSON):
  {"type": "join", "role": "bike" | "car" | "spectator"}
  {"type": "input", "throttle": -1..1, "brake": 0..1, "steer": -1..1, "look": rad}
  {"type": "profile", "id": "sedan" | "suv" | "minivan" | "truck"}   (car only)
  {"type": "respawn"}
  {"type": "scenario", "oncoming": true | false}                     (any participant)
  {"type": "reset"}                                                  (any participant)
Gateway -> browser (JSON):
  {"type": "welcome", "world": {...}, "profiles": [...], "role": ...}
  {"type": "state", "t": sim_sec, "bike": {...}, "car": {...}, "oncoming": {...} | absent,
   "contact": {...}, "occupied": {...}, "scenario": {...}, "passing": {...}}
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import threading
import time
from pathlib import Path

import hakopy
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import pdu_to_py_Twist, py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Vector3 import py_to_pdu_Vector3
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Vector3 import Vector3
from websockets.asyncio.server import serve

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vehicles  # noqa: E402

CH_CMD, CH_POSE, CH_VEL, CH_PROFILE = 0, 1, 2, 3
TWIST_SIZE, VECTOR3_SIZE = 72, 48
USERS = {"bike": "Bike", "car": "Car"}
ROBOTS = {**USERS, "oncoming": "Oncoming"}
INPUT_TIMEOUT_SEC = 1.0
CONTACT_HOLD_SEC = 1.5
RESPAWN_HOLD_SEC = 0.2
BROADCAST_HZ = 30.0
MAX_LAG_SEC = 0.2
HOLD = (0.0, 1.0, 0.0, 1.0)  # brake and keep respawning at the spawn pose


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pdu-def", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8771)
    parser.add_argument("--oncoming", choices=("on", "off"), default="on",
                        help="initial state of the oncoming vehicle when the world defines one")
    parser.add_argument("--delta-usec", type=int, default=int(vehicles.DT_SEC * 1_000_000))
    return parser.parse_args(argv)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def speed_command(speed: float, target: float) -> tuple[float, float]:
    """(throttle, brake) that tracks a target speed."""
    error = target - speed
    return _clamp(error * 0.8, 0.0, 1.0), _clamp(-error * 0.5, 0.0, 1.0)


class Shared:
    """State exchanged between the simulation thread and the WebSocket thread."""

    def __init__(self, world: dict, oncoming_enabled: bool = True):
        self.lock = threading.Lock()
        self.world = world
        self.inputs = {role: {"throttle": 0.0, "brake": 0.0, "steer": 0.0, "look": 0.0, "at": 0.0}
                       for role in USERS}
        self.respawn_until = {role: 0.0 for role in USERS}
        self.contact_until = {role: 0.0 for role in USERS}
        self.car_profile = "sedan"
        self.occupied = {role: 0 for role in USERS}
        self.oncoming_enabled = oncoming_enabled
        self.reset_requested = False
        self.snapshot: dict | None = None

    def apply_message(self, role: str | None, message: dict, now: float) -> None:
        kind = message.get("type")
        with self.lock:
            # Scenario controls are open to every participant, spectators included.
            if kind == "scenario" and isinstance(message.get("oncoming"), bool):
                self.oncoming_enabled = message["oncoming"]
                return
            if kind == "reset":
                self.reset_requested = True
                return
            if role not in USERS:
                return
            if kind == "input":
                self.inputs[role] = {
                    "throttle": float(message.get("throttle", 0.0)),
                    "brake": float(message.get("brake", 0.0)),
                    "steer": float(message.get("steer", 0.0)),
                    "look": float(message.get("look", 0.0)),
                    "at": now,
                }
            elif kind == "respawn":
                self.respawn_until[role] = now + RESPAWN_HOLD_SEC
            elif kind == "profile" and role == "car" and message.get("id") in vehicles.CAR_PROFILE_IDS:
                self.car_profile = message["id"]

    def command(self, role: str, now: float,
                fallback: tuple[float, float, float] | None = None) -> tuple[float, float, float, float]:
        """(throttle, brake, steer, respawn) to send to a plant.

        `fallback` drives the vehicle when nobody has sent input recently (autopilot);
        otherwise a stale input becomes a light brake.
        """
        with self.lock:
            if now < self.respawn_until[role]:
                return HOLD
            if now < self.contact_until[role]:
                return 0.0, 1.0, 0.0, 0.0
            data = self.inputs[role]
            if now - data["at"] > INPUT_TIMEOUT_SEC:
                return (*fallback, 0.0) if fallback else (0.0, 0.3, 0.0, 0.0)
            return data["throttle"], data["brake"], data["steer"], 0.0


def detect_contacts(world: dict, states: dict) -> dict:
    """Which road users touch each other or a static obstacle."""
    polys = {}
    for role, s in states.items():
        polys[role] = vehicles.footprint(vehicles.PROFILES[s["profile"]], s["x"], s["y"], s["yaw"])
    statics = [vehicles.rect_polygon(b) for b in world["buildings"]]
    statics += [vehicles.footprint(vehicles.PROFILES[p["profile"]], p["x"], p["y"], p["yaw"])
                for p in world.get("parked", [])]
    result = {role: None for role in states}
    roles = list(states)
    for i, a in enumerate(roles):
        for b in roles[i + 1:]:
            if vehicles.polygons_overlap(polys[a], polys[b]):
                result[a] = result[b] = "vehicle"
    for role in roles:
        if result[role] is None and any(vehicles.polygons_overlap(polys[role], p) for p in statics):
            result[role] = "obstacle"
    return result


def autopilot_command(state: dict, spawn: dict, speed: float) -> tuple[float, float, float]:
    """Keep the spawn lane and heading at a constant speed (bicycle autopilot)."""
    yaw0 = spawn["yaw"]
    lateral = -math.sin(yaw0) * (state["x"] - spawn["x"]) + math.cos(yaw0) * (state["y"] - spawn["y"])
    steer = _clamp(-0.6 * lateral - 1.5 * _wrap(state["yaw"] - yaw0), -1.0, 1.0)
    throttle, brake = speed_command(state["speed"], speed)
    return throttle, brake, steer


class OncomingScenario:
    """Times an oncoming vehicle so that it meets the car where the car catches the bicycle.

    waiting     held at its spawn (out of sight in the fog) until the car starts moving
    approaching speed chosen every step from the predicted overtaking point and time
    leaving     passed both road users; cruises away
    finished    held at its spawn again until the scenario is reset
    """

    def __init__(self, config: dict, spawn: dict, bounds: dict):
        self.config = config
        self.spawn = spawn
        self.bounds = bounds
        self.heading = (math.cos(spawn["yaw"]), math.sin(spawn["yaw"]))
        self.phase = "waiting"
        self.target_speed = 0.0

    def reset(self) -> None:
        self.phase, self.target_speed = "waiting", 0.0

    def _along(self, state: dict) -> float:
        """Distance travelled from the spawn along the oncoming vehicle's heading."""
        return (state["x"] - self.spawn["x"]) * self.heading[0] + (state["y"] - self.spawn["y"]) * self.heading[1]

    def meeting_speed(self, car: dict, bike: dict, bus: dict) -> float:
        cfg = self.config
        # Positions along the car's direction of travel (opposite to the oncoming heading).
        car_u, bike_u, bus_u = -self._along(car), -self._along(bike), -self._along(bus)
        gap = bike_u - car_u
        closing = car["speed"] - bike["speed"]
        if gap <= 0.0:
            # Side by side or already past: keep the current speed through the encounter.
            return _clamp(bus["speed"], cfg["min_speed"], cfg["max_speed"])
        if closing < 0.5:
            return cfg["min_speed"]
        t_meet = gap / closing
        meet_u = bike_u + bike["speed"] * t_meet
        return _clamp((bus_u - meet_u) / t_meet, cfg["min_speed"], cfg["max_speed"])

    def _near_bounds(self, state: dict) -> bool:
        b, m = self.bounds, 30.0
        return (state["x"] < b["x0"] + m or state["x"] > b["x1"] - m
                or state["y"] < b["y0"] + m or state["y"] > b["y1"] - m)

    def step(self, enabled: bool, states: dict, contact: str | None) -> tuple[float, float, float, float]:
        car, bike, bus = states.get("car"), states.get("bike"), states.get("oncoming")
        if car is None or bike is None or bus is None:
            return HOLD
        cfg = self.config
        if self.phase == "waiting":
            if enabled and car["speed"] > cfg["trigger_speed"]:
                self.phase = "approaching"
            else:
                return HOLD
        if self.phase == "approaching":
            margin = cfg["pass_margin"]
            if self._along(bus) > max(self._along(car), self._along(bike)) + margin:
                self.phase = "leaving"
            else:
                self.target_speed = self.meeting_speed(car, bike, bus)
        if self.phase == "leaving":
            self.target_speed = cfg["leave_speed"]
            if self._along(bus) > 50.0 and self._near_bounds(bus):
                self.phase = "finished"
        if self.phase == "finished":
            return HOLD
        if contact:
            return 0.0, 1.0, 0.0, 0.0
        throttle, brake = speed_command(bus["speed"], self.target_speed)
        return throttle, brake, 0.0, 0.0


class PassingMonitor:
    """Lateral clearance and speed while the car passes the bicycle."""

    ONCOMING_NEAR_M = 40.0
    MIN_CLEARANCE_M = 1.0  # rule: keep at least 1 m beside the bicycle when overtaking

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.current: dict | None = None
        self.last: dict | None = None

    def update(self, car: dict | None, bike: dict | None, oncoming: dict | None) -> dict:
        gap = None
        if car is not None and bike is not None:
            gap = vehicles.side_by_side_gap(vehicles.PROFILES[car["profile"]], car,
                                            vehicles.PROFILES["bicycle"], bike)
        if gap is not None:
            near = oncoming is not None and math.hypot(oncoming["x"] - bike["x"],
                                                       oncoming["y"] - bike["y"]) < self.ONCOMING_NEAR_M
            speed_kmh = abs(car["speed"]) * 3.6
            if self.current is None:
                self.current = {"min_gap": gap, "speed_kmh": speed_kmh, "oncoming_near": near}
            elif gap < self.current["min_gap"]:
                self.current.update(min_gap=gap, speed_kmh=speed_kmh)
            self.current["oncoming_near"] = self.current["oncoming_near"] or near
        elif self.current is not None:
            # Count it as an overtake only when the car ended up ahead of the bicycle.
            ahead = ((car["x"] - bike["x"]) * math.cos(bike["yaw"])
                     + (car["y"] - bike["y"]) * math.sin(bike["yaw"])) > 0
            if ahead:
                self.last = {key: round(value, 2) if isinstance(value, float) else value
                             for key, value in self.current.items()}
            self.current = None
        return {"active": gap is not None, "gap": None if gap is None else round(gap, 2),
                "min_gap": None if self.current is None else round(self.current["min_gap"], 2),
                "warning": gap is not None and gap < self.MIN_CLEARANCE_M,
                "min_clearance_m": self.MIN_CLEARANCE_M,
                "last": self.last}


class Gateway:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.world = json.loads(Path(args.world).read_text(encoding="utf-8"))
        self.scenario_config = self.world.get("scenario", {})
        self.shared = Shared(self.world, oncoming_enabled=getattr(args, "oncoming", "on") == "on")
        self.oncoming = None
        if "oncoming" in self.scenario_config:
            self.oncoming = OncomingScenario(self.scenario_config["oncoming"], self.world["spawn"]["oncoming"],
                                             self.world["bounds"])
        self.autopilot = self.scenario_config.get("bike_autopilot")
        self.autopilot_started = False
        self.passing = PassingMonitor()
        self.roles = list(USERS) + (["oncoming"] if self.oncoming else [])
        self.previous_contact = {role: None for role in self.roles}
        self.wall_start: float | None = None
        self.sim_start: int | None = None

    # ---- simulation thread -------------------------------------------------
    def read_state(self, role: str) -> dict | None:
        robot = ROBOTS[role]
        pose_raw = hakopy.pdu_read(robot, CH_POSE, TWIST_SIZE)
        vel_raw = hakopy.pdu_read(robot, CH_VEL, TWIST_SIZE)
        if not pose_raw or not vel_raw:
            return None
        try:
            # ValueError until the plant has written the channel once (no metadata yet).
            pose, vel = pdu_to_py_Twist(pose_raw), pdu_to_py_Twist(vel_raw)
        except ValueError:
            return None
        try:
            profile = vehicles.profile_by_index(vel.angular.x).id
        except ValueError:
            return None
        if role == "bike":
            profile = "bicycle"
        return {"x": pose.linear.x, "y": pose.linear.y, "yaw": pose.angular.z,
                "speed": vel.linear.x, "steer": vel.angular.z, "profile": profile}

    def reset_scenario(self, now: float) -> None:
        with self.shared.lock:
            for role in USERS:
                self.shared.respawn_until[role] = now + RESPAWN_HOLD_SEC
            self.shared.reset_requested = False
        if self.oncoming:
            self.oncoming.reset()
        self.autopilot_started = False
        self.passing.reset()

    def bike_fallback(self, states: dict) -> tuple[float, float, float] | None:
        """Autopilot for an unoccupied bicycle once the car has started moving."""
        bike, car = states.get("bike"), states.get("car")
        if not self.autopilot or bike is None or car is None:
            return None
        with self.shared.lock:
            occupied = self.shared.occupied["bike"] > 0
        if occupied:
            return None
        if not self.autopilot_started and car["speed"] > 1.0:
            self.autopilot_started = True
        if not self.autopilot_started:
            return None
        return autopilot_command(bike, self.world["spawn"]["bike"], self.autopilot["speed"])

    def tick(self) -> None:
        now = time.monotonic()
        with self.shared.lock:
            reset = self.shared.reset_requested
        if reset:
            self.reset_scenario(now)
        states = {role: self.read_state(role) for role in self.roles}
        ready = {role: s for role, s in states.items() if s is not None}
        contacts = detect_contacts(self.world, ready) if ready else {}
        with self.shared.lock:
            for role, kind in contacts.items():
                # Hold only on a new contact so a driver can back out of it afterwards.
                if kind and not self.previous_contact.get(role) and role in USERS:
                    self.shared.contact_until[role] = now + CONTACT_HOLD_SEC
                self.previous_contact[role] = kind
            car_profile = self.shared.car_profile
            oncoming_enabled = self.shared.oncoming_enabled

        commands = {
            "bike": self.shared.command("bike", now, fallback=self.bike_fallback(states)),
            "car": self.shared.command("car", now),
        }
        if self.oncoming:
            commands["oncoming"] = self.oncoming.step(oncoming_enabled, states, contacts.get("oncoming"))
        for role, (throttle, brake, steer, respawn) in commands.items():
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z, cmd.angular.x = throttle, brake, steer, respawn
            hakopy.pdu_write(ROBOTS[role], CH_CMD, py_to_pdu_Twist(cmd), TWIST_SIZE)
        profile = Vector3()
        profile.x = float(vehicles.profile_index(car_profile))
        hakopy.pdu_write("Car", CH_PROFILE, py_to_pdu_Vector3(profile), VECTOR3_SIZE)

        sim_usec = int(hakopy.simulation_time())
        snapshot = {"type": "state", "t": sim_usec / 1_000_000, "contact": contacts,
                    "car_profile_request": car_profile,
                    "scenario": {"id": self.scenario_config.get("id"), "name": self.scenario_config.get("name"),
                                 "has_oncoming": self.oncoming is not None,
                                 "oncoming_enabled": oncoming_enabled,
                                 "phase": self.oncoming.phase if self.oncoming else None,
                                 "bike_autopilot": self.autopilot_started},
                    "passing": self.passing.update(states.get("car"), states.get("bike"), states.get("oncoming"))}
        snapshot.update(states)
        with self.shared.lock:
            for role in USERS:
                if states.get(role) is not None:
                    # Head direction relative to the vehicle, so each PiP shows where the partner looks.
                    states[role]["look"] = self.shared.inputs[role]["look"]
            snapshot["occupied"] = {role: count > 0 for role, count in self.shared.occupied.items()}
            self.shared.snapshot = snapshot
        self.pace(sim_usec)

    def pace(self, sim_usec: int) -> None:
        """Keep simulation time from running ahead of wall-clock time."""
        if self.wall_start is None:
            self.wall_start, self.sim_start = time.monotonic(), sim_usec
            return
        ahead = (sim_usec - self.sim_start) / 1_000_000 - (time.monotonic() - self.wall_start)
        if ahead > 0:
            time.sleep(ahead)
        elif ahead < -MAX_LAG_SEC:
            # Drop the debt instead of fast-forwarding: a burst would move vehicles faster than drivers can react.
            self.wall_start, self.sim_start = time.monotonic(), sim_usec

    # ---- WebSocket thread --------------------------------------------------
    async def handle(self, websocket) -> None:
        role = None
        try:
            async for text in websocket:
                try:
                    message = json.loads(text)
                except json.JSONDecodeError:
                    continue
                if message.get("type") == "join":
                    wanted = message.get("role")
                    with self.shared.lock:
                        if role in USERS:
                            self.shared.occupied[role] -= 1
                        role = wanted if wanted in USERS else "spectator"
                        if role in USERS:
                            self.shared.occupied[role] += 1
                    await websocket.send(json.dumps({
                        "type": "welcome", "role": role, "world": self.world,
                        "profiles": vehicles.profiles_payload(),
                    }, ensure_ascii=False))
                else:
                    self.shared.apply_message(role, message, time.monotonic())
        finally:
            if role in USERS:
                with self.shared.lock:
                    self.shared.occupied[role] -= 1

    async def serve(self, stop: asyncio.Event) -> None:
        clients = set()

        async def handler(websocket):
            clients.add(websocket)
            try:
                await self.handle(websocket)
            finally:
                clients.discard(websocket)

        async with serve(handler, self.args.host, self.args.port):
            print(f"[INFO] driver gateway listening on ws://{self.args.host}:{self.args.port}", flush=True)
            while not stop.is_set():
                with self.shared.lock:
                    snapshot = self.shared.snapshot
                if snapshot is not None and clients:
                    text = json.dumps(snapshot, ensure_ascii=False)
                    for websocket in list(clients):
                        try:
                            await websocket.send(text)
                        except Exception:
                            clients.discard(websocket)
                await asyncio.sleep(1.0 / BROADCAST_HZ)

    def start_server_thread(self) -> tuple[threading.Thread, callable]:
        loop = asyncio.new_event_loop()
        stop = asyncio.Event()

        def run():
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.serve(stop))

        thread = threading.Thread(target=run, name="driver-gateway-ws", daemon=True)
        thread.start()
        return thread, lambda: loop.call_soon_threadsafe(stop.set)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    gateway = Gateway(args)
    result = {"code": 0}

    def on_manual_timing_control(_context):
        try:
            while True:
                gateway.tick()
                if not hakopy.usleep(args.delta_usec):
                    break
        except Exception as exc:
            print(f"[ERROR] driver gateway failed: {exc}", file=sys.stderr, flush=True)
            result["code"] = 1
        return 0

    callbacks = {
        "on_initialize": lambda _context: 0,
        "on_simulation_step": None,
        "on_manual_timing_control": on_manual_timing_control,
        "on_reset": lambda _context: 0,
    }
    thread, stop_server = gateway.start_server_thread()
    hakopy.conductor_start(args.delta_usec, args.delta_usec)
    try:
        if not hakopy.asset_register("DriverGateway", args.pdu_def, callbacks, args.delta_usec,
                                     hakopy.HAKO_ASSET_MODEL_CONTROLLER):
            print("[ERROR] hakopy.asset_register(DriverGateway) failed", file=sys.stderr)
            return 1
        print("[INFO] DriverGateway registered", flush=True)
        hakopy.start()
    finally:
        stop_server()
        thread.join(timeout=2.0)
        hakopy.conductor_stop()
    return result["code"]


if __name__ == "__main__":
    raise SystemExit(main())
