#!/usr/bin/env python3
"""Hakoniwa plant asset that moves one road user (Bike, Car, or the Oncoming scenario vehicle).

Reads   <Robot>/cmd      Twist: linear.x throttle, linear.y brake, angular.z steer,
                                angular.x > 0.5 requests a respawn
        <Robot>/profile  Vector3: x = profile index (Car only; the Bike is fixed)
Writes  <Robot>/pose     Twist: linear.x/y position, angular.z yaw
        <Robot>/vel      Twist: linear.x speed (m/s), angular.z steering angle (rad),
                                angular.x = profile index in use
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import hakopy
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Twist import pdu_to_py_Twist, py_to_pdu_Twist
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_conv_Vector3 import pdu_to_py_Vector3
from hakoniwa_pdu.pdu_msgs.geometry_msgs.pdu_pytype_Twist import Twist

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vehicles  # noqa: E402

CH_CMD, CH_POSE, CH_VEL, CH_PROFILE = 0, 1, 2, 3
TWIST_SIZE, VECTOR3_SIZE = 72, 48
ROBOTS = {"bike": "Bike", "car": "Car", "oncoming": "Oncoming"}
DEFAULT_PROFILE = {"bike": "bicycle", "car": "sedan", "oncoming": "bus"}


def read_pdu(robot: str, channel: int, size: int, convert):
    """Decoded PDU, or None while nobody has written the channel yet (no metadata)."""
    raw = hakopy.pdu_read(robot, channel, size)
    if not raw:
        return None
    try:
        return convert(raw)
    except ValueError:
        return None


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--role", choices=tuple(ROBOTS), required=True)
    parser.add_argument("--pdu-def", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--profile", default=None, help="initial profile id (default: bicycle / sedan / bus)")
    parser.add_argument("--delta-usec", type=int, default=int(vehicles.DT_SEC * 1_000_000))
    return parser.parse_args(argv)


class Plant:
    def __init__(self, args: argparse.Namespace):
        self.robot = ROBOTS[args.role]
        world = json.loads(Path(args.world).read_text(encoding="utf-8"))
        self.bounds = world["bounds"]
        self.spawn = world["spawn"][args.role]
        self.profile = vehicles.PROFILES[args.profile or DEFAULT_PROFILE[args.role]]
        self.dt = args.delta_usec / 1_000_000
        self.delta_usec = args.delta_usec
        self.state = self._spawned()

    def _spawned(self) -> vehicles.VehicleState:
        # A scenario vehicle may spawn already cruising (spawn.speed), e.g. the oncoming bus.
        return vehicles.VehicleState(self.spawn["x"], self.spawn["y"], self.spawn["yaw"],
                                     self.spawn.get("speed", 0.0))

    def _read_cmd(self) -> Twist | None:
        return read_pdu(self.robot, CH_CMD, TWIST_SIZE, pdu_to_py_Twist)

    def _read_profile(self) -> None:
        if self.robot != "Car":
            return
        requested = read_pdu(self.robot, CH_PROFILE, VECTOR3_SIZE, pdu_to_py_Vector3)
        if requested is None:
            return
        try:
            wanted = vehicles.profile_by_index(requested.x)
        except ValueError:
            return
        # Swapping the body while moving would teleport its footprint; only switch at rest.
        if wanted.kind != "bicycle" and wanted.id != self.profile.id and abs(self.state.speed) < 0.1:
            print(f"[INFO] {self.robot}: profile {self.profile.id} -> {wanted.id}", flush=True)
            self.profile = wanted

    def tick(self) -> None:
        cmd = self._read_cmd()
        self._read_profile()
        if cmd is not None and cmd.angular.x > 0.5:
            self.state = self._spawned()
        elif cmd is not None:
            self.state = vehicles.step(self.profile, self.state, cmd.linear.x, cmd.linear.y,
                                       cmd.angular.z, self.dt)
        else:
            self.state = vehicles.step(self.profile, self.state, 0.0, 1.0, 0.0, self.dt)
        b = self.bounds
        if not (b["x0"] < self.state.x < b["x1"] and b["y0"] < self.state.y < b["y1"]):
            self.state = self._spawned()

        pose = Twist()
        pose.linear.x, pose.linear.y, pose.angular.z = self.state.x, self.state.y, self.state.yaw
        vel = Twist()
        vel.linear.x, vel.angular.z = self.state.speed, self.state.steer
        vel.angular.x = float(vehicles.profile_index(self.profile.id))
        hakopy.pdu_write(self.robot, CH_POSE, py_to_pdu_Twist(pose), TWIST_SIZE)
        hakopy.pdu_write(self.robot, CH_VEL, py_to_pdu_Twist(vel), TWIST_SIZE)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    plant = Plant(args)
    result = {"code": 0}

    def on_manual_timing_control(_context):
        try:
            while True:
                plant.tick()
                if not hakopy.usleep(plant.delta_usec):
                    break
        except Exception as exc:  # report and let the launcher see a non-zero exit
            print(f"[ERROR] {plant.robot} plant failed: {exc}", file=sys.stderr, flush=True)
            result["code"] = 1
        return 0

    callbacks = {
        "on_initialize": lambda _context: 0,
        "on_simulation_step": None,
        "on_manual_timing_control": on_manual_timing_control,
        "on_reset": lambda _context: 0,
    }
    asset_name = f"{plant.robot}Plant"
    if not hakopy.asset_register(asset_name, args.pdu_def, callbacks, plant.delta_usec,
                                 hakopy.HAKO_ASSET_MODEL_PLANT):
        print(f"[ERROR] hakopy.asset_register({asset_name}) failed", file=sys.stderr)
        return 1
    print(f"[INFO] {asset_name} registered (profile={plant.profile.id})", flush=True)
    hakopy.start()
    return result["code"]


if __name__ == "__main__":
    raise SystemExit(main())
