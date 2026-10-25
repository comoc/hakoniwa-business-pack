#!/usr/bin/env python3
"""Driver clients that check the running simulation.

1. Two simultaneous clients (bike + car) pass when both are welcomed, the car
   switches to the requested profile while at rest, and both move forward.
2. A car client (back in a sedan) overtakes the autopilot bicycle twice, first
   with the oncoming bus switched on, then off (switched at runtime), and checks
   that the bus meets the overtake only when it is on.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys

from websockets.asyncio.client import connect


async def drive(uri: str, role: str, profile: str | None, seconds: float, log: dict) -> None:
    async with connect(uri) as websocket:
        await websocket.send(json.dumps({"type": "join", "role": role}))
        welcome = None
        first = last = None
        loop = asyncio.get_running_loop()
        started = loop.time()
        if profile:
            await websocket.send(json.dumps({"type": "profile", "id": profile}))
        while loop.time() - started < seconds:
            elapsed = loop.time() - started
            throttle = 1.0 if elapsed > 1.0 else 0.0
            await websocket.send(json.dumps({"type": "input", "throttle": throttle, "brake": 0.0, "steer": 0.0}))
            try:
                text = await asyncio.wait_for(websocket.recv(), timeout=0.05)
            except asyncio.TimeoutError:
                continue
            message = json.loads(text)
            if message.get("type") == "welcome":
                welcome = message
            elif message.get("type") == "state" and message.get(role):
                if first is None and elapsed > 0.8:
                    first = message[role]
                last = message
        log[role] = {"welcome": welcome is not None,
                     "profiles": len(welcome["profiles"]) if welcome else 0,
                     "first": first, "last": last[role] if last else None,
                     "occupied": last.get("occupied") if last else None}


def evaluate(log: dict, car_profile: str) -> list[str]:
    errors = []
    for role in ("bike", "car"):
        entry = log.get(role) or {}
        if not entry.get("welcome"):
            errors.append(f"{role}: no welcome message")
            continue
        first, last = entry.get("first"), entry.get("last")
        if not first or not last:
            errors.append(f"{role}: no state received")
            continue
        moved = math.hypot(last["x"] - first["x"], last["y"] - first["y"])
        entry["moved_m"] = round(moved, 2)
        if moved < 2.0:
            errors.append(f"{role}: moved only {moved:.2f} m under full throttle")
        if not (entry.get("occupied") or {}).get(role):
            errors.append(f"{role}: gateway does not report the role as occupied")
    car_last = (log.get("car") or {}).get("last") or {}
    if car_last.get("profile") != car_profile:
        errors.append(f"car: profile is {car_last.get('profile')}, expected {car_profile}")
    return errors


async def run(uri: str, seconds: float, car_profile: str) -> dict:
    log: dict = {}
    await asyncio.gather(drive(uri, "bike", None, seconds, log),
                         drive(uri, "car", car_profile, seconds, log))
    return log


OVERTAKE_SPEED = 13.9  # m/s, 50 km/h
OVERTAKE_PROFILE = "sedan"  # the scripted passing line assumes a sedan's width
PASS_LANE_Y, CRUISE_LANE_Y = 0.9, 1.6


def overtake_input(car: dict, bike: dict) -> dict:
    """Car driver: cruise in the lane, shift towards the centre line around the bicycle."""
    beside = bike["x"] - 20.0 < car["x"] < bike["x"] + 12.0
    lane = PASS_LANE_Y if beside else CRUISE_LANE_Y
    steer = max(-1.0, min(1.0, 0.5 * (lane - car["y"]) - 1.5 * car["yaw"]))
    error = OVERTAKE_SPEED - car["speed"]
    return {"type": "input", "throttle": max(0.0, min(1.0, error)), "brake": max(0.0, min(1.0, -error)),
            "steer": steer}


async def overtake_once(websocket, oncoming: bool, timeout: float) -> dict:
    await websocket.send(json.dumps({"type": "scenario", "oncoming": oncoming}))
    await websocket.send(json.dumps({"type": "reset"}))
    loop = asyncio.get_running_loop()
    started = loop.time()
    state = None
    result = {"oncoming": oncoming, "phases": [], "contacts": [], "bike_start": None, "bus_start": None,
              "pass": None, "bus_at_pass": None, "bike_at_pass": None}
    while loop.time() - started < timeout:
        try:
            message = json.loads(await asyncio.wait_for(websocket.recv(), timeout=0.05))
        except asyncio.TimeoutError:
            message = None
        if message and message.get("type") == "state":
            state = message
        if state is None or loop.time() - started < 1.0 or not state.get("car") or not state.get("bike"):
            await websocket.send(json.dumps({"type": "input", "throttle": 0.0, "brake": 1.0, "steer": 0.0}))
            continue
        car, bike, bus = state["car"], state["bike"], state.get("oncoming")
        result["bike_start"] = result["bike_start"] or dict(bike)
        result["bus_start"] = result["bus_start"] or (dict(bus) if bus else None)
        phase = state["scenario"].get("phase")
        if not result["phases"] or result["phases"][-1] != phase:
            result["phases"].append(phase)
        for role, kind in (state.get("contact") or {}).items():
            if kind and (role, kind) not in result["contacts"]:
                result["contacts"].append((role, kind))
        last = (state.get("passing") or {}).get("last")
        if last:
            result.update(passing=last, bike_end=dict(bike), bus_end=dict(bus) if bus else None,
                          autopilot=state["scenario"].get("bike_autopilot"))
            return result
        if (state.get("passing") or {}).get("active"):
            result["bus_at_pass"] = dict(bus) if bus else None
            result["bike_at_pass"] = dict(bike)
        await websocket.send(json.dumps(overtake_input(car, bike)))
    return result


def evaluate_overtake(runs: list[dict]) -> list[str]:
    errors = []
    for run in runs:
        label = "oncoming on" if run["oncoming"] else "oncoming off"
        passing = run.get("passing")
        if not passing:
            errors.append(f"{label}: the car did not complete an overtake")
            continue
        if not run.get("autopilot"):
            errors.append(f"{label}: the unoccupied bicycle did not ride on autopilot")
        if run["bike_end"]["x"] - run["bike_start"]["x"] < 20.0:
            errors.append(f"{label}: the bicycle moved less than 20 m")
        if run["contacts"]:
            errors.append(f"{label}: contacts during the overtake: {run['contacts']}")
        bus_moved = abs(run["bus_end"]["x"] - run["bus_start"]["x"]) if run.get("bus_end") else 0.0
        if run["oncoming"]:
            if "approaching" not in run["phases"]:
                errors.append(f"{label}: the bus never started approaching")
            if not passing.get("oncoming_near"):
                errors.append(f"{label}: the bus was not near the bicycle during the overtake")
        else:
            if bus_moved > 1.0 or passing.get("oncoming_near"):
                errors.append(f"{label}: the bus moved {bus_moved:.1f} m although it is switched off")
    return errors


async def run_overtake(uri: str, timeout: float) -> list[dict]:
    async with connect(uri) as websocket:
        await websocket.send(json.dumps({"type": "join", "role": "car"}))
        # Applied by the plant once the car is at rest after the reset.
        await websocket.send(json.dumps({"type": "profile", "id": OVERTAKE_PROFILE}))
        return [await overtake_once(websocket, True, timeout), await overtake_once(websocket, False, timeout)]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--uri", default="ws://127.0.0.1:8771")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--overtake-timeout", type=float, default=45.0)
    parser.add_argument("--car-profile", default="truck")
    args = parser.parse_args(argv)
    clients = asyncio.run(run(args.uri, args.seconds, args.car_profile))
    overtakes = asyncio.run(run_overtake(args.uri, args.overtake_timeout))
    errors = evaluate(clients, args.car_profile) + evaluate_overtake(overtakes)
    log = {"clients": clients, "overtake": overtakes}
    print(json.dumps({"status": "PASS" if not errors else "FAIL", "errors": errors, "log": log},
                     ensure_ascii=False, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
