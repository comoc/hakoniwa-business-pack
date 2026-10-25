"""Vehicle profiles and kinematic model for the mutual-view driving simulator.

World frame: x east, y north, z up (metres, radians). A vehicle pose is the
centre of its footprint; yaw 0 faces +x. Left-hand traffic (Japan): the driver
of a car sits on the right (negative local y).

This module is pure Python so the plant, the gateway, the smoke client and the
unit tests share one source of truth. The browser receives the same profiles
from the gateway in its welcome message.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

DT_SEC = 0.02


@dataclass(frozen=True)
class Profile:
    id: str
    label: str
    kind: str  # "bicycle" | "car" | "truck" | "bus"
    length: float
    width: float
    height: float
    wheelbase: float
    eye: tuple[float, float, float]  # driver eye in the vehicle frame
    max_speed: float  # m/s
    max_reverse: float  # m/s
    accel: float  # m/s^2 at full throttle
    brake: float  # m/s^2 at full brake
    max_steer: float  # rad
    steer_rate: float  # rad/s
    color: str


PROFILES: dict[str, Profile] = {
    p.id: p
    for p in (
        Profile("bicycle", "自転車", "bicycle", 1.75, 0.6, 1.1, 1.05, (-0.1, 0.0, 1.5),
                8.3, 1.0, 1.2, 3.5, 0.5, 1.5, "#2e8b57"),
        Profile("sedan", "セダン", "car", 4.7, 1.8, 1.45, 2.8, (-0.15, -0.37, 1.15),
                16.7, 3.0, 3.0, 7.0, 0.6, 0.9, "#c0392b"),
        Profile("suv", "SUV", "car", 4.6, 1.85, 1.75, 2.7, (-0.05, -0.38, 1.45),
                16.7, 3.0, 3.0, 7.0, 0.6, 0.9, "#34495e"),
        Profile("minivan", "ミニバン", "car", 4.7, 1.7, 1.85, 2.85, (0.25, -0.36, 1.5),
                16.7, 3.0, 2.7, 6.5, 0.6, 0.9, "#ecf0f1"),
        Profile("truck", "大型トラック", "truck", 11.9, 2.49, 3.5, 6.0, (5.05, -0.6, 2.45),
                16.7, 2.0, 1.0, 5.0, 0.55, 0.6, "#f1c40f"),
        # Scenario-driven oncoming vehicle; drivers cannot select it.
        Profile("bus", "大型バス", "bus", 10.9, 2.49, 3.25, 5.3, (4.6, -0.6, 2.1),
                16.7, 0.0, 1.5, 5.0, 0.55, 0.6, "#2e86c1"),
    )
}
PROFILE_IDS = list(PROFILES)
CAR_PROFILE_IDS = ["sedan", "suv", "minivan", "truck"]


def profile_index(profile_id: str) -> int:
    return PROFILE_IDS.index(profile_id)


def profile_by_index(index: float) -> Profile:
    i = int(round(index))
    if 0 <= i < len(PROFILE_IDS):
        return PROFILES[PROFILE_IDS[i]]
    raise ValueError(f"unknown profile index: {index}")


def profiles_payload() -> list[dict]:
    return [asdict(p) for p in PROFILES.values()]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class VehicleState:
    x: float
    y: float
    yaw: float
    speed: float = 0.0
    steer: float = 0.0


def step(profile: Profile, state: VehicleState, throttle: float, brake: float,
         steer_cmd: float, dt: float = DT_SEC) -> VehicleState:
    """Advance one step of a kinematic bicycle model.

    throttle in [-1, 1] (negative drives backwards), brake in [0, 1],
    steer_cmd in [-1, 1] (positive turns left).
    """
    throttle = _clamp(throttle, -1.0, 1.0)
    brake = _clamp(brake, 0.0, 1.0)
    target_steer = _clamp(steer_cmd, -1.0, 1.0) * profile.max_steer
    max_delta = profile.steer_rate * dt
    steer = state.steer + _clamp(target_steer - state.steer, -max_delta, max_delta)

    speed = state.speed + throttle * profile.accel * dt
    # Brake and rolling resistance both act against the motion and never reverse it.
    decel = (brake * profile.brake + 0.15) * dt
    if speed > 0.0:
        speed = max(0.0, speed - decel)
    elif speed < 0.0:
        speed = min(0.0, speed + decel)
    speed = _clamp(speed, -profile.max_reverse, profile.max_speed)

    yaw = state.yaw + speed / profile.wheelbase * math.tan(steer) * dt
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    x = state.x + speed * math.cos(yaw) * dt
    y = state.y + speed * math.sin(yaw) * dt
    return VehicleState(x, y, yaw, speed, steer)


def footprint(profile: Profile, x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    """Corners of the vehicle footprint in the world frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = profile.length / 2.0, profile.width / 2.0
    return [(x + c * lx - s * ly, y + s * lx + c * ly)
            for lx, ly in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]


def polygons_overlap(a: list[tuple[float, float]], b: list[tuple[float, float]]) -> bool:
    """Separating-axis test for two convex polygons."""
    for poly in (a, b):
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            nx, ny = y1 - y2, x2 - x1
            pa = [nx * px + ny * py for px, py in a]
            pb = [nx * px + ny * py for px, py in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def side_by_side_gap(a: Profile, a_pose: dict, b: Profile, b_pose: dict) -> float | None:
    """Lateral clearance between two road users while they are side by side, else None.

    Measured in b's frame (b is the one being passed), so it is the gap a driver keeps
    to the cyclist while overtaking.
    """
    dx, dy = a_pose["x"] - b_pose["x"], a_pose["y"] - b_pose["y"]
    c, s = math.cos(b_pose["yaw"]), math.sin(b_pose["yaw"])
    along, across = c * dx + s * dy, -s * dx + c * dy
    if abs(along) >= (a.length + b.length) / 2.0:
        return None
    return abs(across) - (a.width + b.width) / 2.0


def rect_polygon(rect: dict) -> list[tuple[float, float]]:
    """Axis-aligned world rectangle {x0, y0, x1, y1} as a polygon."""
    return [(rect["x0"], rect["y0"]), (rect["x1"], rect["y0"]),
            (rect["x1"], rect["y1"]), (rect["x0"], rect["y1"])]
