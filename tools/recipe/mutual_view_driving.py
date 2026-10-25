#!/usr/bin/env python3
"""Operate the bicycle / car mutual-view driving simulator Recipe.

  python tools/recipe/mutual_view_driving.py configure
  python tools/recipe/mutual_view_driving.py doctor
  python tools/recipe/mutual_view_driving.py start      # then open the printed URLs
  python tools/recipe/mutual_view_driving.py start --oncoming off
  python tools/recipe/mutual_view_driving.py status
  python tools/recipe/mutual_view_driving.py smoke      # start, drive both roles, overtake, stop
  python tools/recipe/mutual_view_driving.py stop
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path


RECIPE_ID = "bicycle-car-mutual-view-driving"
BASE_SCRIPT = Path(__file__).with_name("mujoco_turtlebot3_mbody.py")
SPEC = importlib.util.spec_from_file_location("managed_base_for_mutual_view_driving", BASE_SCRIPT)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot load managed Recipe helper: {BASE_SCRIPT}")
base = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = base
SPEC.loader.exec_module(base)
base.RECIPE_ID = RECIPE_ID

APP_SOURCE = Path(__file__).with_name("assets") / "mutual_view_driving"
REQUIREMENTS = base.business_root() / "recipes" / "requirements" / f"{RECIPE_ID}.txt"
APP_FILES = ("vehicles.py", "vehicle_plant.py", "driver_gateway.py", "smoke_client.py", "world-overtake.json")
WORLD_FILE = "world-overtake.json"


def paths() -> dict[str, Path]:
    resolved = base.paths()
    resolved["app"] = resolved["assets"] / "app"
    return resolved


def inspect(args: argparse.Namespace) -> dict[str, object]:
    resolved = paths()
    prefix = resolved["prefix"]
    errors: list[str] = []
    foundation = base.load_foundation_module()
    inspection = foundation.inspect_foundation(base.recipe_file(), prefix)
    if inspection["status"] != "SATISFIED":
        errors.append(f"Foundation is {inspection['status']}; run tools/foundation.py plan/build")
    python = base.foundation_python(prefix)
    base.require_file(python, "Foundation Python", errors)
    for name in APP_FILES:
        base.require_file(APP_SOURCE / name, f"app source {name}", errors)
    base.require_file(APP_SOURCE / "web" / "index.html", "browser app", errors)
    if python.is_file():
        probe = subprocess.run([str(python), "-c", "import websockets, hakopy"],
                               capture_output=True, text=True)
        if probe.returncode != 0:
            errors.append("Foundation Python cannot import websockets/hakopy; run configure")
    return {
        "status": "READY" if not errors else "BLOCKED",
        "errors": errors,
        "workspace": {key: str(value) for key, value in resolved.items()},
        "urls": urls(args),
    }


def urls(args: argparse.Namespace) -> dict[str, str]:
    page = f"http://{args.host}:{args.http_port}/index.html"
    ws = f"ws://{args.host}:{args.ws_port}"
    return {
        "bike": f"{page}?role=bike&ws={ws}",
        "car": f"{page}?role=car&ws={ws}",
        "websocket": ws,
    }


def stage_app() -> Path:
    app = paths()["app"]
    if app.exists():
        shutil.rmtree(app)
    shutil.copytree(APP_SOURCE, app, ignore=shutil.ignore_patterns("__pycache__"))
    return app


def write_launcher(args: argparse.Namespace, app: Path) -> Path:
    resolved = paths()
    prefix = resolved["prefix"]
    python = base.foundation_python(prefix)
    pdu_def = str(app / "config" / "mutual-view-pdudef-compact.json")
    world = str(app / WORLD_FILE)
    plant = str(app / "vehicle_plant.py")
    launcher = {
        "version": "0.1",
        "defaults": {
            "cwd": str(app / "config"),
            "stdout": str(resolved["logs"] / "${asset}.out"),
            "stderr": str(resolved["logs"] / "${asset}.err"),
            "env": {
                "set": {
                    "HAKO_CONFIG_PATH": str(resolved["foundation"] / "config/cpp_core_config.json"),
                    "HAKONIWA_CORE_ROOT": str(prefix),
                    "PYTHONUNBUFFERED": "1",
                },
                "prepend": {
                    "PATH": [str(python.parent), str(prefix / "bin"), str(prefix / "lib")],
                },
            },
            "start_grace_sec": 2,
            "delay_sec": 1,
        },
        "assets": [
            {
                "name": "driver_gateway",
                "activation_timing": "before_start",
                "command": str(python),
                "args": [str(app / "driver_gateway.py"), "--pdu-def", pdu_def, "--world", world,
                         "--oncoming", args.oncoming, "--host", args.host, "--port", str(args.ws_port)],
                "start_grace_sec": 3,
            },
            {
                "name": "bike_plant",
                "activation_timing": "before_start",
                "command": str(python),
                "args": [plant, "--role", "bike", "--pdu-def", pdu_def, "--world", world],
            },
            {
                "name": "car_plant",
                "activation_timing": "before_start",
                "command": str(python),
                "args": [plant, "--role", "car", "--pdu-def", pdu_def, "--world", world,
                         "--profile", args.car_profile],
            },
            {
                "name": "oncoming_plant",
                "activation_timing": "before_start",
                "command": str(python),
                "args": [plant, "--role", "oncoming", "--pdu-def", pdu_def, "--world", world],
            },
            {
                "name": "web_server",
                "activation_timing": "before_start",
                "command": str(python),
                "args": [str(Path(__file__).with_name("viewer_http_server.py")),
                         "--port", str(args.http_port), "--bind", args.host,
                         "--directory", str(app / "web")],
                "start_grace_sec": 1,
            },
        ],
    }
    return base.write_json(resolved["config"] / "launcher.json", launcher)


def configure(args: argparse.Namespace) -> None:
    for name in ("config", "logs", "runtime", "assets"):
        paths()[name].mkdir(parents=True, exist_ok=True)
    python = base.foundation_python(paths()["prefix"])
    base.run([str(python), "-m", "pip", "install", "--disable-pip-version-check",
              "--requirement", str(REQUIREMENTS)], check=True)
    app = stage_app()
    launcher = write_launcher(args, app)
    print(f"Recipe workspace: {paths()['recipe']}")
    print(f"Launcher: {launcher}")


def start(args: argparse.Namespace) -> None:
    state = inspect(args)
    if state["status"] != "READY":
        raise base.RecipeError("\n".join(state["errors"]))
    for name in ("config", "logs", "runtime"):
        paths()[name].mkdir(parents=True, exist_ok=True)
    launcher = write_launcher(args, stage_app())
    python = base.foundation_python(paths()["prefix"])
    base.run([str(python), "-m", "hakoniwa_pdu.apps.launcher.hako_launcher", str(launcher),
              "--mode", "immediate", "--background", str(base.session_file())], check=True)
    base.run(base.control_command("status"), check=True)
    for role, url in urls(args).items():
        print(f"{role:9}: {url}")


def smoke(args: argparse.Namespace) -> int:
    start(args)
    python = base.foundation_python(paths()["prefix"])
    time.sleep(args.warmup_sec)
    result_file = paths()["validation"] / "smoke-result.json"
    result_file.parent.mkdir(parents=True, exist_ok=True)
    try:
        completed = subprocess.run(
            [str(python), str(paths()["app"] / "smoke_client.py"), "--uri", urls(args)["websocket"],
             "--car-profile", "truck"],
            capture_output=True, text=True)
        result_file.write_text(completed.stdout, encoding="utf-8")
        print(completed.stdout)
        if completed.stderr:
            print(completed.stderr, file=sys.stderr)
        return completed.returncode
    finally:
        base.control("terminate")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Operate the bicycle / car mutual-view driving Recipe")
    parser.add_argument("command", choices=("configure", "doctor", "start", "status", "stop", "smoke"))
    parser.add_argument("--host", default="127.0.0.1", help="bind address (LAN use is a human decision)")
    parser.add_argument("--http-port", type=int, default=8770)
    parser.add_argument("--ws-port", type=int, default=8771)
    parser.add_argument("--car-profile", default="sedan", choices=("sedan", "suv", "minivan", "truck"))
    parser.add_argument("--oncoming", default="on", choices=("on", "off"),
                        help="initial oncoming-bus setting (switchable in the browser)")
    parser.add_argument("--warmup-sec", type=float, default=3.0)
    args = parser.parse_args(argv)
    try:
        if args.command == "doctor":
            state = inspect(args)
            print(json.dumps(state, ensure_ascii=False, indent=2))
            return 0 if state["status"] == "READY" else 1
        if args.command == "configure":
            configure(args)
        elif args.command == "start":
            start(args)
        elif args.command == "smoke":
            return smoke(args)
        elif args.command == "status":
            base.control("status")
        else:
            base.control("terminate")
        return 0
    except (base.RecipeError, subprocess.CalledProcessError, OSError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
