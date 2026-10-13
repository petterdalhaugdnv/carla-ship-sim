#!/usr/bin/env python3
"""Conformance check for a CARLA-protocol vessel backend, using the real carla client.

Starts the lightweight backend in-process (or targets --host/--port with
--external, e.g. an Unreal server) and checks spawn, synchronous ticks,
VesselControl round-trip and pose/velocity against a local VesselRunner.

Usage: python test_conformance.py [--external --host H --port P]
"""

import argparse
import math
import socket
import subprocess
import sys
import time
from pathlib import Path

import carla

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from vessel_runner import VesselRunner  # noqa: E402

DELTA = 0.05
TICKS = 200
RUDDER_RAD = 0.3
SHAFT_RAD_S = 11.15

failures = []


def wait_for_port(host, port, timeout=60.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            socket.create_connection((host, port), timeout=1.0).close()
            return
        except OSError:
            time.sleep(0.2)
    raise RuntimeError(f"backend did not start on {host}:{port}")


def check(name, condition, detail=""):
    print(("PASS " if condition else "FAIL ") + name + (f"  [{detail}]" if detail else ""))
    if not condition:
        failures.append(name)


def run(host, port):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = DELTA
    world.apply_settings(settings)
    check("sync mode applied", world.get_settings().synchronous_mode)

    blueprint = world.get_blueprint_library().find("vessel.mass.kvlcc2")
    start = carla.Transform(carla.Location(10.0, -5.0, 0.0), carla.Rotation(yaw=30.0))
    vessel = world.spawn_actor(blueprint, start)
    check("spawned carla.Vessel", isinstance(vessel, carla.Vessel), type(vessel).__name__)

    reference = VesselRunner(10.0, -5.0, 30.0)
    reference.set_control(RUDDER_RAD, SHAFT_RAD_S)
    control = carla.VesselControl(rudder_angle=RUDDER_RAD, propeller_speed=SHAFT_RAD_S)

    frames, stamps, worst_pos, worst_vel, worst_yaw = [], [], 0.0, 0.0, 0.0
    for _ in range(TICKS):
        vessel.apply_control(control)
        frame = world.tick()
        reference.advance(DELTA)
        snapshot = world.get_snapshot()
        frames.append(frame)
        stamps.append(snapshot.timestamp.elapsed_seconds)
        north, east, yaw, v_north, v_east, _, _ = reference.get()
        transform = vessel.get_transform()
        velocity = vessel.get_velocity()
        worst_pos = max(worst_pos, math.hypot(transform.location.x - north, transform.location.y - east))
        worst_vel = max(worst_vel, math.hypot(velocity.x - v_north, velocity.y - v_east))
        dyaw = (transform.rotation.yaw - yaw + 180.0) % 360.0 - 180.0
        worst_yaw = max(worst_yaw, abs(dyaw))

    check("frames strictly +1", all(b - a == 1 for a, b in zip(frames, frames[1:])))
    check("snapshot frame == tick frame", world.get_snapshot().frame == frames[-1])
    steps = [b - a for a, b in zip(stamps, stamps[1:])]
    check("timestamp delta == fixed_delta_seconds",
          all(abs(s - DELTA) < 1e-6 for s in steps), f"max err {max(abs(s - DELTA) for s in steps):.2e}")
    check("elapsed advances by (ticks-1) * delta",
          abs((stamps[-1] - stamps[0]) - (TICKS - 1) * DELTA) < 1e-6, f"{stamps[-1] - stamps[0]:.6f}")
    check("position matches runner", worst_pos < 1e-3, f"worst {worst_pos:.2e} m")
    check("velocity matches runner", worst_vel < 1e-4, f"worst {worst_vel:.2e} m/s")
    check("yaw matches runner", worst_yaw < 1e-3, f"worst {worst_yaw:.2e} deg")
    moved = math.hypot(transform.location.x - 10.0, transform.location.y + 5.0)
    check("vessel actually moved", moved > 1e-3, f"{moved:.4f} m (shaft lag is slow from rest)")

    echoed = vessel.get_control()
    check("get_control round-trips",
          abs(echoed.rudder_angle - RUDDER_RAD) < 1e-6 and abs(echoed.propeller_speed - SHAFT_RAD_S) < 1e-4,
          f"{echoed.rudder_angle}, {echoed.propeller_speed}")

    check("destroy_actor", vessel.destroy())

    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--external", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    args = parser.parse_args()
    backend = None
    if not args.external:
        # Separate process: the carla client holds the GIL during blocking RPCs.
        backend = subprocess.Popen(
            [sys.executable, str(HERE / "server.py"), "--host", args.host, "--port", str(args.port)])
        wait_for_port(args.host, args.port)
    try:
        run(args.host, args.port)
    finally:
        if backend is not None:
            backend.terminate()
            backend.wait(timeout=10)
    print("\nRESULT:", "FAILED " + ", ".join(failures) if failures else "all checks passed")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
