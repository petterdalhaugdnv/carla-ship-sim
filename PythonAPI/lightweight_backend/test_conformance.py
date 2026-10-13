#!/usr/bin/env python3
"""Conformance check for a CARLA-protocol vessel backend, using the real carla client.

Starts the lightweight backend as a subprocess (or targets --host/--port with
--external, e.g. an Unreal server) and checks spawn, synchronous ticks,
VesselControl round-trip and pose/velocity against a local VesselRunner.

Usage: python test_conformance.py [--external --host H --port P]
"""

import argparse
import math
import sys
from pathlib import Path

import carla

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from harness import backend_process, check, finish  # noqa: E402
from vessel_runner import VesselRunner  # noqa: E402

DELTA = 0.05
TICKS = 200
RUDDER_RAD = 0.3
SHAFT_RAD_S = 11.15


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
    if args.external:
        run(args.host, args.port)
    else:
        with backend_process(args.host, args.port):
            run(args.host, args.port)
    finish()


if __name__ == "__main__":
    main()
