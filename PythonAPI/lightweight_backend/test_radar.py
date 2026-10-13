#!/usr/bin/env python3
"""Radar check for a CARLA-protocol vessel backend, using the real carla client.

Default mode starts the lightweight backend with known point targets and checks
that the attached radar delivers detections whose frame, timestamp, sensor
transform and geometry are consistent with the snapshot and the target positions.

With --external (for example an Unreal server) only backend-independent checks
run, because target counts and Doppler depend on the world's geometry and on
sampled rays: stream delivery, frame and timestamp equal to the snapshot, sensor
transform equal to the vessel pose composed with the mount, detections finite and
inside the configured range and field of view, and clean destruction. Use
--expect N,E[,Z] (repeatable) to require that some detection reprojects, through
the sensor transform, within --expect-tol metres of a known point in the world.

Usage: python test_radar.py [--external --host H --port P] [--expect N,E[,Z] ...]
"""

import argparse
import math
import queue
import random
import sys
import time
from pathlib import Path

import carla

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from geometry import axes_to_rotator, local_to_world, rotator_to_axes  # noqa: E402
from harness import add_server_args, backend_process, check, finish  # noqa: E402

DELTA = 0.05
TICKS = 40
FWD_FOV, FWD_RANGE = 60, 1000
SIDE_FOV, SIDE_RANGE = 60, 400
FOV_SLACK_RAD = math.radians(1.0)
TARGET_ARGS = [
    "600,0,0",          # T1 ahead, static
    "600,200,0,-5,0",   # T2 ahead-starboard, closing at 5 m/s
    "-500,0,0",         # T3 astern: outside the forward radar's field of view
    "5000,0,0",         # T4 beyond range
    "0,300,0",          # T5 abeam to starboard: seen by the side-looking radar only
]
STATIC = {"T1": (600.0, 0.0, 0.0), "T5": (0.0, 300.0, 0.0)}
T2_START, T2_SPEED = (600.0, 200.0, 0.0), -5.0


def local_point(data, detection):
    """Sensor-frame cartesian point of a detection (CARLA convention)."""
    c = math.cos(detection.altitude)
    return carla.Location(detection.depth * c * math.cos(detection.azimuth),
                          detection.depth * c * math.sin(detection.azimuth),
                          detection.depth * math.sin(detection.altitude))


def distance(a, b):
    return math.dist(a, b)


def check_geometry_against_carla():
    rng = random.Random(1)
    worst = 0.0
    for _ in range(200):
        pitch, yaw, roll = (rng.uniform(-90, 90), rng.uniform(-180, 180), rng.uniform(-180, 180))
        loc = [rng.uniform(-50, 50) for _ in range(3)]
        point = [rng.uniform(-50, 50) for _ in range(3)]
        carla_world = carla.Transform(carla.Location(*loc), carla.Rotation(pitch, yaw, roll)).transform(
            carla.Location(*point))
        ours = local_to_world(rotator_to_axes(pitch, yaw, roll), point)
        ours = tuple(o + l for o, l in zip(ours, loc))
        worst = max(worst, distance(ours, (carla_world.x, carla_world.y, carla_world.z)))
        back = axes_to_rotator(rotator_to_axes(pitch, yaw, roll))
        again = local_to_world(rotator_to_axes(*back), point)
        worst = max(worst, distance(again, local_to_world(rotator_to_axes(pitch, yaw, roll), point)))
    check("rotation helpers match carla.Transform", worst < 1e-4, f"worst {worst:.2e} m")


def wait_data(q, frame):
    """Sensor streams are independent of the snapshot stream, so wait for the matching frame."""
    while True:
        data = q.get(timeout=5.0)
        if data.frame == frame:
            return data
        if data.frame > frame:
            raise RuntimeError(f"missed radar frame {frame}, got {data.frame}")


def run(host, port, lightweight, expects, expect_tol):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = DELTA
    world.apply_settings(settings)

    library = world.get_blueprint_library()
    vessel = world.spawn_actor(library.find("vessel.mass.kvlcc2"),
                               carla.Transform(carla.Location(0, 0, 0), carla.Rotation(yaw=0)))

    def make_radar(rel, h_fov, range_m):
        bp = library.find("sensor.other.radar")
        bp.set_attribute("horizontal_fov", str(h_fov))
        bp.set_attribute("vertical_fov", "60")
        bp.set_attribute("range", str(range_m))
        sensor = world.spawn_actor(bp, rel, attach_to=vessel)
        q = queue.Queue()
        sensor.listen(q.put)
        return sensor, q

    rel_fwd = carla.Transform(carla.Location(100.0, 0.0, 30.0), carla.Rotation(yaw=0))
    rel_side = carla.Transform(carla.Location(0.0, 0.0, 10.0), carla.Rotation(yaw=90))
    fwd, q_fwd = make_radar(rel_fwd, FWD_FOV, FWD_RANGE)
    side, q_side = make_radar(rel_side, SIDE_FOV, SIDE_RANGE)
    check("radars are carla.Sensor", isinstance(fwd, carla.Sensor) and isinstance(side, carla.Sensor))
    check("sensor parent is the vessel", fwd.parent is not None and fwd.parent.id == vessel.id)

    # Stream connections are established asynchronously (as in CARLA), so the
    # first ticks after listen() can be missed. Tick until both streams deliver.
    for _ in range(100):
        world.tick()
        if not q_fwd.empty() and not q_side.empty():
            break
        time.sleep(0.05)
    check("radar streams connected", not q_fwd.empty() and not q_side.empty())

    control = carla.VesselControl(rudder_angle=0.1, propeller_speed=11.15)
    worst_tf, worst_geo, counts_fwd, counts_side = 0.0, 0.0, set(), set()
    frames_ok = stamps_ok = fov_ok = True
    expect_hit = [False] * len(expects)
    for _ in range(TICKS):
        vessel.apply_control(control)
        frame = world.tick()
        snap = world.get_snapshot()
        d_fwd, d_side = wait_data(q_fwd, frame), wait_data(q_side, frame)
        for d in (d_fwd, d_side):
            frames_ok &= d.frame == snap.frame
            stamps_ok &= abs(d.timestamp - snap.timestamp.elapsed_seconds) < 1e-9

        vessel_tf = snap.find(vessel.id).get_transform()
        for d, rel in ((d_fwd, rel_fwd), (d_side, rel_side)):
            expected = carla.Transform(vessel_tf.location, vessel_tf.rotation).transform(rel.location)
            worst_tf = max(worst_tf, distance((d.transform.location.x, d.transform.location.y, d.transform.location.z),
                                              (expected.x, expected.y, expected.z)))
            yaw_err = abs((d.transform.rotation.yaw - vessel_tf.rotation.yaw - rel.rotation.yaw + 180) % 360 - 180)
            worst_tf = max(worst_tf, yaw_err * 1e-3)

        for d, fov, range_m in ((d_fwd, FWD_FOV, FWD_RANGE), (d_side, SIDE_FOV, SIDE_RANGE)):
            half = math.radians(fov / 2) + FOV_SLACK_RAD
            for det in d:
                values = (det.velocity, det.azimuth, det.altitude, det.depth)
                fov_ok &= (all(math.isfinite(v) for v in values) and 0.0 <= det.depth <= range_m * 1.001
                           and abs(det.azimuth) <= half and abs(det.altitude) <= half)
                point = d.transform.transform(local_point(d, det))
                for i, known in enumerate(expects):
                    if distance((point.x, point.y, point.z), known) <= expect_tol:
                        expect_hit[i] = True

        if lightweight:
            t2 = (T2_START[0] + T2_SPEED * snap.timestamp.elapsed_seconds, T2_START[1], T2_START[2])
            counts_fwd.add(len(d_fwd))
            counts_side.add(len(d_side))
            for d, known in ((d_fwd, [STATIC["T1"], t2]), (d_side, [STATIC["T5"]])):
                origin = (d.transform.location.x, d.transform.location.y, d.transform.location.z)
                for det in d:
                    p = d.transform.transform(local_point(d, det))
                    nearest = min(known, key=lambda k: distance((p.x, p.y, p.z), k))
                    worst_geo = max(worst_geo, distance((p.x, p.y, p.z), nearest),
                                    abs(det.depth - distance(origin, nearest)))

    check("radar frame == snapshot frame", frames_ok)
    check("radar timestamp == snapshot timestamp", stamps_ok)
    check("sensor_transform == vessel pose composed with mount", worst_tf < 1e-3, f"worst {worst_tf:.2e}")
    check("detections finite, within range and field of view", fov_ok)
    for known, hit in zip(expects, expect_hit):
        check(f"a detection reprojects within {expect_tol:g} m of expected point {known}", hit)

    if lightweight:
        check("forward radar sees exactly T1 and T2", counts_fwd == {2}, f"counts {sorted(counts_fwd)}")
        check("side radar sees exactly T5", counts_side == {1}, f"counts {sorted(counts_side)}")
        check("detections reproject onto targets via sensor transform", worst_geo < 0.05, f"worst {worst_geo:.2e} m")

        by_az = sorted(d_fwd, key=lambda det: det.azimuth)
        static_v, closing_v = by_az[0].velocity, by_az[1].velocity
        # T2 moves along -north, so its radial speed is -5 m/s times the north component of the line of sight.
        p = d_fwd.transform.transform(local_point(d_fwd, by_az[1]))
        expected_v = T2_SPEED * (p.x - d_fwd.transform.location.x) / by_az[1].depth
        check("static target radial speed ~ -own speed", abs(static_v) < 0.2, f"{static_v:.4f} m/s")
        check("moving target radial speed matches line-of-sight projection",
              abs(closing_v - expected_v) < 0.1, f"{closing_v:.4f} vs {expected_v:.4f} m/s")

    for sensor in (fwd, side):
        sensor.stop()
    check("destroy sensors", fwd.destroy() and side.destroy())
    check("destroy vessel", vessel.destroy())
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)


def parse_point(text):
    values = [float(v) for v in text.split(",")]
    if not 2 <= len(values) <= 3:
        raise argparse.ArgumentTypeError("expected N,E[,Z]")
    return tuple(values + [0.0] * (3 - len(values)))


def main():
    parser = argparse.ArgumentParser()
    add_server_args(parser)
    parser.add_argument("--expect", action="append", default=[], type=parse_point, metavar="N,E[,Z]",
                        help="world point (CARLA x,y,z in m) that some detection must reproject near")
    parser.add_argument("--expect-tol", type=float, default=5.0, help="tolerance for --expect, m")
    args = parser.parse_args()
    check_geometry_against_carla()
    if args.external:
        run(args.host, args.port, False, args.expect, args.expect_tol)
    else:
        extra = [f"--target={t}" for t in TARGET_ARGS]
        with backend_process(args.host, args.port, extra):
            run(args.host, args.port, True, args.expect, args.expect_tol)
    finish()


if __name__ == "__main__":
    main()
