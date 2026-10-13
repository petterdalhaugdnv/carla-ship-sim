#!/usr/bin/env python3
"""Top-down chart view of a vessel, built only on the public carla client API.

Works against any CARLA-protocol server (lightweight backend or Unreal). The
chart uses nautical orientation: north up, east right. CARLA x is north, y is
east, and yaw is degrees clockwise from north.

    python viewer.py --output run.png               # headless: drive, save PNG
    python viewer.py --live                         # interactive window

Without --external it starts the lightweight backend itself.
"""

import argparse
import math
import sys
from pathlib import Path

import matplotlib

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

# KVLCC2 hull outline, metres, vessel frame (x forward, y starboard).
LENGTH_M, BEAM_M = 320.0, 58.05714285714286


def hull_outline(location, yaw_deg):
    """Hull polygon in chart coordinates (east, north) for a vessel pose."""
    yaw = math.radians(yaw_deg)
    half_l, half_b = LENGTH_M / 2, BEAM_M / 2
    points = [(half_l, 0.0), (half_l * 0.6, half_b), (-half_l, half_b),
              (-half_l, -half_b), (half_l * 0.6, -half_b)]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    out = []
    for forward, starboard in points:
        north = location.x + forward * cos_y - starboard * sin_y
        east = location.y + forward * sin_y + starboard * cos_y
        out.append((east, north))
    return out


class TopDownView:
    def __init__(self, live=False, extent_m=600.0):
        if not live:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        self.plt = plt
        self.live = live
        self.extent = extent_m
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.track = []

    def draw(self, transform, title="", radar=None):
        """`radar`: (sensor transform, horizontal fov deg, range m, detections in chart (east, north))."""
        loc = transform.location
        self.track.append((loc.y, loc.x))
        ax = self.ax
        ax.clear()
        ax.set_aspect("equal")
        ax.set_facecolor("#cfe6f5")
        ax.grid(True, color="white", linewidth=0.8)
        ax.set_xlabel("east [m]")
        ax.set_ylabel("north [m]")
        ax.set_title(title)
        ax.plot(*zip(*self.track), color="#1f4e79", linewidth=1.2, label="track")
        ax.add_patch(matplotlib.patches.Polygon(
            hull_outline(loc, transform.rotation.yaw), closed=True,
            facecolor="#444444", edgecolor="black", zorder=3, label="vessel"))
        if radar is not None:
            sensor_tf, h_fov, range_m, points = radar
            self._draw_radar(ax, sensor_tf, h_fov, range_m, points)
        ax.set_xlim(loc.y - self.extent, loc.y + self.extent)
        ax.set_ylim(loc.x - self.extent, loc.x + self.extent)
        ax.legend(loc="upper right")
        if self.live:
            self.plt.pause(0.001)

    @staticmethod
    def _draw_radar(ax, sensor_tf, h_fov, range_m, points):
        origin = (sensor_tf.location.y, sensor_tf.location.x)
        yaw = sensor_tf.rotation.yaw
        edge = []
        for offset in (-h_fov / 2, 0.0, h_fov / 2):
            a = math.radians(yaw + offset)
            edge.append((origin[0] + range_m * math.sin(a), origin[1] + range_m * math.cos(a)))
        ax.add_patch(matplotlib.patches.Polygon(
            [origin, *edge], closed=True, facecolor="#ffd27f", alpha=0.25, edgecolor="#e69500", zorder=1,
            label="radar field of view"))
        if points:
            ax.scatter(*zip(*points), s=40, color="red", marker="x", zorder=4, label="radar detections")

    def save(self, path):
        self.fig.savefig(path, dpi=100)


def detections_to_chart(data):
    """Radar detections to chart (east, north), via the sensor's own world transform."""
    points = []
    for d in data:
        c = math.cos(d.altitude)
        world = data.transform.transform(carla_location(
            d.depth * c * math.cos(d.azimuth), d.depth * c * math.sin(d.azimuth), d.depth * math.sin(d.altitude)))
        points.append((world.y, world.x))
    return points


def carla_location(x, y, z):
    import carla
    return carla.Location(x, y, z)


def run(host, port, output, live, ticks, rudder, shaft, radar_on=False):
    import queue
    import time
    import carla
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    library = world.get_blueprint_library()
    vessel = world.spawn_actor(library.find("vessel.mass.kvlcc2"),
                               carla.Transform(carla.Location(0, 0, 0), carla.Rotation(yaw=0)))
    sensor, data_queue, h_fov, range_m = None, queue.Queue(), 90.0, 1500.0
    if radar_on:
        bp = library.find("sensor.other.radar")
        bp.set_attribute("horizontal_fov", str(h_fov))
        bp.set_attribute("vertical_fov", "60")
        bp.set_attribute("range", str(range_m))
        sensor = world.spawn_actor(bp, carla.Transform(carla.Location(150.0, 0.0, 30.0)), attach_to=vessel)
        sensor.listen(data_queue.put)
    view = TopDownView(live=live, extent_m=1000.0 if radar_on else 600.0)
    control = carla.VesselControl(rudder_angle=rudder, propeller_speed=shaft)
    try:
        for i in range(ticks):
            vessel.apply_control(control)
            frame = world.tick()
            if live or i == ticks - 1 or i % 20 == 0:
                snapshot = world.get_snapshot()
                radar = None
                if sensor is not None:
                    data = None
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline:
                        try:
                            data = data_queue.get(timeout=0.2)
                        except queue.Empty:
                            continue
                        if data.frame >= frame:
                            break
                    if data is not None and data.frame == frame:
                        radar = (data.transform, h_fov, range_m, detections_to_chart(data))
                view.draw(vessel.get_transform(), f"t = {snapshot.timestamp.elapsed_seconds:.1f} s", radar)
    finally:
        if output:
            view.save(output)
        if sensor is not None:
            sensor.stop()
            sensor.destroy()
        vessel.destroy()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
    return view


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--external", action="store_true", help="use an already running server")
    parser.add_argument("--output", help="save the final frame as PNG")
    parser.add_argument("--live", action="store_true", help="interactive window")
    parser.add_argument("--ticks", type=int, default=2400)
    parser.add_argument("--rudder", type=float, default=0.3, help="rudder angle, rad")
    parser.add_argument("--shaft", type=float, default=11.15, help="propeller speed, rad/s")
    parser.add_argument("--radar", action="store_true", help="attach a forward radar and overlay detections")
    parser.add_argument("--target", action="append", default=[], metavar="N,E[,Z[,VN,VE]]",
                        help="point target for the spawned backend (repeatable, only without --external)")
    args = parser.parse_args()
    radar_on = args.radar or bool(args.target)
    if args.external:
        run(args.host, args.port, args.output, args.live, args.ticks, args.rudder, args.shaft, radar_on)
    else:
        from harness import backend_process
        extra = [f"--target={t}" for t in args.target]
        with backend_process(args.host, args.port, extra):
            run(args.host, args.port, args.output, args.live, args.ticks, args.rudder, args.shaft, radar_on)
    if args.live:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
