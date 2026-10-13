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

    def draw(self, transform, title=""):
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
        ax.set_xlim(loc.y - self.extent, loc.y + self.extent)
        ax.set_ylim(loc.x - self.extent, loc.x + self.extent)
        ax.legend(loc="upper right")
        if self.live:
            self.plt.pause(0.001)

    def save(self, path):
        self.fig.savefig(path, dpi=100)


def run(host, port, output, live, ticks, rudder, shaft):
    import carla
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    blueprint = world.get_blueprint_library().find("vessel.mass.kvlcc2")
    vessel = world.spawn_actor(blueprint, carla.Transform(carla.Location(0, 0, 0), carla.Rotation(yaw=0)))
    view = TopDownView(live=live)
    control = carla.VesselControl(rudder_angle=rudder, propeller_speed=shaft)
    try:
        for i in range(ticks):
            vessel.apply_control(control)
            world.tick()
            if live or i == ticks - 1 or i % 20 == 0:
                snapshot = world.get_snapshot()
                view.draw(vessel.get_transform(), f"t = {snapshot.timestamp.elapsed_seconds:.1f} s")
    finally:
        if output:
            view.save(output)
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
    args = parser.parse_args()
    if args.external:
        run(args.host, args.port, args.output, args.live, args.ticks, args.rudder, args.shaft)
    else:
        from harness import backend_process
        with backend_process(args.host, args.port):
            run(args.host, args.port, args.output, args.live, args.ticks, args.rudder, args.shaft)
    if args.live:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
