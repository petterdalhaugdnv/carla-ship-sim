#!/usr/bin/env python3
"""Lightweight (engine-free) CARLA-protocol backend for vessels.

Speaks the subset of the CARLA msgpack-RPC + streaming protocol documented in
MarineDocs/lightweight-backend-protocol.md, so the unmodified carla Python
client can spawn a vessel, tick and apply VesselControl. Dynamics come from the
shared C++ VesselRunner (via vessel_runner.py). No sensors, map or weather yet.

Usage: python server.py [--host 127.0.0.1] [--port 2000]
"""

import argparse
import itertools
import random
import socket
import socketserver
import struct
import threading
import time

import msgpack

import radar
from geometry import axes_to_rotator, compose, local_to_world, rotator_to_axes
from vessel_runner import VesselRunner

SERVER_VERSION = "0.10.0"
VESSEL_BLUEPRINT = "vessel.mass.kvlcc2"
WORLD_OBSERVER_SENSOR_TYPE = 16  # index of FWorldObserver in carla::sensor::SensorRegistry
RADAR_BLUEPRINT = "sensor.other.radar"
RADAR_SENSOR_TYPE = 9  # index of ARadar in carla::sensor::SensorRegistry
EPISODE_STREAM_ID = 1
ACTOR_STATE_ACTIVE = 1
DEFAULT_SYNC_DELTA = 0.05
ASYNC_PERIOD = 0.05
# KVLCC2 length/beam (VesselDynamicsModel.h); height is a placeholder.
BBOX_EXTENT = (160.0, 29.03, 10.0)


def ok(value):
    """carla::rpc::Response<T> success: [[variant index 1, value]]."""
    return [[1, value]]


def ok_void():
    """carla::rpc::Response<void> success: [[false]] (empty optional)."""
    return [[False]]


def err(message):
    """carla::rpc::Response<T> error: [[variant index 0, [message]]]."""
    return [[0, [message]]]


def err_void(message):
    return [[True, [message]]]


def make_token(stream_id, port, address):
    """carla::streaming::detail::token_data: u32 id, u16 port, tcp, ipv4, 4 bytes, padded to 24."""
    packed = struct.pack("<IHBB", stream_id, port, 1, 1) + socket.inet_aton(address)
    return packed + bytes(24 - len(packed))


class Actor:
    def __init__(self, actor_id, description, x, y, z, yaw_deg):
        self.id = actor_id
        self.description = description
        self.z = z
        self.runner = VesselRunner(x, y, yaw_deg)
        self.control = (0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.acceleration = (0.0, 0.0, 0.0)

    def rpc_actor(self):
        extent = list(BBOX_EXTENT)
        bbox = [[0.0, 0.0, extent[2]], extent, [0.0, 0.0, 0.0]]
        return [self.id, 0, self.description, bbox, b"", b""]


class Sensor:
    """Radar attached to a vessel; relative pose is (x, y, z, pitch, yaw, roll)."""

    def __init__(self, actor_id, parent, description, relative, stream_id, port, address):
        self.id = actor_id
        self.parent = parent
        self.description = description
        self.relative = relative
        self.stream_id = stream_id
        self.token = make_token(stream_id, port, address)
        attrs = {a[0]: a[2] for a in description[2]}
        self.h_fov = float(attrs.get("horizontal_fov", 30.0))
        self.v_fov = float(attrs.get("vertical_fov", 30.0))
        self.range = float(attrs.get("range", 100.0))
        self.sensor_tick = float(attrs.get("sensor_tick", 0.0))
        self.last_emit = None

    def rpc_actor(self):
        bbox = [[0.0, 0.0, 0.0], [0.05, 0.05, 0.05], [0.0, 0.0, 0.0]]
        return [self.id, self.parent.id, self.description, bbox, b"", self.token]

    def world_pose(self):
        """Returns (location, axes) in the world frame: vessel yaw only, no pitch or roll."""
        north, east, yaw, _, _, _, _ = self.parent.runner.get()
        parent_axes = rotator_to_axes(0.0, yaw, 0.0)
        offset = local_to_world(parent_axes, self.relative[:3])
        location = (north + offset[0], east + offset[1], self.parent.z + offset[2])
        return location, compose(parent_axes, rotator_to_axes(*self.relative[3:]))


class Backend:
    def __init__(self, host, port, targets=()):
        self.host = host
        self.port = port
        self.lock = threading.RLock()
        self.episode_id = random.getrandbits(63)
        self.frame = 0
        self.elapsed = 0.0
        self.last_delta = 0.0
        self.synchronous = False
        self.fixed_delta = None
        self.actors = {}
        self.next_actor_id = itertools.count(1)
        self.targets = list(targets)
        self.sensors = {}
        self.next_stream_id = itertools.count(EPISODE_STREAM_ID + 1)
        self.subscribers = {}  # stream id -> connections
        self.stop = threading.Event()
        self.last_wall = time.monotonic()

    # -- ticking and streaming -------------------------------------------------

    def tick(self, delta):
        with self.lock:
            self.frame += 1
            self.elapsed += delta
            self.last_delta = delta
            for actor in self.actors.values():
                actor.runner.set_control(*actor.control)
                actor.runner.advance(delta)
                north, east, _, v_north, v_east, _, _ = actor.runner.get()
                velocity = (v_north, v_east, 0.0)
                actor.acceleration = tuple((n - o) / delta for n, o in zip(velocity, actor.velocity))
                actor.velocity = velocity
            for target in self.targets:
                target.step(delta)
            self.send(EPISODE_STREAM_ID, self.snapshot_message())
            for sensor in self.sensors.values():
                self.emit_sensor(sensor)
            return self.frame

    def snapshot_message(self):
        sensor_header = struct.pack(
            "<QQd6f", WORLD_OBSERVER_SENSOR_TYPE, self.frame, self.elapsed, 0, 0, 0, 0, 0, 0)
        episode_header = struct.pack(
            "<QdfiiiI", self.episode_id, time.time(), self.last_delta, 0, 0, 0, 0)
        states = []
        for actor in self.actors.values():
            north, east, yaw, _, _, yaw_rate, _ = actor.runner.get()
            states.append(
                struct.pack("<IB", actor.id, ACTOR_STATE_ACTIVE)
                + struct.pack("<6f", north, east, actor.z, 0.0, yaw, 0.0)
                + struct.pack("<3f", *actor.velocity)
                + struct.pack("<3f", 0.0, 0.0, yaw_rate)
                + struct.pack("<3f", *actor.acceleration)
                + bytes(54))
        return sensor_header + episode_header + b"".join(states)

    def emit_sensor(self, sensor):
        if sensor.sensor_tick > 0.0 and sensor.last_emit is not None \
                and self.elapsed - sensor.last_emit < sensor.sensor_tick - 1e-9:
            return
        sensor.last_emit = self.elapsed
        location, axes = sensor.world_pose()
        detections = radar.detect(
            self.targets, location, axes, sensor.parent.velocity,
            sensor.h_fov, sensor.v_fov, sensor.range)
        header = struct.pack(
            "<QQd6f", RADAR_SENSOR_TYPE, self.frame, self.elapsed, *location, *axes_to_rotator(axes))
        self.send(sensor.stream_id, header + b"".join(struct.pack("<4f", *d) for d in detections))

    def send(self, stream_id, payload):
        framed = struct.pack("<I", len(payload)) + payload
        alive = []
        for conn in self.subscribers.get(stream_id, []):
            try:
                conn.sendall(framed)
                alive.append(conn)
            except OSError:
                conn.close()
        self.subscribers[stream_id] = alive

    def close_stream(self, stream_id):
        for conn in self.subscribers.pop(stream_id, []):
            conn.close()

    def async_loop(self):
        while not self.stop.wait(ASYNC_PERIOD):
            with self.lock:
                if self.synchronous:
                    self.last_wall = time.monotonic()
                    continue
                now = time.monotonic()
                delta, self.last_wall = now - self.last_wall, now
                self.tick(delta)

    # -- RPC methods -----------------------------------------------------------

    def settings_wire(self):
        fixed = [True, self.fixed_delta] if self.fixed_delta else [False]
        return [self.synchronous, False, fixed, True, 0.01, 10, 0.0, True, 3000.0, 2000.0, True]

    def m_version(self, ctx):
        return ok(SERVER_VERSION)

    def m_query_lights_state(self, ctx, *args):
        return ok([])

    def m_get_required_files(self, ctx, folder=""):
        return ok([])

    def m_get_episode_info(self, ctx):
        token = make_token(EPISODE_STREAM_ID, self.port, "0.0.0.0")
        return ok([self.episode_id, [token]])

    def m_get_sensor_token(self, ctx, stream_id):
        with self.lock:
            known = stream_id == EPISODE_STREAM_ID or any(
                s.stream_id == stream_id for s in self.sensors.values())
        if not known:
            return err("unknown stream")
        return ok([make_token(stream_id, self.port, ctx["local_address"])])

    def m_get_episode_settings(self, ctx):
        return ok(self.settings_wire())

    def m_set_episode_settings(self, ctx, settings):
        with self.lock:
            self.synchronous = bool(settings[0])
            fixed = settings[2]
            self.fixed_delta = fixed[1] if len(fixed) == 2 and fixed[1] else None
            self.last_wall = time.monotonic()
            return ok(self.frame)

    def m_tick_cue(self, ctx):
        with self.lock:
            delta = self.fixed_delta or DEFAULT_SYNC_DELTA
            return ok(self.tick(delta))

    def m_get_actor_definitions(self, ctx):
        role_name = ["role_name", 3, "default", [], True, False]
        floats = {name: [name, 2, value, [value], True, False] for name, value in
                  [("horizontal_fov", "30"), ("vertical_fov", "30"), ("range", "100"),
                   ("sensor_tick", "0.0")]}
        points = ["points_per_second", 1, "1500", ["1500"], True, False]
        return ok([[1, VESSEL_BLUEPRINT, "vessel", [role_name]],
                   [2, RADAR_BLUEPRINT, "sensor", [role_name, *floats.values(), points]]])

    def m_spawn_actor(self, ctx, description, transform):
        if description[1] != VESSEL_BLUEPRINT:
            return err(f"unsupported blueprint '{description[1]}' (only {VESSEL_BLUEPRINT})")
        (x, y, z), (_, yaw, _) = transform
        with self.lock:
            actor = Actor(next(self.next_actor_id), description, x, y, z, yaw)
            self.actors[actor.id] = actor
            return ok(actor.rpc_actor())

    def m_spawn_actor_with_parent(self, ctx, description, transform, parent_id, attachment_type):
        if description[1] != RADAR_BLUEPRINT:
            return err(f"unsupported sensor '{description[1]}' (only {RADAR_BLUEPRINT})")
        with self.lock:
            parent = self.actors.get(parent_id)
            if parent is None:
                return err("parent actor not found")
            (x, y, z), (pitch, yaw, roll) = transform
            sensor = Sensor(next(self.next_actor_id), parent, description,
                            (x, y, z, pitch, yaw, roll), next(self.next_stream_id),
                            self.port, ctx["local_address"])
            self.sensors[sensor.id] = sensor
            return ok(sensor.rpc_actor())

    def m_destroy_actor(self, ctx, actor_id):
        with self.lock:
            sensor = self.sensors.pop(actor_id, None)
            if sensor is not None:
                self.close_stream(sensor.stream_id)
                return ok(True)
            removed = self.actors.pop(actor_id, None)
            for child in [s for s in self.sensors.values() if s.parent is removed]:
                del self.sensors[child.id]
                self.close_stream(child.stream_id)
            return ok(removed is not None)

    def m_get_actors_by_id(self, ctx, ids):
        with self.lock:
            known = {**self.actors, **self.sensors}
            return ok([known[i].rpc_actor() for i in ids if i in known])

    def m_set_actor_transform(self, ctx, actor_id, transform):
        with self.lock:
            actor = self.actors.get(actor_id)
            if actor is None:
                return err_void("actor not found")
            (x, y, z), (_, yaw, _) = transform
            actor.z = z
            actor.runner.set_pose(x, y, yaw)
            return ok_void()

    def m_apply_control_to_vessel(self, ctx, actor_id, control):
        with self.lock:
            actor = self.actors.get(actor_id)
            if actor is None:
                return err_void("actor not found")
            actor.control = (float(control[0]), float(control[1]))
            return ok_void()


class RpcHandler(socketserver.BaseRequestHandler):
    def handle(self):
        backend = self.server.backend
        ctx = {"local_address": self.request.getsockname()[0]}
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        unpacker = msgpack.Unpacker(raw=False)
        while True:
            data = self.request.recv(65536)
            if not data:
                return
            unpacker.feed(data)
            for message in unpacker:
                self.dispatch(backend, ctx, message)

    def dispatch(self, backend, ctx, message):
        kind, msgid, method, params = message
        handler = getattr(backend, "m_" + method, None)
        if handler is None:
            reply = [1, msgid, f"rpc::rpc_error during call: no such function '{method}'", None]
        else:
            try:
                # params[0] is carla::rpc::Metadata ([is_async]); handlers don't need it.
                reply = [1, msgid, None, handler(ctx, *params[1:])]
            except Exception as exc:  # noqa: BLE001 - report to the client like rpclib does
                reply = [1, msgid, f"{type(exc).__name__}: {exc}", None]
        self.request.sendall(msgpack.packb(reply, use_bin_type=True))


class StreamHandler(socketserver.BaseRequestHandler):
    def handle(self):
        backend = self.server.backend
        raw = b""
        while len(raw) < 4:
            chunk = self.request.recv(4 - len(raw))
            if not chunk:
                return
            raw += chunk
        (stream_id,) = struct.unpack("<I", raw)
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.request.settimeout(5.0)
        with backend.lock:
            known = stream_id == EPISODE_STREAM_ID or any(
                s.stream_id == stream_id for s in backend.sensors.values())
            if not known:
                return
            backend.subscribers.setdefault(stream_id, []).append(self.request)
        # Hold the connection open; the broadcaster owns writes.
        while not backend.stop.is_set():
            try:
                if not self.request.recv(1):
                    return
            except socket.timeout:
                continue
            except OSError:
                return


class ThreadedServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def serve(host="127.0.0.1", port=2000, targets=()):
    """Starts the RPC server on `port` and the streaming server on `port + 1`."""
    backend = Backend(host, port + 1, targets)
    rpc_server = ThreadedServer((host, port), RpcHandler)
    stream_server = ThreadedServer((host, port + 1), StreamHandler)
    rpc_server.backend = stream_server.backend = backend
    for server in (rpc_server, stream_server):
        threading.Thread(target=server.serve_forever, daemon=True).start()
    threading.Thread(target=backend.async_loop, daemon=True).start()
    return backend, rpc_server, stream_server


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument(
        "--target", action="append", default=[], metavar="N,E[,Z[,VN,VE]]",
        help="point target for the radar (north, east, z in m; velocity in m/s); repeatable")
    args = parser.parse_args()
    targets = [radar.Target.parse(t) for t in args.target]
    backend, rpc_server, stream_server = serve(args.host, args.port, targets)
    print(f"lightweight vessel backend: rpc {args.host}:{args.port}, "
          f"streaming {args.host}:{args.port + 1}")
    try:
        backend.stop.wait()
    except KeyboardInterrupt:
        pass
    backend.stop.set()
    rpc_server.shutdown()
    stream_server.shutdown()


if __name__ == "__main__":
    main()
