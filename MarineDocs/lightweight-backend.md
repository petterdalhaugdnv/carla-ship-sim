# Lightweight backend

A non-Unreal CARLA-protocol server for fast vessel tests. The unmodified `carla`
Python client connects to it, spawns `vessel.mass.kvlcc2`, ticks and sends
`VesselControl`. Protocol details: [lightweight-backend-protocol.md](lightweight-backend-protocol.md).

## Layout (`PythonAPI/lightweight_backend/`)

| File | Role |
|------|------|
| `vessel_runner_c.cpp` | Thin `extern "C"` ABI over `carla::vessel::VesselRunner`; contains no dynamics |
| `vessel_runner.py` | ctypes wrapper. Compiles the shim with `g++` into `~/.cache/carla-marine/` (override with `CARLA_MARINE_CACHE`) when missing or older than its sources |
| `server.py` | msgpack-RPC server (port 2000) and snapshot streaming server (port 2001), single shared lock, wall-clock ticks in asynchronous mode, `tick_cue` in synchronous mode |
| `geometry.py` | Unreal rotation helpers (pitch/yaw/roll to axes, compose); checked against `carla.Transform` |
| `radar.py` | Point-target radar model: `Target`, `detect` |
| `harness.py` | Shared test helpers (`check`, `backend_process`, `wait_for_port`) |
| `viewer.py` | Top-down chart (north up, east right) over the public client API; works against any CARLA server |
| `test_conformance.py` | Spawn, sync tick and control check using the real client |
| `test_radar.py` | Attached radar check using the real client |

The dynamics are the same compiled `VesselRunner` the Unreal component uses, so
results match by construction; the Python side only does protocol encoding.

## Run

Needs `g++` and [uv](https://docs.astral.sh/uv/). The repo-local environment lives in
`PythonAPI/lightweight_backend/` (`pyproject.toml`, `uv.lock`, git-ignored `.venv/`); it does
not touch the legacy CARLA Python files. The `carla` client wheel is built locally outside the
repository (see `setup.md`) and referenced from `pyproject.toml` by relative path
(`[tool.uv.sources]`: `../../../build-client/PythonAPI/dist/carla-0.10.0-cp312-cp312-linux_x86_64.whl`).
Python is pinned to 3.12 to match that wheel. If your build directory or Python version
differs, edit that path and `requires-python`. `uv sync` fails if the wheel is missing.

```
cd PythonAPI/lightweight_backend
uv sync
```

Then:
```
uv run python server.py --port 2000
uv run python test_conformance.py --port 2100   # starts its own backend
uv run python test_radar.py --port 2110        # use a different port per run
```

Point targets for the radar (north, east, [z, [v_north, v_east]] in metres and m/s;
repeatable; north maps to CARLA x, east to y):

```
uv run python server.py --target=800,0 --target=600,300,0,-2,0
```

Viewer (installed by `uv sync`). Without `--external` it starts its own backend:

```
uv run python viewer.py --output chart.png --ticks 1200 --rudder 0.1 \
    --target=800,0,0 --target=600,300,0,-2,0      # headless PNG with radar overlay
uv run python viewer.py --live --external --port 2000   # window, any server
```

`--radar` (or any `--target`) attaches a forward radar and draws its field of view and
detections. Detections are reprojected to world coordinates with the sensor's own
`data.transform`, so the overlay also exercises the transform chain.

## Radar model

`sensor.other.radar` can be spawned with `attach_to=<vessel>`. Attributes
`horizontal_fov`, `vertical_fov`, `range` and `sensor_tick` are honoured. Each target
inside range and the rectangular field of view yields one detection
`(velocity, azimuth, altitude, depth)`, using the CARLA convention (sensor x forward,
y right, z up; velocity = relative velocity projected on the line of sight, negative
when closing). The sensor pose is the vessel pose composed with the mount transform;
the sensor velocity equals the parent velocity (lever-arm effect of rotation ignored).
Frame and timestamp equal the snapshot's. Limits: no occlusion, no noise,
`points_per_second` ignored, targets are points.

Stream connections are established asynchronously as in CARLA, so the first
ticks after `listen()` may not be delivered; tests tick until data arrives.

## Supported

`version`, `get_episode_info`, `get_sensor_token`, `get_episode_settings`,
`set_episode_settings`, `tick_cue`, `get_actor_definitions`, `spawn_actor`,
`destroy_actor` (also destroys child sensors), `get_actors_by_id`, `set_actor_transform`,
`apply_control_to_vessel`, `spawn_actor_with_parent` (radar only), and empty answers for
`get_required_files` and `query_lights_state`.

## Not supported yet

LiDAR, cameras, other sensors, weather, maps, batch commands, spectator, recorder.
Acceleration in snapshots is a finite difference of velocity; the vessel is flat
(`z`, pitch and roll stay at their spawn values).
