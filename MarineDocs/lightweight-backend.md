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
| `test_conformance.py` | Conformance check using the real client |

The dynamics are the same compiled `VesselRunner` the Unreal component uses, so
results match by construction; the Python side only does protocol encoding.

## Run

Needs `g++`, Python with `msgpack`, and the `carla` wheel (see `setup.md`).

```
python PythonAPI/lightweight_backend/server.py --port 2000
python PythonAPI/lightweight_backend/test_conformance.py --port 2100   # starts its own backend
```

## Supported

`version`, `get_episode_info`, `get_sensor_token`, `get_episode_settings`,
`set_episode_settings`, `tick_cue`, `get_actor_definitions`, `spawn_actor`,
`destroy_actor`, `get_actors_by_id`, `set_actor_transform`, `apply_control_to_vessel`,
plus empty answers for `get_required_files` and `query_lights_state`.

## Not supported yet

Sensors and attached actors, weather, maps, batch commands, spectator, recorder.
Acceleration in snapshots is a finite difference of velocity; the vessel is flat
(`z`, pitch and roll stay at their spawn values).
