# Lightweight backend: minimal CARLA protocol subset

Goal: a non-Unreal server that the unmodified Python/C++ client (`carla.Client`)
can connect to, spawn a vessel and sensors on, drive with `VesselControl`, and tick
synchronously. It must advance the vessel with `carla::vessel::VesselRunner`
(`Unreal/.../Carla/Vessel/VesselRunner.h`, header-only, standard library only) so
the dynamics are identical to the Unreal backend.

Everything below was read from the client sources under `LibCarla/source/carla/`
and has **not** yet been exercised against a server. Items marked *to confirm* need
a check when the backend is written. The client library builds and the wheel
imports (see `setup.md`); the protocol itself is untested.

## Transport layers

1. **RPC**: msgpack-RPC over TCP using rpclib, default port 2000
   (`client/detail/Client.cpp`: `rpc_client.call(...)`). Arguments and results use
   `MSGPACK_DEFINE_ARRAY` layouts (fields in declaration order, no names). The
   client uses a timeout (`GetTimeout`), so every call must answer promptly.
   Server-side errors are returned as `rpc::Response<T>` holding either `T` or a
   `ResponseError` (`rpc/Response.h`).
2. **Streaming**: a second TCP server. The client learns the address from a
   `streaming::Token` (`streaming/detail/Token.h`, packed: `uint32 stream_id`,
   `uint16 port`, `uint8 protocol` (1 = tcp), `uint8 address_type`
   (1 = ipv4), address bytes). Only TCP tokens are accepted by the client
   (`streaming/detail/tcp/Client.cpp`). Handshake: the client connects and sends
   its `uint32 stream_id`; the server then pushes messages, each a size prefix
   (`message_size_type`) followed by the buffer (`tcp/ServerSession.cpp`,
   `tcp/Message.h`; prefix width *to confirm*).

The streaming layer is required, not optional: world snapshots (pose, velocity,
frame, timestamps) and all sensor data arrive only by streaming. `Actor.get_transform()`,
`get_velocity()`, `world.tick()` completion and `wait_for_tick()` are all derived
from snapshots, with no RPC.

## RPC functions required

Names are the strings passed to `rpc_client.call` in `client/detail/Client.cpp`.

| Function | Arguments | Result | Used for |
|----------|-----------|--------|----------|
| `version` | none | string (client expects `"0.10.0"`, see `Version.h`) | Connect; a mismatch only logs a warning (`Simulator::ValidateVersions`). |
| `get_episode_info` | none | `EpisodeInfo [id:uint64, token]` | `client.get_world()`; token of the snapshot stream. |
| `get_episode_settings` | none | `EpisodeSettings` (11-element array, see `rpc/EpisodeSettings.h`) | Startup; async-vs-sync decision. |
| `set_episode_settings` | `EpisodeSettings` | `uint64` frame | Enable synchronous mode and `fixed_delta_seconds`. The client then waits up to 1 s for a snapshot with that frame. |
| `tick_cue` | none | `uint64` frame | `world.tick()`. The client waits until a snapshot with `frame >=` the returned value arrives. |
| `get_actor_definitions` | none | `vector<ActorDefinition>` `[uid, id, tags, attributes]` | Blueprint library: needs `vessel.mass.kvlcc2` and the sensor blueprints in use. |
| `spawn_actor` | `ActorDescription [uid, id, attributes]`, `Transform` | `Actor [id, parent_id, description, bounding_box, semantic_tags, stream_token]` | Vessel spawn. A `vessel.` id makes a `client::Vessel` (`detail/ActorFactory.cpp`). |
| `spawn_actor_with_parent` | description, transform, parent id, attachment type | `Actor` | Sensors attached to the vessel; sensors need a valid `stream_token`. |
| `apply_control_to_vessel` | actor id, `VesselControl [rudder_angle, propeller_speed]` | void (or `Response`) | Command; held until the next call. |
| `destroy_actor` | actor id | bool | Cleanup. |
| `get_actors_by_id` | `vector<ActorId>` | `vector<Actor>` | The client fills in actors it sees in a snapshot but has not spawned itself. |

The backend should treat a snapshot actor as unknown-to-client only if it is not
spawned by that client; it must therefore return correct `Actor` records here.

Recommended, not strictly required for a first test:

| Function | Why |
|----------|-----|
| `set_actor_transform`, `set_actor_location` | Teleports; map to `VesselRunner::SetPose`. |
| `apply_batch` | `Batch` commands and `SpawnActor` batches; Python scripts often use it. |
| `get_weather_parameters`, `set_weather_parameters` | Scripts that touch weather. |
| `get_map_info`, `get_map_data`, `get_available_maps`, `load_new_episode` | `world.get_map()`, map loading. Not needed for a flat-water test. |
| `get_spectator` | `world.get_spectator()`. |

Anything else may return an error; the call sites are explicit, so unimplemented
functions only fail the feature that uses them.

## Streamed data

### World snapshot (episode token)

One message per tick (`sensor/s11n/EpisodeStateSerializer.h`,
`Sensor/WorldObserver.cpp`, `FWorldObserver::BroadcastTick`). Like every streamed
message it starts with the sensor header below (`frame`; `timestamp` = episode
elapsed game time, `Episode.GetElapsedGameTime()`; `sensor_type` = the
episode-state type; transform unused). The episode-state payload follows, packed:

- Header: `uint64 episode_id`, `double platform_timestamp`, `float delta_seconds`,
  `Vector3DInt map_origin`, `simulation_state` (0 = none).
- Then one `ActorDynamicState` (119 bytes, `sensor/data/ActorDynamicState.h`) per
  actor: `id`, `actor_state`, `Transform` (location in m as x, y, z; rotation
  pitch, yaw, roll in degrees), `velocity` (m/s), `angular_velocity` (deg/s),
  `acceleration` (m/s^2), then the type-dependent union (zeros for a vessel).
- `Episode::Listen` drops snapshots whose frame is not newer than the last one
  (except on an episode change), so frame numbers must strictly increase.
- The client's `Timestamp` (frame, elapsed seconds, delta seconds, platform
  timestamp) is assembled from the sensor header and the episode header; both
  must be consistent between snapshots and sensors.

Vessel mapping (shared with Unreal, see `unreal-integration.md`): location x = north
(m), y = east (m); yaw deg = heading; velocity from `VesselRunner::GetWorldVelocity`;
angular velocity z = `GetYawRateDegPerSec`; acceleration = velocity difference over
`delta_seconds`.

### Sensor messages (per-sensor token from `Actor.stream_token`)

Each message starts with `SensorHeaderSerializer::Header` (packed):
`uint64 sensor_type`, `uint64 frame`, `double timestamp`, `Transform sensor_transform`,
followed by the sensor-specific payload (`sensor/s11n/*Serializer.h`). Only the
sensors in use need to be implemented; a first backend can start with radar and
LiDAR (point arrays). `sensor_type` must match the index from the client's
sensor registry (`sensor/SensorRegistry.h`). Camera images need a renderer and are
out of scope for the lightweight tier.

## Tick semantics to reproduce

Synchronous mode (what the Unreal server does, `Game/CarlaEngine.cpp`):

1. Wait for `tick_cue`; apply any RPCs received since the previous tick.
2. Advance every vessel with `VesselRunner::Advance(fixed_delta_seconds)`, using
   the last control received.
3. Publish the snapshot, then the sensor messages, all with the same new `frame`
   and elapsed time.
4. Reply to `tick_cue` with the new frame (the reply may precede publishing; the
   client waits for the snapshot either way).

Asynchronous mode (`synchronous_mode = false`) is not needed for the first test;
the client also waits for one snapshot on startup in that mode.

## Not required

Traffic manager, walkers, vehicles, traffic lights, recorder, map and OpenDRIVE,
file transfer, ROS2, GBuffer, lights, environment objects, debug drawing.

## Conformance script

One script, run against every backend (lightweight, Unreal low quality, Unreal
full): connect, enable synchronous mode at 0.05 s, spawn `vessel.mass.kvlcc2`,
apply a control each tick, and check:

- frame numbers rise by exactly 1 per tick;
- snapshot timestamp delta equals `fixed_delta_seconds`;
- pose and velocity match a local `VesselRunner` run to numerical tolerance;
- `carla.VesselControl` round-trips and `get_control()` returns the last command;
- a spawned sensor reports `frame` and `timestamp` equal to the snapshot's.
