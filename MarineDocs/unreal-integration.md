# Unreal integration verification

Static review of how the vessel plugs into CARLA's tick, snapshot and sensor
paths. Nothing here was run inside Unreal (no UE build available); every
statement is from reading the source, and the file and symbol are given so it can
be re-checked. Paths are under `Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/`.

## Tick order

Per world tick (`Game/CarlaEngine.cpp`):

1. `FCarlaEngine::OnPreTick` (`OnWorldTickStart`): processes RPCs until the tick
   cue in synchronous mode, so `apply_control_to_vessel` has already set the
   control; then `CurrentEpisode->TickTimers(DeltaSeconds)` advances episode time.
2. Actor ticks. `UVesselDynamicsComponent` ticks in `TG_PrePhysics` and moves the
   actor.
3. `FCarlaEngine::OnPostTick` (`OnWorldPostActorTick`): `WorldObserver.BroadcastTick`
   builds the snapshot, then `GetSensorManager().PostPhysTick` runs the sensors.

Result: the snapshot and every sensor of a frame see the vessel pose written in the
same frame, and the control received for frame N is used for frame N. Verified by
reading; not observed at runtime.

## Time synchronisation

- Fixed delta: `FCarlaEngine_SetFixedDeltaSeconds` forces UE's `DeltaTime`, so in
  synchronous mode the component receives exactly `fixed_delta_seconds`.
- Earlier prototype: a 0.02 s accumulator left a remainder when the tick was not
  a multiple of 0.02 s (for example the default 0.05 s), so the vessel's simulated
  time lagged episode time by up to one substep. **Fixed**: `VesselRunner::Advance`
  now integrates exactly `DeltaTime` in equal substeps of at most 0.02 s
  (0.05 s becomes 3 x 0.01667 s). Covered by `TestTimeSync` and
  `TestTickSizeInvariance` in `Vessel/Tests/VesselRunnerTest.cpp`.
- Sensor timestamps come from the episode (`SensorHeaderSerializer::Header`
  holds `frame`, `timestamp`, `sensor_transform`), not from the vessel, so they
  are consistent with the snapshot as long as the vessel is advanced in the same
  tick, which it is.
- `GetSimTime()` on the component/runner is the vessel's own integrated time. In
  synchronous mode it should equal the episode's elapsed game time since
  spawn/reset. A runtime check of this is an acceptance item (see `plan.md`).
- Budget: a `DeltaTime` above 500 x 0.02 s = 10 s is truncated and logged.

## Actor velocity

- `FWorldObserver_Serialize` (`Sensor/WorldObserver.cpp`) and
  `ARadar::CalculateRelativeVelocity` (`Sensor/Radar.cpp`) call
  `AActor::GetVelocity()`; `ACarlaVessel::GetVelocity` is overridden to return the
  kinematic velocity, so the snapshot, `get_velocity()` and radar Doppler see it.
  `FCarlaActor::GetActorVelocity` (used by the recorder) does the same.
- Dormant vessels use `FActorData::Velocity`, captured by `Actor/ActorData.cpp`
  via `Actor->GetVelocity()`.
- Acceleration is derived by `FWorldObserver_GetAcceleration` from the velocity
  difference, so it works without any vessel code.
- **Gap found and fixed:** angular velocity was read from a `UPrimitiveComponent`
  root, which a vessel does not have, so it was always zero. It now returns
  `(0, 0, yaw rate deg/s)` for `ACarlaVessel` (`FWorldObserver_GetAngularVelocity`).
  Roll and pitch rates are zero by construction.
- Radar caveat: `ARadar::CalculateRelativeVelocity` uses `HittedActor->GetVelocity()`,
  which is correct for vessel targets but returns zero for static meshes.

## Coordinate conversions

| Quantity | Model (NED, SI) | UE / CARLA world | Where |
|----------|-----------------|------------------|-------|
| Position | north, east (m) | X, Y (cm); m x 100 | `VesselDynamicsComponent.cpp` (`ToPoseMeters`, `WritePoseToActor`) |
| Heading | rad clockwise from north, wrapped to (-pi, pi] | yaw deg, positive clockwise from above | `VesselRunner::GetPose`, `SetPose` |
| Velocity | body U surge, V sway (starboard +) | world X/Y cm/s | `VesselRunner::GetWorldVelocity` x 100 |
| Yaw rate | rad/s, clockwise + | deg/s, Z component | `VesselRunner::GetYawRateDegPerSec` |

Checks:

- UE and CARLA use a left-handed frame with X forward, Y right, Z up, and yaw is
  positive clockwise seen from above, so north = +X, east = +Y and
  heading = yaw have the same sign. `VesselRunnerTest::TestFrameConversion`
  verifies heading 90 degrees moves east and a starboard rudder gives positive yaw
  and yaw rate.
- `rpc::Rotation` is `(pitch, yaw, roll)` in degrees and maps directly to
  `FRotator` (`geom/Rotation.h`); transforms are sent in meters, so the server's
  `carla::geom::Transform(FTransform)` conversion applies the cm to m scaling.
- Pitch, roll and Z are left untouched by the dynamics. Anything that adds wave
  motion later must write them without feeding back into the planar state.
- Origin rebasing: CARLA's `map_origin` is carried in the snapshot header
  (`EpisodeStateSerializer::Header::map_origin`). The component reads
  `GetActorLocation()` in world coordinates and assumes no rebasing; this is
  unverified and matters only for very large maps.

## External teleport

`set_actor_transform` changes the actor pose between ticks. The component compares
the actor pose with what it last wrote and, if different, calls
`VesselRunner::SetPose`, which keeps velocities and actuator state (covered by
`TestPoseAndReset`). A teleport made by the same RPC that applies a control in the
same frame is processed once per tick.

## Open points that need a running Unreal

- Compile and UHT: `GENERATED_BODY` in `VesselDynamicsComponent.h`/`CarlaVessel.h`,
  the `disable-ue4-macros` include pairing around `VesselRunner.h`, and possible
  name clashes (`Reset`, `GetVelocity` on the component).
- `ACarlaVessel` is invisible until a Blueprint subclass adds a mesh.
- Actual timing of snapshot, sensor frames and `GetSimTime()` against episode time.
- Sensor attach on a vessel without physics: attachment uses the root
  `USceneComponent`, which should follow `SetActorLocationAndRotation` immediately.
