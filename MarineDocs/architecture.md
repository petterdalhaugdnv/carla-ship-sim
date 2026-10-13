# Vessel architecture

## Data flow

```
Python  carla.Vessel.apply_control(carla.VesselControl)
  -> client::Vessel::ApplyControl            (LibCarla/.../client/Vessel.cpp)
  -> RPC "apply_control_to_vessel"           (client/detail/Client.cpp)
  -> BIND_SYNC(apply_control_to_vessel)      (Server/CarlaServer.cpp)
  -> FCarlaActor::ApplyControlToVessel      (Actor/CarlaActor.cpp, FVesselActor)
  -> ACarlaVessel -> UVesselDynamicsComponent::SetControl
  -> fixed 0.02 s MMG substeps in TickComponent -> actor planar pose
```

## Components

Paths are under `Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/` unless noted.

| Component | Path | Role |
|-----------|------|------|
| `carla::rpc::VesselControl` | `LibCarla/source/carla/rpc/VesselControl.h` | RPC contract: `rudder_angle` (rad), `propeller_speed` (rad/s). Converts to/from `FVesselControl` under `LIBCARLA_INCLUDED_FROM_UE4`. |
| `FVesselControl` | `Vessel/VesselControl.h` | UE USTRUCT `{RudderAngle, PropellerSpeed}`. |
| `carla::client::Vessel` | `LibCarla/source/carla/client/Vessel.h/.cpp` | Client actor: `ApplyControl`, `GetControl` (last control sent by this client). `detail/ActorFactory.cpp` creates it for ids starting `vessel.`. |
| `carla.Vessel`, `carla.VesselControl` | `PythonAPI/carla/src/Actor.cpp`, `Control.cpp`; docs in `PythonAPI/docs/actor.yml`, `control.yml` | Python bindings. |
| `carla::vessel::MMG3DOF` | `Vessel/VesselDynamicsModel.h` | Engine-free header: `MMGState`, `MMGInputs`, `MMGParameters::KVLCC2()`, RK4 `Step`, `VesselActuators` (rudder limit and rate, propeller lag). Port of `mass-core-main` `mmg_3dof.py`. |
| `UVesselDynamicsComponent` | `Vessel/VesselDynamicsComponent.h/.cpp` | `TG_PrePhysics` tick; accumulator with fixed 0.02 s substeps; zero-order-hold control; writes planar pose; adopts external teleports. |
| `ACarlaVessel` | `Vessel/CarlaVessel.h/.cpp` | Plain `AActor` with a `USceneComponent` root and the dynamics component. Not a wheeled vehicle. |
| `AVesselActorFactory` | `Actor/Factory/VesselActorFactory.h/.cpp` | Registers `vessel.mass.kvlcc2` (`object_type=vessel`, optional `VesselClass`). Auto-spawned by `ACarlaGameModeBase::SpawnActorFactories` if none is configured. |
| Hook-ins | `Actor/CarlaActor.h/.cpp`, `Actor/ActorRegistry.cpp`, `Server/CarlaServerResponse.h/.cpp` | `ActorType::Vessel`, `FVesselActor`, virtual `ApplyControlToVessel` (base returns `NotAVessel`), registry cast (vessel checked first). |

## Frames and units

- Dynamics state: `[North, East, Heading]` (m, rad) and `[U, V, R]` (m/s, rad/s).
- NED north maps to UE +X, east to UE +Y, heading (clockwise from north) to UE yaw degrees, metres to cm.
- Z is left at the spawn height; pitch and roll are unchanged.
- Dynamics are kinematic: no UE physics, collision or buoyancy.

## Design notes

- The vessel path copies the vehicle path (RPC, client class, `FCarlaActor` subclass, actor type, factory) but is independent of the vehicle classes.
- `ActorType::Vessel` was inserted before `INVALID`, so `INVALID` moved from 6 to 7. This only matters for stored data (recorder, `FrameData`, `CarlaEpisode`).
- Reference actuator values: rudder +/-35 deg at 0.0405166 rad/s; propeller 0 to 25.13 rad/s with a 50 s time constant.
