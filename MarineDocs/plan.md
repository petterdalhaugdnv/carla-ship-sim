# Plan and status

## Commit history since base `b544fa7cb`

| Commit | Change |
|--------|--------|
| `fe6d6f322` | Vessel control contract (`rpc::VesselControl`, `FVesselControl`) |
| `6f7d1fae9` | Client `Vessel` and `apply_control_to_vessel` call |
| `a09de152f` | `carla.Vessel` and `carla.VesselControl` in Python, with docs |
| `a2e47bb94` | `UVesselDynamicsComponent` and `ACarlaVessel` |
| `36645f3ae` | Engine-free MMG 3-DOF model, unit test and reference generator |
| `ff10bafa2` | `ActorType::Vessel` plumbing, `FVesselActor`, `NotAVessel` response |
| `343a494b6` | `AVesselActorFactory`, registered by default |
| `7e9ee7f10` | Server RPC `apply_control_to_vessel` |
| `6f37abec3`, `ef6909b9e` | Merges of the dynamics and UE-structure helper branches |
| `95274dd98` | Fix `carla.Vessel.get_control` binding (needs `copy_const_reference`; found by the Python build) |

## Validation

| Check | Result |
|-------|--------|
| Dynamics unit test (g++ 15, `-Wall -Wextra`) | Passed |
| `carla-client` build (265/265) | Passed, contains `Vessel::ApplyControl` |
| `carla-python-api` wheel, import, `VesselControl`, `Vessel` methods | Passed |
| UE build, UHT, runtime | Not run |
| End-to-end spawn | Not run |

## Known limitations

- Vessels are not recorded or replayed (recorder and `FrameData` skip them).
- `ACarlaVessel` is invisible until a Blueprint subclass adds a mesh.
- Pose can lag one 0.02 s substep when the tick length is not a multiple of 0.02 s.
- Dormant vessels return `FunctionNotAvailiableWhenDormant`.
- No `ApplyVesselControl` batch command.
- No water, waves, buoyancy or collision; radar and LiDAR use `ECC_GameTraceChannel2`, so water must block it.
- KVLCC2 is 320 m long; default sensor ranges are small (LiDAR 10 m, radar 100 m).
- Unverified UE risks: UHT and `GENERATED_BODY`, `disable-ue4-macros` include pairing in `VesselDynamicsComponent.h`, name clashes (`Reset`, `GetVelocity`), large-map origin rebasing.

## Next steps

1. Optional: minimal mock RPC server for an end-to-end spawn and control test without UE; one conformance script usable against any backend (mock, UE low quality, UE full).
2. UE toolchain: CARLA UE 5.3 fork, content, plugin build.
3. Flat water map (blocking `ECC_GameTraceChannel2`) and a ship mesh Blueprint subclass.
4. Synchronous-mode test: spawn `vessel.mass.kvlcc2`, attach camera, LiDAR and radar, send `VesselControl` each tick, check pose, frame numbers and `sensor_transform`.
5. Later: kinematic wave motion, then physics only if needed; sensor overlays; recorder support.
6. Remove vehicle code only after the vessel path runs perfectly.
