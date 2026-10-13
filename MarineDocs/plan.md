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
| `f0ccb5cea` | `MarineDocs/` documentation |
| branch `feature/vessel-engine-free-runner` | `VesselRunner.h` extracted from the component, component refactored onto it, vessel angular velocity in the snapshot, runner tests, Unreal-integration and lightweight-protocol docs (see `git log`) |
| branch `feature/vessel-lightweight-backend` | `PythonAPI/lightweight_backend/`: Python msgpack-RPC + streaming server reusing `VesselRunner` through a ctypes shim, and a conformance test (see `lightweight-backend.md`); then a matplotlib top-down viewer, shared test harness, and an attached point-target radar with overlay and test |

## Validation

| Check | Result |
|-------|--------|
| Dynamics unit test (g++ 15, `-Wall -Wextra`) | Passed |
| Runner unit test (`-O2`, and ASan+UBSan) | Passed |
| `carla-client` build (265/265) | Passed, contains `Vessel::ApplyControl` |
| `carla-python-api` wheel, import, `VesselControl`, `Vessel` methods | Passed |
| UE build, UHT, runtime (incl. refactored component, `WorldObserver` change) | Not run |
| End-to-end spawn, sync ticks, `apply_control`, pose/velocity/timestamps vs `VesselRunner` (lightweight backend, real client, 200 ticks, `test_conformance.py`) | Passed (12 checks, 2 runs) |
| Viewer headless PNG against the lightweight backend (vessel turn, track, radar FOV and detections inspected) | Passed |
| Attached radar through the real client (`test_radar.py`: frame/timestamp match, sensor transform, detection counts, reprojection error < 1e-4 m, radial velocities, destroy; 3 runs) | Passed (14 checks) |
| Same scripts against an Unreal server | Not run |
| End-to-end spawn on Unreal | Not run |

## Known limitations

- Vessels are not recorded or replayed (recorder and `FrameData` skip them).
- `ACarlaVessel` is invisible until a Blueprint subclass adds a mesh.
- Dormant vessels return `FunctionNotAvailiableWhenDormant`.
- No `ApplyVesselControl` batch command.
- No water, waves, buoyancy or collision; radar and LiDAR use `ECC_GameTraceChannel2`, so water must block it.
- KVLCC2 is 320 m long; default sensor ranges are small (LiDAR 10 m, radar 100 m).
- Unverified UE risks: UHT and `GENERATED_BODY`, `disable-ue4-macros` include pairing in `VesselDynamicsComponent.h`, name clashes (`Reset`, `GetVelocity`), large-map origin rebasing. Details in `unreal-integration.md`.
- Resolved on `feature/vessel-engine-free-runner`: the one-substep pose lag for tick lengths that are not multiples of 0.02 s (the runner now integrates exactly the tick length), and zero angular velocity in the snapshot.
- **Accepted decision (2026-10-13):** the dynamics step is an integer fraction of the simulation tick. Substeps are now `ceil(dt/0.02)` equal parts, so a 0.05 s tick uses 3 x 0.0167 s rather than 2 x 0.02 s with a carried remainder. Results at 0.02 s ticks are unchanged.

## Next steps

1. Lightweight backend, next increment: synthetic LiDAR (and optionally a simple camera) payloads. Done: spawn, sync tick, control, viewer, attached radar. Radar limits: no occlusion or noise, one detection per point target.
2. UE toolchain: CARLA UE 5.3 fork, content, plugin build.
3. Flat water map (blocking `ECC_GameTraceChannel2`) and a ship mesh Blueprint subclass.
4. Synchronous-mode test: spawn `vessel.mass.kvlcc2`, attach camera, LiDAR and radar, send `VesselControl` each tick, check pose, frame numbers and `sensor_transform`.
5. Later: kinematic wave motion, then physics only if needed; sensor overlays; recorder support.
6. Remove vehicle code only after the vessel path runs perfectly.
