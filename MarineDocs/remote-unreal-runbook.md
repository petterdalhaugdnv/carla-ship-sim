# Runbook: first Unreal run on a remote machine

For running the vessel plugin on a machine with a GPU, after developing against the
lightweight backend locally. Nothing in this file has been executed: there is no Unreal
Engine on the development machine. Items marked *unverified* come from the stock CARLA
documentation, not from a run of this fork.

## What is already established without Unreal

- The vessel dynamics (`VesselRunner`) and the client (`carla.Vessel`, `VesselControl`)
  behave as specified, see `plan.md` (validation table).
- `test_conformance.py` and `test_radar.py` pass against the lightweight backend.
- The Unreal side has only been reviewed statically: `unreal-integration.md`.

The purpose of the remote session is to move each gate below from "not run" to a recorded
result, stopping at the first failing gate.

## Prerequisites

From `Docs/build_linux_ue5.md` (Windows: `Docs/build_windows_ue5.md`):

- Ubuntu 22.04 or newer; the CARLA fork of **Unreal Engine 5.5** (needs a GitHub account
  linked to Epic Games); `CARLA_UNREAL_ENGINE_PATH` pointing at it. Building the engine
  from scratch takes more than an hour and about 225 GB.
- Build on a local disk, not an external one.
- The repository must be reachable from the remote machine. The `feature/*` branches are
  local only (nothing has been pushed); pushing them is a decision for the repository owner.
- For the Python side: the `carla` wheel built from the same checkout (target
  `carla-python-api-install`, or the wheel in the build's `PythonAPI/dist`), and `g++`
  (`test_conformance.py` compiles `VesselRunner` as its reference). The test scripts are
  plain Python; `uv` is optional there. `viewer.py` also needs `matplotlib`.

## Build

Commands from `Docs/build_linux_ue5.md`:

```
cmake -G Ninja -S . -B Build --toolchain=$PWD/CMake/LinuxToolchain.cmake \
  -DLAUNCH_ARGS="-prefernvidia" -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS2=OFF
cmake --build Build
cmake --build Build --target carla-python-api-install
cmake --build Build --target launch      # opens the editor; Play starts the CARLA server
```

`ENABLE_ROS2=OFF` is a suggestion to remove a dependency, not a requirement. The first
configure also downloads content (`CarlaSetup.sh` does this for a fresh machine).

## Gates

Record the outcome of each gate in the table at the end.

### G1: the plugin compiles

Expect to iterate here. Files to look at first, from the risk list in
`unreal-integration.md`:

| Area | Files |
|------|-------|
| UHT and `GENERATED_BODY` | `Vessel/CarlaVessel.h`, `Vessel/VesselDynamicsComponent.h`, `Actor/Factory/VesselActorFactory.h` |
| `disable-ue4-macros` / `enable-ue4-macros` pairing around the engine-free include | `Vessel/VesselDynamicsComponent.h` |
| Name clashes with engine symbols (`Reset`, `GetVelocity`) | `Vessel/VesselDynamicsComponent.h`, `Vessel/CarlaVessel.h` |
| Switch statements over `FCarlaActor::ActorType` | `Recorder/CarlaRecorder.cpp`, `Game/FrameData.cpp`, `Recorder/CarlaReplayerHelper.cpp` (they already omit `TrafficSign`, so no new exhaustiveness error is expected; vessels are skipped) |

Pass: the `Carla` plugin and the CARLA client library build with no errors. Any fix goes
to the vessel files and a unit test or doc note, not to unrelated CARLA code.

### G2: the server starts and lists the vessel

Start the server (editor Play, or a packaged build; stock flags such as `-RenderOffScreen`,
`-quality-level=Low` and `-carla-rpc-port=2000` are *unverified* for UE5). Then:

```
python - <<'EOF'
import carla
w = carla.Client("127.0.0.1", 2000).get_world()
print([b.id for b in w.get_blueprint_library().filter("vessel.*")])
EOF
```

Pass: prints `['vessel.mass.kvlcc2']`. This proves `SpawnActorFactories` registered
`AVesselActorFactory`.

### G3: spawn, tick and control

```
cd PythonAPI/lightweight_backend
python test_conformance.py --external --host <H> --port 2000 --spawn-z <water height m>
```

The reference runs in double precision on the client; UE stores positions in cm.
Start with the defaults, and if the only failures are position or velocity errors of
rounding size, loosen `--pos-tol` and `--vel-tol` and record the values used. Failures of
frame, timestamp or yaw checks are real findings.

Pass: all checks. Settles whether actor velocity, the control-then-tick order and the
timestamps behave as `unreal-integration.md` claims. Also watch the server log for
`Vessel dynamics step budget exceeded` or `non-finite` warnings.

### G4: attached sensors

Radar needs something to hit on trace channel `ECC_GameTraceChannel2`. A vessel has no
collision, so place a static prop or a wall at a known position, then:

```
python test_radar.py --external --host <H> --port 2000 --expect <x>,<y>,<z> --expect-tol 10
```

Pass: stream delivery, frame and timestamp equal to the snapshot, sensor transform equal
to vessel pose composed with the mount, detections within range and field of view, and
a detection near each expected point. Detection counts and Doppler are not asserted (see
the differences below). Camera and LiDAR are untested on both tiers: spawn them by hand
once G4 passes.

### G5: overlay

`python viewer.py --live --external --port 2000 --radar` shows the vessel and the radar
detections on a chart, against Unreal.

## Differences to expect between the two backends

| Topic | Lightweight backend | Unreal (`Sensor/Radar.cpp`) |
|-------|--------------------|-----------------------------|
| Detections | One per target in range, deterministic | `points_per_second * dt` randomly sampled rays, only where a ray hits geometry |
| Field of view | Rectangle in azimuth and altitude | Rays fill an ellipse on a plane at `range`; angles stay inside the half-FOV |
| Targets | Point targets from `--target`, optionally moving | Actors with collision on `ECC_GameTraceChannel2`; Doppler uses the hit actor's `GetVelocity()`, which is zero for static meshes and the kinematic velocity for a vessel |
| Radar own velocity | Parent's velocity | Finite difference of the radar's own location each tick, so it includes the lever arm of the vessel's yaw rate (100 m forward mount at 0.5 deg/s is about 0.9 m/s) |
| Range of detection hits | Target centre | Surface impact point |

`ARadar::PostPhysTick` runs in the world post-actor-tick delegate, after
`UVesselDynamicsComponent` has moved the actor, so sensors do not lag the vessel by a
tick. Transients are possible on the first frames after attaching a sensor, because
`PrevLocation` is taken in `BeginPlay`, before attachment (*unverified*); ignore the first
few frames when judging Doppler.

## Results to record

| Gate | Date | Machine and engine | Result | Notes |
|------|------|--------------------|--------|-------|
| G1 plugin builds | | | | |
| G2 vessel listed | | | | |
| G3 conformance | | | | |
| G4 radar | | | | |
| G5 overlay | | | | |
