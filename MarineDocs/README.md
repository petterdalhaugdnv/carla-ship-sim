# Marine fork documentation

Fork of CARLA (UE5) towards a marine simulator. The existing navigation,
controller, rudder and thruster stack is meant to drive a vessel in this
simulator with realistic camera, radar and LiDAR input.

Work is additive: the vehicle code is untouched and stays as a reference until
the vessel path runs end to end.

| File | Content |
|------|---------|
| [architecture.md](architecture.md) | Vessel classes, data flow, frames and units |
| [setup.md](setup.md) | Building and testing what exists today |
| [plan.md](plan.md) | Commit history, status, limitations, next steps |
| [unreal-integration.md](unreal-integration.md) | Static verification of tick order, time sync, velocity, frames |
| [lightweight-backend-protocol.md](lightweight-backend-protocol.md) | Minimal CARLA protocol subset for a non-Unreal backend |
| `decisions/` | Reserved for decision records (empty) |

## Status (branch `feature/native-vessel-prototype`, base `b544fa7cb`)

Implemented and committed: vessel control contract, client `carla.Vessel`,
Python API, UE actor/component/factory, server RPC, and an engine-free MMG
3-DOF dynamics model.

Verified: dynamics unit test, C++ client library build, Python wheel build
and import.
Not verified: anything running inside Unreal, and an end-to-end spawn.
