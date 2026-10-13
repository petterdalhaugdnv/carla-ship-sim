# Setup and testing

Only the UE-free parts can be built and tested without the Unreal Engine fork.
Use a build directory outside the repository (`Build/` is not git-ignored).

## Dynamics unit test (no dependencies)

```
cd Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/Vessel/Tests
g++ -std=c++17 -Wall -Wextra -O2 VesselDynamicsModelTest.cpp -o /tmp/vessel_test && /tmp/vessel_test
```

Checks 30 reference samples against the Python model (`generate_mmg_reference.py`, which needs the `mass-core-main` checkout), plus rest, heading wrap, rudder rate and shaft lag. Last result: worst scaled error 1.11e-15.

## C++ client and Python API (no UE)

Needs a Python with headers (3.10 to 3.12; 3.14 is likely incompatible with Boost.Python) and network access for FetchContent dependencies.

```
cmake -G Ninja -S . -B ../build-client -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DBUILD_CARLA_CLIENT=ON -DBUILD_CARLA_SERVER=OFF -DBUILD_PYTHON_API=ON \
  -DBUILD_EXAMPLES=OFF -DBUILD_LIBCARLA_TESTS=OFF -DBUILD_CARLA_UNREAL=OFF \
  -DENABLE_ROS2=OFF -DPython3_EXECUTABLE=<python> -DPython_EXECUTABLE=<python> \
  -DCMAKE_CXX_FLAGS="-I<build>/_deps/zlib-build -include cstdint"
ninja -C ../build-client carla-client carla-python-api
```

Workarounds, none of which change repository sources:

- `BUILD_PYTHON_API=ON` is required even for the client, because `carla-client` links `Boost::python` unconditionally (`LibCarla/CMakeLists.txt`).
- The zlib include path is needed because libpng cannot find the generated `zconf.h`.
- `-include cstdint` is needed because `FileTransfer.h` lacks `#include <cstdint>` (rejected by GCC 15).
- The wheel step needs `build`, `setuptools`, `wheel` and `numpy` in the chosen interpreter (use a venv).

Smoke test after installing the wheel:

```
python -c "import carla; print(carla.VesselControl(rudder_angle=0.3, propeller_speed=12.0))"
```

## Unreal build

Not done. Requires the CARLA UE 5.3 fork, content and `UE5_BUILD_README.md` (`CarlaSetup.sh`). The default map is `Town10HD_Opt`.
