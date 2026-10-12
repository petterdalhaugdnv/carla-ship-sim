#!/usr/bin/env python3
"""Print the reference trajectory embedded in VesselDynamicsModelTest.cpp.

Uses the real mass_core MMG3DOF with the parameters of
data/vessels/core_reference_loop.yaml (needs Python >= 3.10 and PyYAML, no install).
Usage: python3 generate_mmg_reference.py [path/to/mass-core-main]

Scenario: 300 s at 0.02 s, initial surge 7.97 m/s, shaft 11.15 rad/s held, rudder 0
for the first 60 s, then a +35 deg (starboard) step. Actual (post-actuator) values
are fed directly. Samples are printed every 10 s.
"""
import math
import sys
from pathlib import Path

import yaml

root = (
    Path(sys.argv[1])
    if len(sys.argv) > 1
    else Path(__file__).resolve().parents[9] / "mass-core-main"
)
sys.path.insert(0, str(root / "src"))

from mass_core.models.common.kinematics import Pose2D, Velocity2D, VesselState  # noqa: E402
from mass_core.models.dynamics.mmg_3dof import (  # noqa: E402
    MMG3DOF,
    MMG3DOFInputs,
    MMG3DOFParameters,
)

DT = 0.02
SHAFT_RAD_S = 11.15
RUDDER_RAD = math.radians(35.0)
U0 = 7.97

data = yaml.safe_load((root / "data/vessels/core_reference_loop.yaml").read_text())
model = MMG3DOF(
    MMG3DOFParameters(**data["components"]["mmg_3dof_timed"]),
    VesselState(Pose2D(0.0, 0.0, 0.0), Velocity2D(U0, 0.0, 0.0)),
)

steps = int(round(300.0 / DT))
for k in range(1, steps + 1):
    t_start = (k - 1) * DT
    rudder = RUDDER_RAD if t_start >= 60.0 - 1e-9 else 0.0
    s = model.step(DT, MMG3DOFInputs(rudder, SHAFT_RAD_S))
    if k % 500 == 0:
        p, v = s.pose, s.velocity
        print(
            f"  {{{k * DT:.0f}.0, {{{p.north_m!r}, {p.east_m!r}, {p.heading_rad!r}, "
            f"{v.surge_mps!r}, {v.sway_mps!r}, {v.yaw_rate_radps!r}}}}},"
        )
