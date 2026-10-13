// Plain assert-based test for VesselRunner.h.
// Build: g++ -std=c++17 -Wall -Wextra -O2 VesselRunnerTest.cpp -o vessel_runner_test

#undef NDEBUG
#include "../VesselRunner.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace carla::vessel;

namespace {

constexpr double Pi = 3.14159265358979323846;

bool Close(double A, double B, double Tol = 1e-9) { return std::abs(A - B) <= Tol; }

PoseMeters Origin() { return {}; }

// A single Advance(0.02) must be bit-identical to stepping the shared model and
// actuators directly: the runner adds no dynamics of its own.
void TestMatchesDirectStepping()
{
  const ActuatorCommand Cmd{0.3, 11.15};
  VesselRunner Runner;
  Runner.Reset(Origin());
  Runner.SetControl(Cmd);

  MMG3DOF Model;
  VesselActuators Actuators;
  MMGState S;
  for (int i = 0; i < 3000; ++i)
  {
    const AdvanceResult R = Runner.Advance(0.02);
    assert(R.Substeps == 1 && !R.Truncated && !R.NonFinite);
    S = Model.Step(S, Actuators.Step(Cmd, 0.02), 0.02);
  }
  const MMGState &G = Runner.GetState();
  assert(G.North == S.North && G.East == S.East && G.Heading == S.Heading);
  assert(G.U == S.U && G.V == S.V && G.R == S.R);
  assert(Close(Runner.GetSimTime(), 60.0, 1e-9));
}

// Tick lengths that are not a multiple of 0.02 s must still integrate exactly
// that much time (no leftover carried between ticks).
void TestTimeSync()
{
  const double Ticks[] = {0.05, 0.01, 0.0333333, 0.1, 0.02, 0.2};
  for (double Dt : Ticks)
  {
    VesselRunner Runner;
    Runner.Reset(Origin());
    Runner.SetControl({0.0, 11.15});
    double Expected = 0.0;
    for (int i = 0; i < 200; ++i)
    {
      const AdvanceResult R = Runner.Advance(Dt);
      assert(Close(R.Advanced, Dt, 1e-12));
      assert(R.Substeps == static_cast<int>(std::ceil(Dt / 0.02 - 1e-9)));
      Expected += Dt;
    }
    assert(Close(Runner.GetSimTime(), Expected, 1e-9));
  }
}

// The result must be nearly independent of how the same time is split into ticks.
void TestTickSizeInvariance()
{
  VesselRunner Coarse, Fine;
  Coarse.Reset(Origin());
  Fine.Reset(Origin());
  Coarse.SetControl({0.4, 12.0});
  Fine.SetControl({0.4, 12.0});
  for (int i = 0; i < 40; ++i) Coarse.Advance(0.05);
  for (int i = 0; i < 100; ++i) Fine.Advance(0.02);
  assert(Close(Coarse.GetSimTime(), Fine.GetSimTime(), 1e-9));
  assert(Close(Coarse.GetState().North, Fine.GetState().North, 1e-4));
  assert(Close(Coarse.GetState().U, Fine.GetState().U, 1e-6));
}

void TestInvalidDt()
{
  VesselRunner Runner;
  Runner.Reset(Origin());
  const double Bad[] = {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::infinity()};
  for (double Dt : Bad)
  {
    const AdvanceResult R = Runner.Advance(Dt);
    assert(R.Substeps == 0 && R.Advanced == 0.0);
  }
  assert(Runner.GetSimTime() == 0.0);
}

void TestTruncation()
{
  VesselRunner Runner;
  Runner.Reset(Origin());
  const AdvanceResult R = Runner.Advance(100.0);
  assert(R.Truncated && R.Substeps == VesselRunner::DefaultMaxSubsteps);
  assert(Close(R.Advanced, VesselRunner::DefaultMaxSubsteps * 0.02, 1e-9));
}

// SetPose adopts a teleport but keeps velocities and actuators; Reset clears them.
void TestPoseAndReset()
{
  VesselRunner Runner;
  Runner.Reset({10.0, 20.0, 90.0});
  assert(Close(Runner.GetPose().X, 10.0) && Close(Runner.GetPose().Y, 20.0));
  assert(Close(Runner.GetPose().YawDeg, 90.0, 1e-9));

  Runner.SetControl({0.0, 11.15});
  for (int i = 0; i < 500; ++i) Runner.Advance(0.02);
  const double U = Runner.GetState().U;
  assert(U > 0.0);

  Runner.SetPose({-5.0, 7.0, -170.0});
  assert(Runner.GetState().U == U);
  assert(Close(Runner.GetPose().X, -5.0) && Close(Runner.GetPose().Y, 7.0));
  assert(Close(Runner.GetPose().YawDeg, -170.0, 1e-9));

  Runner.Reset({1.0, 2.0, 0.0});
  assert(Runner.GetState().U == 0.0 && Runner.GetSimTime() == 0.0);
}

// Heading is clockwise from north: heading 90 deg moves surge towards east, and a
// starboard (positive) rudder turns the heading to positive yaw (clockwise).
void TestFrameConversion()
{
  VesselRunner Runner;
  Runner.Reset({0.0, 0.0, 90.0});
  Runner.SetControl({0.0, 11.15});
  for (int i = 0; i < 3000; ++i) Runner.Advance(0.02);
  const WorldVelocity V = Runner.GetWorldVelocity();
  assert(V.Y > 0.1 && std::abs(V.X) < 1e-9);
  assert(Runner.GetPose().Y > 0.0);

  VesselRunner Turn;
  Turn.Reset({0.0, 0.0, 0.0});
  Turn.SetControl({0.5, 11.15});
  for (int i = 0; i < 3000; ++i) Turn.Advance(0.02);
  assert(Turn.GetPose().YawDeg > 0.0);
  assert(Turn.GetYawRateDegPerSec() > 0.0);
}

// Heading stays wrapped and the world velocity is rotation-consistent.
void TestWrapAndSpeed()
{
  VesselRunner Runner;
  Runner.Reset({0.0, 0.0, 170.0});
  Runner.SetControl({0.6, 11.15});
  for (int i = 0; i < 5000; ++i)
  {
    Runner.Advance(0.02);
    const double Yaw = Runner.GetPose().YawDeg;
    assert(Yaw > -180.0 - 1e-9 && Yaw <= 180.0 + 1e-9);
    const MMGState &S = Runner.GetState();
    const WorldVelocity V = Runner.GetWorldVelocity();
    assert(Close(std::hypot(V.X, V.Y), std::hypot(S.U, S.V), 1e-9));
  }
  (void)Pi;
}

// A NaN control must not poison the state or crash.
void TestNonFiniteControl()
{
  VesselRunner Runner;
  Runner.Reset(Origin());
  Runner.SetControl({std::numeric_limits<double>::quiet_NaN(), 11.15});
  const AdvanceResult R = Runner.Advance(0.02);
  const MMGState &S = Runner.GetState();
  assert(R.NonFinite || std::isfinite(S.North));
  assert(std::isfinite(S.U) && std::isfinite(S.V) && std::isfinite(S.R));
}

} // namespace

int main()
{
  TestMatchesDirectStepping();
  TestTimeSync();
  TestTickSizeInvariance();
  TestInvalidDt();
  TestTruncation();
  TestPoseAndReset();
  TestFrameConversion();
  TestWrapAndSpeed();
  TestNonFiniteControl();
  std::printf("ALL PASSED\n");
  return 0;
}
