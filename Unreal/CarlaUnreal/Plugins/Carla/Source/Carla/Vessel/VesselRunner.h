// Engine-free state advancement for a single vessel. Owns the MMG model, the
// actuators, the held control and the simulated time, so that the Unreal
// component and a lightweight (non-Unreal) backend advance a vessel identically.
// Only the standard library is used. No dynamics are implemented here: every step
// goes through MMG3DOF and VesselActuators from VesselDynamicsModel.h.
//
// Frame convention (shared with UVesselDynamicsComponent): the model works in NED
// SI units. The "pose in meters" accessors use the CARLA/UE world axes:
// x = north, y = east, yaw in degrees = heading clockwise from north.

#pragma once

#include "VesselDynamicsModel.h"

#include <cmath>

namespace carla {
namespace vessel {

struct AdvanceResult
{
  double Advanced = 0;     // simulated seconds actually integrated
  int Substeps = 0;
  bool Truncated = false;  // dt exceeded the substep budget; time was dropped
  bool NonFinite = false;  // a step produced NaN/Inf; velocities were zeroed
};

struct PoseMeters
{
  double X = 0;      // north, m
  double Y = 0;      // east, m
  double YawDeg = 0; // heading, degrees clockwise from north
};

struct WorldVelocity
{
  double X = 0; // north, m/s
  double Y = 0; // east, m/s
};

class VesselRunner
{
public:
  static constexpr double DefaultMaxSubstep = 0.02;
  static constexpr int DefaultMaxSubsteps = 500;

  explicit VesselRunner(
      const MMGParameters &Params = MMGParameters::KVLCC2(),
      double InMaxSubstep = DefaultMaxSubstep,
      int InMaxSubsteps = DefaultMaxSubsteps)
    : Model(Params),
      Actuators(Params),
      MaxSubstep(InMaxSubstep),
      MaxSubsteps(InMaxSubsteps) {}

  // Place the vessel at rest with amidships rudder and a stopped shaft.
  void Reset(const PoseMeters &Pose)
  {
    State = MMGState{};
    Actuators.Reset();
    SimTime = 0.0;
    SetPose(Pose);
  }

  // Move the vessel without touching velocities or actuators (external teleport).
  void SetPose(const PoseMeters &Pose)
  {
    State.North = Pose.X;
    State.East = Pose.Y;
    State.Heading = MMG3DOF::WrapPi(Pose.YawDeg * (Pi() / 180.0));
  }

  // Command held (zero-order hold) until the next call.
  void SetControl(const ActuatorCommand &Command) { Control = Command; }

  // Advance by Dt seconds using equal substeps no longer than MaxSubstep, so the
  // integrated time always matches Dt (when the budget allows). A step of exactly
  // MaxSubstep or less is a single step.
  AdvanceResult Advance(double Dt)
  {
    AdvanceResult Result;
    if (!(Dt > 0.0) || !std::isfinite(Dt))
    {
      return Result;
    }

    int Count = static_cast<int>(std::ceil(Dt / MaxSubstep - 1e-9));
    if (Count < 1)
    {
      Count = 1;
    }
    double H = Dt / Count;
    if (Count > MaxSubsteps)
    {
      Count = MaxSubsteps;
      H = MaxSubstep;
      Result.Truncated = true;
    }

    for (int i = 0; i < Count; ++i)
    {
      const MMGInputs Inputs = Actuators.Step(Control, H);
      const MMGState Next = Model.Step(State, Inputs, H);
      if (IsFinite(Next))
      {
        State = Next;
      }
      else
      {
        State.U = State.V = State.R = 0.0;
        Result.NonFinite = true;
      }
      SimTime += H;
      Result.Advanced += H;
      ++Result.Substeps;
    }
    return Result;
  }

  const MMGState &GetState() const { return State; }
  const ActuatorCommand &GetControl() const { return Control; }

  // Total simulated seconds since Reset.
  double GetSimTime() const { return SimTime; }

  PoseMeters GetPose() const
  {
    return {State.North, State.East, State.Heading * (180.0 / Pi())};
  }

  // Body-frame surge/sway rotated into the world axes, m/s.
  WorldVelocity GetWorldVelocity() const
  {
    const double CosH = std::cos(State.Heading);
    const double SinH = std::sin(State.Heading);
    return {State.U * CosH - State.V * SinH, State.U * SinH + State.V * CosH};
  }

  // Positive clockwise seen from above, same sense as the yaw angle.
  double GetYawRateDegPerSec() const { return State.R * (180.0 / Pi()); }

private:
  static constexpr double Pi() { return 3.14159265358979323846; }

  static bool IsFinite(const MMGState &S)
  {
    return std::isfinite(S.North) && std::isfinite(S.East) &&
           std::isfinite(S.Heading) && std::isfinite(S.U) &&
           std::isfinite(S.V) && std::isfinite(S.R);
  }

  MMG3DOF Model;
  VesselActuators Actuators;
  MMGState State = {};
  ActuatorCommand Control = {};
  double SimTime = 0.0;
  double MaxSubstep;
  int MaxSubsteps;
};

} // namespace vessel
} // namespace carla
