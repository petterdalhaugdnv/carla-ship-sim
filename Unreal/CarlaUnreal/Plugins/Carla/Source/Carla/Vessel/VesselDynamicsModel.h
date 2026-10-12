// Header-only, engine-free C++17 port of the mass_core 3-DOF MMG ship model
// (models/dynamics/mmg_3dof.py) and its rudder / shaft actuators
// (models/actuators/{rudder,shaft}_actuator.py). Only the standard library is used.
//
// Frames: NED world, body surge forward, sway positive to starboard, yaw rate and
// rudder angle positive to starboard. Heading is clockwise from north, wrapped to
// (-pi, pi]. Propeller speed is in rad/s at the API; the thrust law converts to rev/s.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace carla {
namespace vessel {

struct MMGState
{
  double North = 0;   // m
  double East = 0;    // m
  double Heading = 0; // rad
  double U = 0;       // m/s surge
  double V = 0;       // m/s sway, starboard +
  double R = 0;       // rad/s yaw rate
};

// Actual (post-actuator) values.
struct MMGInputs
{
  double RudderAngleRad = 0;
  double PropellerSpeedRadS = 0;
};

// Defaults are the KVLCC2 values of mass-core data/vessels/core_reference_loop.yaml.
struct MMGParameters
{
  double LengthM = 320.0;
  double BeamM = 58.05714285714286;
  double DraftM = 21.02857142857143;
  double XGM = 11.428571428571429;
  double MassKg = 320204501.45772594;
  double YawInertiaKgm2 = 2049308809329.446;
  double AddedMassXKg = 24278747.428571425;
  double AddedMassYKg = 246098212.57142857;
  double AddedInertiaZKgm2 = 1243071868342.857;

  double PropellerDiameterM = 9.874285714285714;
  double K0 = 0.2931;
  double K1 = -0.2753;
  double K2 = -0.1385;
  double ThrustDeduction = 0.22;
  double WakeFractionStraight = 0.4;
  double PropellerPositionRatio = -0.69;
  double PropellerRaceCorrection = 0.5;

  double RudderAreaM2 = 112.64000000000001;
  double RudderSpanRatio = 0.6260869565217392;
  double RudderLiftGradient = 2.747;
  double SteeringResistanceDeduction = 0.387;
  double RudderPositionM = -160.0;
  double RudderForceIncreaseFactor = 0.312;
  double RudderForcePositionM = -148.48000000000002;
  double FlowStraighteningNegative = 0.395;
  double FlowStraighteningPositive = 0.64;
  double RudderPositionRatio = -0.71;
  double WakeRatioRudderPropeller = 1.09;

  double ResistanceCoefficient = 0.022;
  double XVV = -0.04;
  double XVR = 0.002;
  double XRR = 0.011;
  double XVVVV = 0.771;
  double YV = -0.315;
  double YR = 0.083;
  double YVVV = -1.607;
  double YVVR = 0.379;
  double YVRR = -0.391;
  double YRRR = 0.008;
  double NV = -0.137;
  double NR = -0.049;
  double NVVV = -0.03;
  double NVVR = -0.294;
  double NVRR = 0.055;
  double NRRR = -0.013;

  double RhoKgM3 = 1025.0;

  // Actuator limits (rudder_actuator / shaft_actuator entries of the same file).
  double MaxRudderAngleRad = 0.6108652381980153;
  double MaxRudderRateRadS = 0.040516571921296936;
  double ShaftTimeConstantS = 50.0;
  double MinShaftSpeedRadS = 0.0;
  double MaxShaftSpeedRadS = 25.132741228718345;

  static MMGParameters KVLCC2() { return MMGParameters(); }
};

class MMG3DOF
{
public:
  explicit MMG3DOF(const MMGParameters &Params = MMGParameters::KVLCC2())
    : P(Params) {}

  // One RK4 step with the inputs held constant over Dt.
  MMGState Step(const MMGState &S, const MMGInputs &In, double Dt) const
  {
    const double Rudder = In.RudderAngleRad;
    const double ShaftRps = In.PropellerSpeedRadS / (2.0 * Pi());

    const Vec X = {S.North, S.East, S.Heading, S.U, S.V, S.R};
    const Vec K1 = Derivative(X, Rudder, ShaftRps);
    const Vec K2 = Derivative(Advance(X, K1, 0.5 * Dt), Rudder, ShaftRps);
    const Vec K3 = Derivative(Advance(X, K2, 0.5 * Dt), Rudder, ShaftRps);
    const Vec K4 = Derivative(Advance(X, K3, Dt), Rudder, ShaftRps);

    Vec N;
    for (size_t i = 0; i < 6; ++i)
    {
      N[i] = X[i] + (Dt / 6.0) * (K1[i] + 2.0 * K2[i] + 2.0 * K3[i] + K4[i]);
    }
    MMGState Out;
    Out.North = N[0];
    Out.East = N[1];
    Out.Heading = WrapPi(N[2]);
    Out.U = N[3];
    Out.V = N[4];
    Out.R = N[5];
    return Out;
  }

  // Wrap to (-pi, pi].
  static double WrapPi(double Angle)
  {
    const double W = std::remainder(Angle, 2.0 * Pi());
    return W == -Pi() ? Pi() : W;
  }

private:
  using Vec = std::array<double, 6>;

  struct Loads { double X, Y, N; };

  struct Flow
  {
    double Speed, Drift, VDash, RDash, Wake;
  };

  static constexpr double Pi() { return 3.14159265358979323846; }
  static constexpr double MinSpeed = 1.0e-6;
  static constexpr double MinShaftRps = 1.0e-9;

  MMGParameters P;

  static Vec Advance(const Vec &X, const Vec &D, double H)
  {
    Vec Out;
    for (size_t i = 0; i < 6; ++i)
    {
      Out[i] = X[i] + D[i] * H;
    }
    return Out;
  }

  Flow ComputeFlow(double U, double V, double R) const
  {
    Flow F;
    const double Lateral = V - R * P.XGM;
    F.Speed = std::hypot(U, Lateral);
    if (F.Speed < MinSpeed)
    {
      F.Drift = 0.0;
      F.VDash = 0.0;
      F.RDash = 0.0;
    }
    else
    {
      F.Drift = std::asin(std::max(-1.0, std::min(1.0, -Lateral / F.Speed)));
      F.VDash = V / F.Speed;
      F.RDash = R * P.LengthM / F.Speed;
    }
    const double Arg = F.Drift - P.PropellerPositionRatio * F.RDash;
    const double Wake = P.WakeFractionStraight * std::exp(-4.0 * Arg * Arg);
    F.Wake = std::max(0.0, std::min(0.95, Wake));
    return F;
  }

  double AdvanceRatio(const Flow &F, double U, double ShaftRps) const
  {
    if (std::abs(ShaftRps) < MinShaftRps)
    {
      return 0.0;
    }
    return (1.0 - F.Wake) * U / (ShaftRps * P.PropellerDiameterM);
  }

  double ThrustCoefficient(const Flow &F, double U, double ShaftRps) const
  {
    const double J = AdvanceRatio(F, U, ShaftRps);
    return P.K0 + P.K1 * J + P.K2 * J * J;
  }

  Loads HullForce(const Flow &F) const
  {
    const double Pressure = 0.5 * P.RhoKgM3 * P.LengthM * P.DraftM * F.Speed * F.Speed;
    const double V = F.VDash;
    const double R = F.RDash;
    Loads L;
    L.X = Pressure * (-P.ResistanceCoefficient + P.XVV * V * V + P.XVR * V * R +
                      P.XRR * R * R + P.XVVVV * V * V * V * V);
    L.Y = Pressure * (P.YV * V + P.YR * R + P.YVVV * V * V * V + P.YVVR * V * V * R +
                      P.YVRR * V * R * R + P.YRRR * R * R * R);
    L.N = Pressure * P.LengthM *
          (P.NV * V + P.NR * R + P.NVVV * V * V * V + P.NVVR * V * V * R +
           P.NVRR * V * R * R + P.NRRR * R * R * R);
    return L;
  }

  double PropellerForceX(const Flow &F, double U, double ShaftRps) const
  {
    const double D2 = P.PropellerDiameterM * P.PropellerDiameterM;
    return (1.0 - P.ThrustDeduction) * P.RhoKgM3 * ThrustCoefficient(F, U, ShaftRps) *
           ShaftRps * std::abs(ShaftRps) * D2 * D2;
  }

  double RudderAxialInflow(double U, double Wake, double J, double Kt, double ShaftRps) const
  {
    const double Span = P.RudderSpanRatio;
    if (std::abs(ShaftRps) < MinShaftRps || Kt <= 0.0)
    {
      return P.WakeRatioRudderPropeller * (1.0 - Wake) * U;
    }
    if (std::abs(J) < 1.0e-4)
    {
      return P.WakeRatioRudderPropeller * P.PropellerRaceCorrection * ShaftRps *
             P.PropellerDiameterM * std::sqrt(8.0 * Span * Kt / Pi());
    }
    const double RaceFactor = 1.0 + 8.0 * Kt / (Pi() * J * J);
    const double SpeedUp =
        1.0 + P.PropellerRaceCorrection * (std::sqrt(std::max(0.0, RaceFactor)) - 1.0);
    return U * (1.0 - Wake) * P.WakeRatioRudderPropeller *
           std::sqrt(Span * SpeedUp * SpeedUp + (1.0 - Span));
  }

  Loads RudderForce(const Flow &F, double U, double Rudder, double ShaftRps) const
  {
    const double Kt = ThrustCoefficient(F, U, ShaftRps);
    const double J = AdvanceRatio(F, U, ShaftRps);

    const double RudderDrift = F.Drift - P.RudderPositionRatio * F.RDash;
    const double Straightening =
        RudderDrift < 0.0 ? P.FlowStraighteningNegative : P.FlowStraighteningPositive;
    const double Lateral = F.Speed * Straightening * RudderDrift;
    const double Axial = RudderAxialInflow(U, F.Wake, J, Kt, ShaftRps);
    const double SpeedSq = Axial * Axial + Lateral * Lateral;
    const double Attack = Rudder - std::atan2(Lateral, Axial);
    const double Normal = 0.5 * P.RudderAreaM2 * P.RhoKgM3 * P.RudderLiftGradient *
                          SpeedSq * std::sin(Attack);
    Loads L;
    L.X = -(1.0 - P.SteeringResistanceDeduction) * Normal * std::sin(Rudder);
    L.Y = -(1.0 + P.RudderForceIncreaseFactor) * Normal * std::cos(Rudder);
    L.N = -(P.RudderPositionM + P.RudderForceIncreaseFactor * P.RudderForcePositionM) *
          Normal * std::cos(Rudder);
    return L;
  }

  Vec Derivative(const Vec &X, double Rudder, double ShaftRps) const
  {
    const double Heading = X[2], U = X[3], V = X[4], R = X[5];
    const Flow F = ComputeFlow(U, V, R);
    const Loads Hull = HullForce(F);
    const Loads Rud = RudderForce(F, U, Rudder, ShaftRps);
    const double Surge = Hull.X + PropellerForceX(F, U, ShaftRps) + Rud.X;
    const double Sway = Hull.Y + Rud.Y;
    const double Yaw = Hull.N + Rud.N;

    const double M = P.MassKg;
    const double XG = P.XGM;
    const double MX = M + P.AddedMassXKg;
    const double MY = M + P.AddedMassYKg;
    const double Iz = P.YawInertiaKgm2 + P.AddedInertiaZKgm2 + XG * XG * M;

    const double UDot = (Surge + MY * V * R + XG * M * R * R) / MX;
    const double Den = Iz * MY - XG * XG * M * M;
    const double VDot =
        (XG * XG * M * M * U * R - Yaw * XG * M + (Sway - MX * U * R) * Iz) / Den;
    const double RDot = (Yaw - XG * M * (VDot + U * R)) / Iz;

    return {U * std::cos(Heading) - V * std::sin(Heading),
            U * std::sin(Heading) + V * std::cos(Heading), R, UDot, VDot, RDot};
  }
};

struct ActuatorCommand
{
  double RudderAngleRad = 0;
  double PropellerSpeedRadS = 0;
};

// Rudder: clamp then rate-limited slew. Shaft: clamp then exact first-order lag.
// Both start at zero (amidships, stopped).
class VesselActuators
{
public:
  explicit VesselActuators(const MMGParameters &Params = MMGParameters::KVLCC2())
    : P(Params) {}

  void Reset()
  {
    Rudder = 0.0;
    Shaft = 0.0;
  }

  MMGInputs Step(const ActuatorCommand &Cmd, double Dt)
  {
    const double Limit = P.MaxRudderAngleRad;
    const double Target = std::min(Limit, std::max(-Limit, Cmd.RudderAngleRad));
    const double Travel = P.MaxRudderRateRadS * Dt;
    const double Change = Target - Rudder;
    Rudder = std::abs(Change) <= Travel ? Target : Rudder + std::copysign(Travel, Change);

    const double ShaftTarget =
        std::min(P.MaxShaftSpeedRadS, std::max(P.MinShaftSpeedRadS, Cmd.PropellerSpeedRadS));
    const double Decay = std::exp(-Dt / P.ShaftTimeConstantS);
    Shaft = std::min(P.MaxShaftSpeedRadS,
                     std::max(P.MinShaftSpeedRadS, ShaftTarget + (Shaft - ShaftTarget) * Decay));

    MMGInputs Out;
    Out.RudderAngleRad = Rudder;
    Out.PropellerSpeedRadS = Shaft;
    return Out;
  }

private:
  MMGParameters P;
  double Rudder = 0.0;
  double Shaft = 0.0;
};

} // namespace vessel
} // namespace carla
