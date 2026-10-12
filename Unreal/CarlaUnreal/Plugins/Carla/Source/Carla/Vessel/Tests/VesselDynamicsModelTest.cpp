// Plain assert-based test for VesselDynamicsModel.h.
// Build: g++ -std=c++17 -Wall -Wextra -O2 VesselDynamicsModelTest.cpp -o vessel_dynamics_test
// Reference values come from generate_mmg_reference.py (mass_core MMG3DOF).

#undef NDEBUG
#include "../VesselDynamicsModel.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace carla::vessel;

namespace {

constexpr double Dt = 0.02;
constexpr double Pi = 3.14159265358979323846;

struct RefRow
{
  double T;
  std::array<double, 6> S; // North, East, Heading, U, V, R
};

// 300 s, U0 = 7.97 m/s, shaft 11.15 rad/s, rudder 0 until t = 60 s then +35 deg.
const RefRow Reference[] = {
  {10.0, {79.72157295813595, 0.0, 0.0, 7.974283561524999, 0.0, 0.0}},
  {20.0, {159.4850638071885, 0.0, 0.0, 7.978384885047428, 0.0, 0.0}},
  {30.0, {239.28868882917732, 0.0, 0.0, 7.982311649023552, 0.0, 0.0}},
  {40.0, {319.1307394936164, 0.0, 0.0, 7.986071214631998, 0.0, 0.0}},
  {50.0, {399.00957934804705, 0.0, 0.0, 7.989670638358861, 0.0, 0.0}},
  {60.0, {478.9236410321377, 0.0, 0.0, 7.993116684128119, 0.0, 0.0}},
  {70.0, {558.5444383499673, -0.43494059129201995, 0.015610661332116853, 7.929836341598366, -0.1819836068833964, 0.002952478647843574}},
  {80.0, {637.5418110093248, -0.6519429091700364, 0.05674574593069005, 7.854779977086577, -0.40804922144316974, 0.005196490025227153}},
  {90.0, {715.7626318183557, 0.7642512085139264, 0.11836265356892889, 7.749551506927915, -0.6539130316048408, 0.00707879796927163}},
  {100.0, {792.8261789597314, 5.051440585292295, 0.19739877193254382, 7.601279247662042, -0.8985946817587899, 0.008680404314322808}},
  {110.0, {868.1210696229549, 13.277643424687184, 0.29091347353233893, 7.4050693076218135, -1.1216987811404173, 0.009964546576283312}},
  {120.0, {940.8521957783599, 26.258995763433806, 0.39545893958485406, 7.165429772420336, -1.3064539146162129, 0.01088143289921941}},
  {130.0, {1010.1400025402576, 44.46308467870716, 0.507315939444946, 6.894431041158907, -1.4441145346574262, 0.011432554534230324}},
  {140.0, {1075.1440937045268, 67.96797613406898, 0.6230860596961474, 6.607235475810389, -1.5350854169733315, 0.011677367970258139}},
  {150.0, {1135.1660622234854, 96.5041429177325, 0.7401311144338775, 6.317755537670022, -1.586174547103489, 0.01170229187664802}},
  {160.0, {1189.7024223613114, 129.55272218131304, 0.8566739070056101, 6.036350870244335, -1.6066753162129894, 0.011589111289888002}},
  {170.0, {1238.4487137949507, 166.45440751342906, 0.9716628319809016, 5.769523548556794, -1.6055364467412314, 0.011400065412354169}},
  {180.0, {1281.2734059226987, 206.4981508357907, 1.0845623518145577, 5.5206546359961495, -1.5900788544747573, 0.011176618281750294}},
  {190.0, {1318.1806414357632, 248.98052710308303, 1.1951658030215304, 5.290973958405922, -1.5657615684091766, 0.010943991922893817}},
  {200.0, {1349.2737775463231, 293.2394868255062, 1.3034597893358393, 5.080381916353707, -1.5364265125325696, 0.010716422518876723}},
  {210.0, {1374.724930938489, 338.6700840421015, 1.4095361675330238, 4.888032730264867, -1.504672392904371, 0.010501296318474322}},
  {220.0, {1394.751654134929, 384.729091088502, 1.513538542829692, 4.712710095320511, -1.4722025852244487, 0.010301944093503723}},
  {230.0, {1409.6000251609378, 430.9334659987392, 1.6156311781500325, 4.553054103409301, -1.440099877224625, 0.010119390267071277}},
  {240.0, {1419.532860603582, 476.85583420970306, 1.7159816358100177, 4.40769112590281, -1.409027170781915, 0.00995340227314849}},
  {250.0, {1424.8217690557597, 522.1188435036869, 1.8147515524851177, 4.275303684053111, -1.379368063207461, 0.009803103774192733}},
  {260.0, {1425.7419723478122, 566.3894118788869, 1.912092122603028, 4.154664520280363, -1.351322977593825, 0.009667325813117836}},
  {270.0, {1422.569066655427, 609.3733820242153, 2.0081422525109662, 4.044649961796327, -1.3249739583214637, 0.009544803540557468}},
  {280.0, {1415.5771111749807, 650.8108051486153, 2.1030281914540407, 3.944241719151751, -1.3003279444949591, 0.009434282982473795}},
  {290.0, {1405.0376025893702, 690.4719169308966, 2.1968639466722397, 3.8525225502628055, -1.2773454737728311, 0.009334575869884329}},
  {300.0, {1391.219021419254, 728.1537851411158, 2.2897520845170325, 3.768668959354603, -1.2559596025052024, 0.009244584803639143}},
};

bool Close(double A, double B, double Tol = 1e-6)
{
  return std::abs(A - B) <= Tol * std::max(1.0, std::abs(B));
}

void TestReference()
{
  MMG3DOF Model;
  MMGState S;
  S.U = 7.97;
  const int Steps = 15000;
  const int Every = 500;
  size_t Row = 0;
  double Worst = 0.0;
  for (int k = 1; k <= Steps; ++k)
  {
    const double TStart = (k - 1) * Dt;
    MMGInputs In;
    In.PropellerSpeedRadS = 11.15;
    In.RudderAngleRad = TStart >= 60.0 - 1e-9 ? 35.0 * Pi / 180.0 : 0.0;
    S = Model.Step(S, In, Dt);
    if (k % Every == 0)
    {
      const auto &Ref = Reference[Row++];
      const double Got[6] = {S.North, S.East, S.Heading, S.U, S.V, S.R};
      for (int i = 0; i < 6; ++i)
      {
        Worst = std::max(Worst, std::abs(Got[i] - Ref.S[i]) / std::max(1.0, std::abs(Ref.S[i])));
        assert(Close(Got[i], Ref.S[i]));
      }
    }
  }
  assert(Row == sizeof(Reference) / sizeof(Reference[0]));
  std::printf("reference: %zu samples ok, worst scaled error %.3e\n", Row, Worst);
}

void TestRest()
{
  MMG3DOF Model;
  MMGState S;
  for (int k = 0; k < 5000; ++k)
  {
    S = Model.Step(S, MMGInputs{}, Dt);
  }
  assert(S.North == 0 && S.East == 0 && S.Heading == 0);
  assert(S.U == 0 && S.V == 0 && S.R == 0);
  std::printf("rest: ok\n");
}

void TestHeadingWrap()
{
  MMG3DOF Model;
  MMGState S;
  S.U = 7.97;
  S.Heading = 3.14;
  S.R = 0.02;
  MMGInputs In;
  In.PropellerSpeedRadS = 11.15;
  In.RudderAngleRad = 0.3;
  bool Wrapped = false;
  for (int k = 0; k < 2000; ++k)
  {
    const double Prev = S.Heading;
    S = Model.Step(S, In, Dt);
    assert(S.Heading >= -Pi && S.Heading <= Pi);
    if (Prev > 3.0 && S.Heading < -3.0)
    {
      Wrapped = true;
    }
  }
  assert(Wrapped);
  assert(Close(MMG3DOF::WrapPi(3.0 * Pi), Pi));
  assert(Close(MMG3DOF::WrapPi(-Pi), Pi));
  assert(Close(MMG3DOF::WrapPi(2.5 * Pi), 0.5 * Pi));
  std::printf("heading wrap: ok\n");
}

void TestRudderRate()
{
  const MMGParameters P = MMGParameters::KVLCC2();
  VesselActuators A(P);
  ActuatorCommand Cmd;
  Cmd.RudderAngleRad = 10.0; // beyond the stops
  double T = 0.0;
  double Prev = 0.0;
  for (int k = 1; k <= 500; ++k)
  {
    const MMGInputs Out = A.Step(Cmd, Dt);
    T = k * Dt;
    assert(Out.RudderAngleRad - Prev <= P.MaxRudderRateRadS * Dt + 1e-15);
    assert(Out.RudderAngleRad <= 0.6108652381980153 + 1e-15);
    Prev = Out.RudderAngleRad;
  }
  assert(Close(Prev, 0.040516571921296936 * T, 1e-9));
  assert(Prev < 0.6108652381980153);
  // Reach the +-35 deg stop and clamp there; reverse command slews back at the same rate.
  for (int k = 0; k < 2000; ++k)
  {
    Prev = A.Step(Cmd, Dt).RudderAngleRad;
  }
  assert(Prev == 0.6108652381980153);
  Cmd.RudderAngleRad = -10.0;
  const double Back = A.Step(Cmd, Dt).RudderAngleRad;
  assert(Close(Prev - Back, 0.040516571921296936 * Dt, 1e-9));
  A.Reset();
  assert(A.Step(ActuatorCommand{}, Dt).RudderAngleRad == 0.0);
  std::printf("rudder rate limit: ok\n");
}

void TestShaftLag()
{
  VesselActuators A;
  ActuatorCommand Cmd;
  Cmd.PropellerSpeedRadS = 10.0;
  MMGInputs Out;
  for (int k = 1; k <= 2500; ++k) // 50 s = one time constant
  {
    Out = A.Step(Cmd, Dt);
  }
  assert(Close(Out.PropellerSpeedRadS, 10.0 * (1.0 - std::exp(-1.0)), 1e-9));
  Cmd.PropellerSpeedRadS = 100.0; // clamped to max
  for (int k = 0; k < 100000; ++k)
  {
    Out = A.Step(Cmd, Dt);
  }
  assert(Out.PropellerSpeedRadS <= 25.132741228718345);
  assert(Close(Out.PropellerSpeedRadS, 25.132741228718345, 1e-9));
  Cmd.PropellerSpeedRadS = -5.0; // no astern: clamped to 0
  for (int k = 0; k < 100000; ++k)
  {
    Out = A.Step(Cmd, Dt);
  }
  assert(Out.PropellerSpeedRadS >= 0.0 && Out.PropellerSpeedRadS < 1e-9);
  std::printf("shaft lag: ok\n");
}

} // namespace

int main()
{
  TestReference();
  TestRest();
  TestHeadingWrap();
  TestRudderRate();
  TestShaftLag();
  std::printf("ALL PASSED\n");
  return 0;
}
