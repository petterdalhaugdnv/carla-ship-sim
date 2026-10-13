// Thin C ABI over carla::vessel::VesselRunner so non-C++ hosts (the Python
// lightweight backend) reuse the exact dynamics used by the Unreal component.
// Built on demand by vessel_runner.py; no dynamics live here.

#include "VesselRunner.h"

using carla::vessel::ActuatorCommand;
using carla::vessel::PoseMeters;
using carla::vessel::VesselRunner;

extern "C" {

void *vr_create() { return new VesselRunner(); }

void vr_destroy(void *Handle) { delete static_cast<VesselRunner *>(Handle); }

void vr_reset(void *Handle, double X, double Y, double YawDeg)
{
  static_cast<VesselRunner *>(Handle)->Reset(PoseMeters{X, Y, YawDeg});
}

void vr_set_pose(void *Handle, double X, double Y, double YawDeg)
{
  static_cast<VesselRunner *>(Handle)->SetPose(PoseMeters{X, Y, YawDeg});
}

void vr_set_control(void *Handle, double RudderRad, double PropellerRadS)
{
  static_cast<VesselRunner *>(Handle)->SetControl(ActuatorCommand{RudderRad, PropellerRadS});
}

// Bit 0: truncated, bit 1: non-finite step.
int vr_advance(void *Handle, double Dt)
{
  const auto Result = static_cast<VesselRunner *>(Handle)->Advance(Dt);
  return (Result.Truncated ? 1 : 0) | (Result.NonFinite ? 2 : 0);
}

// Out: north m, east m, yaw deg, v_north m/s, v_east m/s, yaw rate deg/s, sim time s.
void vr_get(void *Handle, double *Out)
{
  const auto *Runner = static_cast<VesselRunner *>(Handle);
  const auto Pose = Runner->GetPose();
  const auto Vel = Runner->GetWorldVelocity();
  Out[0] = Pose.X;
  Out[1] = Pose.Y;
  Out[2] = Pose.YawDeg;
  Out[3] = Vel.X;
  Out[4] = Vel.Y;
  Out[5] = Runner->GetYawRateDegPerSec();
  Out[6] = Runner->GetSimTime();
}

} // extern "C"
