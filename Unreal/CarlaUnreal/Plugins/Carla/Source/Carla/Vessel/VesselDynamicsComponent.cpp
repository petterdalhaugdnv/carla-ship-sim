// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Vessel/VesselDynamicsComponent.h"
#include "Carla.h"

#include <util/ue-header-guard-begin.h>
#include "GameFramework/Actor.h"
#include <util/ue-header-guard-end.h>

#include <cmath>

namespace {

  constexpr double MetersToCm = 100.0;
  constexpr double CmToMeters = 0.01;

  carla::vessel::PoseMeters ToPoseMeters(const FVector &Location, const FRotator &Rotation)
  {
    return {
        Location.X * CmToMeters,
        Location.Y * CmToMeters,
        static_cast<double>(Rotation.Yaw)};
  }

} // namespace

UVesselDynamicsComponent::UVesselDynamicsComponent()
{
  PrimaryComponentTick.bCanEverTick = true;
  PrimaryComponentTick.bStartWithTickEnabled = true;
  PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UVesselDynamicsComponent::BeginPlay()
{
  Super::BeginPlay();
  InitializeFromActorTransform();
}

void UVesselDynamicsComponent::ApplyControl(const FVesselControl &InControl)
{
  Control = InControl;
}

void UVesselDynamicsComponent::InitializeFromActorTransform()
{
  carla::vessel::PoseMeters Pose;

  const AActor *Owner = GetOwner();
  if (Owner != nullptr)
  {
    const FVector Location = Owner->GetActorLocation();
    const FRotator Rotation = Owner->GetActorRotation();
    Pose = ToPoseMeters(Location, Rotation);
    LastWrittenLocation = Location;
    LastWrittenRotation = Rotation;
    bInitialized = true;
  }
  Runner.Reset(Pose);
}

void UVesselDynamicsComponent::Reset()
{
  InitializeFromActorTransform();
}

FVector UVesselDynamicsComponent::GetVelocity() const
{
  const carla::vessel::WorldVelocity V = Runner.GetWorldVelocity();
  return FVector(V.X * MetersToCm, V.Y * MetersToCm, 0.0);
}

float UVesselDynamicsComponent::GetYawRateDegPerSec() const
{
  return static_cast<float>(Runner.GetYawRateDegPerSec());
}

void UVesselDynamicsComponent::TickComponent(
    float DeltaTime,
    ELevelTick TickType,
    FActorComponentTickFunction *ThisTickFunction)
{
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

  AActor *Owner = GetOwner();
  if (Owner == nullptr || DeltaTime <= 0.0f)
  {
    return;
  }
  if (!bInitialized)
  {
    InitializeFromActorTransform();
  }

  // The actor was moved externally (e.g. set_transform): adopt its pose but
  // keep the velocities.
  const FVector CurrentLocation = Owner->GetActorLocation();
  const FRotator CurrentRotation = Owner->GetActorRotation();
  if (!CurrentLocation.Equals(LastWrittenLocation, 0.01f) ||
      !CurrentRotation.Equals(LastWrittenRotation, 0.01f))
  {
    Runner.SetPose(ToPoseMeters(CurrentLocation, CurrentRotation));
  }

  Runner.SetControl({
      static_cast<double>(Control.RudderAngle),
      static_cast<double>(Control.PropellerSpeed)});

  const carla::vessel::AdvanceResult Result = Runner.Advance(static_cast<double>(DeltaTime));
  if (Result.NonFinite)
  {
    UE_LOG(LogCarla, Warning, TEXT("Vessel dynamics produced a non-finite state; resetting velocities."));
  }
  if (Result.Truncated)
  {
    UE_LOG(LogCarla, Warning, TEXT("Vessel dynamics step budget exceeded; simulated time was dropped."));
  }

  if (Result.Substeps > 0)
  {
    WritePoseToActor();
  }
}

void UVesselDynamicsComponent::WritePoseToActor()
{
  AActor *Owner = GetOwner();
  if (Owner == nullptr)
  {
    return;
  }
  const FVector OldLocation = Owner->GetActorLocation();
  const FRotator OldRotation = Owner->GetActorRotation();
  const carla::vessel::PoseMeters Pose = Runner.GetPose();
  const FVector NewLocation(
      Pose.X * MetersToCm,
      Pose.Y * MetersToCm,
      OldLocation.Z);
  const FRotator NewRotation(
      OldRotation.Pitch,
      static_cast<float>(Pose.YawDeg),
      OldRotation.Roll);
  Owner->SetActorLocationAndRotation(
      NewLocation, NewRotation, false, nullptr, ETeleportType::None);
  LastWrittenLocation = Owner->GetActorLocation();
  LastWrittenRotation = Owner->GetActorRotation();
}
