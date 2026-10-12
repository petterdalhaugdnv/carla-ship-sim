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
  constexpr double Pi = 3.14159265358979323846;
  constexpr double RadToDeg = 180.0 / Pi;
  constexpr double DegToRad = Pi / 180.0;
  constexpr double Epsilon = 1e-9;
  // Bound the catch-up work after a long hitch.
  constexpr int32 MaxSubstepsPerTick = 500;

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
  State = carla::vessel::MMGState{};
  Actuators.Reset();
  Accumulator = 0.0;

  const AActor *Owner = GetOwner();
  if (Owner != nullptr)
  {
    const FVector Location = Owner->GetActorLocation();
    const FRotator Rotation = Owner->GetActorRotation();
    State.North = Location.X * CmToMeters;
    State.East = Location.Y * CmToMeters;
    State.Heading = static_cast<double>(Rotation.Yaw) * DegToRad;
    LastWrittenLocation = Location;
    LastWrittenRotation = Rotation;
    bInitialized = true;
  }
}

void UVesselDynamicsComponent::Reset()
{
  InitializeFromActorTransform();
}

FVector UVesselDynamicsComponent::GetVelocity() const
{
  const double CosH = std::cos(State.Heading);
  const double SinH = std::sin(State.Heading);
  const double NorthRate = State.U * CosH - State.V * SinH;
  const double EastRate = State.U * SinH + State.V * CosH;
  return FVector(NorthRate * MetersToCm, EastRate * MetersToCm, 0.0);
}

float UVesselDynamicsComponent::GetYawRateDegPerSec() const
{
  return static_cast<float>(State.R * RadToDeg);
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
    State.North = CurrentLocation.X * CmToMeters;
    State.East = CurrentLocation.Y * CmToMeters;
    State.Heading = static_cast<double>(CurrentRotation.Yaw) * DegToRad;
  }

  Accumulator += static_cast<double>(DeltaTime);

  const carla::vessel::ActuatorCommand Command{
      static_cast<double>(Control.RudderAngle),
      static_cast<double>(Control.PropellerSpeed)};

  int32 Steps = 0;
  while (Accumulator + Epsilon >= FixedSubstep && Steps < MaxSubstepsPerTick)
  {
    const carla::vessel::MMGInputs Inputs = Actuators.Step(Command, FixedSubstep);
    const carla::vessel::MMGState Next = Model.Step(State, Inputs, FixedSubstep);
    if (std::isfinite(Next.North) && std::isfinite(Next.East) &&
        std::isfinite(Next.Heading) && std::isfinite(Next.U) &&
        std::isfinite(Next.V) && std::isfinite(Next.R))
    {
      State = Next;
    }
    else
    {
      UE_LOG(LogCarla, Warning, TEXT("Vessel dynamics produced a non-finite state; resetting velocities."));
      State.U = State.V = State.R = 0.0;
    }
    Accumulator -= FixedSubstep;
    ++Steps;
  }
  if (Steps == MaxSubstepsPerTick)
  {
    Accumulator = 0.0;
  }
  if (Accumulator < 0.0)
  {
    Accumulator = 0.0;
  }

  if (Steps > 0)
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
  const FVector NewLocation(
      State.North * MetersToCm,
      State.East * MetersToCm,
      OldLocation.Z);
  const FRotator NewRotation(
      OldRotation.Pitch,
      static_cast<float>(State.Heading * RadToDeg),
      OldRotation.Roll);
  Owner->SetActorLocationAndRotation(
      NewLocation, NewRotation, false, nullptr, ETeleportType::None);
  LastWrittenLocation = Owner->GetActorLocation();
  LastWrittenRotation = Owner->GetActorRotation();
}
