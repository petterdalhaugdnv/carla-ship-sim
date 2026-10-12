// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Vessel/CarlaVessel.h"
#include "Carla.h"

ACarlaVessel::ACarlaVessel()
{
  PrimaryActorTick.bCanEverTick = false;

  VesselRoot = CreateDefaultSubobject<USceneComponent>(TEXT("VesselRoot"));
  VesselRoot->SetMobility(EComponentMobility::Movable);
  RootComponent = VesselRoot;

  VesselDynamics = CreateDefaultSubobject<UVesselDynamicsComponent>(TEXT("VesselDynamics"));
}

void ACarlaVessel::ApplyVesselControl(const FVesselControl &Control)
{
  if (VesselDynamics != nullptr)
  {
    VesselDynamics->ApplyControl(Control);
  }
}

FVesselControl ACarlaVessel::GetVesselControl() const
{
  return VesselDynamics != nullptr ? VesselDynamics->GetControl() : FVesselControl();
}

FVector ACarlaVessel::GetVesselVelocity() const
{
  return VesselDynamics != nullptr ? VesselDynamics->GetVelocity() : FVector::ZeroVector;
}

FVector ACarlaVessel::GetVelocity() const
{
  return GetVesselVelocity();
}
