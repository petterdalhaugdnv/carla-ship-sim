// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Vessel/VesselControl.h"
#include "Carla/Vessel/VesselDynamicsComponent.h"

#include <util/ue-header-guard-begin.h>
#include "Components/SceneComponent.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <util/ue-header-guard-end.h>

#include "CarlaVessel.generated.h"

/// Surface vessel actor. It has no physics body: its pose is driven by
/// UVesselDynamicsComponent (MMG 3-DOF). Attach visual meshes in a Blueprint
/// subclass.
UCLASS()
class CARLA_API ACarlaVessel : public AActor
{
  GENERATED_BODY()

public:

  ACarlaVessel();

  /// Command held until the next call.
  UFUNCTION(BlueprintCallable, Category = "Vessel")
  void ApplyVesselControl(const FVesselControl &Control);

  UFUNCTION(BlueprintCallable, Category = "Vessel")
  FVesselControl GetVesselControl() const;

  /// World-frame velocity in cm/s.
  UFUNCTION(BlueprintCallable, Category = "Vessel")
  FVector GetVesselVelocity() const;

  /// Yaw rate in deg/s, positive clockwise seen from above.
  UFUNCTION(BlueprintCallable, Category = "Vessel")
  float GetVesselYawRateDegPerSec() const;

  UFUNCTION(BlueprintCallable, Category = "Vessel")
  UVesselDynamicsComponent *GetVesselDynamics() const { return VesselDynamics; }

  /// Overridden so that snapshots and sensors see the kinematic velocity.
  virtual FVector GetVelocity() const override;

private:

  UPROPERTY(VisibleAnywhere, Category = "Vessel")
  USceneComponent *VesselRoot = nullptr;

  UPROPERTY(VisibleAnywhere, Category = "Vessel")
  UVesselDynamicsComponent *VesselDynamics = nullptr;
};
