// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Vessel/VesselControl.h"

#include <util/disable-ue4-macros.h>
#include "Carla/Vessel/VesselRunner.h"
#include <util/enable-ue4-macros.h>

#include <util/ue-header-guard-begin.h>
#include "Components/ActorComponent.h"
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

#include "VesselDynamicsComponent.generated.h"

/// Kinematic (non-physics) vessel driver. Advances an engine-free
/// carla::vessel::VesselRunner (MMG 3-DOF) by the tick's DeltaTime, in equal
/// substeps of at most 0.02 s, and writes the resulting planar pose to the owning
/// actor.
///
/// Frame mapping: NED north -> UE +X, NED east -> UE +Y, heading (clockwise
/// from north) -> UE yaw in degrees. Model units are SI, UE units are cm. The
/// actor Z, pitch and roll are left untouched (flat water prototype).
UCLASS(ClassGroup = (Carla), meta = (BlueprintSpawnableComponent))
class CARLA_API UVesselDynamicsComponent : public UActorComponent
{
  GENERATED_BODY()

public:

  UVesselDynamicsComponent();

  virtual void BeginPlay() override;

  virtual void TickComponent(
      float DeltaTime,
      ELevelTick TickType,
      FActorComponentTickFunction *ThisTickFunction) override;

  /// Set the command held (zero-order hold) until the next call.
  UFUNCTION(BlueprintCallable, Category = "Vessel")
  void ApplyControl(const FVesselControl &Control);

  UFUNCTION(BlueprintCallable, Category = "Vessel")
  FVesselControl GetControl() const { return Control; }

  /// Reset the state from the owner's transform with zero velocities and
  /// actuators.
  UFUNCTION(BlueprintCallable, Category = "Vessel")
  void InitializeFromActorTransform();

  /// Same as InitializeFromActorTransform.
  UFUNCTION(BlueprintCallable, Category = "Vessel")
  void Reset();

  const carla::vessel::MMGState &GetState() const { return Runner.GetState(); }

  /// Simulated seconds integrated since the last reset.
  double GetSimTime() const { return Runner.GetSimTime(); }

  /// World-frame velocity in cm/s.
  UFUNCTION(BlueprintCallable, Category = "Vessel")
  FVector GetVelocity() const;

  UFUNCTION(BlueprintCallable, Category = "Vessel")
  float GetYawRateDegPerSec() const;

private:

  void WritePoseToActor();

  carla::vessel::VesselRunner Runner;

  FVesselControl Control;

  bool bInitialized = false;

  FVector LastWrittenLocation = FVector::ZeroVector;

  FRotator LastWrittenRotation = FRotator::ZeroRotator;
};
