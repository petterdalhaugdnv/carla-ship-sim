// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/ActorSpawnResult.h"
#include "Carla/Actor/CarlaActorFactory.h"
#include "Carla/Vessel/CarlaVessel.h"

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include <util/ue-header-guard-end.h>

#include "VesselActorFactory.generated.h"

/// Factory in charge of spawning surface vessels (ACarlaVessel). It does not
/// depend on any wheeled-vehicle code.
UCLASS()
class CARLA_API AVesselActorFactory : public ACarlaActorFactory
{
  GENERATED_BODY()

  /// Retrieve the definitions of the vessels, currently "vessel.mass.kvlcc2".
  virtual TArray<FActorDefinition> GetDefinitions() override;

  virtual FActorSpawnResult SpawnActor(
      const FTransform &SpawnAtTransform,
      const FActorDescription &ActorDescription) override;

protected:

  /// Class spawned by the vessel definition. Defaults to ACarlaVessel.
  UPROPERTY(Category = "Vessel", EditAnywhere)
  TSubclassOf<ACarlaVessel> VesselClass = ACarlaVessel::StaticClass();
};
