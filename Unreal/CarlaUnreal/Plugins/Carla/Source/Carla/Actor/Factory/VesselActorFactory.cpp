// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Actor/Factory/VesselActorFactory.h"
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Actor/ActorDefinition.h"

TArray<FActorDefinition> AVesselActorFactory::GetDefinitions()
{
  // Produces id "vessel.mass.kvlcc2" and tags "vessel,mass,kvlcc2", plus the
  // standard role_name and ros_name variations.
  FActorDefinition Definition = UActorBlueprintFunctionLibrary::MakeGenericDefinition(
      TEXT("vessel"),
      TEXT("mass"),
      TEXT("kvlcc2"));
  Definition.Class = VesselClass != nullptr ? VesselClass.Get() : ACarlaVessel::StaticClass();

  FActorAttribute ObjectType;
  ObjectType.Id = TEXT("object_type");
  ObjectType.Type = EActorAttributeType::String;
  ObjectType.Value = TEXT("vessel");
  Definition.Attributes.Emplace(ObjectType);

  return {Definition};
}

FActorSpawnResult AVesselActorFactory::SpawnActor(
    const FTransform &SpawnAtTransform,
    const FActorDescription &ActorDescription)
{
  FActorSpawnResult SpawnResult;
  UClass *Class = ActorDescription.Class;
  if (!IsValid(Class) || !Class->IsChildOf(ACarlaVessel::StaticClass()))
  {
    UE_LOG(LogCarla, Warning, TEXT("Vessel actor description has no ACarlaVessel class, using the default."));
    Class = ACarlaVessel::StaticClass();
  }

  AActor *SpawnedActor = GetWorld()->SpawnActor<AActor>(Class, SpawnAtTransform);
  SpawnResult.Actor = SpawnedActor;
  SpawnResult.Status = (SpawnedActor != nullptr) ?
      EActorSpawnResultStatus::Success :
      EActorSpawnResultStatus::Collision;
  return SpawnResult;
}
