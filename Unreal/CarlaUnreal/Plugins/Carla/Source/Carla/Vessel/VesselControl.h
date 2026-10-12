// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "VesselControl.generated.h"

/// Rudder and propeller command. Mirrors carla::rpc::VesselControl.
USTRUCT(BlueprintType)
struct CARLA_API FVesselControl
{
  GENERATED_BODY()

  /// Radians, positive to starboard.
  UPROPERTY(Category = "Vessel Control", EditAnywhere, BlueprintReadWrite)
  float RudderAngle = 0.0f;

  /// Rad/s, non-negative.
  UPROPERTY(Category = "Vessel Control", EditAnywhere, BlueprintReadWrite)
  float PropellerSpeed = 0.0f;
};
