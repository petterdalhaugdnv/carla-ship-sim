// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/MsgPack.h"

#ifdef LIBCARLA_INCLUDED_FROM_UE4
#include <util/enable-ue4-macros.h>
#include "Carla/Vessel/VesselControl.h"
#include <util/disable-ue4-macros.h>
#endif // LIBCARLA_INCLUDED_FROM_UE4

namespace carla {
namespace rpc {

  /// Rudder and propeller command for a vessel.
  /// rudder_angle: radians, positive to starboard.
  /// propeller_speed: rad/s, non-negative.
  class VesselControl {
  public:

    VesselControl() = default;

    VesselControl(float in_rudder_angle, float in_propeller_speed)
      : rudder_angle(in_rudder_angle),
        propeller_speed(in_propeller_speed) {}

    float rudder_angle = 0.0f;
    float propeller_speed = 0.0f;

#ifdef LIBCARLA_INCLUDED_FROM_UE4

    VesselControl(const FVesselControl &Control)
      : rudder_angle(Control.RudderAngle),
        propeller_speed(Control.PropellerSpeed) {}

    operator FVesselControl() const {
      FVesselControl Control;
      Control.RudderAngle = rudder_angle;
      Control.PropellerSpeed = propeller_speed;
      return Control;
    }

#endif // LIBCARLA_INCLUDED_FROM_UE4

    bool operator!=(const VesselControl &rhs) const {
      return rudder_angle != rhs.rudder_angle || propeller_speed != rhs.propeller_speed;
    }

    bool operator==(const VesselControl &rhs) const {
      return !(*this != rhs);
    }

    MSGPACK_DEFINE_ARRAY(rudder_angle, propeller_speed);
  };

} // namespace rpc
} // namespace carla
