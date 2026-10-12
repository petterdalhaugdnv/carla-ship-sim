// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/client/Actor.h"
#include "carla/rpc/VesselControl.h"

namespace carla {
namespace client {

  class Vessel : public Actor {
  public:

    using Control = rpc::VesselControl;

    explicit Vessel(ActorInitializer init)
      : Actor(std::move(init)) {}

    /// Apply @a control to this vessel.
    void ApplyControl(const Control &control);

    /// Return the last control applied by this client.
    const Control &GetControl() const {
      return _control;
    }

  private:

    Control _control;
  };

} // namespace client
} // namespace carla
