// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/client/Vessel.h"

#include "carla/client/detail/Simulator.h"

namespace carla {
namespace client {

  void Vessel::ApplyControl(const Control &control) {
    GetEpisode().Lock()->ApplyControlToVessel(*this, control);
    _control = control;
  }

} // namespace client
} // namespace carla
