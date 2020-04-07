#ifndef PICKUP_STATES_H
#define PICKUP_STATES_H

#include <iostream>

namespace pickup_states {
enum class MasterPickupStates { OFF, SEARCH, ACTION };

enum class GlobalPickupStates { OFF, APPROACH, SEARCH, ATTEMPT_PICKUP, DROPOFF };

enum class LocalPickupState {
  OFF,
  BRICK_ALIGNMENT,
  DESCENT,
  TOUCHDOWN_ALIGNMENT,
  TOUCHDOWN
};


std::ostream &operator<<(std::ostream &o, const MasterPickupStates &state)
{
  switch (state) {
  case MasterPickupStates::OFF:
    o << "[OFF]";
    break;

  case MasterPickupStates::SEARCH:
    o << "[SEARCH]";
    break;

  case MasterPickupStates::ACTION:
    o << "[ACTION]";
    break;
  }
  return o;
}

}// namespace pickup_states

#endif /* PICKUP_STATES_H */