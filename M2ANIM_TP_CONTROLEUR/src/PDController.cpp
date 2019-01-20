
#include "PDController.h"

// ================================ TODO PARTIE II ==================================== //

double PDController::compute (double currentValue) {
    double currentVelocity = currentValue - _previousCurrentValue;
    _previousCurrentValue = currentValue;
    return _kp * (_targetValue - currentValue) + _kd * (_targetVelocity - currentVelocity);
}

// ================================================================================== //
