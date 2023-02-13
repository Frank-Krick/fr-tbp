#include "tbp/tbp.h"

#include <boost/qvm/vec_operations.hpp>

using boost::qvm::vec;
using std::array;

void fr_tbp::ThreeBodyProblem::simulate() {
    auto accelerations = _calculateBodyAccelerations();
    _updateBodyPositions(accelerations);
    _updateVelocities(accelerations);
}

boost::qvm::vec<double, 3> accelerationComponent(double mass, double G,
                                                 vec<double, 3> a,
                                                 vec<double, 3> b) {
    return G * mass * ((a - b) / pow(mag(a - b), 3));
}

std::array<boost::qvm::vec<double, 3>, 3>
fr_tbp::ThreeBodyProblem::_calculateBodyAccelerations() {
    array<vec<double, 3>, 3> bodyAccelerations{};
    bodyAccelerations[0] =
        accelerationComponent(_masses[1], _gravitationalConstant,
                              _bodyPositions[0], _bodyPositions[1]) -
        accelerationComponent(_masses[2], _gravitationalConstant,
                              _bodyPositions[0], _bodyPositions[2]);
    bodyAccelerations[1] =
        accelerationComponent(_masses[2], _gravitationalConstant,
                              _bodyPositions[1], _bodyPositions[2]) -
        accelerationComponent(_masses[0], _gravitationalConstant,
                              _bodyPositions[1], _bodyPositions[0]);
    bodyAccelerations[2] =
        accelerationComponent(_masses[0], _gravitationalConstant,
                              _bodyPositions[2], _bodyPositions[0]) -
        accelerationComponent(_masses[1], _gravitationalConstant,
                              _bodyPositions[2], _bodyPositions[1]);
    return bodyAccelerations;
}

void fr_tbp::ThreeBodyProblem::_updateBodyPositions(
    std::array<boost::qvm::vec<double, 3>, 3> accelerations) {
    for (int i = 0; i < 3; i++) {
        _bodyPositions[i] = _bodyPositions[i] +
                            _bodyVelocities[i] * _deltaTime +
                            0.5 * accelerations[i] * _deltaTime * _deltaTime;
    }
}

void fr_tbp::ThreeBodyProblem::_updateVelocities(
    std::array<boost::qvm::vec<double, 3>, 3> accelerations) {
    for (int i = 0; i < 3; i++) {
        _bodyVelocities[i] = _bodyVelocities[i] + accelerations[i] * _deltaTime;
    }
}
