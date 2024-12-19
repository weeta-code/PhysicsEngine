#include "rigid_body.h"
#include <cmath>

namespace vic_scs {

    RigidBody::RigidBody() {
        position = {0.0, 0.0, 0.0};     // Origin
        orientation = {1.0, 0.0, 0.0, 0.0};  // Identity quaternion
        angularVelocity = {0.0, 0.0, 0.0};  // Zero angular velocity

        p_x = p_y = p_z = 0.0;  // No momentum
        v_x = v_y = v_z = 0.0;  // No velocity

        m = 1.0;  // Default mass
        I = 1.0;  // Default moment of inertia
    }

    void RigidBody::applyTorque(const std::array<double, 3>& torque, double dt) {
        // Torque to angularacceleration update
        angularVelocity[0] += (torque[0] / I) * dt;
        angularVelocity[1] += (torque[1] / I) * dt;
        angularVelocity[2] += (torque[2] / I) * dt;
    }

    void RigidBody::updateOrientation(double dt) {
        double angle = std::sqrt(
            angularVelocity[0] * angularVelocity[0] +
            angularVelocity[1] * angularVelocity[1] + // Updating our Orientation
            angularVelocity[2] * angularVelocity[2]
        );

        if (angle == 0.0) return;

        double halfAngle = 0.5 * angle * dt;
        double sinHalfAngle = std::sin(halfAngle); // Half-sin to create/initialize new Quaternions 

        std::array<double, 4> deltaRotation = {
            std::cos(halfAngle),
            sinHalfAngle * angularVelocity[0] / angle,
            sinHalfAngle * angularVelocity[1] / angle,
            sinHalfAngle * angularVelocity[2] / angle
        };

        orientation = multiplyQuaternions(orientation, deltaRotation); // Quaternion Multiplication (See Quaternion.h)
        normalizeQuaternion(orientation); // Accounting for all float-point errors to retain accuracy
    }
}
