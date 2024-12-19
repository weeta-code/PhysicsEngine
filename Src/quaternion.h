#ifndef CUSTOM_PHYSICS_ENGINE_QUATERNION_H
#define CUSTOM_PHYSICS_ENGINE_QUATERNION_H

#include <array>

namespace vic_scs {

    std::array<double, 4> multiplyQuaternions(const std::array<double, 4>& q1, const std::array<double, 4>& q2);
    void normalizeQuaternion(std::array<double, 4>& q);
    std::array<double, 3> rotateVector(const std::array<double, 4>& q, const std::array<double, 3>& v);

    std::array<double, 4> createQuaternionFromAxisAngle(const std::array<double, 3>& axis, double angle);
    std::array<double, 4> invertQuaternion(const std::array<double, 4>& q);
    std::array<double, 4> slerp(const std::array<double, 4>& q1, const std::array<double, 4>& q2, double t);

}

#endif
