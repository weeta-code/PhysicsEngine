#include "quaternion.h"
#include <cmath>

namespace vic_scs {

    std::array<double, 4> multiplyQuaternions(
        const std::array<double, 4>& q1, 
        const std::array<double, 4>& q2
    ) {
        return {
            q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],  // W
            q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],  // X
            q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],  // Y
            q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]   // Z
        };
    }

    void normalizeQuaternion(std::array<double, 4>& q) {
        double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        for (auto& val : q) val /= norm; // gets rid of floating point errors, |q|
    }

    std::array<double, 3> rotateVector(
        const std::array<double, 4>& q, 
        const std::array<double, 3>& v
    ) {
        std::array<double, 4> vecQuat = {0.0, v[0], v[1], v[2]};
        std::array<double, 4> qConj = {q[0], -q[1], -q[2], -q[3]};
        auto rotatedQuat = multiplyQuaternions(multiplyQuaternions(q, vecQuat), qConj);
        return {rotatedQuat[1], rotatedQuat[2], rotatedQuat[3]};
    }
}
