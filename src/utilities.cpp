
#include "../include/utilities.h"
#include <cmath>

void atg_scs::Quaternion::normalize() {
    double mag = std::sqrt(w * w + x * x + y * y + z * z);
    if (mag > 0) {
        w /= mag;
        x /= mag;
        y /= mag;
        z /= mag;
    }
}

atg_scs::Quaternion atg_scs::Quaternion::operator*(const Quaternion &q) const {
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    );
}

void atg_scs::freeArray(double *&data) {
    delete[] data;
    data = nullptr;
}

void atg_scs::freeArray(int *&data) {
    delete[] data;
    data = nullptr;
}
