#ifndef ATG_SIMPLE_3D_CONSTRAINT_SOLVER_UTILITIES_H
#define ATG_SIMPLE_3D_CONSTRAINT_SOLVER_UTILITIES_H

#include <cfloat> // for DBL_MAX if needed
#include <cmath>  // for math functions

// Define scs_force_inline if your code uses it
#ifndef scs_force_inline
#define scs_force_inline inline
#endif

namespace atg_scs {
    struct Quaternion {
        double w, x, y, z;

        Quaternion(double w = 1, double x = 0, double y = 0, double z = 0)
            : w(w), x(x), y(y), z(z) {}

        // Normalize in place
        void normalize() {
            double mag = std::sqrt(w*w + x*x + y*y + z*z);
            if (mag > 1e-15) {
                w /= mag; x /= mag; y /= mag; z /= mag;
            }
        }

        // Multiply quaternions
        Quaternion operator*(const Quaternion &q) const {
            Quaternion result;
            result.w = w*q.w - x*q.x - y*q.y - z*q.z;
            result.x = w*q.x + x*q.w + y*q.z - z*q.y;
            result.y = w*q.y - x*q.z + y*q.w + z*q.x;
            result.z = w*q.z + x*q.y - y*q.x + z*q.w;
            return result;
        }
    };
     
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_3D_CONSTRAINT_SOLVER_UTILITIES_H */
