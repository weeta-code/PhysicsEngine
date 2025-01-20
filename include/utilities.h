
#ifndef ATG_SIMPLE_3D_CONSTRAINT_SOLVER_UTILITIES_H
#define ATG_SIMPLE_3D_CONSTRAINT_SOLVER_UTILITIES_H

namespace atg_scs {
    struct Quaternion {
        double w, x, y, z;

        Quaternion(double w = 1, double x = 0, double y = 0, double z = 0)
            : w(w), x(x), y(y), z(z) {}

        void normalize();
        Quaternion operator*(const Quaternion &q) const;
    };

    void freeArray(double *&data);
    void freeArray(int *&data);
} /* atg_scs */

#endif /* ATG_SIMPLE_3D_CONSTRAINT_SOLVER_UTILITIES_H */
