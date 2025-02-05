#ifndef ATG_SIMPLE_3D_CONSTRAINT_SOLVER_RIGID_BODY_H
#define ATG_SIMPLE_3D_CONSTRAINT_SOLVER_RIGID_BODY_H

#include "utilities.h"

namespace atg_scs {
    struct RigidBody {
    public:
        RigidBody() {
            p_x = p_y = p_z = 0.0;
            v_x = v_y = v_z = 0.0;
            w_x = w_y = w_z = 0.0;
            q = Quaternion(); // identity orientation
            m = 1.0;
            I_xx = I_yy = I_zz = 1.0;
            index = -1;
        }

        ~RigidBody() { }

        // Simple local->world transform ignoring advanced scaling/shears
        // Replaces 2D localToWorld calls from older code
        void localToWorld(double x, double y, double z,
                          double *w_x, double *w_y, double *w_z)
        {
            // naive approach: just translate for now
            *w_x = p_x + x;
            *w_y = p_y + y;
            *w_z = p_z + z;
        }

        // The inverse transform
        void worldToLocal(double x, double y, double z,
                          double *l_x, double *l_y, double *l_z)
        {
            *l_x = x - p_x;
            *l_y = y - p_y;
            *l_z = z - p_z;
        }

        // -- Data: position, velocity, orientation --
        double p_x, p_y, p_z; // position
        double v_x, v_y, v_z; // linear velocity
        double w_x, w_y, w_z; // angular velocity
        Quaternion q;         // orientation

        double m;             // mass
        double I_xx, I_yy, I_zz; // diagonal inertia

        int index; // -1 if not in system
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_3D_CONSTRAINT_SOLVER_RIGID_BODY_H */
