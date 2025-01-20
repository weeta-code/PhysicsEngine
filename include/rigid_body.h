
#ifndef ATG_SIMPLE_3D_CONSTRAINT_SOLVER_RIGID_BODY_H
#define ATG_SIMPLE_3D_CONSTRAINT_SOLVER_RIGID_BODY_H

#include "utilities.h"

namespace atg_scs {
    struct RigidBody {
        public:
            RigidBody();
            ~RigidBody();

            void localToWorld(double x, double y, double z, double *w_x, double *w_y, double *w_z);
            void worldToLocal(double x, double y, double z, double *l_x, double *l_y, double *l_z);

            double p_x, p_y, p_z; // Position
            double v_x, v_y, v_z; // Velocity

            double w_x, w_y, w_z; // Angular velocity

            Quaternion q; // Orientation

            double m; // Mass
            double I_xx, I_yy, I_zz; // Moment of inertia

            int index;
    };
} /* atg_scs */

#endif /* ATG_SIMPLE_3D_CONSTRAINT_SOLVER_RIGID_BODY_H */
