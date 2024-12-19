#ifndef CUSTOM_PHYSICS_ENGINE_RIGID_BODY_H
#define CUSTOM_PHYSICS_ENGINE_RIGID_BODY_H

#include <array>
#include "quaternion.h"

namespace vic_scs {
    struct RigidBody {
        public:
            // Member Variables
            std::array<double, 3> position;              // Position in 3D space
            std::array<double, 4> orientation;           // Quaternion (w, x, y, z)
            std::array<double, 3> angularVelocity;      // Angular velocity

            double p_x, p_y, p_z;  // Momentum
            double v_x, v_y, v_z;  // Linear velocity
            double m;              // Mass
            double I;              // Moment of inertia

            // Constructor & Destructor
            RigidBody();
            ~RigidBody() = default;

            // Methods
            void localToWorld(double l_x, double l_y, double l_z, double *w_x, double *w_y, double *w_z);
            void worldToLocal(double w_x, double w_y, double w_z, double *l_x, double *l_y, double *l_z);
            void updateOrientation(double dt);
            void applyTorque(const std::array<double, 3>& torque, double dt);
    };
}

#endif
