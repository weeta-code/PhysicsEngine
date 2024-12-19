#ifndef CUSTOM_PHYSICS_ENGINE_SYSTEM_STATE_H
#define CUSTOM_PHYSICS_ENGINE_SYSTEM_STATE_H

#include "quaternion.h"
#include <array>
#include <vector>

namespace vic_scs {

    class SystemState {
        public:
            // Constructor & Destructor
            SystemState();
            ~SystemState();

            // Core Functions for constructor & Destructor
            void copy(const SystemState *state);
            void resize(int bodyCount, int constraintCount);
            void destroy();

            // Physics Methods
            void localToWorld(double x, double y, double z, double *x_t, double *y_t, double *z_t, int body);
            void velocityAtPoint(double x, double y, double z, double *v_x, double *v_y, double *v_z, int body);
            void applyForce(double x, double y, double z, double f_x, double f_y, double f_z, int body);

            // System State Variables
            std::vector<std::array<double, 4>> orientation;         // Quaternion (w, x, y, z)
            std::vector<std::array<double, 3>> angularVelocity;    // Angular velocity (wx, wy, wz)
            std::vector<std::array<double, 3>> angularAcceleration; // Angular acceleration (ax, ay, az)

            std::vector<std::array<double, 3>> a;    // Linear acceleration (a_x, a_y, a_z)
            std::vector<std::array<double, 3>> v;    // Linear velocity (v_x, v_y, v_z)
            std::vector<std::array<double, 3>> r;    // Position (r_x, r_y, r_z)

            std::vector<std::array<double, 3>> forces;   // Applied forces
            std::vector<std::array<double, 3>> torque;   // Applied torques

            std::vector<double> mass;   // Mass 
            std::vector<int> indexMap;  // Body constraint index map

            // Metadata
            int n, n_c;  // Body and constraint counts
            double dt;   // Time step
    };
}

#endif  // CUSTOM_PHYSICS_ENGINE_SYSTEM_STATE_H
