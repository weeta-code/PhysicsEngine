#include "../include/constant_rotation_constraint.h"
#include <cfloat> // for DBL_MAX

namespace atg_scs {
    ConstantRotationConstraint::ConstantRotationConstraint()
        : Constraint(1, 1) // 1 constraint row, 1 body
    {
        m_rotationSpeed = 0.0;
        m_maxTorque     = DBL_MAX;
        m_minTorque     = -DBL_MAX;
        m_ks            = 10.0;
        m_kd            = 1.0;
    }

    ConstantRotationConstraint::~ConstantRotationConstraint() {
        /* void */
    }

    void ConstantRotationConstraint::calculate(Output *output, const SystemState &state) {
        // For 1 row => index 0
        // This sets rotation about Z (if you're in a 2D plane, for instance).
        output->J[0][0] = 0; // d(theta)/dx body = 0
        output->J[0][1] = 0; // d(theta)/dy body = 0
        output->J[0][2] = 1; // d(theta)/dtheta  = 1

        // If bodies had 6 DOF each, you'd index up to [MaxBodyCount*6] 
        // but here we only reference the first 3 for a single body.

        // J_dot, spring/damper, etc.
        output->J_dot[0][0] = 0;
        output->J_dot[0][1] = 0;
        output->J_dot[0][2] = 0;

        output->ks[0]      = m_ks;
        output->kd[0]      = m_kd;
        output->C[0]       = 0;   // no position error if you want a constant rotation speed
        output->v_bias[0]  = m_rotationSpeed; // desired rotational velocity

        // Set torque limits
        output->limits[0][0] = m_minTorque;
        output->limits[0][1] = m_maxTorque;
    }
}
