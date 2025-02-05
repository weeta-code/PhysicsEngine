#ifndef ATG_SIMPLE_3D_CONSTRAINT_SOLVER_CONSTRAINT_H
#define ATG_SIMPLE_3D_CONSTRAINT_SOLVER_CONSTRAINT_H

#include "system_state.h"   // Must exist in your project
#include "rigid_body.h"     // For RigidBody definition
#include "matrix.h"
#include "utilities.h"      // For noLimits(...) if you want it here

namespace atg_scs {
    class Constraint {
    public:
        static constexpr int MaxConstraintCount = 6; // up to 6 DOFs per rigid body in 3D
        static constexpr int MaxBodyCount       = 2; // typically 1 or 2 bodies in a constraint

        struct Output {
            double C[MaxConstraintCount];                     // positional error
            double J[MaxConstraintCount][MaxBodyCount * 6];   // Jacobian
            double lambda[MaxConstraintCount];                 // Lagrange multipliers

            double J_dot[MaxConstraintCount][MaxBodyCount * 6]; // derivative of Jacobian
            double ks[MaxConstraintCount];                     // spring constants
            double kd[MaxConstraintCount];                     // damping constants
            double v_bias[MaxConstraintCount];                 // velocity bias
            double limits[MaxConstraintCount][2];              // lower/upper limits per row
        };

        // constructor: specify how many constraint rows and how many bodies
        Constraint(int constraintCount, int bodyCount);
        virtual ~Constraint();

        // fill the Output struct with the constraint equation, etc.
        virtual void calculate(Output *output, const SystemState &state) = 0;

        // Accessors
        int getConstraintCount() const { return m_constraintCount; }
        int getBodyCount() const       { return m_bodyCount; }

    protected:
        int m_constraintCount;                  // how many rows in this constraint
        int m_bodyCount;                        // how many bodies (1 or 2 typically)
    public:
        int m_index;                            // used by solvers to track constraints
        RigidBody *m_bodies[MaxBodyCount];      // up to 2 bodies
    };
    inline void noLimits(struct Constraint::Output *output) {
        for (int i = 0; i < 6 /* MaxConstraintCount */; ++i) {
            output->limits[i][0] = -DBL_MAX;
            output->limits[i][1] =  DBL_MAX;
        }
    }
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_3D_CONSTRAINT_SOLVER_CONSTRAINT_H */
