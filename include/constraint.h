
#ifndef ATG_SIMPLE_3D_CONSTRAINT_SOLVER_CONSTRAINT_H
#define ATG_SIMPLE_3D_CONSTRAINT_SOLVER_CONSTRAINT_H

#include "system_state.h"
#include "rigid_body.h"
#include "matrix.h"
#include "utilities.h"

namespace atg_scs {
    class Constraint {
        public:
            static constexpr int MaxConstraintCount = 6; // 3 for position, 3 for rotation
            static constexpr int MaxBodyCount = 2;

            struct Output {
                double C[MaxConstraintCount];
                double J[MaxConstraintCount][MaxBodyCount * 6]; // Jacobian for 3D (position + rotation)
                double lambda[MaxConstraintCount];
            };

            Constraint(int constraintCount, int bodyCount);
            virtual ~Constraint();

            virtual void calculate(Output *output, const SystemState &state) = 0;

        protected:
            int m_constraintCount;
            int m_bodyCount;
            int m_index;
            RigidBody *m_bodies[MaxBodyCount];
    };
} /* atg_scs */

#endif /* ATG_SIMPLE_3D_CONSTRAINT_SOLVER_CONSTRAINT_H */
