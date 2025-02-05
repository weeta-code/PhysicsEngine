#include "../include/optimized_nsv_rigid_body_system.h"

#include <chrono>
#include <cmath>

namespace atg_scs {

OptimizedNsvRigidBodySystem::OptimizedNsvRigidBodySystem() {
    m_sleSolver = nullptr;
    m_biasFactor = 1.0;
    m_t = 0.0;
}

OptimizedNsvRigidBodySystem::~OptimizedNsvRigidBodySystem() {
    m_iv.J_sparse.destroy();
    m_iv.sreg0.destroy();
    m_iv.C.destroy();
    m_iv.M.destroy();
    m_iv.M_inv.destroy();
    m_iv.b_err.destroy();
    m_iv.v_bias.destroy();
    m_iv.limits.destroy();
    m_iv.q_dot.destroy();
    m_iv.q_dot_prime.destroy();
    m_iv.reg0.destroy();
    m_iv.reg1.destroy();
    m_iv.reg2.destroy();
    m_iv.right.destroy();
    m_iv.F_ext.destroy();
    m_iv.F_C.destroy();
    m_iv.R.destroy();
    m_iv.lambda.destroy();
}

void OptimizedNsvRigidBodySystem::initialize(SleSolver *sleSolver) {
    m_sleSolver = sleSolver;
}

void OptimizedNsvRigidBodySystem::process(double dt, int steps) {
    long long
        odeSolveTime = 0,
        constraintSolveTime = 0,
        forceEvalTime = 0,
        constraintEvalTime = 0;

    populateSystemState();
    populateMassMatrices(&m_iv.M, &m_iv.M_inv);

    for (int i = 0; i < steps; ++i) {
        m_odeSolver.start(&m_state, dt / steps);

        while (true) {
            const bool done = m_odeSolver.step(&m_state);

            long long evalTime = 0, solveTime = 0;

            auto s0 = std::chrono::steady_clock::now();
            processForces();
            auto s1 = std::chrono::steady_clock::now();

            processConstraints(dt / steps, &evalTime, &solveTime);

            auto s2 = std::chrono::steady_clock::now();
            m_odeSolver.solve(&m_state);
            auto s3 = std::chrono::steady_clock::now();

            constraintSolveTime += solveTime;
            constraintEvalTime += evalTime;
            odeSolveTime +=
                std::chrono::duration_cast<std::chrono::microseconds>(s3 - s2).count();
            forceEvalTime +=
                std::chrono::duration_cast<std::chrono::microseconds>(s1 - s0).count();

            if (done) break;
        }

        m_odeSolver.end();
    }

    propagateResults();

    m_odeSolveMicroseconds[m_frameIndex]         = odeSolveTime;
    m_constraintSolveMicroseconds[m_frameIndex]  = constraintSolveTime;
    m_forceEvalMicroseconds[m_frameIndex]        = forceEvalTime;
    m_constraintEvalMicroseconds[m_frameIndex]   = constraintEvalTime;
    m_frameIndex = (m_frameIndex + 1) % ProfilingSamples;

    m_t += dt;
}

void OptimizedNsvRigidBodySystem::propagateResults() {
    const int n = getRigidBodyCount();
    for (int i = 0; i < n; ++i) {
        m_rigidBodies[i]->v_x    = m_state.v_x[i];
        m_rigidBodies[i]->v_y    = m_state.v_y[i];
        // v_z if we were fully 3D

        m_rigidBodies[i]->p_x    = m_state.p_x[i];
        m_rigidBodies[i]->p_y    = m_state.p_y[i];
        // p_z for 3D

       // m_rigidBodies[i]->v_theta = m_state.v_theta[i]; // leftover from 2D approach
        // For real 3D, you'd handle w_x, w_y, w_z or quaternions here

        //m_rigidBodies[i]->theta = m_state.theta[i]; // leftover from 2D
    }

    const int m = getConstraintCount();
    for (int i = 0, i_f = 0; i < m; ++i) {
        Constraint *constraint = m_constraints[i];

        for (int j = 0; j < constraint->getConstraintCount(); ++j, ++i_f) {
            for (int k = 0; k < constraint->m_bodyCount; ++k) {
                constraint->F_x[j][k] = m_state.r_x[i_f * 2 + k];
                constraint->F_y[j][k] = m_state.r_y[i_f * 2 + k];
                constraint->F_t[j][k] = m_state.r_t[i_f * 2 + k];
            }
        }
    }
}

void OptimizedNsvRigidBodySystem::processConstraints(
    double dt,
    long long *evalTime,
    long long *solveTime)
{
    *evalTime = -1;
    *solveTime = -1;

    auto s0 = std::chrono::steady_clock::now();

    const int n   = getRigidBodyCount();
    const int m_f = getFullConstraintCount();
    const int m   = getConstraintCount();

    m_iv.J_sparse.initialize(3 * n, m_f);
    m_iv.v_bias.initialize(1, m_f);
    m_iv.C.initialize(1, m_f);
    m_iv.limits.initialize(2, m_f);

    Constraint::Output constraintOutput;
    for (int j = 0, j_f = 0; j < m; ++j) {
        m_constraints[j]->calculate(&constraintOutput, &m_state);

        const int n_f = m_constraints[j]->getConstraintCount();
        for (int k = 0; k < n_f; ++k, ++j_f) {
            // etc. ...
            // omitted for brevity, same logic as you posted
        }
    }

    // solver logic ...
    auto s1 = std::chrono::steady_clock::now();

    // calls to m_sleSolver->solve or solveWithLimits

    auto s2 = std::chrono::steady_clock::now();

    // final constraint force derivation, storing in r_x, r_y, r_t, etc.

    auto s3 = std::chrono::steady_clock::now();

    *evalTime =
        std::chrono::duration_cast<std::chrono::microseconds>(s1 - s0 + s3 - s2).count();
    *solveTime =
        std::chrono::duration_cast<std::chrono::microseconds>(s2 - s1).count();
}

} // namespace atg_scs
