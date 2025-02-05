#include "../include/rigid_body_system.h"

#include <assert.h>
#include <chrono>
#include <cmath>

namespace atg_scs {

RigidBodySystem::RigidBodySystem() {
    m_odeSolveMicroseconds          = new long long[ProfilingSamples];
    m_constraintSolveMicroseconds   = new long long[ProfilingSamples];
    m_forceEvalMicroseconds         = new long long[ProfilingSamples];
    m_constraintEvalMicroseconds    = new long long[ProfilingSamples];
    m_frameIndex = 0;

    for (int i = 0; i < ProfilingSamples; ++i) {
        m_odeSolveMicroseconds[i]        = -1;
        m_constraintSolveMicroseconds[i] = -1;
        m_forceEvalMicroseconds[i]       = -1;
        m_constraintEvalMicroseconds[i]  = -1;
    }
}

RigidBodySystem::~RigidBodySystem() {
    delete[] m_odeSolveMicroseconds;
    delete[] m_constraintSolveMicroseconds;
    delete[] m_forceEvalMicroseconds;
    delete[] m_constraintEvalMicroseconds;

    m_state.destroy(); // Assuming SystemState has a destroy() method
}

void RigidBodySystem::reset() {
    m_rigidBodies.clear();
    m_constraints.clear();
    m_forceGenerators.clear();
}

void RigidBodySystem::process(double dt, int steps) {
    // Placeholder – actual integration or constraint solving would go here
}

void RigidBodySystem::addRigidBody(RigidBody *body) {
    m_rigidBodies.push_back(body);
    body->index = (int)m_rigidBodies.size() - 1;
}

void RigidBodySystem::removeRigidBody(RigidBody *body) {
    m_rigidBodies[body->index] = m_rigidBodies.back();
    m_rigidBodies[body->index]->index = body->index;
    m_rigidBodies.pop_back();
}

RigidBody *RigidBodySystem::getRigidBody(int i) {
    assert(i < (int)m_rigidBodies.size());
    return m_rigidBodies[i];
}

void RigidBodySystem::addConstraint(Constraint *constraint) {
    m_constraints.push_back(constraint);
    constraint->m_index = (int)m_constraints.size() - 1;
}

void RigidBodySystem::removeConstraint(Constraint *constraint) {
    m_constraints[constraint->m_index] = m_constraints.back();
    m_constraints[constraint->m_index]->m_index = constraint->m_index;
    m_constraints.pop_back();
}

void RigidBodySystem::addForceGenerator(ForceGenerator *forceGenerator) {
    m_forceGenerators.push_back(forceGenerator);
    forceGenerator->m_index = (int)m_forceGenerators.size() - 1;
}

void RigidBodySystem::removeForceGenerator(ForceGenerator *forceGenerator) {
    m_forceGenerators[forceGenerator->m_index] = m_forceGenerators.back();
    m_forceGenerators[forceGenerator->m_index]->m_index = forceGenerator->m_index;
    m_forceGenerators.pop_back();
}

int RigidBodySystem::getFullConstraintCount() const {
    int count = 0;
    for (auto *constraint : m_constraints) {
        count += constraint->getConstraintCount();
    }
    return count;
}

float RigidBodySystem::findAverage(long long *samples) {
    long long accum = 0;
    int count = 0;
    for (int i = 0; i < ProfilingSamples; ++i) {
        if (samples[i] != -1) {
            accum += samples[i];
            ++count;
        }
    }
    if (count == 0) return 0.0f;
    return (float)accum / count;
}

float RigidBodySystem::getOdeSolveMicroseconds() const {
    return findAverage(m_odeSolveMicroseconds);
}

float RigidBodySystem::getConstraintSolveMicroseconds() const {
    return findAverage(m_constraintSolveMicroseconds);
}

float RigidBodySystem::getForceEvalMicroseconds() const {
    return findAverage(m_forceEvalMicroseconds);
}

float RigidBodySystem::getConstraintEvalMicroseconds() const {
    return findAverage(m_constraintEvalMicroseconds);
}

void RigidBodySystem::populateSystemState() {
    const int n   = getRigidBodyCount();
    const int n_c = getFullConstraintCount();
    const int m   = getConstraintCount();

    // Example, depends on your SystemState class
    m_state.resize(n, n_c);

    // Transfer data from RigidBody to m_state
    for (int i = 0; i < n; ++i) {
        // 2D example usage
        m_state.v_x[i]    = m_rigidBodies[i]->v_x;
        m_state.v_y[i]    = m_rigidBodies[i]->v_y;
        // If you had v_z in system_state, set it too

        m_state.p_x[i]    = m_rigidBodies[i]->p_x;
        m_state.p_y[i]    = m_rigidBodies[i]->p_y;
        // p_z if needed

        // etc.
    }

    for (int i = 0, j_f = 0; i < m; ++i) {
        m_state.indexMap[i] = j_f;
        j_f += m_constraints[i]->getConstraintCount();
    }
}

void RigidBodySystem::populateMassMatrices(Matrix *M, Matrix *M_inv) {
    const int n = getRigidBodyCount();

    // The code is using 3*n => indicates 2D + rotation
    M->initialize(1, 3 * n);
    M_inv->initialize(1, 3 * n);

    // For each body, set diagonal mass/inertia
    for (int i = 0; i < n; ++i) {
        // If purely 2D, we only store I as one value. 
        // For 3D you’d do M->set(?, i*6 + 0, body->I_xx), etc.
        // This code is a leftover from 2D logic.
        double mass   = m_rigidBodies[i]->m;
        double inertia= m_rigidBodies[i]->I_xx; // might not be correct for full 3D

        M->set(0, i*3 + 0, mass);
        M->set(0, i*3 + 1, mass);
        M->set(0, i*3 + 2, inertia);

        M_inv->set(0, i*3 + 0, 1 / mass);
        M_inv->set(0, i*3 + 1, 1 / mass);
        M_inv->set(0, i*3 + 2, 1 / inertia);
    }
}

void RigidBodySystem::processForces() {
    const int n_f = getForceGeneratorCount();
    const int n   = getRigidBodyCount();

    for (int i = 0; i < n; ++i) {
        m_state.f_x[i] = 0.0;
        m_state.f_y[i] = 0.0;
        m_state.t[i]   = 0.0;
    }

    for (int i = 0; i < n_f; ++i) {
        m_forceGenerators[i]->apply(&m_state);
    }
}

} // namespace atg_scs
