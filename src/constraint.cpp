
#include "../include/constraint.h"
#include <cstring>
#include <cassert>

atg_scs::Constraint::Constraint(int constraintCount, int bodyCount) {
    assert(constraintCount <= MaxConstraintCount);
    assert(bodyCount <= MaxBodyCount);

    m_constraintCount = constraintCount;
    m_bodyCount = bodyCount;
    m_index = -1;

    std::memset(m_bodies, 0, sizeof(RigidBody *) * MaxBodyCount);
}

atg_scs::Constraint::~Constraint() {
    /* void */
}
