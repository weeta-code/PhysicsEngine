#include "../include/spring.h"
#include <cmath>

namespace atg_scs {

Spring::Spring() {
    m_restLength = 1.0;
    m_ks = 0;
    m_kd = 0;
    m_p1_x = m_p1_y = 0;
    m_p2_x = m_p2_y = 0;
    m_body1 = nullptr;
    m_body2 = nullptr;
}

Spring::~Spring() {
    /* void */
}

void Spring::apply(SystemState *state) {
    if (m_body1 == nullptr || m_body2 == nullptr) return;

    double x1, y1;
    double x2, y2;
    double v_x1 = 0, v_y1 = 0;
    double v_x2 = 0, v_y2 = 0;

    // localToWorld calls...
    // leftover 2D logic
    // ...
}

void Spring::getEnds(double *x_1, double *y_1, double *x_2, double *y_2) {
    if (m_body1 == nullptr || m_body2 == nullptr) return;
    // ...
}

double Spring::energy() const {
    if (m_body1 == nullptr || m_body2 == nullptr) return 0;
    double x1, y1, x2, y2;
    // ...
    return 0.0; // etc.
}

} // namespace atg_scs
