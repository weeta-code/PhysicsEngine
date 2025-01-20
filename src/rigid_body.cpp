
#include "../include/rigid_body.h"
#include <cmath>

atg_scs::RigidBody::RigidBody() {
    p_x = p_y = p_z = 0.0;
    v_x = v_y = v_z = 0.0;
    w_x = w_y = w_z = 0.0;
    q = Quaternion(1, 0, 0, 0); // Default orientation
    m = 1.0;
    I_xx = I_yy = I_zz = 1.0;
    index = -1;
}

atg_scs::RigidBody::~RigidBody() {
    /* void */
}

void atg_scs::RigidBody::localToWorld(double x, double y, double z, double *w_x, double *w_y, double *w_z) {
    // Transform local coordinates to world using quaternion rotation
    Quaternion point(0, x, y, z);
    Quaternion rotated = q * point * Quaternion(q.w, -q.x, -q.y, -q.z);
    *w_x = rotated.x;
    *w_y = rotated.y;
    *w_z = rotated.z;
}

void atg_scs::RigidBody::worldToLocal(double x, double y, double z, double *l_x, double *l_y, double *l_z) {
    // Transform world coordinates to local using quaternion inverse
    Quaternion point(0, x, y, z);
    Quaternion conjugate(q.w, -q.x, -q.y, -q.z);
    Quaternion rotated = conjugate * point * q;
    *l_x = rotated.x;
    *l_y = rotated.y;
    *l_z = rotated.z;
}
