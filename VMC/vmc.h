#ifndef VMC_H
#define VMC_H

#include <stdint.h>

/* Map one leg's virtual wrench to its two joint torques: tau = J^T [F, Tp]. */
uint8_t VMC_Calculate2D(
    const float jacobian[2][2],
    float force_N,
    float pitch_torque_Nm,
    float joint_torque_Nm[2]);

#endif
