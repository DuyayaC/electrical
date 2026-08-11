#ifndef WHEEL_LEG_STATE_ESTIMATOR_H
#define WHEEL_LEG_STATE_ESTIMATOR_H

#include <stdint.h>

#include "wheel_leg_control_types.h"

typedef struct
{
    float xb_m;
    uint8_t initialized;
} WheelLegStateEstimator;

void WheelLeg_StateEstimator_Reset(WheelLegStateEstimator *estimator);

/*
 * Fuse already-converted SI measurements with the supplied five-bar states.
 * Invalid or incomplete inputs produce a zeroed estimate and return 0.
 */
uint8_t WheelLeg_StateEstimator_Update(
    WheelLegStateEstimator *estimator,
    const WheelLegEstimatorInput *input,
    const WheelLegEstimatorParams *params,
    const LegKinematicState leg_state[WL_WHEEL_COUNT],
    WheelLegEstimate *estimate);

#endif
