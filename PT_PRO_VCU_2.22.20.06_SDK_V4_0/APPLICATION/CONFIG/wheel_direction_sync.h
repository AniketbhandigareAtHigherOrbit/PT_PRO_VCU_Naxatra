#ifndef WHEEL_DIRECTION_SYNC_H
#define WHEEL_DIRECTION_SYNC_H

#include "app_typedef.h"

void WheelDirectionSync_Reset(void);

/*
 * base_rpm is the dead-zone-processed wheel target.
 * requested_direction must be DIR_FORWARD or DIR_REVERSE.
 * This function is the exclusive owner of the common wheel RPM and the actual
 * CAN direction during direction transitions and synchronized pickup.
 */
void WheelDirectionSync_Step(uint16_t base_rpm,
                             uint8_t requested_direction);

/* Turning and independent left/right slew are allowed only when this is 0. */
uint8_t WheelDirectionSync_IsTransitionActive(void);

#endif /* WHEEL_DIRECTION_SYNC_H */
