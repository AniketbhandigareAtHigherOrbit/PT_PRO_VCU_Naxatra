#ifndef VEHICLE_CONFIG_DIRECTION_SYNC_BLOCK_H
#define VEHICLE_CONFIG_DIRECTION_SYNC_BLOCK_H

/* ============================================================
 * PT_PRO_VCU_2.11.10.04 — wheel direction/pickup synchronization
 * ============================================================ */

/* Controller feedback freshness */
#define VC_CONTROLLER_MSG1_TIMEOUT_MS                  (400U)
#define VC_CONTROLLER_MSG2_TIMEOUT_MS                  (800U)

/*
 * Dynamic direction-change active braking / hill hold.
 *
 * While a running machine changes direction, the controller speed loop must
 * never be opened by a zero-RPM coast interval. The common wheel command is
 * reduced to VC_ACTIVE_BRK_MIN_RPM in the old direction and both controllers
 * remain enabled. The handover to the requested direction is then performed
 * without an intermediate DIR_STOP/0-RPM coast command.
 *
 * Initial direction arming from rest still uses the zero-RPM path.
 */
#define VC_DIR_ACTIVE_BRAKE_HANDOVER_ENABLE             (1U)
#define VC_DIR_ACTIVE_BRAKE_HOLD_RPM                    (VC_ACTIVE_BRK_MIN_RPM)
#define VC_DIR_ACTIVE_BRAKE_DETECT_RPM                  (20U)
#define VC_DIR_ACTIVE_BRAKE_APPLY_HOLD_MS               (220U)

/* Direction change: low-speed/stop verification while braking remains active. */
#define VC_DIR_ZERO_FEEDBACK_RPM                        (20U)
#define VC_DIR_ZERO_CONFIRM_FRESH_PAIRS                 (3U)
#define VC_DIR_WAIT_ZERO_TIMEOUT_MS                     (2500U)

/*
 * SLOPE / ROLLBACK HANDOVER
 * -------------------------
 * On a slope the machine can keep creeping after the command has been reduced
 * to VC_ACTIVE_BRK_MIN_RPM. Requiring an exact near-zero feedback condition can
 * therefore prevent the direction handover forever. The old firmware then
 * changed direction after a fixed delay, which avoided this loss of hill hold.
 *
 * The new algorithm keeps feedback supervision and separates two behaviours:
 *   1. Flat/normal deceleration: direction handover is blocked until BOTH
 *      wheels are confirmed near zero from fresh feedback.
 *   2. Slope/load condition: no-coast handover is allowed only when either
 *      both feedback directions prove rollback in the requested direction or
 *      the low-speed feedback stops decelerating for several fresh samples.
 *
 * During an active-brake handover, a zero-confirm timeout NEVER sends 0 RPM /
 * DIR_STOP and NEVER blindly reverses at a large residual RPM. It keeps the
 * active-brake floor and waits for trustworthy flat-stop or slope evidence.
 */
#define VC_DIR_SLOPE_HANDOVER_ENABLE                    (1U)

/*
 * Adaptive flat/slope classification.
 *
 * Flat ground: keep the old direction + active-brake floor until both wheel
 * feedbacks genuinely settle near zero, then hand over both controllers
 * together at the minimum braking RPM and start synchronized pickup. The key
 * jerk-prevention rule is that no flat-ground handover occurs while normal
 * deceleration is still progressing.
 *
 * Slope/load: never open the speed loop. A slope is accepted only when the
 * feedback provides evidence that gravity/load is preventing normal decay:
 *   A) both wheel feedback directions already show the requested direction, or
 *   B) low-speed wheel feedback stops decelerating for several fresh samples.
 * This removes the old blind 500-ms / <=120-RPM forced handover that could
 * reverse the controllers while the machine was still decelerating on flat
 * ground and create a short random-side jerk.
 */
#define VC_DIR_SLOPE_ROLLBACK_MIN_RPM                   (20U)
#define VC_DIR_SLOPE_ROLLBACK_MAX_RPM                   (160U)
#define VC_DIR_SLOPE_ROLLBACK_MAX_DIFF_RPM              (80U)
#define VC_DIR_SLOPE_ROLLBACK_CONFIRM_FRESH_PAIRS       (2U)

#define VC_DIR_SLOPE_CREEP_DETECT_DELAY_MS              (650U)
#define VC_DIR_SLOPE_CREEP_MIN_RPM                      (25U)
#define VC_DIR_SLOPE_CREEP_MAX_RPM                      (120U)
#define VC_DIR_SLOPE_CREEP_MAX_DIFF_RPM                 (70U)
#define VC_DIR_SLOPE_CREEP_WINDOW_FRESH_PAIRS           (4U)
#define VC_DIR_SLOPE_CREEP_MAX_TOTAL_DECEL_RPM          (8U)

/*
 * Last-resort near-zero handover. This is deliberately much lower than the
 * previous 220-RPM timeout handover. It is not used as a slope detector; it
 * only prevents a controller quantisation/noise edge case from waiting forever
 * when both wheels are already almost stopped.
 */
#define VC_DIR_TIMEOUT_NEAR_ZERO_RPM                    (35U)
#define VC_DIR_TIMEOUT_NEAR_ZERO_MAX_DIFF_RPM           (35U)

/*
 * Zero-RPM neutral is retained only as the fallback path for initial/static
 * direction selection where active-brake handover is not required.
 */
#define VC_DIR_TRUE_NEUTRAL_HOLD_MS                     (200U)
#define VC_DIR_APPLY_ZERO_HOLD_MS                       (220U)

/*
 * command_dir feedback is observed only. It is not a hard startup gate because
 * some controllers update it only after a non-zero RPM command is received.
 */
#define VC_DIR_ACK_CONFIRM_FRESH_PAIRS                  (2U)
#define VC_DIR_ACK_OBSERVE_MS                           (250U)

/* Synchronized pickup — restored to the previously validated values. */
#define VC_DIR_PICKUP_START_RPM                         (50U)
#define VC_DIR_PICKUP_MAX_RPM                           (260U)
#define VC_DIR_PICKUP_RAMP_STEP_RPM                     (10U)
#define VC_DIR_PICKUP_MOVING_RPM                        (30U)
#define VC_DIR_PICKUP_ALLOWED_DIFF_RPM                  (60U)
#define VC_DIR_PICKUP_STABLE_FRESH_PAIRS                (3U)
#define VC_DIR_PICKUP_SINGLE_WHEEL_TIMEOUT_MS           (650U)
#define VC_DIR_PICKUP_SINGLE_WHEEL_ABORT_RPM            (220U)
#define VC_DIR_PICKUP_FEEDBACK_START_TIMEOUT_MS         (1200U)
#define VC_DIR_PICKUP_TOTAL_TIMEOUT_MS                  (5000U)

/* Adaptive equal-command retry; no left/right hunting is permitted. */
#define VC_DIR_PICKUP_RETRY_HOLD_MS                     (180U)
#define VC_DIR_PICKUP_RETRY_INCREMENT_RPM               (40U)
#define VC_DIR_PICKUP_MAX_RETRIES                       (3U)

/* Direction mismatch validation after actual movement begins. */
#define VC_DIR_DIRECTION_MISMATCH_CONFIRM_PAIRS         (4U)

/* Equal ramp after pickup before normal steering is released. */
#define VC_DIR_RAMP_UP_STEP_RPM                         (20U)

/*
 * Pickup supervision is intentionally released at low speed. The supervisor is
 * only for the pickup region, not full-throttle acceleration.
 */
#define VC_DIR_RAMPUP_SYNC_RELEASE_RPM                  (400U)
#define VC_DIR_AUTO_RESYNC_MAX_ATTEMPTS                 (2U)

/* Fault recovery */
#define VC_DIR_FAULT_CLEAR_FEEDBACK_RPM                 (20U)
#define VC_DIR_FAULT_CLEAR_CONFIRM_FRESH_PAIRS          (3U)
#define VC_DIR_FAULT_CLEAR_NO_FEEDBACK_MS               (800U)

/* Low-speed hard imbalance diagnostic during equal ramp-up */
#define VC_DIR_RAMPUP_HARD_DIFF_RPM                     (240U)
#define VC_DIR_RAMPUP_HARD_DIFF_CONFIRM_PAIRS           (4U)

#endif /* VEHICLE_CONFIG_DIRECTION_SYNC_BLOCK_H */
