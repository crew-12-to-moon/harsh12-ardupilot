#include "Plane.h"

#ifndef STANDBY_ALT_TC
// time constant, in seconds, of the lag applied to the standby altitude target
#define STANDBY_ALT_TC 10.0
#endif

#ifndef STANDBY_ALT_MAX_LAG_CM
// hard limit on how far the standby altitude target may lag the current
// altitude, in centimetres
#define STANDBY_ALT_MAX_LAG_CM 500.0
#endif

/*
  Stand-By mode, for use with multiple flight controllers running in
  parallel on one airframe. Only one of them is the controller in
  command (CIC); its outputs reach the servos through an external
  multiplexer. The others "ride along": they sense the same flight but
  do not command it.

  A ride-along controller accumulates error, because the aircraft is
  doing what another controller commands rather than what it commands.
  On a fixed wing a large part of the trim is carried in the rate
  controller I terms and in the TECS integrators, so a stale ride-along
  controller would command a large control surface step the moment it
  is promoted to CIC. Its self monitoring logic, which compares its own
  commands against sensed motion, would also false trigger.

  Standby solves both with a single flag. While it is set, the control
  loops keep running at full rate - only the accumulated state is
  flushed - so the controller is always "fresh" and can take command
  with a minimal transient.

  This is the fixed wing equivalent of ArduCopter/standby.cpp
 */

/*
  run standby functions at approximately 100Hz to limit maximum
  variable build up.

  When standby is active:
      the roll, pitch, yaw and steering rate controller I terms are
          continually reset
      the TECS throttle and energy balance integrators are reset
      the L1 cross-track integrator is reset
      the altitude target tracks the current altitude
      the flight mode drops any navigation state of its own
      the VTOL attitude and position controllers are reset
      crash detection, parachute release, hover throttle learning and
          servo auto trim are disabled
 */
void Plane::standby_update()
{
    if (!standby_active) {
        return;
    }

    // rate controller integrators. On a fixed wing these carry much of
    // the trim, so they are the largest single source of a takeover
    // transient
    rollController.reset_I();
    pitchController.reset_I();
    yawController.reset_I();
    steerController.reset_I();

    // don't hold a ground course that we are not actually steering
    steer_state.locked_course = false;
    steer_state.locked_course_err = 0;

    // flush the TECS integrators. Deliberately NOT TECS_controller.reset():
    // a full reset also re-anchors the pitch demand rate limiter and the
    // pitch crossover filters to the current pitch every cycle, which leaves
    // the speed/height loop with no reference and produces an undamped
    // phugoid. The height demand is pinned by set_target_altitude_current().
    TECS_controller.reset_throttle_I();
    TECS_controller.reset_pitch_I();

    // flush the L1 cross-track integrator so that no lateral
    // correction is stored up
    nav_controller->standby_reset();

    // Pin the height demand to a lagged version of the current altitude, so
    // that taking command does not command a climb or dive to recover an old
    // target.
    //
    // An exact pin would make the height error zero by construction, leaving no
    // restoring force at all, so any drift persists for as long as standby is
    // asserted. A first order lag leaves a small error proportional to the
    // drift rate, which TECS opposes, while bounding how stale the target can
    // become to roughly drift rate * STANDBY_ALT_TC - metres, rather than the
    // hundreds of metres an unpinned target could accumulate.
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t dt_ms = now_ms - standby_alt.last_ms;
    if (standby_alt.last_ms == 0 || dt_ms > 500) {
        // standby has just been asserted
        standby_alt.amsl_cm = current_loc.alt;
    } else {
        const float dt = dt_ms * 0.001;
        const float alpha = dt / (dt + STANDBY_ALT_TC);
        standby_alt.amsl_cm += (current_loc.alt - standby_alt.amsl_cm) * alpha;
    }
    // Clamp how far the target may lag. The lag is the aircraft's rate of
    // altitude change multiplied by the time constant, and while standing by
    // that rate is set by whichever controller is actually flying - which may
    // be climbing far faster than this controller ever drifts. The clamp keeps
    // the restoring force for slow drift while bounding the stale target a
    // fast climb would otherwise leave behind.
    standby_alt.amsl_cm = constrain_float(standby_alt.amsl_cm,
                                          current_loc.alt - STANDBY_ALT_MAX_LAG_CM,
                                          current_loc.alt + STANDBY_ALT_MAX_LAG_CM);
    standby_alt.last_ms = now_ms;

    // let this keep the terrain following bookkeeping right, then override the
    // absolute demand with the lagged one
    set_target_altitude_current();
    target_altitude.amsl_cm = standby_alt.amsl_cm;

    // let the flight mode drop any accumulated navigation state of its
    // own, such as the CRUISE locked heading
    control_mode->standby_reset();

#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        // the VTOL controllers accumulate state in exactly the same way
        // as they do on copter
        quadplane.attitude_control->reset_rate_controller_I_terms();
        quadplane.attitude_control->reset_yaw_target_and_rate();
        quadplane.pos_control->NED_standby_reset();
    }
#endif
}
