#include "mode.h"
#include "Plane.h"

bool ModeCircle::_enter()
{
    // the altitude to circle at is taken from the current altitude
    plane.next_WP_loc = plane.current_loc;

    return true;
}

void ModeCircle::update()
{
    // we have no GPS installed and have lost radio contact
    // or we just want to fly around in a gentle circle w/o GPS,
    // holding altitude at the altitude we set when we
    // switched into the mode
    plane.nav_roll_cd  = plane.roll_limit_cd / 3;
    plane.update_load_factor();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}


/*
  keep the circle centre under the aircraft while standing by, for the same
  reason as LOITER: it is captured from current_loc on entry.
 */
void ModeCircle::standby_reset()
{
    plane.next_WP_loc = plane.current_loc;
}
