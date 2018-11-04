#include "Copter.h"

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    update_super_simple_bearing(false);

    flightmode->update_navigation();
}

// distance between vehicle and home in cm
uint32_t Copter::home_distance()
{
    if (position_ok()) {
        _home_distance = get_distance_cm(current_loc, ahrs.get_home());
    }
    return _home_distance;
}

// The location of home in relation to the vehicle in centi-degrees
int32_t Copter::home_bearing()
{
    if (position_ok()) {
        _home_bearing = get_bearing_cd(current_loc, ahrs.get_home());
    }
    return _home_bearing;
}

/*
void Plane::calc_airspeed_errors()
{
    float airspeed_measured = 0;

    // we use the airspeed estimate function not direct sensor as TECS
    // may be using synthetic airspeed
    ahrs.airspeed_estimate(&airspeed_measured);

    // FBW_B airspeed target
    if (control_mode == FLY_BY_WIRE_B ||
        control_mode == CRUISE) {
        target_airspeed_cm = ((int32_t)(aparm.airspeed_max -
                                        aparm.airspeed_min) *
                              channel_throttle->get_control_in()) +
                             ((int32_t)aparm.airspeed_min * 100);

    } else if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
        // Landing airspeed target
        target_airspeed_cm = landing.get_target_airspeed_cm();

    } else {
        // Normal airspeed target
        target_airspeed_cm = aparm.airspeed_cruise_cm;
    }

    // Set target to current airspeed + ground speed undershoot,
    // but only when this is faster than the target airspeed commanded
    // above.
    if (auto_throttle_mode &&
        aparm.min_gndspeed_cm > 0 &&
        control_mode != CIRCLE) {
        int32_t min_gnd_target_airspeed = airspeed_measured*100 + groundspeed_undershoot;
        if (min_gnd_target_airspeed > target_airspeed_cm) {
            target_airspeed_cm = min_gnd_target_airspeed;
        }
    }

    // Bump up the target airspeed based on throttle nudging
    if (throttle_allows_nudging && airspeed_nudge_cm > 0) {
        target_airspeed_cm += airspeed_nudge_cm;
    }

    // Apply airspeed limit
    if (target_airspeed_cm > (aparm.airspeed_max * 100))
        target_airspeed_cm = (aparm.airspeed_max * 100);

    // use the TECS view of the target airspeed for reporting, to take
    // account of the landing speed
    airspeed_error = SpdHgt_Controller->get_target_airspeed() - airspeed_measured;
}
*/
