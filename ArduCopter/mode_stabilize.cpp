#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::ModeStabilize::init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::ModeStabilize::run()
{
    const int CUTOFF_VAL = 1750;
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    uint16_t theta;
    theta = hal.rcin->read(7);


    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    AP_Vehicle::MultiCopter &aparm = copter.aparm;

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(target_roll, target_pitch, aparm.angle_max, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);

    //get speed scaler
    float aspeed, speed_scaler;
    //speed_scaler = 1.67f;  if this function doesn't work out
    // if (ahrs.airspeed_estimate(&aspeed)) {
    //     if (aspeed > auto_state.highest_airspeed) {
    //         auto_state.highest_airspeed = aspeed;
    //     }
    //     if (aspeed > 0.0001f) {
    //         speed_scaler = g.scaling_speed / aspeed;
    //     } else {
    //         speed_scaler = 2.0;
    //     }
    //     speed_scaler = constrain_float(speed_scaler, 0.5f, 2.0f);
    // } else {
    //     if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > 0) {
    //         speed_scaler = 0.5f + ((float)THROTTLE_CRUISE / SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 2.0f);                 // First order taylor expansion of square root
    //         // Should maybe be to the 2/7 power, but we aren't going to implement that...
    //     }else{
    //         speed_scaler = 1.67f;
    //     }
    //     // This case is constrained tighter as we don't have real speed info
    //     speed_scaler = constrain_float(speed_scaler, 0.6f, 1.67f);
    // }


    speed_scaler = constrain_float(speed_scaler, 0.6f, 1.67f);


//stabilize_roll(speed_scaler)
    bool disable_integrator = false;
    if (channel_roll->get_control_in() != 0) {
        disable_integrator = true;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, rollController.get_servo_out(roll_out - ahrs.roll_sensor, 
                                                                                         speed_scaler, 
                                                                                         disable_integrator));

    disable_integrator = false;
     if (channel_pitch->get_control_in() != 0) {
        disable_integrator = true;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitchController.get_servo_out(pitch_out - ahrs.pitch_sensor, 
                                                                                           speed_scaler, 
                                                                                           disable_integrator));

    if (channel_throttle->get_control_in() == 0 &&
        fabsf(relative_altitude) < 5.0f && 
        fabsf(barometer.get_climb_rate()) < 0.5f &&
        gps.ground_speed() < 3) {
        // we are low, with no climb rate, and zero throttle, and very
        // low ground speed. Zero the attitude controller
        // integrators. This prevents integrator buildup pre-takeoff.
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();

        // if moving very slowly also zero the steering integrator
        if (gps.ground_speed() < 1) {
            steerController.reset_I();            
        }
    }
}
