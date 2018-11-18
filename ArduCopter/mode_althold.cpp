#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Copter::ModeAltHold::init(bool ignore_checks)
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    logIndex = 0;
    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::ModeAltHold::run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

    //should be parameters
    int tlty_althld_cut1 = 1300;
    int tlty_althld_cut2 = 1600;
    int tlty_m1_pit_up = 2000;
    int tlty_m1_pit_dn = -500;
    float tlty_Vstall = 15;
    float tlty_Vmax = 25;

    //added for alt hold only
    int in = hal.rcin->read(10);
    float tlty_Vdes = 0;
    float tlty_dV = 0;

    tlty_Vdes = ((tlty_Vmax-1.1*tlty_Vstall)/(2000-tlty_althld_cut2))*(in-tlty_althld_cut2)+1.1*tlty_Vstall;
    tlty_dV = tlty_Vdes-copter.smoothed_airspeed;
    if(hal.rcout->read_last_sent(8)>g.tiltEPMax-50){
        target_pitch = g.Pvp_elev*tlty_dV*100;
    }else if (hal.rcout->read_last_sent(8)>g.Tilt_Mix){
        target_pitch = g.Pvp_elev_derate*g.Pvp_elev*tlty_dV*100;
    }else{
        target_pitch = 0;
    }
    if (in < tlty_althld_cut1){
        target_pitch = -1*((tlty_m1_pit_up-tlty_m1_pit_dn)/(tlty_althld_cut1-1000))*(in-1000)+tlty_m1_pit_up;
        copter.tilt = g.tiltEPMin;
    } else if (in > tlty_althld_cut1 && in < tlty_althld_cut2){
        copter.tilt = ((g.tiltEPMax-g.tiltEPMin)/(tlty_althld_cut2-tlty_althld_cut1))*(in-tlty_althld_cut1)+g.tiltEPMin;
    } else if (in > tlty_althld_cut2){
        copter.tilt = g.tiltEPMax;
    }

    // get pilot's desired yaw rate
    float target_yaw_rate = 0.2*get_pilot_desired_yaw_rate(channel_roll->get_control_in());
    if (copter.tilt > 1850)
    { //hard coded cutoff
        target_yaw_rate = g.roll_yaw_mix * target_roll;
    }
    else if (copter.tilt > g.Tilt_Mix)
    {
        target_roll = target_yaw_rate / g.roll_yaw_mix;
    }
    else
    {
        target_roll = 0;
    }

    logIndex++;
    if (logIndex==60)
    {
        DataFlash_Class::instance()->Log_Write("Quad", "TimeUS,Vdes,dV,tilt,pitch,test",
                                               "QffIfI", // format: uint64_t, float
                                               AP_HAL::micros64(),
                                               (double)tlty_Vdes,
                                               (double)tlty_dV,
                                               copter.tilt,
                                               (double)target_pitch/100,
                                               100);
        logIndex = 0;
    }

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_pitch->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff.running() || takeoff.triggered(target_climb_rate)) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    althold_state = AltHold_Flying;

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        heli_flags.init_targets_on_arming=true;
        if (ap.land_complete_maybe) {
            pos_control->relax_alt_hold_controllers(0.0f);
        }
#else
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            heli_flags.init_targets_on_arming=false;
        }
#endif
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            if (motors->get_interlock()) {
                heli_flags.init_targets_on_arming=false;
            }
        }
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#endif
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // adjust climb rate using rangefinder
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }

}
