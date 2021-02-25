#include "Copter.h"

#include <skybrush/colors.h>

/*
 * Implementation of drone show flight mode
 */

// Constructor.
ModeDroneShow::ModeDroneShow(void) : Mode(),
    _stage(DroneShow_Off),
    _last_stage_change_at(0),
    _next_status_report_due_at(0)
{
}

bool ModeDroneShow::init(bool ignore_checks)
{
    initialization_start();
    return true;
}

void ModeDroneShow::exit()
{
    // If we haven't taken off yet, disarm the motors. Apparently
    // copter.ap.land_complete is sometimes false when we are in the
    // DroneShow_WaitForStartTime stage and we are in the time window where
    // the motors were armed already but we haven't taken off yet, so we need
    // to disarm if we are in the DroneShow_WaitForStartTime unconditionally
    // (otherwise we would end up in poshold mode the next time we enter the
    // drone show mode)
    if ((copter.ap.land_complete || _stage == DroneShow_WaitForStartTime) && motors->armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }

    // Clear the timestamp when we last attempted to arm the drone
    _prevent_arming_until_msec = 0;

    // Clear all the status information that depends on the start time
    notify_start_time_changed();

    // Set the stage to "off"
    _set_stage(DroneShow_Off);

    // Notify the drone show manager that the drone show mode exited
    copter.g2.drone_show_manager.notify_drone_show_mode_exited();
}

bool ModeDroneShow::allows_arming(AP_Arming::Method method) const
{
    return (
        // Always allow arming from GCS in case the operator wants to test
        // the motors before takeoff. When the command does not come from the
        // GCS, arm only if we have loaded the show and the takeoff time is
        // valid.
        method == AP_Arming::Method::MAVLINK || (
            copter.g2.drone_show_manager.loaded_show_data_successfully() &&
            copter.g2.drone_show_manager.has_valid_takeoff_time()
        )
    );
}

bool ModeDroneShow::cancel_requested() const
{
    return copter.g2.drone_show_manager.cancel_requested();
}

// ModeDroneShow::run - runs the main drone show controller
// should be called at 25hz or more. This function is actually running at
// 400 Hz
void ModeDroneShow::run()
{
    check_changes_in_parameters();

    // call the correct auto controller
    switch (_stage) {

    case DroneShow_Init:
        // mode has just been initialized
        initialization_run();
        break;

    case DroneShow_WaitForStartTime:
        // waiting for start time
        wait_for_start_time_run();
        break;

    case DroneShow_Takeoff:
        // taking off
        takeoff_run();
        break;

    case DroneShow_Performing:
        // performing show
        performing_run();
        break;

    case DroneShow_Landing:
        // landing at the end of the show (normal termination)
        landing_run();
        break;

    case DroneShow_RTL:
        // returning to home position (abnormal termination)
        rtl_run();
        break;

    case DroneShow_Loiter:
        // holding position (joined show while airborne)
        loiter_run();
        break;

    case DroneShow_Landed:
        // landed successfully after a show
        landed_run();
        break;

    case DroneShow_Error:
        // failed to start a show
        error_run();
        break;

    default:
        break;
    }
}

// Checks changes in relevant parameter values and reports them to the console
void ModeDroneShow::check_changes_in_parameters()
{
    static uint64_t last_seen_start_time_usec;
    uint64_t current_start_time_usec = copter.g2.drone_show_manager.get_start_time_usec();

    if (current_start_time_usec != last_seen_start_time_usec) {
        last_seen_start_time_usec = current_start_time_usec;
        notify_start_time_changed();
    }
}

bool ModeDroneShow::get_wp(Location& destination)
{
    switch (_stage) {
    case DroneShow_Performing:
        return copter.mode_guided.get_wp(destination);
    case DroneShow_Loiter:
        return copter.mode_loiter.get_wp(destination);
    case DroneShow_Landing:
        return copter.mode_land.get_wp(destination);
    case DroneShow_RTL:
        return copter.mode_rtl.get_wp(destination);
    default:
        return false;
    }
}

int32_t ModeDroneShow::wp_bearing() const
{
    switch (_stage) {
    case DroneShow_Performing:
        return copter.mode_guided.wp_bearing();
    case DroneShow_Landing:
        return copter.mode_land.wp_bearing();
    case DroneShow_RTL:
        return copter.mode_rtl.wp_bearing();
    default:
        return false;
    }
}

uint32_t ModeDroneShow::wp_distance() const
{
    switch (_stage) {
    case DroneShow_Performing:
        return copter.mode_guided.wp_distance();
    case DroneShow_Landing:
        return copter.mode_land.wp_distance();
    case DroneShow_RTL:
        return copter.mode_rtl.wp_distance();
    default:
        return false;
    }
}

float ModeDroneShow::crosstrack_error() const
{
    switch (_stage) {
    case DroneShow_Performing:
        return copter.mode_guided.crosstrack_error();
    case DroneShow_Loiter:
        return copter.mode_loiter.crosstrack_error();
    case DroneShow_Landing:
        return copter.mode_land.crosstrack_error();
    case DroneShow_RTL:
        return copter.mode_rtl.crosstrack_error();
    default:
        return false;
    }
}

bool ModeDroneShow::is_landing() const
{
    switch (_stage) {
        case DroneShow_Landing:
            return true;
        case DroneShow_RTL:
            return copter.mode_rtl.is_landing();
        default:
            return false;
    }
    return false;
}

bool ModeDroneShow::is_taking_off() const
{
    return ((_stage == DroneShow_Takeoff) && !wp_nav->reached_wp_destination());
}

// starts the initialization phase of the drone
void ModeDroneShow::initialization_start()
{
    // Set the appropriate stage
    _set_stage(DroneShow_Init);

    // Clear the timestamp when we last attempted to arm the drone
    // TODO(ntamas): prevent arming from the current timestamp until the next
    // five seconds _after_ the drone is armed to prevent injury when someone
    // presses the safety button of the drone while a close start time is
    // already set
    _prevent_arming_until_msec = 0;

    // This is copied from ModeAuto::init()

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // Clear the limits of the guided mode; we will use guided mode internally
    // to control the show
    copter.mode_guided.limit_clear();

    // Set auto-yaw mode to HOLD -- we don't want the drone to start turning
    // towards waypoints
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // Part from ModeAuto::init() ends here

    // Clear all the status information that depends on the start time
    notify_start_time_changed();

    // Notify the drone show manager that the drone show mode was initialized
    copter.g2.drone_show_manager.notify_drone_show_mode_initialized();
}

// initializes the drone show mode after it has been activated the first time
void ModeDroneShow::initialization_run()
{
    // First we need to decide whether we are in the air or not
    if (is_disarmed_or_landed()) {
        // Great, let's move to the state where we wait for the start time
        wait_for_start_time_start();
    } else {
        // We are already in the air. We enter position hold mode as we don't
        // know where the show clock is
        loiter_start();
    }
}

// starts the phase where we are waiting for the start time of the show
void ModeDroneShow::wait_for_start_time_start()
{
    _set_stage(DroneShow_WaitForStartTime);
}

// waits for the start time of the show
void ModeDroneShow::wait_for_start_time_run()
{
    float time_until_takeoff_sec = copter.g2.drone_show_manager.get_time_until_takeoff_sec();
    float time_since_takeoff_sec = -time_until_takeoff_sec;
    const float latest_takeoff_attempt_after_scheduled_takeoff_time_in_seconds = 5.0f;

    // TODO(ntamas): what if cancel_requested() is true in AC_DroneShowManager?
    /*
    uint32_t now = AP_HAL::millis();

    if (now >= _next_status_report_due_at)
    {
        float elapsed = copter.g2.drone_show_manager.get_elapsed_time_since_start_sec();

        if (isfinite(elapsed)) {
            if (elapsed >= 0) {
                gcs().send_text(
                    MAV_SEVERITY_INFO, "Time since show start: %.2fs", elapsed
                );
            } else if (elapsed > -86400) {
                gcs().send_text(
                    MAV_SEVERITY_INFO, "Time until show start: %.2fs", -elapsed
                );
            }
        }

        if (time_until_takeoff_sec > 15)
        {
            _next_status_report_due_at = now + 5000;
        }
        else
        {
            _next_status_report_due_at = now + 1000;
        }
    }
    */

    if (time_since_takeoff_sec > latest_takeoff_attempt_after_scheduled_takeoff_time_in_seconds + 1) {
        // We are late to the party, just move to the landed or poshold state.
        // The +1 second is needed to ensure that we show a "giving up"
        // failure message below
        if (is_disarmed_or_landed()) {
            landed_start();
        } else {
            // In theory, this branch should not happen because we don't move to
            // the "wait for start time" phase if we are flying
            loiter_start();
        }
    } else {
        if (time_until_takeoff_sec <= 10) {
            if (!_preflight_calibration_done) {
                // We calibrate the barometer 10 seconds before our takeoff time.
                // This will reset the internal AGL measurement to zero.
                //
                // Preflight calibration does not hurt anyone so we don't need the
                // takeoff authorization for this

                // This is copied from GCS_MAVLINK::_handle_command_preflight_calibration_baro()
                AP::baro().update_calibration();

                _preflight_calibration_done = true;
            }
        } else {
            // We still have plenty of time until takeoff so note that we haven't
            // done the preflight calibration yet.
            _preflight_calibration_done = false;
        }

        // For the remaining parts, we need takeoff authorization
        if (copter.g2.drone_show_manager.has_authorization_to_start()) {
            if (time_until_takeoff_sec <= 5 && !_motors_started) {
                // We attempt to start the motors 5 seconds before our takeoff time,
                // and we keep on doing so until 5 seconds after the takeoff time, when
                // we give up
                if (time_since_takeoff_sec < latest_takeoff_attempt_after_scheduled_takeoff_time_in_seconds) {
                    if (start_motors_if_needed()) {
                        // Great, motors started
                        _motors_started = true;
                    }
                } else {
                    // Do not change or remove this message; it is used in test cases
                    error_start();
                }
            }

            if (time_until_takeoff_sec <= 0 && _motors_started) {
                // Time to take off!
                takeoff_start();
            }
        } else {
            // No authorization; stop the motors if we have started them. Note
            // that we don't do anything if _motors_started is false. The motors
            // may still be running in this case, but in this case they were
            // started by the operator with a MAVLink command for testing
            // purposes.
            if (_motors_started) {
                if (AP::arming().is_armed()) {
                    AP::arming().disarm(AP_Arming::Method::SCRIPTING);
                }

                _motors_started = false;
            }
        }
    }
}

// starts the phase where we are taking off at the start of the show
void ModeDroneShow::takeoff_start()
{
    Location current_loc(copter.current_loc);
    int32_t current_alt;

    // check whether the drone knows its own position
    if (!copter.current_loc.initialised())
    {
        // This should not happen, but nevertheless let's move to the
        // error state if we don't know where we are
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to take off, no known location");
        error_start();
        return;
    }

    // get current altitude above home because the home is not necessarily at
    // the place where we are currently taking off
    if (!current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, current_alt)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        error_start();
        return;
    }

    // check if user has set show origin and orientation and do not allow
    // takeoff if not
    if (!(copter.g2.drone_show_manager.has_explicit_show_origin_set_by_user() &&
        copter.g2.drone_show_manager.has_explicit_show_orientation_set_by_user()))
    {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_INITIALISE);
        error_start();
        return;
    }

    // now that we are past the basic checks, we can commit ourselves to entering
    // takeoff mode
    _set_stage(DroneShow_Takeoff);

    // notify the drone show manager that we are about to take off. The drone
    // show manager _may_ use our current position and heading as show origin
    // if no explicit show origin was set
    copter.g2.drone_show_manager.notify_takeoff();

    // set speed limits on the waypoint navigation subsystem. This is copied
    // from ModeLand::init()
    /*
    pos_control->set_max_speed_accel_z(
        wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(),
        wp_nav->get_accel_z()
    );
    pos_control->set_correction_speed_accel_z(
        wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(),
        wp_nav->get_accel_z()
    );
    */

    // initialise position and desired velocity
    /*
    pos_control->init_z_controller();
    pos_control->set_vel_desired_z_cms(AC_DroneShowManager::TAKEOFF_SPEED_METERS_PER_SEC * 100);
    */

    // the body of this function from here on is mostly copied from
    // ModeAuto::takeoff_start()

    // set the current waypoint destination above the current position
    Location target_loc(current_loc);
    target_loc.set_alt_cm(
        current_alt + AC_DroneShowManager::TAKEOFF_ALTITUDE_METERS * 100.0f,
        Location::AltFrame::ABOVE_HOME
    );

    // gcs().send_text(MAV_SEVERITY_INFO, "Takeoff target set to %ld %ld %ld", (long int)target_loc.lat, (long int)target_loc.lng, (long int)target_loc.alt);

    if (!wp_nav->set_wp_destination_loc(target_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        error_start();
        return;
    }

    if (wp_nav->reached_wp_destination()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff target already reached");
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear I term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    auto_takeoff_set_start_alt();

    // pretend that we were armed by the user by raising the throttle; the auto
    // takeoff routine won't work without this.
    copter.set_auto_armed(true);

    // gcs().send_text(MAV_SEVERITY_INFO, "Taking off");
}

// performs the takeoff stage
void ModeDroneShow::takeoff_run()
{
    auto_takeoff_run();

    if (cancel_requested()) {
        // if a cancellation was requested, land immediately
        landing_start();
    } else if (!motors->armed()) {
        // if the motors are not armed any more, something is wrong so move to the
        // error stage. This typically happens if we crash during a show.
        error_start();
    } else if (takeoff_completed()) {
        // if the takeoff has finished, move to the "show" stage
        performing_start();
    } else if (takeoff_timed_out()) {
        // if the takeoff takes too long, move to the "show" stage anyway after
        // emitting a warning
        gcs().send_text(MAV_SEVERITY_WARNING, "Takeoff took too long");
        performing_start();
    }
}

// returns whether the takeoff operation has finished successfully. Must be called
// from the takeoff stage only.
bool ModeDroneShow::takeoff_completed() const
{
    if (_stage == DroneShow_Takeoff) {
        return wp_nav->reached_wp_destination();
    } else if (_stage >= DroneShow_Performing && _stage <= DroneShow_Landed) {
        return true;
    } else {
        return false;
    }
}

// returns whether the current takeoff attempt is taking too long time
bool ModeDroneShow::takeoff_timed_out() const
{
    if (_stage == DroneShow_Takeoff) {
        return AP_HAL::millis() - _last_stage_change_at > 5000;
    } else {
        return false;
    }
}

// starts the phase where we are actually performing the show
void ModeDroneShow::performing_start()
{
    _set_stage(DroneShow_Performing);

#if LANDING_GEAR_ENABLED == ENABLED
    // optionally retract landing gear
    copter.landinggear.retract_after_takeoff();
#endif

    // call regular guided flight mode initialisation
    copter.mode_guided.init(true);

    // initialise guided start time and position as reference for limit checking
    copter.mode_guided.limit_init_time_and_pos();

    // gcs().send_text(MAV_SEVERITY_INFO, "Starting show");
}

// performs the takeoff stage
void ModeDroneShow::performing_run()
{
    static uint32_t last_guided_command = 0;
    bool exited_mode = 0;
    uint32_t now = AP_HAL::millis();

    if (now - last_guided_command >= 250) {
        if (!send_guided_mode_command_during_performance()) {
            // Failed to send guided mode command; try to switch to position
            // hold instead. This should not happen anyway.
            loiter_start();
            exited_mode = 1;
        }
        last_guided_command = now;
    }

    // call regular guided flight mode run function
    if (!exited_mode) {
        copter.mode_guided.run();
    }

    if (cancel_requested()) {
        // if a cancellation was requested, return to home and then land
        rtl_start();
    } else if (!motors->armed()) {
        // if the motors are not armed any more, something is wrong so move to the
        // error stage. This typically happens if we crash during a show.
        error_start();
    } else if (performing_completed()) {
        // if we have finished the show, land
        landing_start();
    }
}

bool ModeDroneShow::performing_completed() const
{
    // TODO(ntamas): what if we are late and we are not at the designated landing
    // position yet?
    return copter.g2.drone_show_manager.get_time_until_landing_sec() <= 0;
}

// starts the phase where we are landing at the place where we are, used at
// the end of a show
void ModeDroneShow::landing_start()
{
    _set_stage(DroneShow_Landing);

    // gcs().send_text(MAV_SEVERITY_INFO, "Landing");

    // TODO(ntamas): set stopping point of loiter nav properly so we land as
    // close to our destination as possible

    // call regular land flight mode initialisation and ask it to ignore checks
    copter.mode_land.init(/* ignore_checks = */ true);
}

// performs the landing stage
void ModeDroneShow::landing_run()
{
    /*
    uint64_t now = AP_HAL::micros64();
    static uint64_t last = 0;

    if (now - last >= 1000000) {
        gcs().send_text(
            MAV_SEVERITY_INFO, "Land complete: %s, spool state: %d",
            copter.ap.land_complete ? "yes" : "no",
            static_cast<int>(motors->get_spool_state())
        );
        last = now;
    }
    */

    // call regular land flight mode run function
    copter.mode_land.run();

    // if we have finished landing, move to the "landed" state
    if (landing_completed()) {
        landed_start();
    }
}

// returns whether the landing operation has finished successfully. Must be called
// from the landing stage only.
bool ModeDroneShow::landing_completed() const
{
    if (_stage == DroneShow_Landing) {
        return (
            copter.ap.land_complete &&
            motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE
        );
    } else {
        return false;
    }
}

// starts the phase where we are returning to our home position, used during
// aborted shows
void ModeDroneShow::rtl_start()
{
    _set_stage(DroneShow_RTL);

    // gcs().send_text(MAV_SEVERITY_INFO, "Return to home");

    // call regular RTL flight mode initialisation and ask it to ignore checks
    copter.mode_rtl.init(/* ignore_checks = */ true);
}

// performs the return to landing position stage
void ModeDroneShow::rtl_run()
{
    // call regular rtl flight mode run function
    copter.mode_rtl.run(/* disarm_on_land = */ false);

    // if we have finished landing, move to the "landed" state
    if (rtl_completed()) {
        landed_start();
    }
}

// returns whether the RTL operation has finished successfully. Must be called
// from the RTL stage only.
bool ModeDroneShow::rtl_completed() const
{
    if (_stage == DroneShow_RTL) {
        return (
            copter.mode_rtl.state_complete() && 
            (copter.mode_rtl.state() == ModeRTL::SubMode::FINAL_DESCENT || copter.mode_rtl.state() == ModeRTL::SubMode::LAND) &&
            (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
        );
    } else {
        return false;
    }
}

// starts the phase where we are holding our position indefinitely; this happens
// when we exited show mode and then entered it again while in the air
void ModeDroneShow::loiter_start()
{
    _set_stage(DroneShow_Loiter);

    // call regular position hold flight mode initialisation
    copter.mode_loiter.init(true);

    // gcs().send_text(MAV_SEVERITY_INFO, "Holding position");
}

// performs the return to landing position stage
void ModeDroneShow::loiter_run()
{
    // call regular position hold flight mode run function
    copter.mode_loiter.run();
}

// starts the phase where we have landed after a show and we do nothing any more
void ModeDroneShow::landed_start()
{
    _set_stage(DroneShow_Landed);

    // gcs().send_text(MAV_SEVERITY_INFO, "Landed successfully");

    copter.g2.drone_show_manager.notify_landed();
}

// performs the landed stage where we do nothing any more
void ModeDroneShow::landed_run()
{
    bool has_start_time = copter.g2.drone_show_manager.has_scheduled_start_time();

    // Ensure that we stay disarmed even if someone tries to arm us remotely
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }

    // If the start time of the show is moved to the future, start the
    // initialization process again
    if (has_start_time) {
        float time_until_start_sec = copter.g2.drone_show_manager.get_time_until_start_sec();

        // The upper limit (43200 sec = 12 hours) is needed to cater for the
        // case when the GCS sends us the _original_ start time of the show
        // again (after landing) as it will then be interpreted in the next
        // GPS week (since it is in the past in the current GPS week). We don't
        // want to go back to the "waiting for start time" state in this case.
        if (time_until_start_sec > 10.0f && time_until_start_sec <= 43200.f) {
            initialization_start();
        }
    }
}

// starts the error phase where we have failed to start a show and we do nothing any more
void ModeDroneShow::error_start()
{
    _set_stage(DroneShow_Error);
}

// performs the error stage where we do nothing any more
void ModeDroneShow::error_run()
{
    // Ensure that we stay disarmed even if someone tries to arm us remotely
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }
}

// Handler function that is called when the start time of the show has changed in the
// drone show manager
void ModeDroneShow::notify_start_time_changed()
{
    // Clear whether the preflight calibration was performed
    _preflight_calibration_done = false;

    // Clear whether the motors were started
    _motors_started = false;

    // Report the new start time immediately in a STATUSTEXT message
    _next_status_report_due_at = 0;
}

// Sends a guided mode command during the show performance, calculated from the
// trajectory that the drone should follow
bool ModeDroneShow::send_guided_mode_command_during_performance()
{
    Location loc;
    Vector3f pos;
    Vector3f vel;
    // static uint8_t counter = 0;

    float elapsed = copter.g2.drone_show_manager.get_elapsed_time_since_start_sec();

    copter.g2.drone_show_manager.get_desired_global_position_in_cms_at_seconds(elapsed, loc);
    copter.g2.drone_show_manager.get_desired_velocity_neu_in_cms_per_seconds_at_seconds(elapsed, vel);

    if (loc.get_vector_from_origin_NEU(pos))
    {
        /*
        counter++;
        if (counter > 4) {
            gcs().send_text(MAV_SEVERITY_INFO, "%.2f %.2f %.2f -- %.2f %.2f %.2f", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z);
        }
        */

        if (copter.g2.drone_show_manager.is_velocity_control_enabled())
        {
            copter.mode_guided.set_destination_posvel(pos, vel);
        }
        else
        {
            copter.mode_guided.set_destination(pos);
        }

        return true;
    }
    else
    {
        // No EKF origin yet, this should not have happened
        return false;
    }
}

// Starts the motors before the show if they are not running already
bool ModeDroneShow::start_motors_if_needed()
{
    if (AP::arming().is_armed()) {
        // Already armed
        return true;
    } else if (_prevent_arming_until_msec > AP_HAL::millis()) {
        // Arming prevented because we have tried it recently
        return false;
    } else if (AP::arming().arm(AP_Arming::Method::SCRIPTING, /* do_arming_checks = */ true)) {
        // Started motors successfully
        return true;
    } else {
        // Prearm checks failed; prevent another attempt for the next second
        _prevent_arming_until_msec = AP_HAL::millis() + 1000;
        return false;
    }
}

// Sets the stage of the drone show module and synchronizes it with the DroneShowManager
void ModeDroneShow::_set_stage(DroneShowModeStage value)
{
    _stage = value;
    _last_stage_change_at = AP_HAL::millis();
    copter.g2.drone_show_manager.notify_drone_show_mode_entered_stage(_stage);
}