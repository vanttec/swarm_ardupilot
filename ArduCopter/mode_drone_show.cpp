#include "Copter.h"

/*
 * Implementation of drone show flight mode
 */

const AP_Param::GroupInfo ModeDroneShow::var_info[] = {
    // @Param: START_TIME
    // @DisplayName: Start time
    // @Description: Start time of drone show as a GPS time of week timestamp (sec), negative if unset
    // @Range: -1 604799
    // @Increment: 1
    // @Units: sec
    // @User: Standard
    // 
    // Note that we cannot use UNIX timestamps here because ArduPilot stores
    // all parameters as floats, and floats can represent integers accurately
    // only up to 2^23 - 1
    AP_GROUPINFO("START_TIME", 1, ModeDroneShow, _start_time_gps_sec, -1),

    AP_GROUPEND
};

// Constructor.
ModeDroneShow::ModeDroneShow(void) : Mode(),
    _stage(DroneShow_WaitForStartTime)
{
    // Ensures that the parameters always revert to the defaults when the
    // UAV is rebooted.
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeDroneShow::init(bool ignore_checks)
{
    // Clear the last seen start time so we show a message to acknowledge any
    // new start time set by the user, even if it was set earlier
    _last_seen_start_time_gps_sec = -1;

    // Clear the timestamp when we last attempted to arm the drone
    _prevent_arming_until_msec = 0;

    // Clear all the status information that depends on the start time
    notify_start_time_changed();

    return true;
}

// ModeDroneShow::run - runs the main drone show controller
// should be called at 25hz or more
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

    case DroneShow_RTL:
        // returning to landing position
        rtl_run();
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
    if (_start_time_gps_sec != _last_seen_start_time_gps_sec) {
        _last_seen_start_time_gps_sec = _start_time_gps_sec;

        notify_start_time_changed();

        if (_start_time_gps_sec >= 0) {
            gcs().send_text(
                MAV_SEVERITY_INFO, "Start time set to %llu",
                static_cast<unsigned long long int>(_start_time_usec)
            );
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Start time cleared");
        }
    }
}

bool ModeDroneShow::get_wp(Location& destination)
{
    switch (_stage) {
    // TODO(ntamas): implement this for the other stages
    case DroneShow_RTL:
        return copter.mode_rtl.get_wp(destination);
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

// initializes the drone show mode after it has been activated the first time
void ModeDroneShow::initialization_run()
{
    // First we need to decide whether we are in the air or not
    if (is_disarmed_or_landed()) {
        // Great, let's move to the state where we wait for the start time
        wait_for_start_time_start();
    } else {
        // We are already in the air
        // TODO(ntamas): this is not handled yet; we should attempt to resume
        // the show if possible
        rtl_start();
    }
}

// starts the phase where we are waiting for the start time of the show
void ModeDroneShow::wait_for_start_time_start()
{
    _stage = DroneShow_WaitForStartTime;

    gcs().send_text(MAV_SEVERITY_INFO, "Waiting for start time of show");
}

// waits for the start time of the show
void ModeDroneShow::wait_for_start_time_run()
{
    uint64_t now = AP_HAL::micros64();
    float time_until_start_sec = get_time_until_start_sec();
    static uint64_t last;

    if (now - last >= 1000000) {
        float elapsed = get_elapsed_time_since_start_sec();
        if (elapsed >= 0) {
            gcs().send_text(
                MAV_SEVERITY_INFO, "Time since show start: %.2fs", elapsed
            );
        } else {
            gcs().send_text(
                MAV_SEVERITY_INFO, "Time until show start: %.2fs", -elapsed
            );
        }

        last = now;
    }

    if (time_until_start_sec <= 15 && !_preflight_calibration_done) {
        // This is copied from GCS_MAVLINK::_handle_command_preflight_calibration_baro()
        gcs().send_text(MAV_SEVERITY_INFO, "Updating barometer calibration");
        AP::baro().update_calibration();
        gcs().send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");

        // TODO(ntamas): also perform gyro calibration?

        _preflight_calibration_done = true;
    }

    if (time_until_start_sec <= 10 && !_motors_started) {
        // We attempt to start the motors 10 seconds before the show, and we
        // keep on doing so until 5 seconds into the show, after which we give up
        if (time_until_start_sec >= -5) {
            start_motors_if_needed();
        } else {
            error_start();
        }
    }
}

// starts the phase where we are returning to our landing position
void ModeDroneShow::rtl_start()
{
    _stage = DroneShow_RTL;

    gcs().send_text(MAV_SEVERITY_INFO, "Return to land");

    // call regular rtl flight mode initialisation and ask it to ignore checks
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
bool ModeDroneShow::rtl_completed()
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

// starts the phase where we have landed after a show and we do nothing any more
void ModeDroneShow::landed_start()
{
    _stage = DroneShow_Landed;

    gcs().send_text(MAV_SEVERITY_INFO, "Landed successfully");
}

// performs the landed stage where we do nothing any more
void ModeDroneShow::landed_run()
{
    // Ensure that we stay disarmed even if someone tries to arm us remotely 
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }
}

// starts the error phase where we have failed to start a show and we do nothing any more
void ModeDroneShow::error_start()
{
    _stage = DroneShow_Error;

    // Do not change or remove this message; it is used in test cases
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to start, giving up");
}

// performs the error stage where we do nothing any more
void ModeDroneShow::error_run()
{
    // Ensure that we stay disarmed even if someone tries to arm us remotely 
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }
}

// returns the elapsed time since the start of the show, in microseconds
int64_t ModeDroneShow::get_elapsed_time_since_start_usec() const
{
    if (_start_time_usec > 0) {
        uint64_t now = AP::gps().time_epoch_usec();
        uint64_t diff;
        if (_start_time_usec > now) {
            diff = _start_time_usec - now;
            if (diff < INT64_MAX) {
                return -diff;
            } else {
                return INT64_MIN;
            }
        } else if (_start_time_usec < now) {
            diff = now - _start_time_usec;
            if (diff < INT64_MAX) {
                return diff;
            } else {
                return INT64_MAX;
            }
        } else {
            return 0;
        }
    } else {
        return INT64_MIN;
    }
}

// returns the elapsed time since the start of the show, in seconds
float ModeDroneShow::get_elapsed_time_since_start_sec() const
{
    int64_t elapsed_usec = get_elapsed_time_since_start_usec();
    return static_cast<float>(elapsed_usec / 1000) / 1000.0f;    
}

// returns the time until the start of the show, in microseconds
int64_t ModeDroneShow::get_time_until_start_usec() const
{
    return -get_elapsed_time_since_start_usec();
}

// returns the time until the start of the show, in seconds
float ModeDroneShow::get_time_until_start_sec() const
{
    return -get_elapsed_time_since_start_sec();
}

// Handler function that is called when the start time of the show has changed in the parameters
void ModeDroneShow::notify_start_time_changed()
{
    uint32_t start_time_gps_msec;

    // Clear whether the preflight calibration was performed
    _preflight_calibration_done = false;

    // Clear whether the motors were started
    _motors_started = false;
    
    // GPS-based start time changed, let's report it and convert it to
    // a UNIX timestamp that we can work with
    start_time_gps_msec = _start_time_gps_sec * 1000;
    if (AP::gps().time_week_ms() < start_time_gps_msec) {
        // Interpret the given timestamp in the current GPS week as it is in
        // the future even with the same GPS week number
        _start_time_usec = AP::gps().time_epoch_convert(
            AP::gps().time_week(), start_time_gps_msec
        ) * 1000ULL;
    } else {
        // Interpret the given timestamp in the next GPS week as it is in
        // the past with the same GPS week number
        _start_time_usec = AP::gps().time_epoch_convert(
            AP::gps().time_week() + 1, start_time_gps_msec
        ) * 1000ULL;
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
    } else if (AP::arming().arm(AP_Arming::Method::MAVLINK, /* do_arming_checks = */ true)) {
        // Started motors successfully
        gcs().send_text(MAV_SEVERITY_INFO, "Armed successfully");
        return true;
    } else {
        // Prearm checks failed; prevent another attempt for the next second
        _prevent_arming_until_msec = AP_HAL::millis() + 1000;
        return false;
    }
}