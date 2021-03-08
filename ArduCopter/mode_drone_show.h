#pragma once

#include <AC_DroneShowManager/AC_DroneShowManager.h>

// Provide Copter-specific implementation of the drone show mode. While most of
// the logic for performing a show is present in AC_DroneShowManager, this class
// allows Copter to override base functionality - for example, to switch flight
// mode when the show is authorized.
class AC_DroneShowManager_Copter : public AC_DroneShowManager {
public:

    using AC_DroneShowManager::AC_DroneShowManager;

    /* Do not allow copies */
    AC_DroneShowManager_Copter(const AC_DroneShowManager_Copter &other) = delete;
    AC_DroneShowManager_Copter &operator=(const AC_DroneShowManager_Copter&) = delete;

    virtual bool _get_current_location(Location& loc) const override;
    virtual void _request_switch_to_show_mode() override;
    
};

class ModeDroneShow : public Mode {


public:
    ModeDroneShow();
    Number mode_number() const override { return Number::DRONE_SHOW; }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override;
    virtual void exit() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return true; }

    bool is_landing() const override;
    bool is_taking_off() const override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    const char *name() const override { return "DRONE_SHOW"; }
    const char *name4() const override { return "SHOW"; }

    // for reporting to GCS
    bool get_wp(Location &loc) override;
    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override;

private:

    // --- Internal variables ---

    // Execution stage of the show
    DroneShowModeStage _stage;

    // Stores whether we have attempted to start the motors, due 10 seconds
    // before takeoff.
    bool _motors_started;

    // Stores the timestamp when we have changed the execution stage the
    // last time
    uint32_t _last_stage_change_at;

    // Stores when we are supposed to print the next status report as a STATUSTEXT
    // message
    uint64_t _next_status_report_due_at;

    // Stores whether we have set the home position to the takeoff position
    // before takeoff.
    bool _home_position_set;

    // Stores whether we have performed the preflight calibration before takeoff.
    bool _preflight_calibration_done;

    // Timestamp until we block arming attempts during the startup phase if we
    // have attempted to arm the drone recently
    uint32_t _prevent_arming_until_msec;

    // Sets the stage of the execution to the given value
    void _set_stage(DroneShowModeStage value);

    bool cancel_requested() const;

    void check_changes_in_parameters();
    void notify_start_time_changed();
    bool send_guided_mode_command_during_performance();
    bool start_motors_if_needed();

    void initialization_start();
    void initialization_run();

    void wait_for_start_time_start();
    void wait_for_start_time_run();

    void takeoff_start();
    void takeoff_run();
    bool takeoff_completed() const;
    bool takeoff_timed_out() const;

    void performing_start();
    void performing_run();
    bool performing_completed() const;

    void landing_start();
    void landing_run();
    bool landing_completed() const;

    void rtl_start();
    void rtl_run();
    bool rtl_completed() const;

    void loiter_start();
    void loiter_run();

    void landed_start();
    void landed_run();

    void error_start();
    void error_run();
};
