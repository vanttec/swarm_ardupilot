#include <AP_Filesystem/AP_Filesystem_Available.h>
#include <GCS_MAVLink/GCS.h>

#if HAVE_FILESYSTEM_SUPPORT

#include <sys/stat.h>
#include <sys/types.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/DroneShowNotificationBackend.h>
#include <AP_Param/AP_Param.h>

#include "AC_DroneShowManager.h"
#include <AC_Fence/AC_Fence.h>

#include <skybrush/skybrush.h>

#include "DroneShowLEDFactory.h"

#undef RED     // from AP_Notify/RGBLed.h
#undef GREEN   // from AP_Notify/RGBLed.h
#undef BLUE    // from AP_Notify/RGBLed.h
#undef YELLOW  // from AP_Notify/RGBLed.h
#undef WHITE   // from AP_Notify/RGBLed.h

#ifndef HAL_BOARD_COLLMOT_DIRECTORY
#  if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#    define HAL_BOARD_COLLMOT_DIRECTORY "./collmot"
#  else
#    define HAL_BOARD_COLLMOT_DIRECTORY "/COLLMOT"
#  endif
#endif

#define SHOW_FILE (HAL_BOARD_COLLMOT_DIRECTORY "/show.skyb")

// Default update rate for position and velocity targets
#define DEFAULT_UPDATE_RATE_HZ 10

// Length of a GPS week in seconds
#define GPS_WEEK_LENGTH_SEC 604800

// Smallest valid value of show AMSL. Values smaller than this are considered unset.
#define SMALLEST_VALID_AMSL -9999999

// Largest valid value of show AMSL. Values smaller than this are considered invalid.
#define LARGEST_VALID_AMSL 10000000

// Group mask indicating all groups
#define ALL_GROUPS 0

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// UDP port that the drone show manager uses to broadcast the status of the RGB light
// when compiled with the SITL simulator. Uncomment if you need it.
// #  define RGB_SOCKET_PORT 4245
#endif

extern const AP_HAL::HAL &hal;

// Undefine some macros from RGBLed.h that are in comflict with the code below
#undef BLACK
#undef RED
#undef GREEN
#undef BLUE
#undef YELLOW
#undef WHITE

namespace Colors {
    static const sb_rgb_color_t BLACK = { 0, 0, 0 };
    static const sb_rgb_color_t RED = { 255, 0, 0 };
    static const sb_rgb_color_t YELLOW = { 255, 255, 0 };
    static const sb_rgb_color_t GREEN = { 0, 255, 0 };
    static const sb_rgb_color_t GREEN_DIM = { 0, 128, 0 };
    static const sb_rgb_color_t ORANGE = { 255, 128, 0 };
    static const sb_rgb_color_t WHITE_DIM = { 128, 128, 128 };
    static const sb_rgb_color_t LIGHT_BLUE = { 0, 128, 255 };
    static const sb_rgb_color_t MAGENTA = { 255, 0, 255 };
    static const sb_rgb_color_t WHITE = { 255, 255, 255 };
};

namespace CustomPackets {
    static const uint8_t START_CONFIG = 1;

    typedef struct PACKED {
        // Start time to set on the drone, in GPS time of week (sec). Anything
        // larger than 604799 means not to touch the start time that is
        // currently set. Negative number means that the start time must be
        // cleared.
        int32_t start_time;
        uint8_t is_authorized;
    } start_config_t;
};

const AP_Param::GroupInfo AC_DroneShowManager::var_info[] = {
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
    AP_GROUPINFO("START_TIME", 1, AC_DroneShowManager, _params.start_time_gps_sec, -1),

    // @Param: ORIGIN_LAT
    // @DisplayName: Origin (latitude)
    // @Description: Latitude of the origin of the drone show coordinate system, zero if unset
    // @Range: -900000000 900000000
    // @Increment: 1
    // @Units: 1e-7 degrees
    // @User: Standard
    AP_GROUPINFO("ORIGIN_LAT", 2, AC_DroneShowManager, _params.origin_lat, 0),

    // @Param: ORIGIN_LNG
    // @DisplayName: Origin (longitude)
    // @Description: Longitude of the origin of the drone show coordinate system, zero if unset
    // @Range: -1800000000 1800000000
    // @Increment: 1
    // @Units: 1e-7 degrees
    // @User: Standard
    AP_GROUPINFO("ORIGIN_LNG", 3, AC_DroneShowManager, _params.origin_lng, 0),

    // @Param: ORIGIN_AMSL
    // @DisplayName: Origin (altitude)
    // @Description: Altitude of the origin of the drone show coordinate system, -10000000 or smaller if unset
    // @Range: -10000000 10000000
    // @Increment: 1
    // @Units: mm
    // @User: Standard
    AP_GROUPINFO("ORIGIN_AMSL", 12, AC_DroneShowManager, _params.origin_amsl, SMALLEST_VALID_AMSL - 1),

    // @Param: ORIENTATION
    // @DisplayName: Orientation
    // @Description: Orientation of the X axis of the show coordinate system, -1 if unset
    // @Range: -1 360
    // @Increment: 1
    // @Units: degrees
    // @User: Standard
    AP_GROUPINFO("ORIENTATION", 4, AC_DroneShowManager, _params.orientation_deg, -1),

    // @Param: START_AUTH
    // @DisplayName: Authorization to start
    // @Description: Whether the drone is authorized to start
    // @Range: 0 1
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("START_AUTH", 5, AC_DroneShowManager, _params.authorized_to_start, 0),

    // @Param: LED0_TYPE
    // @DisplayName: Assignment of LED channel 0 to a LED output type
    // @Description: Specifies where the output of the main LED light track of the show should be sent
    // @Values: 0:Off, 1:MAVLink, 2:NeoPixel, 3:ProfiLED, 4:Debug, 5:SITL, 6:Servo
    // @User: Advanced
    AP_GROUPINFO("LED0_TYPE", 6, AC_DroneShowManager, _params.led_specs[0].type, 0),

    // @Param: LED0_CHAN
    // @DisplayName: PWM or MAVLink channel to use for the LED output
    // @Description: PWM channel to use for the LED output (1-based) if the LED type is "NeoPixel" or "ProfiLED", or the MAVLink channel to use if the LEF type is "MAVLink"
    // @User: Advanced
    AP_GROUPINFO("LED0_CHAN", 8, AC_DroneShowManager, _params.led_specs[0].channel, 0),

    // @Param: LED0_COUNT
    // @DisplayName: Number of individual LEDs on a LED channel
    // @Description: Specifies how many LEDs there are on a NeoPixel or ProfiLED LED strip
    // @User: Advanced
    AP_GROUPINFO("LED0_COUNT", 7, AC_DroneShowManager, _params.led_specs[0].count, 16),

    // @Param: MODE_BOOT
    // @DisplayName: Conditions for entering show mode
    // @Description: Bitfield that Specifies when the drone should switch to show mode automatically
    // @Values: 3:At boot and when authorized,2:When authorized,1:At boot,0:Never
    // @Bitmask: 0:At boot,1:When authorized
    // @User: Standard
    AP_GROUPINFO("MODE_BOOT", 9, AC_DroneShowManager, _params.show_mode_settings, 2),

    // @Param: PRE_LIGHTS
    // @DisplayName: Brightness of preflight check related lights
    // @Description: Controls the brightness of light signals on the drone that are used to report status information when the drone is on the ground
    // @Range: 0 4
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PRE_LIGHTS", 10, AC_DroneShowManager, _params.preflight_light_signal_brightness, 2),

    // @Param: CTRL_MODE
    // @DisplayName: Flags to configure the position control algorithm
    // @Description: Controls various aspects of the position control algorithm built into the firmware
    // @Values: 1:Position and velocity control,0:Position control only
    // @Bitmask: 0:VelCtrl
    // @User: Advanced
    AP_GROUPINFO("CTRL_MODE", 11, AC_DroneShowManager, _params.control_mode_flags, DroneShowControl_VelocityControlEnabled),

    // @Param: GROUP
    // @DisplayName: Group index
    // @Description: Index of the group that this drone belongs to
    // @Range: 0 7
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GROUP", 13, AC_DroneShowManager, _params.group_index, 0),

    // @Param: CTRL_RATE
    // @DisplayName: Target update rate
    // @Description: Update rate of the target position and velocity during the show
    // @Range: 1 50
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("CTRL_RATE", 14, AC_DroneShowManager, _params.control_rate_hz, DEFAULT_UPDATE_RATE_HZ),

    AP_GROUPEND
};

// LED factory that is used to create new RGB LED instances
static DroneShowLEDFactory _rgb_led_factory_singleton;

static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage);
static float get_modulation_factor_for_light_effect(
    uint32_t timestamp, LightEffectType effect, uint16_t period_msec, uint16_t phase_msec
);

AC_DroneShowManager::AC_DroneShowManager() :
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _sock_rgb(true),
    _sock_rgb_open(false),
#endif
    _show_data(0),
    _trajectory_valid(false),
    _light_program_valid(false),
    _stage_in_drone_show_mode(DroneShow_Off),
    _start_time_source(StartTimeSource::NONE),
    _start_time_usec(0),
    _takeoff_time_sec(0),
    _landing_time_sec(0),
    _total_duration_sec(0),
    _cancel_requested(false),
    _controller_update_delta_msec(1000 / DEFAULT_UPDATE_RATE_HZ),
    _rgb_led(0),
    _rc_start_switch_blocked_until(0),
    _boot_count(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    _trajectory = new sb_trajectory_t;
    sb_trajectory_init_empty(_trajectory);

    _trajectory_player = new sb_trajectory_player_t;
    sb_trajectory_player_init(_trajectory_player, _trajectory);

    _light_program = new sb_light_program_t;
    sb_light_program_init_empty(_light_program);

    _light_player = new sb_light_player_t;
    sb_light_player_init(_light_player, _light_program);

    // Don't call _update_rgb_led_instance() here, servo framework is not set
    // up yet
}

AC_DroneShowManager::~AC_DroneShowManager()
{
    sb_light_player_destroy(_light_player);
    delete _light_player;

    sb_light_program_destroy(_light_program);
    delete _light_program;

    sb_trajectory_player_destroy(_trajectory_player);
    delete _trajectory_player;

    sb_trajectory_destroy(_trajectory);
    delete _trajectory;
}

void AC_DroneShowManager::early_init()
{
    // AP::FS().mkdir() apparently needs lots of free memory, see:
    // https://github.com/ArduPilot/ardupilot/issues/16103
    EXPECT_DELAY_MS(3000);

    if (AP::FS().mkdir(HAL_BOARD_COLLMOT_DIRECTORY) < 0) {
        if (errno == EEXIST) {
            // Directory already exists, this is okay
        } else {
            hal.console->printf(
                "Failed to create directory %s: %s (code %d)\n",
                 HAL_BOARD_COLLMOT_DIRECTORY, strerror(errno), errno
            );
        }
    }
}

void AC_DroneShowManager::init()
{
    // Get the boot count from the parameters
    enum ap_var_type ptype;
    AP_Int16* boot_count_param = static_cast<AP_Int16*>(AP_Param::find("STAT_BOOTCNT", &ptype));
    _boot_count = boot_count_param ? (*boot_count_param) : 0;

    // Get a reference to the RGB LED factory
    _rgb_led_factory = &_rgb_led_factory_singleton;

    // Clear start time and authorization now; at this point the parameter
    // subsystem has already loaded back the previous value from the EEPROM so
    // we are safe to overwrite it
    _params.start_time_gps_sec = -1;
    _params.authorized_to_start = 0;

    _load_show_file_from_storage();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _open_rgb_led_socket();
#endif
    _update_rgb_led_instance();
}

bool AC_DroneShowManager::configure_show_coordinate_system(
    int32_t lat, int32_t lng, int32_t amsl_mm, float orientation_deg
) {
    if (!check_latlng(lat, lng)) {
        return false;
    }

    if (amsl_mm >= LARGEST_VALID_AMSL) {
        return false;
    }

    _params.origin_lat = lat;
    _params.origin_lng = lng;
    _params.origin_amsl = amsl_mm;
    _params.orientation_deg = orientation_deg;

    return true;
}

void AC_DroneShowManager::get_color_of_rgb_light_at_seconds(float time, sb_rgb_color_t* color)
{
    *color = sb_light_player_get_color_at(_light_player, time < 0 || time > 86400000 ? 0 : time * 1000);
}

void AC_DroneShowManager::get_desired_global_position_in_cms_at_seconds(float time, Location& loc)
{
    sb_vector3_with_yaw_t vec;
    float offset_north, offset_east;

    sb_trajectory_player_get_position_at(_trajectory_player, time, &vec);

    // We need to rotate the X axis by -_orientation_rad radians so it points
    // North. At the same time, we also flip the Y axis so it pointsEast and
    // not West.
    offset_north = cosf(_orientation_rad) * vec.x + sinf(_orientation_rad) * vec.y;
    offset_east = sinf(_orientation_rad) * vec.x - cosf(_orientation_rad) * vec.y;

    // We have millimeters so far, need to convert the North and East offsets
    // to meters first. ALso, in the Z axis, we will need centimeters.
    offset_north = offset_north / 1000.0f;
    offset_east = offset_east / 1000.0f;
    vec.z = vec.z / 10.0f;

    // Finally, we need to offset the show origin with the calculated North and
    // East offset to get a global position

    // TODO(ntamas): handle yaw!

    loc.zero();
    loc.lat = _origin_lat;
    loc.lng = _origin_lng;

    if (_origin_amsl_is_valid) {
        // Show is controlled in AMSL
        loc.set_alt_cm(
            static_cast<int32_t>(vec.z) /* [cm] */ +
            _origin_amsl / 10.0 /* [mm] -> [cm] */,
            Location::AltFrame::ABSOLUTE
        );
    } else {
        // Show is controlled in AGL. We use altitude above home because the
        // EKF origin could be anywhere -- it is typically established early
        // during the initialization process, while the home is set to the
        // point where the drone is armed.
        loc.set_alt_cm(
            static_cast<int32_t>(vec.z) /* [cm] */,
            Location::AltFrame::ABOVE_HOME
        );
    }

    loc.offset(offset_north, offset_east);
}

void AC_DroneShowManager::get_desired_velocity_neu_in_cms_per_seconds_at_seconds(float time, Vector3f& vel)
{
    sb_vector3_with_yaw_t vec;

    sb_trajectory_player_get_velocity_at(_trajectory_player, time, &vec);

    // We need to rotate the X axis by -_orientation_rad degrees so it
    // points North. At the same time, we also flip the Y axis so it points
    // East and not West.
    vec.x = cosf(_orientation_rad) * vec.x + sinf(_orientation_rad) * vec.y;
    vec.y = sinf(_orientation_rad) * vec.x - cosf(_orientation_rad) * vec.y;

    // We have mm/s so far, need to convert to cm/s
    vel.x = vec.x / 10.0f;
    vel.y = vec.y / 10.0f;
    vel.z = vec.z / 10.0f;

    // TODO(ntamas): handle yaw!
}

// returns the elapsed time since the start of the show, in microseconds
int64_t AC_DroneShowManager::get_elapsed_time_since_start_usec() const
{
    if (_start_time_usec > 0) {
        // AP::gps().time_epoch_usec() is smart enough to handle the case when
        // the GPS fix was lost so no need to worry about loss of GPS fix here.
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

// returns the elapsed time since the start of the show, in milliseconds
int32_t AC_DroneShowManager::get_elapsed_time_since_start_msec() const
{
    int64_t elapsed_usec = get_elapsed_time_since_start_usec();

    // Using -INFINITY here can lead to FPEs on macOS in the SITL simulator
    // when compiling in release mode, hence we use a large negative number
    // representing one day
    if (elapsed_usec <= -86400000000) {
        return -86400000;
    } else if (elapsed_usec >= 86400000) {
        return 86400000;
    } else {
        return static_cast<int32_t>(elapsed_usec / 1000);
    }
}

float AC_DroneShowManager::get_elapsed_time_since_start_sec() const
{
    int64_t elapsed_usec = get_elapsed_time_since_start_usec();

    // Using -INFINITY here can lead to FPEs on macOS in the SITL simulator
    // when compiling in release mode, hence we use a large negative number
    // representing one day
    return elapsed_usec == INT64_MIN ? -86400 : static_cast<float>(elapsed_usec / 1000) / 1000.0f;
}

int64_t AC_DroneShowManager::get_time_until_start_usec() const
{
    return -get_elapsed_time_since_start_usec();
}

float AC_DroneShowManager::get_time_until_start_sec() const
{
    return -get_elapsed_time_since_start_sec();
}

float AC_DroneShowManager::get_time_until_takeoff_sec() const
{
    return get_time_until_start_sec() + get_relative_takeoff_time_sec();
}

float AC_DroneShowManager::get_time_until_landing_sec() const
{
    return get_time_until_start_sec() + get_relative_landing_time_sec();
}

MAV_RESULT AC_DroneShowManager::handle_command_int_packet(const mavlink_command_int_t &packet)
{
    // If you modify anything here, try to implement the same modification(s)
    // in the COMMAND_LONG handler as well!

    switch (packet.command) {

    case MAV_CMD_USER_1: {
        // param1: command code
        // remaining params depend on param1
        if (is_zero(packet.param1)) {
            // Reload current show
            if (reload_or_clear_show(/* do_clear = */ 0)) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        } else if (is_zero(packet.param1 - 1)) {
            // Clear current show
            if (reload_or_clear_show(/* do_clear = */ 1)) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        }

        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }

    case MAV_CMD_USER_2: {
        // param1: command code
        // remaining params depend on param1
        if (is_zero(packet.param1)) {
            // Set show origin, orientation and AMSL with a single command.
            // This is supported with COMMAND_INT MAVLink packets only as we
            // do not want to lose precision in the lat/lng direction due to
            // float representation
            //
            // param4: orientation
            // param5 (x): latitude (degE7)
            // param6 (y): longitude (degE7)
            // param7 (z): AMSL (mm)
            if (configure_show_coordinate_system(
                packet.x, packet.y, static_cast<int32_t>(packet.z),
                packet.param4
            )) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        }

        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }

    default:
        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }
}

MAV_RESULT AC_DroneShowManager::handle_command_long_packet(const mavlink_command_long_t &packet)
{
    // If you modify anything here, try to implement the same modification(s)
    // in the COMMAND_INT handler as well!

    switch (packet.command) {

    case MAV_CMD_USER_1: {
        // param1: command code
        // remaining params depend on param1
        if (is_zero(packet.param1)) {
            // Reload current show
            if (reload_or_clear_show(/* do_clear = */ 0)) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        } else if (is_zero(packet.param1 - 1)) {
            // Clear current show
            if (reload_or_clear_show(/* do_clear = */ 1)) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        }

        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }

    case MAV_CMD_USER_2: {
        // param1: command code
        // remaining params depend on param1
        if (is_zero(packet.param1)) {
            // Set show origin, orientation and AMSL with a single command.
            // This is supported with COMMAND_INT MAVLink packets only as we
            // do not want to lose precision in the lat/lng direction due to
            // float representation
            return MAV_RESULT_UNSUPPORTED;
        }

        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }

    default:
        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }
}

bool AC_DroneShowManager::handle_message(const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        // DATA16, DATA32, DATA64, DATA96 packets are used for custom commands.
        // We do not distinguish between them because MAVLink2 truncates the
        // trailing zeros anyway.
        case MAVLINK_MSG_ID_DATA16:
            return _handle_data16_message(msg);

        case MAVLINK_MSG_ID_DATA32:
            return _handle_data32_message(msg);

        case MAVLINK_MSG_ID_DATA64:
            return _handle_data64_message(msg);

        case MAVLINK_MSG_ID_DATA96:
            return _handle_data96_message(msg);

        case MAVLINK_MSG_ID_LED_CONTROL:
            // The drone show LED listens on the "secret" LED ID 42 with a
            // pattern of 42 as well. Any message that does not match this
            // specification is handled transparently by the "core" MAVLink
            // GCS module.
            return _handle_led_control_message(msg);

        default:
            return false;
    }
}

bool AC_DroneShowManager::has_authorization_to_start() const
{
    return _params.authorized_to_start;
}

bool AC_DroneShowManager::has_explicit_show_altitude_set_by_user() const
{
    return _params.origin_amsl >= SMALLEST_VALID_AMSL;
}

bool AC_DroneShowManager::has_explicit_show_orientation_set_by_user() const
{
    return _params.orientation_deg >= 0;
}

bool AC_DroneShowManager::has_explicit_show_origin_set_by_user() const
{
    return _params.origin_lat != 0 && _params.origin_lng != 0;
}

bool AC_DroneShowManager::loaded_show_data_successfully() const
{
    return _trajectory_valid;
}

void AC_DroneShowManager::notify_drone_show_mode_initialized()
{
    _cancel_requested = false;
    _update_rgb_led_instance();
    _clear_start_time_if_set_by_switch();
}

void AC_DroneShowManager::notify_drone_show_mode_entered_stage(DroneShowModeStage stage)
{
    _stage_in_drone_show_mode = stage;
}

void AC_DroneShowManager::notify_drone_show_mode_exited()
{
    _cancel_requested = false;
    _update_rgb_led_instance();
    _clear_start_time_if_set_by_switch();
}

void AC_DroneShowManager::notify_landed()
{
    _cancel_requested = false;

    // Let's not clear the start time; there's not really much point but at
    // least we don't confuse the GCS (not Skybrush but Mission Planner) with
    // a parameter suddenly changing behind its back. This is just a theoretical
    // possibility but let us be on the safe side.
    // _clear_start_time_after_landing();
}

bool AC_DroneShowManager::notify_takeoff_attempt()
{
    if (!has_explicit_show_origin_set_by_user() || !has_explicit_show_orientation_set_by_user()) {
        return false;
    }
        
    _orientation_rad = radians(_params.orientation_deg);
    _origin_lat = static_cast<int32_t>(_params.origin_lat);
    _origin_lng = static_cast<int32_t>(_params.origin_lng);

    if (has_explicit_show_altitude_set_by_user()) {
        _origin_amsl = _params.origin_amsl;
        if (_origin_amsl >= LARGEST_VALID_AMSL) {
            _origin_amsl = LARGEST_VALID_AMSL;
        }
        _origin_amsl_is_valid = true;
    } else {
        _origin_amsl_is_valid = false;
    }

    return true;
}

bool AC_DroneShowManager::reload_or_clear_show(bool do_clear)
{
    if (do_clear) {
        if (AP::FS().unlink(SHOW_FILE)) {
            // Error while removing the file; did it exist?
            if (errno == ENOENT) {
                // File was missing already, this is OK.
            } else {
                // This is a genuine failure
                return false;
            }
        }
    }

    return reload_show_from_storage();
}

bool AC_DroneShowManager::reload_show_from_storage()
{
    // Don't reload the show if the motors are armed
    if (AP::motors()->armed()) {
        return false;
    }

    // This will return whether a new show was loaded, but we want to return
    // success even if there is no show file, so the return value is ignored
    _load_show_file_from_storage();

    return true;
}

void AC_DroneShowManager::send_drone_show_status(const mavlink_channel_t chan) const
{
    const AP_GPS& gps = AP::gps();

    uint8_t packet[16] = { 0x62, };
    uint8_t flags, gps_health;
    float elapsed_time;
    int16_t encoded_elapsed_time;

    /* make sure that we can make use of MAVLink packet truncation */
    memset(packet, 0, sizeof(uint8_t));

    /* calculate status flags */
    flags = 0;
    if (loaded_show_data_successfully() && has_valid_takeoff_time()) {
        flags |= (1 << 7);
    }
    if (has_scheduled_start_time()) {
        flags |= (1 << 6);
    }
    if (has_explicit_show_origin_set_by_user()) {
        flags |= (1 << 5);
    }
    if (has_explicit_show_orientation_set_by_user()) {
        flags |= (1 << 4);
    }
    if (AP::fence()->enabled()) {
        flags |= (1 << 3);
    }
    if (has_authorization_to_start()) {
        flags |= (1 << 2);
    }
    if (!_is_gps_time_ok()) {
        flags |= (1 << 1);
    }

    /* calculate GPS health */
    gps_health = gps.status();
    if (gps_health > 7) {
        gps_health = 7;
    }
    gps_health |= (gps.num_sats() > 31 ? 31 : gps.num_sats()) << 3;

    /* calculate elapsed time */
    elapsed_time = get_elapsed_time_since_start_sec();
    if (elapsed_time > 32767) {
        encoded_elapsed_time = 32767;
    } else if (elapsed_time <= -32768) {
        encoded_elapsed_time = -32768;
    } else {
        encoded_elapsed_time = static_cast<int16_t>(elapsed_time);
    }

    /* fill the packet */
    *(( int32_t*) (packet + 0)) = _params.start_time_gps_sec;
    *((uint16_t*) (packet + 4)) = sb_rgb_color_encode_rgb565(_last_rgb_led_color);
    packet[6] = flags;
    packet[7] = static_cast<uint8_t>(_stage_in_drone_show_mode) & 0x0f;
    packet[8] = gps_health;
    packet[9] = _boot_count & 0x03; // upper 6 bits are unused yet
    *(( int16_t*) (packet + 10)) = encoded_elapsed_time;

    mavlink_msg_data16_send(
        chan,
        0x5b,   // Skybrush status packet type marker
        12,     // effective packet length
        packet
    );
}

void AC_DroneShowManager::handle_rc_start_switch()
{
    if (_stage_in_drone_show_mode == DroneShow_WaitForStartTime)
    {
        if (!_rc_start_switch_blocked_until || _rc_start_switch_blocked_until < AP_HAL::millis())
        {
            start_if_not_running();

            if (_start_time_source == StartTimeSource::START_METHOD) {
                _start_time_source = StartTimeSource::RC_SWITCH;
            }
        }
    }
}

bool AC_DroneShowManager::should_switch_to_show_mode_at_boot() const
{
    return _params.show_mode_settings & 1;
}

bool AC_DroneShowManager::should_switch_to_show_mode_when_authorized() const
{
    return _params.show_mode_settings & 2;
}

void AC_DroneShowManager::start_if_not_running()
{
    _cancel_requested = false;

    if (_is_gps_time_ok()) {
        _params.start_time_gps_sec = (AP::gps().time_week_ms() / 1000 + 10) % GPS_WEEK_LENGTH_SEC;
        _start_time_source = StartTimeSource::START_METHOD;
    }
}

void AC_DroneShowManager::stop_if_running()
{
    _cancel_requested = true;
}

void AC_DroneShowManager::update()
{
    _check_changes_in_parameters();
    _check_events();
    _check_radio_failsafe();
    _update_lights();
}

void AC_DroneShowManager::_check_changes_in_parameters()
{
    static int32_t last_seen_start_time_gps_sec = -1;
    static bool last_seen_show_authorization_state = false;
    static int16_t last_seen_control_rate_hz = DEFAULT_UPDATE_RATE_HZ;
    uint32_t start_time_gps_msec;

    bool new_control_rate_pending = _params.control_rate_hz != last_seen_control_rate_hz;    
    bool new_start_time_pending = _params.start_time_gps_sec != last_seen_start_time_gps_sec;
    bool new_show_authorization_pending = _params.authorized_to_start != last_seen_show_authorization_state;

    if (new_start_time_pending) {
        // We don't allow the user to mess around with the start time if we are
        // already performing the show
        if (!is_safe_to_change_start_time_in_stage(_stage_in_drone_show_mode)) {
            new_start_time_pending = false;
        }
    }

    if (new_start_time_pending && (_is_gps_time_ok() || _params.start_time_gps_sec < 0)) {
        last_seen_start_time_gps_sec = _params.start_time_gps_sec;

        if (last_seen_start_time_gps_sec >= 0) {
            start_time_gps_msec = last_seen_start_time_gps_sec * 1000;
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

            if (_start_time_source == StartTimeSource::NONE) {
                _start_time_source = StartTimeSource::PARAMETER;
            }
        } else {
            _start_time_usec = 0;
            _start_time_source = StartTimeSource::NONE;
        }

        /*
        if (has_scheduled_start_time()) {
            gcs().send_text(
                MAV_SEVERITY_INFO, "Start time set to %llu",
                static_cast<unsigned long long int>(_start_time_usec)
            );
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Start time cleared");
        }
        */
    }

    if (new_show_authorization_pending) {
        last_seen_show_authorization_state = _params.authorized_to_start;

        // Show authorization state changed recently. We might need to switch
        // flight modes, but we cannot change flight modes from here so we just
        // set a flag.
        if (has_authorization_to_start() && should_switch_to_show_mode_when_authorized()) {
            _request_switch_to_show_mode();
        }
    }

    if (new_control_rate_pending) {
        last_seen_control_rate_hz = _params.control_rate_hz;

        // Validate the control rate from the parameters and convert it to
        // milliseconds
        if (last_seen_control_rate_hz < 1) {
            _controller_update_delta_msec = 1000;
        } else if (last_seen_control_rate_hz > 50) {
            _controller_update_delta_msec = 20;
        } else {
            _controller_update_delta_msec = 1000 / last_seen_control_rate_hz;
        }
    }
}

void AC_DroneShowManager::_check_events()
{
    DroneShowNotificationBackend* backend = DroneShowNotificationBackend::get_singleton();

    if (DroneShowNotificationBackend::events.compass_cal_failed) {
        _flash_leds_after_failure();
    } else if (DroneShowNotificationBackend::events.compass_cal_saved) {
        _flash_leds_after_success();
    }

    if (backend) {
        backend->clear_events();
    }
}

void AC_DroneShowManager::_check_radio_failsafe()
{
    if (AP_Notify::flags.failsafe_radio) {
        _rc_start_switch_blocked_until = AP_HAL::millis() + 1000;
    }
}

void AC_DroneShowManager::_clear_start_time_after_landing()
{
    _params.start_time_gps_sec = -1;
    _check_changes_in_parameters();
}

void AC_DroneShowManager::_clear_start_time_if_set_by_switch()
{
    if (_start_time_source == StartTimeSource::RC_SWITCH) {
        _params.start_time_gps_sec = -1;
        _start_time_source = StartTimeSource::NONE;
        _start_time_usec = 0;
    }
}

uint32_t AC_DroneShowManager::_get_gps_synced_timestamp_in_millis_for_lights() const
{
    // No need to worry about loss of GPS fix; AP::gps().time_epoch_usec() is
    // smart enough to extrapolate from the timestamp of the latest fix.
    //
    // Also no need to worry about overflow; AP::gps().time_epoch_usec() / 1000
    // is too large for an uint32_t but it doesn't matter as we will truncate
    // the high bits.
    if (_is_gps_time_ok()) {
        return AP::gps().time_epoch_usec() / 1000;
    } else {
        return AP_HAL::millis();
    }
}

void AC_DroneShowManager::_flash_leds_after_failure()
{
    _flash_leds_with_color(255, 0, 0, /* count = */ 3, LightEffectPriority_Internal);
}

void AC_DroneShowManager::_flash_leds_after_success()
{
    _flash_leds_with_color(0, 255, 0, /* count = */ 3, LightEffectPriority_Internal);
}

void AC_DroneShowManager::_flash_leds_with_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t count, LightEffectPriority priority)
{
    _light_signal.started_at_msec = AP_HAL::millis();
    _light_signal.priority = priority;

    _light_signal.duration_msec = count * 300 - 200;  /* 100ms on, 200ms off */
    _light_signal.color[0] = red;
    _light_signal.color[1] = green;
    _light_signal.color[2] = blue;
    _light_signal.effect = LightEffect_Blinking;
    _light_signal.period_msec = 300;  /* 100ms on, 200ms off */
    _light_signal.phase_msec = 0;     /* exact sync with GPS clock */
}

bool AC_DroneShowManager::_handle_custom_data_message(uint8_t type, void* data, uint8_t length)
{
    if (data == nullptr) {
        return false;
    }

    // We allocate type 0x5C for the GCS-to-drone packets (0X5B is the drone-to-GCS
    // status packet), and sacrifice the first byte of the payload to identify
    // the _real_ message type. This reduces the chance of clashes with other
    // DATA* messages from third parties. The type that we receive in this
    // function is the _real_ message type.
    switch (type) {
        // Broadcast start time and authorization state of the show
        case CustomPackets::START_CONFIG:
            if (length >= sizeof(CustomPackets::start_config_t)) {
                CustomPackets::start_config_t* start_config = static_cast<CustomPackets::start_config_t*>(data);
                if (start_config->start_time < 0) {
                    _params.start_time_gps_sec = -1;
                } else if (start_config->start_time < GPS_WEEK_LENGTH_SEC) {
                    _params.start_time_gps_sec = start_config->start_time;
                }
                _params.authorized_to_start = start_config->is_authorized;

                return true;
            }
            break;
    }

    return false;
}

bool AC_DroneShowManager::_handle_data16_message(const mavlink_message_t& msg)
{
    mavlink_data16_t packet;
    mavlink_msg_data16_decode(&msg, &packet);
    if (packet.type != 0x5C || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data32_message(const mavlink_message_t& msg)
{
    mavlink_data32_t packet;
    mavlink_msg_data32_decode(&msg, &packet);
    if (packet.type != 0x5C || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data64_message(const mavlink_message_t& msg)
{
    mavlink_data64_t packet;
    mavlink_msg_data64_decode(&msg, &packet);
    if (packet.type != 0x5C || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data96_message(const mavlink_message_t& msg)
{
    mavlink_data96_t packet;
    mavlink_msg_data96_decode(&msg, &packet);
    if (packet.type != 0x5C || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_led_control_message(const mavlink_message_t& msg)
{
    mavlink_led_control_t packet;
    LightEffectPriority priority;
    uint8_t mask = ALL_GROUPS;

    mavlink_msg_led_control_decode(&msg, &packet);

    if (packet.instance != 42 || packet.pattern != 42) {
        // Not handled by us
        return false;
    }

    // LED control packets exist in five variants:
    //
    // custom_len == 0 --> simple blink request
    // custom_len == 1 --> simple blink request, but only if the drone matches
    //                     the group mask given in the last byte
    // custom_len == 3 --> set the LED to a specific color for five seconds
    // custom_len == 5 --> set the LED to a specific color for a given duration (msec)
    // custom_len == 6 --> set the LED to a specific color for a given duration (msec),
    //                     modulated by a given effect
    // custom_len == 7 --> set the LED to a specific color for a given duration (msec),
    //                     modulated by a given effect, but only if the drone matches
    //                     the group mask given in the last byte

    if (packet.custom_len >= 8 || packet.custom_len == 2 || packet.custom_len == 4) {
        // Not handled by us
        return false;
    }
    
    // Individual messages take precedence over broadcast messages so we need to
    // know whether this message is broadcast
    priority = (packet.target_system == 0)
        ? LightEffectPriority_Broadcast
        : LightEffectPriority_Individual;
    if (priority < _light_signal.priority) {
        // Previous light signal has a higher priority, but maybe it ended already?
        if (_light_signal.started_at_msec + _light_signal.duration_msec < AP_HAL::millis()) {
            _light_signal.priority = LightEffectPriority_None;
        } else {
            // Handled but ignored by us because a higher priority effect is still
            // playing.
            return true;
        }
    }

    _light_signal.started_at_msec = AP_HAL::millis();
    _light_signal.priority = priority;

    if (packet.custom_len < 2) {
        // Start blinking the drone show LED
        if (packet.custom_len == 1) {
            mask = packet.custom_bytes[0];
        }
        if (matches_group_mask(mask)) {
            _flash_leds_with_color(255, 255, 255, /* count = */ 5, priority);
        }
    } else if (packet.custom_len == 3) {
        // Set the drone show LED to a specific color for five seconds
        _light_signal.duration_msec = 5000;
        _light_signal.color[0] = packet.custom_bytes[0];
        _light_signal.color[1] = packet.custom_bytes[1];
        _light_signal.color[2] = packet.custom_bytes[2];
        _light_signal.effect = LightEffect_Solid;
        _light_signal.period_msec = 0;    /* doesn't matter */
        _light_signal.phase_msec = 0;     /* doesn't matter */
    } else if (packet.custom_len == 5) {
        // Set the drone show LED to a specific color for a given number of
        // milliseconds
        _light_signal.duration_msec = packet.custom_bytes[3] + (packet.custom_bytes[4] << 8);
        _light_signal.color[0] = packet.custom_bytes[0];
        _light_signal.color[1] = packet.custom_bytes[1];
        _light_signal.color[2] = packet.custom_bytes[2];
        _light_signal.effect = LightEffect_Solid;
        _light_signal.period_msec = 0;    /* doesn't matter */
        _light_signal.phase_msec = 0;     /* doesn't matter */
    } else if (packet.custom_len == 6 || packet.custom_len == 7) {
        if (packet.custom_len == 7) {
            mask = packet.custom_bytes[6];
        }
        if (matches_group_mask(mask)) {
            // Set the drone show LED to a specific color for a given number of
            // milliseconds, modulated by a given effect
            _light_signal.duration_msec = packet.custom_bytes[3] + (packet.custom_bytes[4] << 8);
            _light_signal.period_msec = 5000;
            _light_signal.color[0] = packet.custom_bytes[0];
            _light_signal.color[1] = packet.custom_bytes[1];
            _light_signal.color[2] = packet.custom_bytes[2];

            // Reset the phase of the effect if the previous effect was of a
            // different type, but keep the phase counter at its current value
            if (_light_signal.effect != packet.custom_bytes[5]) {
                bool is_effect_synced_to_gps;

                if (packet.custom_bytes[5] > LightEffect_Last) {
                    _light_signal.effect = LightEffect_Last;
                } else {
                    _light_signal.effect = static_cast<LightEffectType>(packet.custom_bytes[5]);
                }

                is_effect_synced_to_gps = (
                    _light_signal.effect == LightEffect_Blinking ||
                    _light_signal.effect == LightEffect_Off ||
                    _light_signal.effect == LightEffect_Solid
                );

                _light_signal.phase_msec = is_effect_synced_to_gps ? 0 : get_random16() % _light_signal.period_msec;
            }
        }
    }

    // Handle zero duration; it means that we need to turn off whatever
    // effect we have now.
    if (matches_group_mask(mask) && _light_signal.duration_msec == 0) {
        _light_signal.started_at_msec = 0;
        _light_signal.effect = LightEffect_Off;
    }

    return true;
}

bool AC_DroneShowManager::_is_gps_time_ok() const
{
    // AP::gos().time_week() starts from zero and gets set to a non-zero value
    // when we start receiving full time information from the GPS. It may happen
    // that the GPS subsystem receives iTOW information from the GPS module but
    // no week number; we deem this unreliable so we return false in this case.
    return AP::gps().time_week() > 0;
}

bool AC_DroneShowManager::_load_show_file_from_storage()
{
    int fd;
    int retval;
    struct stat stat_data;
    uint8_t *show_data, *write_ptr, *end_ptr;
    ssize_t to_read, actually_read;
    bool success = false;

    // Clear any previously loaded show
    _set_light_program_and_take_ownership(0);
    _set_trajectory_and_take_ownership(0);
    _set_show_data_and_take_ownership(0);

    // Check whether the show file exists
    retval = AP::FS().stat(SHOW_FILE, &stat_data);
    if (retval)
    {
        return false;
    }

    // Ensure that we have a sensible block size that we will use when reading
    // the show file
    if (stat_data.st_blksize < 1)
    {
        stat_data.st_blksize = 4096;
    }

    // Allocate memory for the whole content of the file
    show_data = static_cast<uint8_t *>(calloc(stat_data.st_size, sizeof(uint8_t)));
    if (show_data == 0)
    {
        hal.console->printf(
            "Show file too large: %ld bytes\n",
            static_cast<long int>(stat_data.st_size));
        return false;
    }

    // Read the entire show file into memory
    fd = AP::FS().open(SHOW_FILE, O_RDONLY);
    if (fd < 0)
    {
        free(show_data);
        show_data = write_ptr = end_ptr = 0;
    }
    else
    {
        write_ptr = show_data;
        end_ptr = show_data + stat_data.st_size;
    }

    while (write_ptr < end_ptr)
    {
        to_read = end_ptr - write_ptr;
        if (to_read > stat_data.st_blksize)
        {
            to_read = stat_data.st_blksize;
        }

        if (to_read == 0)
        {
            break;
        }

        actually_read = AP::FS().read(fd, write_ptr, to_read);
        if (actually_read < 0)
        {
            /* Error while reading */
            hal.console->printf(
                "IO error while reading show file near byte %ld, errno = %d\n",
                static_cast<long int>(write_ptr - show_data),
                static_cast<int>(errno)
            );
            free(show_data);
            show_data = 0;
            break;
        }
        else if (actually_read == 0)
        {
            /* EOF */
            break;
        }
        else
        {
            write_ptr += actually_read;
        }
    }

    if (fd > 0)
    {
        AP::FS().close(fd);
    }

    // Parse the show file and find the trajectory and the light program in it
    if (show_data)
    {
        sb_trajectory_t loaded_trajectory;
        sb_light_program_t loaded_light_program;

        _set_show_data_and_take_ownership(show_data);

        retval = sb_trajectory_init_from_binary_file_in_memory(&loaded_trajectory, show_data, stat_data.st_size);
        if (retval)
        {
            hal.console->printf("Error while parsing show file: %d\n", (int) retval);
        }
        else
        {
            _set_trajectory_and_take_ownership(&loaded_trajectory);

            if (has_valid_takeoff_time())
            {
                hal.console->printf(
                    "Loaded show: %.1fs, takeoff at %.1fs, landing at %.1fs\n",
                    _total_duration_sec, _takeoff_time_sec, _landing_time_sec
                );
                success = true;
            }
        }

        retval = sb_light_program_init_from_binary_file_in_memory(&loaded_light_program, show_data, stat_data.st_size);
        if (retval)
        {
            hal.console->printf("No light program in show file: %d\n", (int) retval);
        }
        else
        {
            _set_light_program_and_take_ownership(&loaded_light_program);
        }
    }

    return success;
}

void AC_DroneShowManager::_set_light_program_and_take_ownership(sb_light_program_t *value)
{
    sb_light_player_destroy(_light_player);
    sb_light_program_destroy(_light_program);

    if (value)
    {
        *_light_program = *value;
        _light_program_valid = true;
    }
    else
    {
        sb_light_program_init_empty(_light_program);
        _light_program_valid = false;
    }

    sb_light_player_init(_light_player, _light_program);
}

void AC_DroneShowManager::_set_show_data_and_take_ownership(uint8_t *value)
{
    if (_show_data == value)
    {
        return;
    }

    if (_show_data)
    {
        free(_show_data);
    }

    _show_data = value;
}

void AC_DroneShowManager::_set_trajectory_and_take_ownership(sb_trajectory_t *value)
{
    sb_trajectory_player_destroy(_trajectory_player);
    sb_trajectory_destroy(_trajectory);

    if (value)
    {
        *_trajectory = *value;
        _trajectory_valid = true;
    }
    else
    {
        sb_trajectory_init_empty(_trajectory);
        _trajectory_valid = false;
    }

    sb_trajectory_player_init(_trajectory_player, _trajectory);

    _total_duration_sec = sb_trajectory_get_total_duration_sec(_trajectory);

    _takeoff_time_sec = sb_trajectory_propose_takeoff_time_sec(
        _trajectory, AC_DroneShowManager::TAKEOFF_ALTITUDE_METERS * 1000.0f /* [mm] */,
        AC_DroneShowManager::TAKEOFF_SPEED_METERS_PER_SEC * 1000.0f /* [mm/s] */
    );

    /* We assume that we need to trigger landing at the end of the trajectory;
     * in other words, the trajectory should end above the landing position
     * by a safe altitude margin. This is because calculating an exact landing
     * time onboard with the current trajectory format is too slow on a
     * Pixhawk1 and we trigger a watchdog timer that resets the Pixhawk */
    _landing_time_sec = _total_duration_sec;

    // Make sure that we never take off before the scheduled start of the
    // show, even if we are going to be a bit late with the takeoff
    if (_takeoff_time_sec < 0)
    {
        _takeoff_time_sec = 0;
    }

    // Check whether the landing time is later than the takeoff time. If it is
    // earlier, it shows that there's something wrong with the trajectory so
    // let's not take off at all.
    if (_landing_time_sec < _takeoff_time_sec)
    {
        // This should ensure that has_valid_takeoff_time() returns false
        _landing_time_sec = _takeoff_time_sec = -1;
    }
}

void AC_DroneShowManager::_update_lights()
{
    // TODO(ntamas): mode numbers are hardcoded here; we cannot import them
    // from ../ArduCopter/mode.h due to circular imports. We should inject them
    // from Copter.cpp after the construction of AC_DroneShowManager.cpp instead.
    const uint32_t MODE_RTL = 6, MODE_SMART_RTL = 21, MODE_LAND = 9, MODE_DRONE_SHOW = 127;
    sb_rgb_color_t color = Colors::BLACK;
    bool light_signal_affected_by_brightness_setting = true;
    uint8_t pattern = 0b11111111;
    const uint8_t BLINK = 0b11110000;
    const uint8_t BLINK_TWICE_PER_SECOND = 0b11001100;
    const uint8_t FLASH_ONCE_PER_SECOND = 0b10000000;
    const uint8_t FLASH_TWICE_PER_SECOND = 0b10100000;
    const uint8_t FLASH_FOUR_TIMES_PER_SECOND = 0b10101010;
    float elapsed_time;
    float pulse = 0.0f;
    float factor;

#define IS_LANDING(mode) (  \
    mode == MODE_LAND ||    \
    (mode == MODE_DRONE_SHOW && _stage_in_drone_show_mode == DroneShow_Landing) \
)

#define IS_LANDED(mode) (  \
    mode == MODE_LAND ||    \
    (mode == MODE_DRONE_SHOW && _stage_in_drone_show_mode == DroneShow_Landed) \
)

#define IS_RTL(mode) (        \
    mode == MODE_RTL ||       \
    mode == MODE_SMART_RTL || \
    (mode == MODE_DRONE_SHOW && _stage_in_drone_show_mode == DroneShow_RTL) \
)

    // During compass calibration, the light should be purple no matter what.
    // Compass calibration is always requested by the user so he can rightly
    // expect any light signal that was previously set up from the GCS to be
    // overridden.
    if (AP_Notify::flags.compass_cal_running) {
        color = Colors::MAGENTA;
        pulse = 0.5;
    } else if (_light_signal.started_at_msec) {
        // If the user requested a light signal, it trumps everything except
        // the compass calibration.
        uint32_t now = AP_HAL::millis();
        if (now < _light_signal.started_at_msec) {
            // Something is wrong, let's just clear the light signal
            _light_signal.started_at_msec = 0;
            _light_signal.priority = LightEffectPriority_None;
        } else {
            // "Where are you signal" is 100 msec on, 200 msec off, five times.
            uint32_t diff = (now - _light_signal.started_at_msec);
            if (diff > _light_signal.duration_msec) {
                // Light signal ended
                _light_signal.started_at_msec = 0;
                _light_signal.priority = LightEffectPriority_None;
            } else {
                // Light signal is in progress
                factor = get_modulation_factor_for_light_effect(
                    _get_gps_synced_timestamp_in_millis_for_lights(),
                    _light_signal.effect, _light_signal.period_msec,
                    _light_signal.phase_msec
                );
                color.red = _light_signal.color[0] * factor;
                color.green = _light_signal.color[1] * factor;
                color.blue = _light_signal.color[2] * factor;
            }
        }

        // If the user explicitly requested a light signal, do not dim it
        light_signal_affected_by_brightness_setting = false;
    } else if (
        AP_Notify::flags.ekf_bad || AP_Notify::flags.failsafe_battery ||
        AP_Notify::flags.failsafe_gcs || AP_Notify::flags.failsafe_radio
    ) {
        // In error conditions, the light should be red.
        //
        // Ideally, the condition above should be the same as the one that triggers
        // MAV_STATE_CRITICAL in GCS_Mavlink.cpp. However, we cannot reach out to
        // the Copter class because everything that we would need there is private.
        color = Colors::RED;
        pattern = BLINK;
    } else if (AP_Notify::flags.flying) {
        uint32_t mode = gcs().custom_mode();

        // If we are flying, we don't want to dim the LED light
        light_signal_affected_by_brightness_setting = false;

        if (IS_RTL(mode)) {
            // If we are flying and we are in RTL or smart RTL mode, blink with orange color
            color = Colors::ORANGE;
            pattern = BLINK;
        } else if (IS_LANDING(mode)) {
            // If we are flying and we are in landing mode, show a solid orange color
            color = Colors::ORANGE;
        } else if (mode == MODE_DRONE_SHOW) {
            // If we are flying in drone show mode, show the color that we are
            // supposed to show during the drone show if the show has started.
            // If the show has not started, show white, although this should not
            // happen (we cannot be in drone show mode and flying if we are
            // before the scheduled start time).
            //
            // TODO(ntamas): what if we were late to the party and we are loitering
            // only?
            elapsed_time = get_elapsed_time_since_start_sec();
            if (elapsed_time >= 0) {
                get_color_of_rgb_light_at_seconds(elapsed_time, &color);
            } else {
                color = Colors::WHITE_DIM;
            }
        } else {
            // Otherwise, show a bright white color so we can see the drone from the ground
            color = Colors::WHITE;
        }
    } else if (AP::motors()->get_spool_state() != AP_Motors::SpoolState::SHUT_DOWN) {
        uint32_t mode = gcs().custom_mode();

        if (IS_LANDING(mode)) {
            // If the landing algorithm is running but we are not flying, _but_
            // the motors have not shut down yet, flash four times per second
            // as an alarm signal so people know not to approach the drone yet
            color = Colors::ORANGE;
            pattern = FLASH_FOUR_TIMES_PER_SECOND;
        } else if (mode == MODE_DRONE_SHOW) {
            // If we are not flying in drone show mode but the motors are
            // running, show the color that we are supposed to show if the
            // show has started already; otherwise blink green twice per second.
            elapsed_time = get_elapsed_time_since_start_sec();
            if (elapsed_time >= 0) {
                get_color_of_rgb_light_at_seconds(elapsed_time, &color);
                light_signal_affected_by_brightness_setting = false;
            } else {
                color = Colors::GREEN;
                pattern = BLINK_TWICE_PER_SECOND;
            }
        } else {
            // In all other cases, blink green twice per second.
            color = Colors::GREEN;
            pattern = BLINK_TWICE_PER_SECOND;
        }
    } else if (
        AP_Notify::flags.initialising || !AP_Notify::flags.pre_arm_check ||
        !AP_Notify::flags.pre_arm_gps_check
    ) {
        // If the prearm checks are running, flash yellow once per second
        color = Colors::YELLOW;
        pattern = FLASH_ONCE_PER_SECOND;
    } else {
        // We are on the ground, motors are not running and the prearm checks
        // have passed. We are essentially in standby.
        uint32_t mode = gcs().custom_mode();

        if (IS_LANDED(mode)) {
            // Landing-related mode, show a green dim light to indicate success.
            color = Colors::GREEN_DIM;
        } else if (mode == MODE_DRONE_SHOW) {
            // Drone show mode is distinguished from the rest by a pulsating
            // light ("breathing" pattern).

            // No authorization yet --> slow pulsating blue light; dark if no
            // start time was set, not-so-dark if some start time was set
            // Authorized, far from start --> slow pulsating green light
            // Authorized, about to start --> green flashes, twice per second,
            // synced to GPS

            if (has_authorization_to_start()) {
                if (get_time_until_landing_sec() < 0) {
                    // if we have already landed but show mode is reset from
                    // another mode, we just keep calm with solid green
                    color = Colors::GREEN_DIM;
                } else if (get_time_until_takeoff_sec() > 10) {
                    // if there is plenty of time until takeoff, we pulse slowly
                    color = Colors::GREEN_DIM;
                    pulse = 0.5;
                } else {
                    // if we are about to take off soon, flash quickly
                    color = Colors::GREEN;
                    pattern = FLASH_TWICE_PER_SECOND;
                }
            } else {
                color = Colors::LIGHT_BLUE;
                pulse = has_scheduled_start_time() && get_time_until_takeoff_sec() >= 0 ? 0.5 : 0.3;
            }
        } else {
            // Show a green dim light to indicate that we are ready.
            color = Colors::GREEN_DIM;
        }
    }

    if (pulse > 0) {
        // "Pulsating", "breathing" light pattern. Modulate intensity with a
        // sine wave.
        factor = pulse * get_modulation_factor_for_light_effect(
            _get_gps_synced_timestamp_in_millis_for_lights(),
            LightEffect_Breathing, /* period_msec = */ 5000, /* phase_msec = */ 0
        );
        color.red *= factor;
        color.green *= factor;
        color.blue *= factor;
    } else if (pattern < 255) {
        // check the time and set the color to black if needed - this creates a
        // blinking pattern when needed
        uint32_t timestamp = _get_gps_synced_timestamp_in_millis_for_lights() % 1000;
        if (!(pattern & (0x80 >> (timestamp / 125))))
        {
            color = Colors::BLACK;
        }
    }

    // Dim the lights if we are on the ground before the flight
    if (light_signal_affected_by_brightness_setting) {
        uint8_t shift = 0;

        if (_params.preflight_light_signal_brightness <= 0) {
            // <= 0 = completely off, shift by 8 bits
            shift = 8;
        } else if (_params.preflight_light_signal_brightness == 1) {
            // 1 = low brightness, keep the 6 MSB so the maximum is 64
            shift = 2;
        } else if (_params.preflight_light_signal_brightness == 2) {
            // 2 = medium brightness, keep the 7 MSB so the maximum is 128
            shift = 1;
        } else {
            // >= 2 = full brightness
            shift = 0;
        }

        color.red >>= shift;
        color.green >>= shift;
        color.blue >>= shift;
    }

    _last_rgb_led_color = color;

    if (_rgb_led) {
        // No need to test whether the RGB values changed because the RGBLed
        // drivers do this on their own
        _rgb_led->set_rgb(color.red, color.green, color.blue);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_sock_rgb_open) {
        uint8_t data[4];

        data[0] = mavlink_system.sysid;
        data[1] = color.red;
        data[2] = color.green;
        data[3] = color.blue;

        _sock_rgb.send(data, sizeof(data));
    }
#endif

#undef IS_LANDING
#undef IS_RTL

}

void AC_DroneShowManager::_update_rgb_led_instance()
{
    if (_rgb_led)
    {
        _rgb_led->set_rgb(0, 0, 0);

        delete _rgb_led;
        _rgb_led = NULL;
    }

    if (_rgb_led_factory) {
        int led_type = _params.led_specs[0].type;
        uint8_t channel = _params.led_specs[0].channel;
        uint8_t num_leds = _params.led_specs[0].count;

        _rgb_led = _rgb_led_factory->new_rgb_led_by_type(
            static_cast<DroneShowLEDType>(led_type), channel, num_leds
        );
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

bool AC_DroneShowManager::_open_rgb_led_socket()
{
#if defined(RGB_SOCKET_PORT)
    if (_sock_rgb_open) {
        return true;
    }

    if (!_sock_rgb.connect("127.0.0.1", RGB_SOCKET_PORT)) {
        return false;
    }

    _sock_rgb.set_blocking(false);
    _sock_rgb_open = true;
#endif

    return true;
}

#endif

static float get_modulation_factor_for_light_effect(
    uint32_t timestamp, LightEffectType effect, uint16_t period_msec, uint16_t phase_msec
) {
    if (effect == LightEffect_Off) {
        return 0.0;
    } else if (effect == LightEffect_Solid || period_msec == 0) {
        return 1.0;
    }

    timestamp = (timestamp + phase_msec) % period_msec;

    switch (effect) {
        case LightEffect_Off:
            return 0.0;

        case LightEffect_Solid:
            return 1.0;

        case LightEffect_Blinking:
            return timestamp < period_msec / 3.0f ? 1.0 : 0.0;

        case LightEffect_Breathing:
            return 0.5f * (1 + sinf(timestamp / 5000.0f * 2.0f * M_PI));

        default:
            return 0.0;
    }
}

static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage) {
    return (
        stage == DroneShow_Off ||
        stage == DroneShow_Init ||
        stage == DroneShow_WaitForStartTime ||
        stage == DroneShow_Landed
    );
}

#endif  // HAVE_FILESYSTEM_SUPPORT
