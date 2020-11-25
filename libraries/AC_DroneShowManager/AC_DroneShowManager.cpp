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
#    define HAL_BOARD_COLLMOT_DIRECTORY "/collmot"
#  endif
#endif

#define SHOW_FILE (HAL_BOARD_COLLMOT_DIRECTORY "/show.skyb")

#if HAVE_FILESYSTEM_SUPPORT && defined(HAL_BOARD_TERRAIN_DIRECTORY)
#define AP_TERRAIN_AVAILABLE 1
#else
#define AP_TERRAIN_AVAILABLE 0
#endif

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
    static const sb_rgb_color_t WHITE = { 255, 255, 255 };
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
    // @Description: Latitude of drone show coordinate system, zero if unset
    // @Range: -900000000 900000000
    // @Increment: 1
    // @Units: 1e-7 degrees
    // @User: Standard
    AP_GROUPINFO("ORIGIN_LAT", 2, AC_DroneShowManager, _params.origin_lat, 0),

    // @Param: ORIGIN_LNG
    // @DisplayName: Origin (longitude)
    // @Description: Longitude of drone show coordinate system, zero if unset
    // @Range: -1800000000 1800000000
    // @Increment: 1
    // @Units: 1e-7 degrees
    // @User: Standard
    AP_GROUPINFO("ORIGIN_LNG", 3, AC_DroneShowManager, _params.origin_lng, 0),

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
    // @Values: 0:Off, 1:MAVLink channel 0, 2:MAVLink channel 1, 3:MAVLink channel 2, 4:MAVLink channel 3, 5:SITL, 6:Servo, 7:NeoPixel, 8:ProfiLED, 9:Debug
    // @User: Standard
    AP_GROUPINFO("LED0_TYPE", 6, AC_DroneShowManager, _params.led_specs[0].type, 0),

    // @Param: LED0_CHAN
    // @DisplayName: PWM or MAVLink channel to use for the LED output
    // @Description: PWM channel to use for the LED output (1-based) if the LED type is "NeoPixel" or "ProfiLED", or the MAVLink channel to use if the LEF type is "MAVLink"
    // @User: Standard
    AP_GROUPINFO("LED0_CHAN", 8, AC_DroneShowManager, _params.led_specs[0].channel, 0),

    // @Param: LED0_COUNT
    // @DisplayName: Number of individual LEDs on a LED channel
    // @Description: Specifies how many LEDs there are on a NeoPixel or ProfiLED LED strip
    // @User: Standard
    AP_GROUPINFO("LED0_COUNT", 7, AC_DroneShowManager, _params.led_specs[0].count, 16),

    AP_GROUPEND
};

// LED factory that is used to create new RGB LED instances
static DroneShowLEDFactory _rgb_led_factory_singleton;

static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage);

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
    _rgb_led(0),
    _rc_start_switch_blocked_until(0)
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

void AC_DroneShowManager::init()
{
    // Get a reference to the RGB LED factory
    _rgb_led_factory = &_rgb_led_factory_singleton;

    // Clear start time and authorization now; at this point the parameter
    // subsystem has already loaded back the previous value from the EEPROM so
    // we are safe to overwrite it
    _params.start_time_gps_sec = -1;
    _params.authorized_to_start = 0;

    AP::FS().mkdir(HAL_BOARD_COLLMOT_DIRECTORY);

    _load_show_file_from_storage();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _open_rgb_led_socket();
#endif
    _update_rgb_led_instance();
}

void AC_DroneShowManager::get_color_of_rgb_light_at_seconds(float time, sb_rgb_color_t* color)
{
    *color = sb_light_player_get_color_at(_light_player, time < 0 || time > 86400000 ? 0 : time * 1000);
}

void AC_DroneShowManager::get_desired_global_position_in_cms_at_seconds(float time, Location& loc)
{
    sb_vector3_with_yaw_t vec;
    float offset_north, offset_east, orientation_rad;

    sb_trajectory_player_get_position_at(_trajectory_player, time, &vec);

    // We have millimeters so far, need to convert X and Y to meters and
    // Z to centimeters first
    vec.x = vec.x / 1000.0f;
    vec.y = vec.y / 1000.0f;
    vec.z = vec.z / 10.0f;

    // Then we need to rotate the X axis by -_orientation_deg degrees so it
    // points North. At the same time, we also flip the Y axis so it points
    // East and not West.
    orientation_rad = radians(_orientation_deg);
    offset_north = cosf(orientation_rad) * vec.x + sinf(orientation_rad) * vec.y;
    offset_east = sinf(orientation_rad) * vec.x - cosf(orientation_rad) * vec.y;

    // Finally, we need to offset the show origin with the calculated North and
    // East offset to get a global position

    // TODO(ntamas): handle yaw!
    // TODO(ntamas): figure out whether using Location::AltFrame::ABOVE_ORIGIN
    // below for the altitude reference is okay or not

    loc.zero();
    loc.lat = _origin_lat;
    loc.lng = _origin_lng;
    loc.set_alt_cm(static_cast<int32_t>(vec.z), Location::AltFrame::ABOVE_ORIGIN);
    loc.offset(offset_north, offset_east);
}

void AC_DroneShowManager::get_desired_velocity_neu_in_cms_per_seconds_at_seconds(float time, Vector3f& vel)
{
    sb_vector3_with_yaw_t vec;
    float orientation_rad;

    sb_trajectory_player_get_velocity_at(_trajectory_player, time, &vec);

    // We have mm/s so far, need to convert to cm/s
    vec.x = vec.x / 10.0f;
    vec.y = vec.y / 10.0f;
    vec.z = vec.z / 10.0f;

    // Then we need to rotate the X axis by -_orientation_deg degrees so it
    // points North. At the same time, we also flip the Y axis so it points
    // East and not West.
    orientation_rad = radians(_orientation_deg);
    vel.x = cosf(orientation_rad) * vec.x + sinf(orientation_rad) * vec.y;
    vel.y = sinf(orientation_rad) * vec.x - cosf(orientation_rad) * vec.y;

    // Copy vec.z to vel.z intact
    vel.z = vec.z;

    // TODO(ntamas): handle yaw!
}

// returns the elapsed time since the start of the show, in microseconds
int64_t AC_DroneShowManager::get_elapsed_time_since_start_usec() const
{
    if (_start_time_usec > 0) {
        // TODO(ntamas): what if we lose GPS fix? In that case, now will be
        // zero, and we don't know what to do with that.
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

bool AC_DroneShowManager::handle_message(const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_LED_CONTROL:
            // The drone show LED listens on the "secret" LED ID 42 with a
            // pattern of 42 as well. No custom payload should be submitted
            // here. Any message that does not match this specification is
            // handled transparently by the "core" MAVLink GCS module.
            return _handle_led_control_message(msg);

        default:
            return false;
    }
}

bool AC_DroneShowManager::has_authorization_to_start() const
{
    return _params.authorized_to_start;
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

void AC_DroneShowManager::notify_takeoff(const Location& loc, float yaw)
{
    if (has_explicit_show_origin_set_by_user()) {
        // Set origin from parameters
        _origin_lat = static_cast<int32_t>(_params.origin_lat);
        _origin_lng = static_cast<int32_t>(_params.origin_lng);
    } else {
        // Set origin from current location
        _origin_lat = loc.lat;
        _origin_lng = loc.lng;
    }

    if (has_explicit_show_orientation_set_by_user()) {
        // Set orientation from parameters
        _orientation_deg = _params.orientation_deg;
    } else {
        // Set origin from current heading
        _orientation_deg = degrees(yaw);
    }
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
    sb_rgb_color_t color;
    float elapsed_time;
    int16_t encoded_elapsed_time;

    /* make sure that we can make use of MAVLink packet truncation */
    memset(packet, 0, sizeof(uint8_t));

    /* convert the last RGB light color */
    color.red = _last_rgb_led_color.red;
    color.green = _last_rgb_led_color.green;
    color.blue = _last_rgb_led_color.blue;

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
    *((uint16_t*) (packet + 4)) = sb_rgb_color_encode_rgb565(color);
    packet[6] = flags;
    packet[7] = static_cast<uint8_t>(_stage_in_drone_show_mode) & 0x0f;
    packet[8] = gps_health;
    packet[9] = 0; // could be used for something else, currently it is padding
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

void AC_DroneShowManager::start_if_not_running()
{
    _cancel_requested = false;

    if (_is_gps_time_ok()) {
        _params.start_time_gps_sec = (AP::gps().time_week_ms() / 1000 + 10) % 604800;
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
    _check_radio_failsafe();
    _update_lights();
}

void AC_DroneShowManager::_check_changes_in_parameters()
{
    static int32_t last_seen_start_time_gps_sec = -1;
    uint32_t start_time_gps_msec;
    bool new_start_time_pending = _params.start_time_gps_sec != last_seen_start_time_gps_sec;

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
    // TODO(ntamas): ensure continuity if the GPS fix goes away; right now this
    // timestamp would "jump" suddenly, which is not a big deal if we never use
    // differences between these timestamps and we only use these through the
    // modulo operator (which we currently do).
    if (_is_gps_time_ok()) {
        return AP::gps().time_epoch_usec() / 1000;
    } else {
        return AP_HAL::millis();
    }
}

bool AC_DroneShowManager::_handle_led_control_message(const mavlink_message_t& msg)
{
    mavlink_led_control_t packet;
    mavlink_msg_led_control_decode(&msg, &packet);

    if (packet.instance == 42 && packet.pattern == 42 && packet.custom_len == 0) {
        // Start blinking the drone show LED
        _light_signal_started_at_msec = AP_HAL::millis();
        return true;
    } else {
        // Not understood
        return false;
    }
}

bool AC_DroneShowManager::_is_gps_time_ok() const
{
    return AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D;
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
    bool light_should_be_dim = true;
    uint8_t pattern = 0b11111111;
    const uint8_t BLINK = 0b11110000;
    const uint8_t BLINK_TWICE_PER_SECOND = 0b11001100;
    const uint8_t FLASH_ONCE_PER_SECOND = 0b10000000;
    const uint8_t FLASH_TWICE_PER_SECOND = 0b10100000;
    const uint8_t FLASH_FOUR_TIMES_PER_SECOND = 0b10101010;
    float elapsed_time;
    float pulse = 0.0f;

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

    // If the user requested a light signal, it trumps everything.
    if (_light_signal_started_at_msec) {
        uint32_t now = AP_HAL::millis();
        if (now < _light_signal_started_at_msec) {
            // Something is wrong, let's just clear the light signal
            _light_signal_started_at_msec = 0;
        } else {
            // "Where are you signal" is 100 msec on, 200 msec off, five times.
            int diff = (now - _light_signal_started_at_msec) / 100;
            if (diff > 12 || diff < 0) {
                // Light signal ended
                _light_signal_started_at_msec = 0;
            } else if (diff % 3 == 0) {
                // Light signal is in progress and we need white now
                color = Colors::WHITE;
            } else {
                // Light signal is in progress and we need black now
            }
        }
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
        light_should_be_dim = false;

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
                light_should_be_dim = false;
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
                if (get_time_until_takeoff_sec() > 10) {
                    color = Colors::GREEN_DIM;
                    pulse = 0.5;
                } else {
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
        uint32_t timestamp = _get_gps_synced_timestamp_in_millis_for_lights() % 5000;
        float rate = pulse * 0.5f * (1 + sinf(timestamp / 5000.0f * 2.0f * M_PI));
        color.red *= rate;
        color.green *= rate;
        color.blue *= rate;
    } else if (pattern < 255) {
        // check the time and set the color to black if needed - this creates a
        // blinking pattern when needed

        uint32_t timestamp = _get_gps_synced_timestamp_in_millis_for_lights() % 1000;
        if (!(pattern & (0x80 >> (timestamp / 125))))
        {
            color = Colors::BLACK;
        }
    }

    if (light_should_be_dim) {
        // Dim the lights if we are on the ground
        color.red >>= 1;
        color.green >>= 1;
        color.blue >>= 1; 
    }

    _last_rgb_led_color.red = color.red;
    _last_rgb_led_color.green = color.green;
    _last_rgb_led_color.blue = color.blue;

    if (_rgb_led) {
        // No need to test whether the RGB values changed because the RGBLed
        // drivers do this on their own
        _rgb_led->set_rgb(color.red, color.green, color.blue);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_sock_rgb_open) {
        uint8_t data[4];

        // TODO(ntamas): insert own ID as the first byte
        data[0] = 0;
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

static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage) {
    return (
        stage == DroneShow_Off ||
        stage == DroneShow_Init ||
        stage == DroneShow_WaitForStartTime ||
        stage == DroneShow_Landed
    );
}

#endif  // HAVE_FILESYSTEM_SUPPORT
