#include "sc2_link_protocol.h"

#include <string.h>

#include <Limelight.h>

#define CONTROLS_INTERFACE_SUFFIX "/input1"
#define PADS_INTERFACE_SUFFIX "/input2"
#define MOTION_INTERFACE_SUFFIX "/input3"

#define STATUS_CONNECTED 0x0001
#define STATUS_CHARGE_STATE_SHIFT 1
#define STATUS_CHARGE_STATE_MASK 0x7u
#define STATUS_BATTERY_PERCENT_SHIFT 4
#define STATUS_BATTERY_PERCENT_MASK 0x7Fu
#define STATUS_BATTERY_KNOWN 0x0800
#define PADS_LEFT_TOUCHED 0x0001
#define PADS_RIGHT_TOUCHED 0x0002

#define TRIGGER_FULL_SCALE 32767u

// EChargeState of SDL's src/joystick/hidapi/steam/controller_structs.h.
#define CHARGE_STATE_DISCHARGING 1
#define CHARGE_STATE_CHARGING 2
#define CHARGE_STATE_CHARGING_DONE 4

// Full scales of the raw IMU mode, which the dongle forces, as SDL's triton driver reads them.
#define SENSOR_RAW_FULL_SCALE 32768.0f
#define ACCELEROMETER_FULL_SCALE_G 2.0f
#define GYROSCOPE_FULL_SCALE_DEGREES_PER_SECOND 2000.0f
#define STANDARD_GRAVITY 9.80665f

typedef struct button_mapping_t {
    uint32_t controller_bit;
    int moonlight_flag;
} button_mapping_t;

/*
 * The flags from which the host rebuilds the same report bits: the inverse of
 * Sunshine's make_gamepad_state() followed by libvirtualhid's
 * steam_controller_button_bits(). The pad touch bits 0x00200000 (right) and
 * 0x02000000 (left) have no flag: the host sets them from active touch
 * contacts.
 */
static const button_mapping_t button_mappings[] = {
        {0x00000001, A_FLAG},
        {0x00000002, B_FLAG},
        {0x00000004, X_FLAG},
        {0x00000008, Y_FLAG},
        {0x00000010, MISC_FLAG}, // QAM
        {0x00000020, RS_CLK_FLAG},
        {0x00000040, PLAY_FLAG}, // View
        {0x00000080, PADDLE1_FLAG}, // R4
        {0x00000100, PADDLE3_FLAG}, // R5
        {0x00000200, RB_FLAG},
        {0x00000400, DOWN_FLAG},
        {0x00000800, RIGHT_FLAG},
        {0x00001000, LEFT_FLAG},
        {0x00002000, UP_FLAG},
        {0x00004000, BACK_FLAG}, // Menu
        {0x00008000, LS_CLK_FLAG},
        {0x00010000, SPECIAL_FLAG}, // Steam
        {0x00020000, PADDLE2_FLAG}, // L4
        {0x00040000, PADDLE4_FLAG}, // L5
        {0x00080000, LB_FLAG},
        {0x00100000, STEAM_RIGHT_STICK_TOUCH_FLAG},
        {0x00400000, STEAM_RIGHT_TOUCHPAD_FLAG},
        {0x00800000, STEAM_RIGHT_TRIGGER_CLICK_FLAG},
        {0x01000000, STEAM_LEFT_STICK_TOUCH_FLAG},
        {0x04000000, STEAM_LEFT_TOUCHPAD_FLAG},
        {0x08000000, STEAM_LEFT_TRIGGER_CLICK_FLAG},
        {0x10000000, STEAM_RIGHT_GRIP_TOUCH_FLAG},
        {0x20000000, STEAM_LEFT_GRIP_TOUCH_FLAG},
};

#define BUTTON_MAPPING_COUNT (sizeof(button_mappings) / sizeof(button_mappings[0]))

static bool ends_with(const char *text, const char *suffix);

static sc2_link_pad_t decode_pad(const int32_t axis_values[ABS_CNT], int x_code, int y_code, int pressure_code,
                                 bool touched);

static sc2_link_vector_t sdl_sensor_axes(int16_t raw_x, int16_t raw_y, int16_t raw_z, float full_scale);

static uint8_t moonlight_battery_state(unsigned int charge_state);

sc2_link_node_role_t sc2_link_node_role(uint16_t vendor, uint16_t product, const char *name, const char *phys) {
    if (vendor != SC2_LINK_VENDOR_ID || product != SC2_LINK_PRODUCT_ID ||
        strcmp(name, SC2_LINK_HID_NODE_NAME) != 0) {
        return SC2_LINK_NODE_OTHER;
    }
    if (ends_with(phys, CONTROLS_INTERFACE_SUFFIX)) {
        return SC2_LINK_NODE_CONTROLS;
    }
    if (ends_with(phys, PADS_INTERFACE_SUFFIX)) {
        return SC2_LINK_NODE_PADS;
    }
    if (ends_with(phys, MOTION_INTERFACE_SUFFIX)) {
        return SC2_LINK_NODE_MOTION;
    }
    return SC2_LINK_NODE_OTHER;
}

bool sc2_link_same_dongle(const char *phys, const char *other_phys) {
    const char *slash = strrchr(phys, '/');
    const char *other_slash = strrchr(other_phys, '/');
    if (slash == NULL || other_slash == NULL) {
        return false;
    }
    size_t prefix_length = (size_t) (slash - phys);
    return prefix_length == (size_t) (other_slash - other_phys) && strncmp(phys, other_phys, prefix_length) == 0;
}

bool sc2_link_is_xpad(uint16_t vendor, uint16_t product, uint16_t version) {
    return vendor == SC2_LINK_VENDOR_ID && product == SC2_LINK_PRODUCT_ID && version == SC2_LINK_XPAD_VERSION;
}

/*
 * Every field travels as a 16-bit axis with a logical range of -32768 to
 * 32767, and an unsigned field as its bit pattern.
 */
sc2_link_controls_t sc2_link_decode_controls(const int32_t axis_values[ABS_CNT]) {
    sc2_link_controls_t controls = {
            .buttons = (uint32_t) (uint16_t) axis_values[ABS_THROTTLE] |
                       (uint32_t) (uint16_t) axis_values[ABS_RUDDER] << 16,
            .left_stick_x = (int16_t) axis_values[ABS_X],
            .left_stick_y = (int16_t) axis_values[ABS_Z],
            .right_stick_x = (int16_t) axis_values[ABS_RX],
            .right_stick_y = (int16_t) axis_values[ABS_RY],
            .left_trigger = (uint16_t) axis_values[ABS_GAS],
            .right_trigger = (uint16_t) axis_values[ABS_BRAKE],
            .status = (uint16_t) axis_values[ABS_WHEEL],
    };
    return controls;
}

sc2_link_pads_t sc2_link_decode_pads(const int32_t axis_values[ABS_CNT]) {
    uint16_t touched = (uint16_t) axis_values[ABS_WHEEL];
    sc2_link_pads_t pads = {
            .pads = {
                    decode_pad(axis_values, ABS_X, ABS_Z, ABS_RX, (touched & PADS_LEFT_TOUCHED) != 0),
                    decode_pad(axis_values, ABS_RY, ABS_THROTTLE, ABS_RUDDER, (touched & PADS_RIGHT_TOUCHED) != 0),
            },
    };
    return pads;
}

sc2_link_motion_t sc2_link_decode_motion(const int32_t axis_values[ABS_CNT]) {
    sc2_link_motion_t motion = {
            .accelerometer_x = (int16_t) axis_values[ABS_X],
            .accelerometer_y = (int16_t) axis_values[ABS_Z],
            .accelerometer_z = (int16_t) axis_values[ABS_RX],
            .gyroscope_x = (int16_t) axis_values[ABS_RY],
            .gyroscope_y = (int16_t) axis_values[ABS_THROTTLE],
            .gyroscope_z = (int16_t) axis_values[ABS_RUDDER],
    };
    return motion;
}

bool sc2_link_controller_connected(const sc2_link_controls_t *controls) {
    return (controls->status & STATUS_CONNECTED) != 0;
}

sc2_link_battery_t sc2_link_battery(uint16_t status) {
    if ((status & STATUS_BATTERY_KNOWN) == 0) {
        return sc2_link_unknown_battery();
    }
    sc2_link_battery_t battery = {
            .state = moonlight_battery_state((status >> STATUS_CHARGE_STATE_SHIFT) & STATUS_CHARGE_STATE_MASK),
            .percentage = (uint8_t) ((status >> STATUS_BATTERY_PERCENT_SHIFT) & STATUS_BATTERY_PERCENT_MASK),
    };
    return battery;
}

sc2_link_battery_t sc2_link_unknown_battery(void) {
    sc2_link_battery_t battery = {
            .state = LI_BATTERY_STATE_UNKNOWN,
            .percentage = LI_BATTERY_PERCENTAGE_UNKNOWN,
    };
    return battery;
}

bool sc2_link_battery_equal(const sc2_link_battery_t *battery, const sc2_link_battery_t *other) {
    return battery->state == other->state && battery->percentage == other->percentage;
}

int sc2_link_moonlight_buttons(uint32_t buttons) {
    int flags = 0;
    for (size_t index = 0; index < BUTTON_MAPPING_COUNT; index++) {
        if (buttons & button_mappings[index].controller_bit) {
            flags |= button_mappings[index].moonlight_flag;
        }
    }
    return flags;
}

uint32_t sc2_link_supported_button_flags(void) {
    uint32_t flags = 0;
    for (size_t index = 0; index < BUTTON_MAPPING_COUNT; index++) {
        flags |= (uint32_t) button_mappings[index].moonlight_flag;
    }
    return flags;
}

/*
 * The inverse of libvirtualhid, which writes the host's trigger value of 0 to
 * 1 times 32767. A full pull reads about 0x8000, hence the clamp.
 */
unsigned char sc2_link_moonlight_trigger(uint16_t raw) {
    uint32_t scaled = raw * 255u / TRIGGER_FULL_SCALE;
    return (unsigned char) (scaled > 255u ? 255u : scaled);
}

/*
 * Sticks pass unchanged: Moonlight's Y points up as the controller's does, and
 * libvirtualhid writes the host's values into its report without a sign change.
 */
sc2_link_gamepad_state_t sc2_link_gamepad_state(const sc2_link_controls_t *controls) {
    sc2_link_gamepad_state_t state = {
            .buttons = sc2_link_moonlight_buttons(controls->buttons),
            .left_trigger = sc2_link_moonlight_trigger(controls->left_trigger),
            .right_trigger = sc2_link_moonlight_trigger(controls->right_trigger),
            .left_stick_x = controls->left_stick_x,
            .left_stick_y = controls->left_stick_y,
            .right_stick_x = controls->right_stick_x,
            .right_stick_y = controls->right_stick_y,
    };
    return state;
}

bool sc2_link_gamepad_state_equal(const sc2_link_gamepad_state_t *state, const sc2_link_gamepad_state_t *other) {
    return state->buttons == other->buttons && state->left_trigger == other->left_trigger &&
           state->right_trigger == other->right_trigger && state->left_stick_x == other->left_stick_x &&
           state->left_stick_y == other->left_stick_y && state->right_stick_x == other->right_stick_x &&
           state->right_stick_y == other->right_stick_y;
}

/* As SDL's triton driver converts them, whose inverse libvirtualhid applies on the host. */
float sc2_link_pad_x(int16_t raw) {
    return (float) raw / 65536.0f + 0.5f;
}

float sc2_link_pad_y(int16_t raw) {
    return -(float) raw / 65536.0f + 0.5f;
}

float sc2_link_pad_pressure(uint16_t raw) {
    return (float) raw / 32768.0f;
}

int sc2_link_touch_event_type(const sc2_link_pad_t *sent, const sc2_link_pad_t *current) {
    if (current->touched != sent->touched) {
        return current->touched ? LI_TOUCH_EVENT_DOWN : LI_TOUCH_EVENT_UP;
    }
    if (current->touched &&
        (current->x != sent->x || current->y != sent->y || current->pressure != sent->pressure)) {
        return LI_TOUCH_EVENT_MOVE;
    }
    return SC2_LINK_NO_TOUCH_EVENT;
}

sc2_link_vector_t sc2_link_acceleration(const sc2_link_motion_t *motion) {
    return sdl_sensor_axes(motion->accelerometer_x, motion->accelerometer_y, motion->accelerometer_z,
                           ACCELEROMETER_FULL_SCALE_G * STANDARD_GRAVITY);
}

sc2_link_vector_t sc2_link_angular_velocity(const sc2_link_motion_t *motion) {
    return sdl_sensor_axes(motion->gyroscope_x, motion->gyroscope_y, motion->gyroscope_z,
                           GYROSCOPE_FULL_SCALE_DEGREES_PER_SECOND);
}

bool sc2_link_motion_due(uint64_t next_due_us, uint64_t now_us, uint32_t period_us) {
    return period_us != 0 && now_us >= next_due_us;
}

/*
 * The next sample is due a full period after this one was due, or half a
 * period after this one went out if that is later. So a sample can go out up
 * to half a period early, and the jitter of the samples' arrival does not drop
 * those that come in early: the dongle sends one about every 4 ms and USB polls
 * it every 1 ms, while the host asks for 250 Hz, a period of 4 ms. The due time
 * still moves on by at least a period per sample sent, so on average no more
 * samples go out than the host asked for, and no two less than half a period
 * apart.
 */
uint64_t sc2_link_motion_next_due(uint64_t next_due_us, uint64_t now_us, uint32_t period_us) {
    uint64_t period_after_due_us = next_due_us + period_us;
    uint64_t half_period_after_now_us = now_us + (period_us - period_us / 2);
    return period_after_due_us > half_period_after_now_us ? period_after_due_us : half_period_after_now_us;
}

/*
 * The host's virtual controller repeats its last state, so without a final
 * zero the last gyroscope reading keeps turning the view while input is
 * blocked. A zero gyroscope is the halt that moonlight-common-c sends reliably
 * (InputStream.c, motion packets).
 */
bool sc2_link_gyroscope_halt_due(bool arrived, bool was_accepting_input, bool accepting_input,
                                 uint32_t gyroscope_period_us) {
    return arrived && was_accepting_input && !accepting_input && gyroscope_period_us != 0;
}

static sc2_link_pad_t decode_pad(const int32_t axis_values[ABS_CNT], int x_code, int y_code, int pressure_code,
                                 bool touched) {
    sc2_link_pad_t pad = {
            .x = (int16_t) axis_values[x_code],
            .y = (int16_t) axis_values[y_code],
            .pressure = (uint16_t) axis_values[pressure_code],
            .touched = touched,
    };
    return pad;
}

/*
 * Raw x, z and the negated raw y become the x, y and z that
 * LiSendControllerMotionEvent() takes, scaled to full_scale at a raw 32768.
 * This is what SDL's triton driver does with the same raw values
 * (HIDAPI_DriverSteamTriton_HandleState() in
 * src/joystick/hidapi/SDL_hidapi_steam_triton.c, SDL fda4ece277), except that
 * Moonlight takes degrees per second where SDL gives radians. Sunshine passes
 * the values to libvirtualhid unchanged (gamepad_motion() in
 * src/platform/virtualhid_input.cpp), and libvirtualhid's
 * pack_steam_controller_input_report() (src/core/report.cpp, e9ede639) writes
 * x, -z and y back at 32768 / (2 g) and 32768 / 2000, so Steam on the host
 * reads the controller's raw values again.
 */
static sc2_link_vector_t sdl_sensor_axes(int16_t raw_x, int16_t raw_y, int16_t raw_z, float full_scale) {
    sc2_link_vector_t vector = {
            .x = (float) raw_x / SENSOR_RAW_FULL_SCALE * full_scale,
            .y = (float) raw_z / SENSOR_RAW_FULL_SCALE * full_scale,
            .z = -(float) raw_y / SENSOR_RAW_FULL_SCALE * full_scale,
    };
    return vector;
}

/*
 * The charge states that SDL's triton driver tells apart
 * (HIDAPI_DriverSteamTriton_HandleBatteryStatus()), which are also the ones
 * libvirtualhid writes back from Moonlight's. Reset and source validation are
 * unknown to both, and Sunshine drops the battery of a controller whose state
 * is unknown.
 */
static uint8_t moonlight_battery_state(unsigned int charge_state) {
    switch (charge_state) {
        case CHARGE_STATE_DISCHARGING:
            return LI_BATTERY_STATE_DISCHARGING;
        case CHARGE_STATE_CHARGING:
            return LI_BATTERY_STATE_CHARGING;
        case CHARGE_STATE_CHARGING_DONE:
            return LI_BATTERY_STATE_FULL;
        default:
            return LI_BATTERY_STATE_UNKNOWN;
    }
}

static bool ends_with(const char *text, const char *suffix) {
    size_t text_length = strlen(text);
    size_t suffix_length = strlen(suffix);
    return text_length >= suffix_length && strcmp(text + text_length - suffix_length, suffix) == 0;
}
