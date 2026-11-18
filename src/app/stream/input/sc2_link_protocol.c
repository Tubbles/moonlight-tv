#include "sc2_link_protocol.h"

#include <string.h>

#include <Limelight.h>

#define CONTROLS_INTERFACE_SUFFIX "/input1"
#define PADS_INTERFACE_SUFFIX "/input2"

#define STATUS_CONNECTED 0x0001
#define PADS_LEFT_TOUCHED 0x0001
#define PADS_RIGHT_TOUCHED 0x0002

#define TRIGGER_FULL_SCALE 32767u

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

bool sc2_link_controller_connected(const sc2_link_controls_t *controls) {
    return (controls->status & STATUS_CONNECTED) != 0;
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

static bool ends_with(const char *text, const char *suffix) {
    size_t text_length = strlen(text);
    size_t suffix_length = strlen(suffix);
    return text_length >= suffix_length && strcmp(text + text_length - suffix_length, suffix) == 0;
}
