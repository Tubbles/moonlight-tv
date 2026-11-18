#pragma once

/*
 * OpenPuck dongle in its "SC2 Link" mode, which carries one Steam Controller
 * (2nd generation) as evdev nodes: xpad binds USB interface 0 as an ordinary
 * Xbox 360 pad, and hid-generic makes one input node of each HID interface 1
 * to 3 (controls, pads, motion). Reference: docs/PROTOCOL.md, section 9.5, of
 * the OpenPuck sources.
 *
 * Everything here is free of I/O so it can be tested on any Linux host.
 */

#include <stdbool.h>
#include <stdint.h>
#include <linux/input.h>

#define SC2_LINK_VENDOR_ID 0x045E
#define SC2_LINK_PRODUCT_ID 0x028E
// The mode's bcdDevice, which xpad reports as the version of its node.
#define SC2_LINK_XPAD_VERSION 0x5C21
// usbhid names the HID nodes after the USB manufacturer and product strings.
#define SC2_LINK_HID_NODE_NAME "OpenPuck SC2 Link"

// Index 0 is the left pad and 1 the right one, as in Moonlight's touchpadIndex.
#define SC2_LINK_PAD_COUNT 2

// Returned by sc2_link_touch_event_type() when no touch event is due.
#define SC2_LINK_NO_TOUCH_EVENT (-1)

typedef enum sc2_link_node_role_t {
    SC2_LINK_NODE_OTHER = 0,
    SC2_LINK_NODE_CONTROLS,
    SC2_LINK_NODE_PADS,
    SC2_LINK_NODE_MOTION,
} sc2_link_node_role_t;

/** The controls node's values, raw as the controller sent them. */
typedef struct sc2_link_controls_t {
    // Button bits of the controller's state report 0x45.
    uint32_t buttons;
    int16_t left_stick_x;
    int16_t left_stick_y;
    int16_t right_stick_x;
    int16_t right_stick_y;
    uint16_t left_trigger;
    uint16_t right_trigger;
    // Bit 0 controller connected, bits 1 to 3 charge state, 4 to 10 battery percent, 11 battery known.
    uint16_t status;
} sc2_link_controls_t;

/** One trackpad from the pads node, raw as the controller sent it. */
typedef struct sc2_link_pad_t {
    int16_t x;
    int16_t y;
    uint16_t pressure;
    bool touched;
} sc2_link_pad_t;

typedef struct sc2_link_pads_t {
    sc2_link_pad_t pads[SC2_LINK_PAD_COUNT];
} sc2_link_pads_t;

/** The motion node's values, raw as the controller sent them. */
typedef struct sc2_link_motion_t {
    int16_t accelerometer_x;
    int16_t accelerometer_y;
    int16_t accelerometer_z;
    int16_t gyroscope_x;
    int16_t gyroscope_y;
    int16_t gyroscope_z;
} sc2_link_motion_t;

/** A sensor reading as LiSendControllerMotionEvent() takes it. */
typedef struct sc2_link_vector_t {
    float x;
    float y;
    float z;
} sc2_link_vector_t;

/** A battery as LiSendControllerBatteryEvent() takes it. */
typedef struct sc2_link_battery_t {
    // LI_BATTERY_STATE_*
    uint8_t state;
    // The controller's percent, or LI_BATTERY_PERCENTAGE_UNKNOWN.
    uint8_t percentage;
} sc2_link_battery_t;

/** A controller state as LiSendMultiControllerEvent() takes it. */
typedef struct sc2_link_gamepad_state_t {
    int buttons;
    unsigned char left_trigger;
    unsigned char right_trigger;
    short left_stick_x;
    short left_stick_y;
    short right_stick_x;
    short right_stick_y;
} sc2_link_gamepad_state_t;

/**
 * Which of the dongle's HID nodes an evdev node is, from its EVIOCGID vendor
 * and product, its name and its phys string. The phys string ends in
 * "/input" and the USB interface number.
 */
sc2_link_node_role_t sc2_link_node_role(uint16_t vendor, uint16_t product, const char *name, const char *phys);

/** Whether two nodes belong to the same USB device: their phys strings differ only after the last '/'. */
bool sc2_link_same_dongle(const char *phys, const char *other_phys);

/** Whether a controller with this vendor, product and version is the dongle's xpad node. */
bool sc2_link_is_xpad(uint16_t vendor, uint16_t product, uint16_t version);

/** The controls node's state, from its last value of every ABS code. */
sc2_link_controls_t sc2_link_decode_controls(const int32_t axis_values[ABS_CNT]);

/** The pads node's state, from its last value of every ABS code. */
sc2_link_pads_t sc2_link_decode_pads(const int32_t axis_values[ABS_CNT]);

/** The motion node's state, from its last value of every ABS code. */
sc2_link_motion_t sc2_link_decode_motion(const int32_t axis_values[ABS_CNT]);

bool sc2_link_controller_connected(const sc2_link_controls_t *controls);

/** The battery from the controls node's status. */
sc2_link_battery_t sc2_link_battery(uint16_t status);

/** A battery the controller has not reported yet, which is also where the host starts. */
sc2_link_battery_t sc2_link_unknown_battery(void);

bool sc2_link_battery_equal(const sc2_link_battery_t *battery, const sc2_link_battery_t *other);

/** Moonlight button flags for the controller's button bits. */
int sc2_link_moonlight_buttons(uint32_t buttons);

/** Every Moonlight button flag the controller can produce, for the arrival event. */
uint32_t sc2_link_supported_button_flags(void);

unsigned char sc2_link_moonlight_trigger(uint16_t raw);

sc2_link_gamepad_state_t sc2_link_gamepad_state(const sc2_link_controls_t *controls);

bool sc2_link_gamepad_state_equal(const sc2_link_gamepad_state_t *state, const sc2_link_gamepad_state_t *other);

/** Trackpad coordinates and pressure in Moonlight's range of 0 to 1, y pointing down. */
float sc2_link_pad_x(int16_t raw);

float sc2_link_pad_y(int16_t raw);

float sc2_link_pad_pressure(uint16_t raw);

/**
 * The LI_TOUCH_EVENT_* that takes the host from the pad state it was last
 * sent to the current one, or SC2_LINK_NO_TOUCH_EVENT.
 */
int sc2_link_touch_event_type(const sc2_link_pad_t *sent, const sc2_link_pad_t *current);

/** Acceleration in m/s², gravity included, on SDL's sensor axes. */
sc2_link_vector_t sc2_link_acceleration(const sc2_link_motion_t *motion);

/** Angular velocity in degrees per second, on SDL's sensor axes. */
sc2_link_vector_t sc2_link_angular_velocity(const sc2_link_motion_t *motion);

/**
 * Whether a sample read at now_us may go out to a host that asked for one
 * sample every period_us, a period of 0 meaning none. next_due_us starts at 0
 * and follows sc2_link_motion_next_due() after every sample sent.
 */
bool sc2_link_motion_due(uint64_t next_due_us, uint64_t now_us, uint32_t period_us);

/** When the sample after the one sent at now_us is due. */
uint64_t sc2_link_motion_next_due(uint64_t next_due_us, uint64_t now_us, uint32_t period_us);
