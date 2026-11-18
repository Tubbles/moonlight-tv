#include "unity.h"
#include "stream/input/sc2_link_protocol.h"

#include <string.h>

#include <Limelight.h>

#define DONGLE_PHYS "usb-0000:00:14.0-1"

#define STANDARD_GRAVITY 9.80665f
#define STATUS_CONNECTED 0x0001
#define STATUS_BATTERY_KNOWN 0x0800
// The host asks for 250 Hz, and the dongle sends a sample about every 4 ms.
#define PERIOD_250_HZ_US 4000u
// Any start well past 0, as the monotonic clock is.
#define START_US 1000000000u

static int32_t axis_values[ABS_CNT];

static uint16_t battery_status(unsigned int charge_state, unsigned int percent) {
    return (uint16_t) (STATUS_CONNECTED | charge_state << 1 | percent << 4 | STATUS_BATTERY_KNOWN);
}

/* libvirtualhid's pack_steam_controller_input_report(): x, -z and y at the given scale, rounded. */
static void pack_like_the_host(const sc2_link_vector_t *vector, float scale, int16_t raw[3]) {
    float values[3] = {vector->x * scale, -vector->z * scale, vector->y * scale};
    for (int index = 0; index < 3; index++) {
        raw[index] = (int16_t) (values[index] + (values[index] >= 0 ? 0.5f : -0.5f));
    }
}

/** Sends one sample at every time given and counts those that went out. */
static int count_motion_sent(const uint64_t *sample_times_us, int sample_count, uint32_t period_us) {
    uint64_t next_due_us = 0;
    int sent = 0;
    for (int index = 0; index < sample_count; index++) {
        if (sc2_link_motion_due(next_due_us, sample_times_us[index], period_us)) {
            next_due_us = sc2_link_motion_next_due(next_due_us, sample_times_us[index], period_us);
            sent++;
        }
    }
    return sent;
}

void setUp(void) {
    memset(axis_values, 0, sizeof(axis_values));
}

void tearDown(void) {
}

void test_node_role_by_interface(void) {
    TEST_ASSERT_EQUAL(SC2_LINK_NODE_CONTROLS,
                      sc2_link_node_role(0x045E, 0x028E, "OpenPuck SC2 Link", DONGLE_PHYS "/input1"));
    TEST_ASSERT_EQUAL(SC2_LINK_NODE_PADS,
                      sc2_link_node_role(0x045E, 0x028E, "OpenPuck SC2 Link", DONGLE_PHYS "/input2"));
    TEST_ASSERT_EQUAL(SC2_LINK_NODE_MOTION,
                      sc2_link_node_role(0x045E, 0x028E, "OpenPuck SC2 Link", DONGLE_PHYS "/input3"));
    TEST_ASSERT_EQUAL(SC2_LINK_NODE_OTHER,
                      sc2_link_node_role(0x045E, 0x028E, "OpenPuck SC2 Link", DONGLE_PHYS "/input4"));
    TEST_ASSERT_EQUAL(SC2_LINK_NODE_OTHER,
                      sc2_link_node_role(0x045E, 0x028E, "OpenPuck SC2 Link", DONGLE_PHYS "/input11"));
}

void test_node_role_rejects_other_devices(void) {
    TEST_ASSERT_EQUAL(SC2_LINK_NODE_OTHER,
                      sc2_link_node_role(0x045E, 0x028E, "Microsoft X-Box 360 pad", DONGLE_PHYS "/input1"));
    TEST_ASSERT_EQUAL(SC2_LINK_NODE_OTHER,
                      sc2_link_node_role(0x28DE, 0x028E, "OpenPuck SC2 Link", DONGLE_PHYS "/input1"));
    TEST_ASSERT_EQUAL(SC2_LINK_NODE_OTHER,
                      sc2_link_node_role(0x045E, 0x0B12, "OpenPuck SC2 Link", DONGLE_PHYS "/input2"));
    TEST_ASSERT_EQUAL(SC2_LINK_NODE_OTHER, sc2_link_node_role(0x045E, 0x028E, "OpenPuck SC2 Link", ""));
}

void test_same_dongle_compares_phys_before_interface(void) {
    TEST_ASSERT_TRUE(sc2_link_same_dongle(DONGLE_PHYS "/input1", DONGLE_PHYS "/input2"));
    TEST_ASSERT_FALSE(sc2_link_same_dongle(DONGLE_PHYS "/input1", "usb-0000:00:14.0-2/input2"));
    TEST_ASSERT_FALSE(sc2_link_same_dongle(DONGLE_PHYS "/input1", DONGLE_PHYS ".1/input2"));
    TEST_ASSERT_FALSE(sc2_link_same_dongle(DONGLE_PHYS "/input1", "input2"));
}

void test_xpad_needs_the_mode_version(void) {
    TEST_ASSERT_TRUE(sc2_link_is_xpad(0x045E, 0x028E, 0x5C21));
    TEST_ASSERT_FALSE(sc2_link_is_xpad(0x045E, 0x028E, 0x0114));
    TEST_ASSERT_FALSE(sc2_link_is_xpad(0x045E, 0x02EA, 0x5C21));
}

void test_decode_controls_keeps_bit_patterns(void) {
    axis_values[ABS_MISC] = 1234;
    axis_values[ABS_THROTTLE] = (int16_t) 0x8001;
    axis_values[ABS_RUDDER] = 0x2008;
    axis_values[ABS_X] = -32768;
    axis_values[ABS_Z] = 32767;
    axis_values[ABS_RX] = -5;
    axis_values[ABS_RY] = 7;
    axis_values[ABS_WHEEL] = 0x0B51;
    axis_values[ABS_GAS] = (int16_t) 0x8000;
    axis_values[ABS_BRAKE] = 100;

    sc2_link_controls_t controls = sc2_link_decode_controls(axis_values);
    TEST_ASSERT_EQUAL_HEX32(0x20088001, controls.buttons);
    TEST_ASSERT_EQUAL_INT16(-32768, controls.left_stick_x);
    TEST_ASSERT_EQUAL_INT16(32767, controls.left_stick_y);
    TEST_ASSERT_EQUAL_INT16(-5, controls.right_stick_x);
    TEST_ASSERT_EQUAL_INT16(7, controls.right_stick_y);
    TEST_ASSERT_EQUAL_UINT16(0x8000, controls.left_trigger);
    TEST_ASSERT_EQUAL_UINT16(100, controls.right_trigger);
    TEST_ASSERT_EQUAL_HEX16(0x0B51, controls.status);
    TEST_ASSERT_TRUE(sc2_link_controller_connected(&controls));

    controls.status = 0x0B50;
    TEST_ASSERT_FALSE(sc2_link_controller_connected(&controls));
}

void test_decode_pads(void) {
    axis_values[ABS_X] = -100;
    axis_values[ABS_Z] = 200;
    axis_values[ABS_RX] = (int16_t) 0x9000;
    axis_values[ABS_RY] = 300;
    axis_values[ABS_THROTTLE] = -400;
    axis_values[ABS_RUDDER] = 500;
    axis_values[ABS_WHEEL] = 0x2;
    axis_values[ABS_GAS] = 0x1234;
    axis_values[ABS_BRAKE] = 0x5678;

    sc2_link_pads_t pads = sc2_link_decode_pads(axis_values);
    TEST_ASSERT_EQUAL_INT16(-100, pads.pads[0].x);
    TEST_ASSERT_EQUAL_INT16(200, pads.pads[0].y);
    TEST_ASSERT_EQUAL_UINT16(0x9000, pads.pads[0].pressure);
    TEST_ASSERT_FALSE(pads.pads[0].touched);
    TEST_ASSERT_EQUAL_INT16(300, pads.pads[1].x);
    TEST_ASSERT_EQUAL_INT16(-400, pads.pads[1].y);
    TEST_ASSERT_EQUAL_UINT16(500, pads.pads[1].pressure);
    TEST_ASSERT_TRUE(pads.pads[1].touched);

    axis_values[ABS_WHEEL] = 0x1;
    pads = sc2_link_decode_pads(axis_values);
    TEST_ASSERT_TRUE(pads.pads[0].touched);
    TEST_ASSERT_FALSE(pads.pads[1].touched);
}

void test_every_button_bit_maps_to_its_flag(void) {
    static const struct {
        uint32_t controller_bit;
        int moonlight_flag;
    } expected[] = {
            {0x00000001, A_FLAG},
            {0x00000002, B_FLAG},
            {0x00000004, X_FLAG},
            {0x00000008, Y_FLAG},
            {0x00000010, MISC_FLAG},
            {0x00000020, RS_CLK_FLAG},
            {0x00000040, PLAY_FLAG},
            {0x00000080, PADDLE1_FLAG},
            {0x00000100, PADDLE3_FLAG},
            {0x00000200, RB_FLAG},
            {0x00000400, DOWN_FLAG},
            {0x00000800, RIGHT_FLAG},
            {0x00001000, LEFT_FLAG},
            {0x00002000, UP_FLAG},
            {0x00004000, BACK_FLAG},
            {0x00008000, LS_CLK_FLAG},
            {0x00010000, SPECIAL_FLAG},
            {0x00020000, PADDLE2_FLAG},
            {0x00040000, PADDLE4_FLAG},
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
    for (size_t index = 0; index < sizeof(expected) / sizeof(expected[0]); index++) {
        TEST_ASSERT_EQUAL_HEX32(expected[index].moonlight_flag,
                                sc2_link_moonlight_buttons(expected[index].controller_bit));
    }
}

/* The host derives the pad touch bits from touch contacts, so they are no buttons. */
void test_pad_touch_bits_are_no_buttons(void) {
    TEST_ASSERT_EQUAL_HEX32(0, sc2_link_moonlight_buttons(0x00200000));
    TEST_ASSERT_EQUAL_HEX32(0, sc2_link_moonlight_buttons(0x02000000));
    TEST_ASSERT_EQUAL_HEX32(0, sc2_link_moonlight_buttons(0xC0000000));
}

void test_buttons_combine(void) {
    TEST_ASSERT_EQUAL_HEX32(A_FLAG | LB_FLAG | STEAM_LEFT_GRIP_TOUCH_FLAG,
                            sc2_link_moonlight_buttons(0x00000001 | 0x00080000 | 0x02000000 | 0x20000000));
}

void test_supported_flags_are_every_mapped_flag(void) {
    uint32_t expected = A_FLAG | B_FLAG | X_FLAG | Y_FLAG | UP_FLAG | DOWN_FLAG | LEFT_FLAG | RIGHT_FLAG |
                        LB_FLAG | RB_FLAG | PLAY_FLAG | BACK_FLAG | LS_CLK_FLAG | RS_CLK_FLAG | SPECIAL_FLAG |
                        PADDLE1_FLAG | PADDLE2_FLAG | PADDLE3_FLAG | PADDLE4_FLAG | MISC_FLAG |
                        STEAM_LEFT_TOUCHPAD_FLAG | STEAM_RIGHT_TOUCHPAD_FLAG | STEAM_LEFT_TRIGGER_CLICK_FLAG |
                        STEAM_RIGHT_TRIGGER_CLICK_FLAG | STEAM_LEFT_STICK_TOUCH_FLAG |
                        STEAM_RIGHT_STICK_TOUCH_FLAG | STEAM_LEFT_GRIP_TOUCH_FLAG | STEAM_RIGHT_GRIP_TOUCH_FLAG;
    TEST_ASSERT_EQUAL_HEX32(expected, sc2_link_supported_button_flags());
}

void test_trigger_scale(void) {
    TEST_ASSERT_EQUAL_UINT8(0, sc2_link_moonlight_trigger(0));
    TEST_ASSERT_EQUAL_UINT8(127, sc2_link_moonlight_trigger(16384));
    TEST_ASSERT_EQUAL_UINT8(255, sc2_link_moonlight_trigger(32767));
    TEST_ASSERT_EQUAL_UINT8(255, sc2_link_moonlight_trigger(0x8000));
    TEST_ASSERT_EQUAL_UINT8(255, sc2_link_moonlight_trigger(0xFFFF));
}

void test_gamepad_state_passes_sticks_unchanged(void) {
    sc2_link_controls_t controls = {
            .buttons = 0x00010000,
            .left_stick_x = -32768,
            .left_stick_y = 32767,
            .right_stick_x = 12,
            .right_stick_y = -34,
            .left_trigger = 32767,
            .right_trigger = 0,
    };
    sc2_link_gamepad_state_t state = sc2_link_gamepad_state(&controls);
    TEST_ASSERT_EQUAL_HEX32(SPECIAL_FLAG, state.buttons);
    TEST_ASSERT_EQUAL_INT16(-32768, state.left_stick_x);
    TEST_ASSERT_EQUAL_INT16(32767, state.left_stick_y);
    TEST_ASSERT_EQUAL_INT16(12, state.right_stick_x);
    TEST_ASSERT_EQUAL_INT16(-34, state.right_stick_y);
    TEST_ASSERT_EQUAL_UINT8(255, state.left_trigger);
    TEST_ASSERT_EQUAL_UINT8(0, state.right_trigger);

    sc2_link_gamepad_state_t same = state;
    TEST_ASSERT_TRUE(sc2_link_gamepad_state_equal(&state, &same));
    same.right_stick_y = -35;
    TEST_ASSERT_FALSE(sc2_link_gamepad_state_equal(&state, &same));
}

/* Raw Y points up, Moonlight's touch Y points down. */
void test_pad_coordinates(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.5f, sc2_link_pad_x(0));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sc2_link_pad_x(-32768));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, sc2_link_pad_x(32767));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, sc2_link_pad_y(0));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, sc2_link_pad_y(-32768));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, sc2_link_pad_y(32767));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sc2_link_pad_pressure(0));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, sc2_link_pad_pressure(32768));
}

void test_touch_events(void) {
    sc2_link_pad_t lifted = {.x = 10, .y = 20, .pressure = 0, .touched = false};
    sc2_link_pad_t down = {.x = 10, .y = 20, .pressure = 5, .touched = true};
    sc2_link_pad_t moved = {.x = 11, .y = 20, .pressure = 5, .touched = true};
    sc2_link_pad_t pressed = {.x = 11, .y = 20, .pressure = 6, .touched = true};
    sc2_link_pad_t lifted_elsewhere = {.x = 99, .y = 99, .pressure = 0, .touched = false};

    TEST_ASSERT_EQUAL(LI_TOUCH_EVENT_DOWN, sc2_link_touch_event_type(&lifted, &down));
    TEST_ASSERT_EQUAL(LI_TOUCH_EVENT_MOVE, sc2_link_touch_event_type(&down, &moved));
    TEST_ASSERT_EQUAL(LI_TOUCH_EVENT_MOVE, sc2_link_touch_event_type(&moved, &pressed));
    TEST_ASSERT_EQUAL(SC2_LINK_NO_TOUCH_EVENT, sc2_link_touch_event_type(&pressed, &pressed));
    TEST_ASSERT_EQUAL(LI_TOUCH_EVENT_UP, sc2_link_touch_event_type(&pressed, &lifted));
    TEST_ASSERT_EQUAL(SC2_LINK_NO_TOUCH_EVENT, sc2_link_touch_event_type(&lifted, &lifted_elsewhere));
}

void test_decode_motion(void) {
    axis_values[ABS_X] = -1;
    axis_values[ABS_Z] = 2;
    axis_values[ABS_RX] = (int16_t) 0x8000;
    axis_values[ABS_RY] = 32767;
    axis_values[ABS_THROTTLE] = -5;
    axis_values[ABS_RUDDER] = 6;

    sc2_link_motion_t motion = sc2_link_decode_motion(axis_values);
    TEST_ASSERT_EQUAL_INT16(-1, motion.accelerometer_x);
    TEST_ASSERT_EQUAL_INT16(2, motion.accelerometer_y);
    TEST_ASSERT_EQUAL_INT16(-32768, motion.accelerometer_z);
    TEST_ASSERT_EQUAL_INT16(32767, motion.gyroscope_x);
    TEST_ASSERT_EQUAL_INT16(-5, motion.gyroscope_y);
    TEST_ASSERT_EQUAL_INT16(6, motion.gyroscope_z);
}

/* SDL's axes: x is raw x, y is raw z, z is the negated raw y. Full scale is 2 g. */
void test_acceleration_axes_and_units(void) {
    sc2_link_motion_t motion = {.accelerometer_x = 16384, .accelerometer_y = 0, .accelerometer_z = 0};
    sc2_link_vector_t acceleration = sc2_link_acceleration(&motion);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, STANDARD_GRAVITY, acceleration.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, acceleration.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, acceleration.z);

    motion = (sc2_link_motion_t) {.accelerometer_y = 16384};
    acceleration = sc2_link_acceleration(&motion);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, acceleration.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, acceleration.y);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -STANDARD_GRAVITY, acceleration.z);

    motion = (sc2_link_motion_t) {.accelerometer_z = -32768};
    acceleration = sc2_link_acceleration(&motion);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, acceleration.x);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -2.0f * STANDARD_GRAVITY, acceleration.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, acceleration.z);
}

/* The same axes in degrees per second. Full scale is 2000. */
void test_angular_velocity_axes_and_units(void) {
    sc2_link_motion_t motion = {.gyroscope_x = -16384, .accelerometer_x = 12345};
    sc2_link_vector_t angular_velocity = sc2_link_angular_velocity(&motion);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1000.0f, angular_velocity.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, angular_velocity.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, angular_velocity.z);

    motion = (sc2_link_motion_t) {.gyroscope_y = -32768};
    angular_velocity = sc2_link_angular_velocity(&motion);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, angular_velocity.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, angular_velocity.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2000.0f, angular_velocity.z);

    motion = (sc2_link_motion_t) {.gyroscope_z = 8192};
    angular_velocity = sc2_link_angular_velocity(&motion);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, angular_velocity.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 500.0f, angular_velocity.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, angular_velocity.z);
}

/* Steam on the host must read the controller's raw values again. */
void test_host_packs_motion_back_to_raw(void) {
    static const int16_t samples[][3] = {
            {0, 0, 0},
            {1, -1, 2},
            {-32768, -32768, -32768},
            {32767, 32767, 32767},
            {-32768, 32767, 1234},
            {16384, -8192, -4321},
    };
    for (size_t index = 0; index < sizeof(samples) / sizeof(samples[0]); index++) {
        sc2_link_motion_t motion = {
                .accelerometer_x = samples[index][0],
                .accelerometer_y = samples[index][1],
                .accelerometer_z = samples[index][2],
                .gyroscope_x = samples[index][2],
                .gyroscope_y = samples[index][0],
                .gyroscope_z = samples[index][1],
        };
        sc2_link_vector_t acceleration = sc2_link_acceleration(&motion);
        sc2_link_vector_t angular_velocity = sc2_link_angular_velocity(&motion);
        int16_t accelerometer[3], gyroscope[3];
        pack_like_the_host(&acceleration, 32768.0f / (2.0f * STANDARD_GRAVITY), accelerometer);
        pack_like_the_host(&angular_velocity, 32768.0f / 2000.0f, gyroscope);
        TEST_ASSERT_EQUAL_INT16(motion.accelerometer_x, accelerometer[0]);
        TEST_ASSERT_EQUAL_INT16(motion.accelerometer_y, accelerometer[1]);
        TEST_ASSERT_EQUAL_INT16(motion.accelerometer_z, accelerometer[2]);
        TEST_ASSERT_EQUAL_INT16(motion.gyroscope_x, gyroscope[0]);
        TEST_ASSERT_EQUAL_INT16(motion.gyroscope_y, gyroscope[1]);
        TEST_ASSERT_EQUAL_INT16(motion.gyroscope_z, gyroscope[2]);
    }
}

void test_motion_not_due_without_request(void) {
    TEST_ASSERT_FALSE(sc2_link_motion_due(0, START_US, 0));
    TEST_ASSERT_FALSE(sc2_link_motion_due(START_US, START_US + 1000000u, 0));
}

void test_motion_due_times(void) {
    TEST_ASSERT_TRUE(sc2_link_motion_due(0, START_US, PERIOD_250_HZ_US));
    // After a pause the next sample may follow half a period later.
    uint64_t next_due_us = sc2_link_motion_next_due(0, START_US, PERIOD_250_HZ_US);
    TEST_ASSERT_EQUAL_UINT32(2000u, (uint32_t) (next_due_us - START_US));
    TEST_ASSERT_FALSE(sc2_link_motion_due(next_due_us, START_US + 1999u, PERIOD_250_HZ_US));
    TEST_ASSERT_TRUE(sc2_link_motion_due(next_due_us, START_US + 2000u, PERIOD_250_HZ_US));
    // Taken early, the sample still moves the due time on by a full period.
    next_due_us = sc2_link_motion_next_due(next_due_us, START_US + 2000u, PERIOD_250_HZ_US);
    TEST_ASSERT_EQUAL_UINT32(6000u, (uint32_t) (next_due_us - START_US));
}

/* USB polls the dongle every 1 ms, so a 4 ms sample can arrive 3 to 5 ms after the last one. */
void test_motion_keeps_every_jittered_sample(void) {
    static const uint32_t delays_us[] = {0, 999, 0, 500, 999, 999, 0, 0, 999, 250};
    uint64_t sample_times_us[250];
    for (int index = 0; index < 250; index++) {
        sample_times_us[index] = START_US + (uint64_t) index * 4032u + delays_us[index % 10];
    }
    TEST_ASSERT_EQUAL_INT(250, count_motion_sent(sample_times_us, 250, PERIOD_250_HZ_US));

    for (int index = 0; index < 250; index++) {
        sample_times_us[index] = START_US + (uint64_t) index * 4000u + delays_us[index % 10];
    }
    TEST_ASSERT_EQUAL_INT(250, count_motion_sent(sample_times_us, 250, PERIOD_250_HZ_US));
}

/* One second of samples never yields more than the host asked for, give or take one at the ends. */
void test_motion_holds_the_requested_rate(void) {
    uint64_t sample_times_us[1000];
    for (int index = 0; index < 1000; index++) {
        sample_times_us[index] = START_US + (uint64_t) index * 1000u;
    }
    int sent = count_motion_sent(sample_times_us, 1000, PERIOD_250_HZ_US);
    TEST_ASSERT_INT_WITHIN(1, 250, sent);

    for (int index = 0; index < 250; index++) {
        sample_times_us[index] = START_US + (uint64_t) index * 4000u;
    }
    sent = count_motion_sent(sample_times_us, 250, 10000u);
    TEST_ASSERT_INT_WITHIN(1, 100, sent);
}

void test_motion_never_sends_two_within_half_a_period(void) {
    uint64_t next_due_us = 0;
    uint64_t last_sent_us = 0;
    int sent = 0;
    for (uint64_t now_us = START_US; now_us < START_US + 100000u; now_us += 300u) {
        if (!sc2_link_motion_due(next_due_us, now_us, PERIOD_250_HZ_US)) {
            continue;
        }
        if (sent > 0) {
            TEST_ASSERT_GREATER_OR_EQUAL_UINT32(PERIOD_250_HZ_US / 2, (uint32_t) (now_us - last_sent_us));
        }
        next_due_us = sc2_link_motion_next_due(next_due_us, now_us, PERIOD_250_HZ_US);
        last_sent_us = now_us;
        sent++;
    }
    TEST_ASSERT_INT_WITHIN(1, 25, sent);
}

void test_battery_charge_states(void) {
    sc2_link_battery_t battery = sc2_link_battery(battery_status(1, 80));
    TEST_ASSERT_EQUAL_UINT8(LI_BATTERY_STATE_DISCHARGING, battery.state);
    TEST_ASSERT_EQUAL_UINT8(80, battery.percentage);

    battery = sc2_link_battery(battery_status(2, 0));
    TEST_ASSERT_EQUAL_UINT8(LI_BATTERY_STATE_CHARGING, battery.state);
    TEST_ASSERT_EQUAL_UINT8(0, battery.percentage);

    battery = sc2_link_battery(battery_status(4, 100));
    TEST_ASSERT_EQUAL_UINT8(LI_BATTERY_STATE_FULL, battery.state);
    TEST_ASSERT_EQUAL_UINT8(100, battery.percentage);
}

/* Reset, source validation and states SDL does not name are unknown, and the percent still passes. */
void test_battery_other_charge_states_are_unknown(void) {
    static const unsigned int charge_states[] = {0, 3, 5, 6, 7};
    for (size_t index = 0; index < sizeof(charge_states) / sizeof(charge_states[0]); index++) {
        sc2_link_battery_t battery = sc2_link_battery(battery_status(charge_states[index], 42));
        TEST_ASSERT_EQUAL_UINT8(LI_BATTERY_STATE_UNKNOWN, battery.state);
        TEST_ASSERT_EQUAL_UINT8(42, battery.percentage);
    }
}

/* The dongle clamps the percent to its 7 bits, and the host clamps it to 100. */
void test_battery_percent_passes_as_reported(void) {
    TEST_ASSERT_EQUAL_UINT8(127, sc2_link_battery(battery_status(1, 127)).percentage);
    TEST_ASSERT_EQUAL_UINT8(1, sc2_link_battery(battery_status(1, 1)).percentage);
}

void test_battery_unknown_until_reported(void) {
    sc2_link_battery_t unknown = sc2_link_unknown_battery();
    TEST_ASSERT_EQUAL_UINT8(LI_BATTERY_STATE_UNKNOWN, unknown.state);
    TEST_ASSERT_EQUAL_UINT8(LI_BATTERY_PERCENTAGE_UNKNOWN, unknown.percentage);

    sc2_link_battery_t battery = sc2_link_battery(STATUS_CONNECTED);
    TEST_ASSERT_TRUE(sc2_link_battery_equal(&unknown, &battery));
    battery = sc2_link_battery(0);
    TEST_ASSERT_TRUE(sc2_link_battery_equal(&unknown, &battery));
    // Charge state and percent without the known bit.
    battery = sc2_link_battery((uint16_t) (battery_status(1, 80) & ~STATUS_BATTERY_KNOWN));
    TEST_ASSERT_TRUE(sc2_link_battery_equal(&unknown, &battery));
}

void test_battery_ignores_the_connected_bit(void) {
    sc2_link_battery_t connected = sc2_link_battery(battery_status(2, 55));
    sc2_link_battery_t disconnected = sc2_link_battery((uint16_t) (battery_status(2, 55) & ~STATUS_CONNECTED));
    TEST_ASSERT_TRUE(sc2_link_battery_equal(&connected, &disconnected));
}

void test_battery_equal(void) {
    sc2_link_battery_t battery = {.state = LI_BATTERY_STATE_CHARGING, .percentage = 50};
    sc2_link_battery_t same = battery;
    TEST_ASSERT_TRUE(sc2_link_battery_equal(&battery, &same));
    same.percentage = 51;
    TEST_ASSERT_FALSE(sc2_link_battery_equal(&battery, &same));
    same = battery;
    same.state = LI_BATTERY_STATE_FULL;
    TEST_ASSERT_FALSE(sc2_link_battery_equal(&battery, &same));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_node_role_by_interface);
    RUN_TEST(test_node_role_rejects_other_devices);
    RUN_TEST(test_same_dongle_compares_phys_before_interface);
    RUN_TEST(test_xpad_needs_the_mode_version);
    RUN_TEST(test_decode_controls_keeps_bit_patterns);
    RUN_TEST(test_decode_pads);
    RUN_TEST(test_every_button_bit_maps_to_its_flag);
    RUN_TEST(test_pad_touch_bits_are_no_buttons);
    RUN_TEST(test_buttons_combine);
    RUN_TEST(test_supported_flags_are_every_mapped_flag);
    RUN_TEST(test_trigger_scale);
    RUN_TEST(test_gamepad_state_passes_sticks_unchanged);
    RUN_TEST(test_pad_coordinates);
    RUN_TEST(test_touch_events);
    RUN_TEST(test_decode_motion);
    RUN_TEST(test_acceleration_axes_and_units);
    RUN_TEST(test_angular_velocity_axes_and_units);
    RUN_TEST(test_host_packs_motion_back_to_raw);
    RUN_TEST(test_motion_not_due_without_request);
    RUN_TEST(test_motion_due_times);
    RUN_TEST(test_motion_keeps_every_jittered_sample);
    RUN_TEST(test_motion_holds_the_requested_rate);
    RUN_TEST(test_motion_never_sends_two_within_half_a_period);
    RUN_TEST(test_battery_charge_states);
    RUN_TEST(test_battery_other_charge_states_are_unknown);
    RUN_TEST(test_battery_percent_passes_as_reported);
    RUN_TEST(test_battery_unknown_until_reported);
    RUN_TEST(test_battery_ignores_the_connected_bit);
    RUN_TEST(test_battery_equal);
    return UNITY_END();
}
