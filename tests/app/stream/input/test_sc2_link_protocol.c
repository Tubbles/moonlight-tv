#include "unity.h"
#include "stream/input/sc2_link_protocol.h"

#include <string.h>

#include <Limelight.h>

#define DONGLE_PHYS "usb-0000:00:14.0-1"

static int32_t axis_values[ABS_CNT];

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
    return UNITY_END();
}
