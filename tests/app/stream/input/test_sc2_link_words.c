#include "unity.h"
#include "stream/input/sc2_link_words.h"

#include <string.h>

#include <Limelight.h>

/*
 * The dongle's side, ported from the Model of OpenPuck's
 * tools/sc2link/sc2link_test.py, which mirrors OpenPuck/mode_sc2link.cpp: the
 * payload words a header takes and the report a complete message builds.
 */

#define OPCODE_CONTROL 0
#define OPCODE_SCRIPT 1
#define OPCODE_RUMBLE 2
#define OPCODE_PULSE 3
#define OPCODE_SWEEP 4
#define OPCODE_TONE 5
#define OPCODE_COMMAND 8
#define REPORT_LENGTH_MAX 10
#define HAPTIC_TYPE_RUMBLE 4

static int opcode_of(uint16_t header) {
    return header & 0x4000 ? OPCODE_COMMAND : header >> 11 & 7;
}

/** The payload words after a header, -1 for a header the dongle rejects. */
static int payload_words(uint16_t header) {
    int opcode = opcode_of(header);
    if (opcode == OPCODE_CONTROL) {
        int sub = header >> 8 & 7;
        int argument = header & 0xFF;
        if (sub == 0x00 || sub == 0x02 || sub == 0x04) {
            return argument ? -1 : 0;
        }
        if (sub == 0x05) {
            return argument ? -1 : 1;
        }
        if (sub == 0x06) {
            return argument <= 13 ? 1 : -1;
        }
        return -1;
    }
    static const struct {
        int opcode;
        uint16_t unused_bits;
        int payload_words;
    } layouts[] = {
            {OPCODE_COMMAND, 0x2000, 0},
            {OPCODE_SCRIPT, 0x0400, 1},
            {OPCODE_RUMBLE, 0x07F8, 2},
            {OPCODE_PULSE, 0x04F8, 3},
            {OPCODE_SWEEP, 0x0400, 4},
            {OPCODE_TONE, 0x0400, 4},
    };
    for (size_t index = 0; index < sizeof(layouts) / sizeof(layouts[0]); index++) {
        if (layouts[index].opcode != opcode) {
            continue;
        }
        if (header & layouts[index].unused_bits) {
            return -1;
        }
        if (opcode == OPCODE_RUMBLE && (header & 4)) {
            return header & 3 ? -1 : 0;
        }
        return layouts[index].payload_words;
    }
    return -1;
}

static bool payload_valid(uint16_t header, int index, uint16_t data) {
    switch (opcode_of(header)) {
        case OPCODE_CONTROL:
            return (header >> 8 & 7) == 0x06 ? data == (0x4D00 | (header & 0xFF)) : data == 0x5546;
        case OPCODE_SCRIPT:
            return data <= 0xFF;
        case OPCODE_SWEEP:
            return index != 3 || data <= 7;
        case OPCODE_TONE:
            return index != 3 || data <= 0x7FF;
        default:
            return true;
    }
}

/** A 16-bit field: bit 15 from bit `at` of high_bits, the rest from low_bits. */
static uint16_t join(uint16_t high_bits, int at, uint16_t low_bits) {
    return (uint16_t) ((high_bits >> at & 1) << 15 | low_bits);
}

static void put16(uint8_t *report, uint16_t value) {
    report[0] = (uint8_t) value;
    report[1] = (uint8_t) (value >> 8);
}

/** The report a complete message queues, report id first, as SDL's controller_structs.h lays them out. */
static size_t build_report(uint16_t header, const uint16_t *payload, uint8_t report[REPORT_LENGTH_MAX]) {
    uint8_t side = header >> 8 & 3;
    uint8_t gain = header & 0xFF;
    switch (opcode_of(header)) {
        case OPCODE_COMMAND:
            report[0] = 0x82;
            report[1] = side;
            report[2] = header >> 10 & 7;
            report[3] = gain;
            return 4;
        case OPCODE_SCRIPT:
            report[0] = 0x85;
            report[1] = side;
            report[2] = (uint8_t) payload[0];
            report[3] = gain;
            return 4;
        case OPCODE_RUMBLE: {
            uint16_t left = header & 4 ? 0 : join(header, 1, payload[0]);
            uint16_t right = header & 4 ? 0 : join(header, 0, payload[1]);
            report[0] = 0x80;
            report[1] = left || right ? HAPTIC_TYPE_RUMBLE : 0;
            put16(report + 2, left > right ? left : right);
            put16(report + 4, left);
            report[6] = 0;
            put16(report + 7, right);
            report[9] = 0;
            return 10;
        }
        case OPCODE_PULSE:
            report[0] = 0x81;
            report[1] = side;
            for (int index = 0; index < 3; index++) {
                put16(report + 2 + 2 * index, join(header, 2 - index, payload[index]));
            }
            return 8;
        case OPCODE_SWEEP:
            report[0] = 0x84;
            report[1] = side;
            report[2] = gain;
            for (int index = 0; index < 3; index++) {
                put16(report + 3 + 2 * index, join(payload[3], 2 - index, payload[index]));
            }
            return 9;
        case OPCODE_TONE:
            report[0] = 0x83;
            report[1] = side;
            report[2] = gain;
            for (int index = 0; index < 3; index++) {
                put16(report + 3 + 2 * index, join(payload[3], 10 - index, payload[index]));
            }
            report[9] = (uint8_t) payload[3];
            return 10;
        default:
            return 0;
    }
}

/** Checks the message as the dongle parses it and returns the report it builds. */
static size_t report_of(const sc2_link_message_t *message, uint8_t report[REPORT_LENGTH_MAX]) {
    TEST_ASSERT_GREATER_THAN_UINT8(0, message->word_count);
    for (uint8_t index = 0; index < message->word_count; index++) {
        TEST_ASSERT_LESS_OR_EQUAL_HEX16(0x7FFF, message->data_words[index]);
    }
    uint16_t header = message->data_words[0];
    TEST_ASSERT_EQUAL_INT(message->word_count - 1, payload_words(header));
    for (uint8_t index = 1; index < message->word_count; index++) {
        TEST_ASSERT_TRUE(payload_valid(header, index - 1, message->data_words[index]));
    }
    return build_report(header, message->data_words + 1, report);
}

static void assert_report(const uint8_t *expected, size_t expected_length, const sc2_link_message_t *message) {
    uint8_t report[REPORT_LENGTH_MAX];
    size_t length = report_of(message, report);
    TEST_ASSERT_EQUAL_size_t(expected_length, length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, report, expected_length);
}

#define ASSERT_REPORT(message, ...)                                                                                    \
    do {                                                                                                               \
        static const uint8_t expected[] = {__VA_ARGS__};                                                               \
        sc2_link_message_t actual = (message);                                                                         \
        assert_report(expected, sizeof(expected), &actual);                                                            \
    } while (0)

static void assert_words(const uint16_t *expected, uint8_t expected_count, const sc2_link_message_t *message) {
    TEST_ASSERT_EQUAL_UINT8(expected_count, message->word_count);
    TEST_ASSERT_EQUAL_HEX16_ARRAY(expected, message->data_words, expected_count);
}

#define ASSERT_WORDS(message, ...)                                                                                     \
    do {                                                                                                               \
        static const uint16_t expected[] = {__VA_ARGS__};                                                              \
        sc2_link_message_t actual = (message);                                                                         \
        assert_words(expected, sizeof(expected) / sizeof(expected[0]), &actual);                                       \
    } while (0)

/* What libvirtualhid makes of report 0x82 and what Sunshine passes on. */
static LI_CONTROLLER_HAPTIC_EFFECT command_effect(uint8_t target, uint8_t kind, int8_t gain_db) {
    LI_CONTROLLER_HAPTIC_EFFECT effect;
    memset(&effect, 0, sizeof(effect));
    effect.target = target;
    effect.kind = kind;
    effect.gainDb = gain_db;
    return effect;
}

static sc2_link_message_t effect_message(const LI_CONTROLLER_HAPTIC_EFFECT *effect) {
    sc2_link_message_t message;
    memset(&message, 0, sizeof(message));
    TEST_ASSERT_TRUE(sc2_link_haptic_effect_message(effect, &message));
    return message;
}

void setUp(void) {
}

void tearDown(void) {
}

/* docs/PROTOCOL.md: a click on both pads at -10 dB is COMMAND data 0x4BF6, report 82 03 02 F6. */
void test_protocol_example(void) {
    ASSERT_WORDS(sc2_link_command_message(3, 2, -10), 0x4BF6);
    ASSERT_REPORT(sc2_link_command_message(3, 2, -10), 0x82, 0x03, 0x02, 0xF6);
}

/* The vectors of test_sc2link.py's Encoder.test_layouts. */
void test_layouts(void) {
    ASSERT_WORDS(sc2_link_script_message(1, 0xFF, -1), 0x09FF, 0x00FF);
    ASSERT_WORDS(sc2_link_rumble_message(0xFFFF, 0x1234), 0x1002, 0x7FFF, 0x1234);
    ASSERT_WORDS(sc2_link_pulse_message(3, 0xFFFF, 1, 0x8000), 0x1B05, 0x7FFF, 0x0001, 0x0000);
    ASSERT_WORDS(sc2_link_sweep_message(1, -128, 0xFFFF, 2, 0x8003), 0x2180, 0x7FFF, 0x0002, 0x0003, 0x0005);
    ASSERT_WORDS(sc2_link_tone_message(2, 127, 0xFFFF, 4, 0x8005, 0xFF), 0x2A7F, 0x7FFF, 0x0004, 0x0005, 0x05FF);
}

void test_control_words(void) {
    ASSERT_WORDS(sc2_link_rumble_message(0, 0), 0x1004);
    ASSERT_WORDS(sc2_link_nop_message(), 0x0000);
    ASSERT_WORDS(sc2_link_bye_message(), 0x0200);
    TEST_ASSERT_EQUAL_INT(0, payload_words(sc2_link_nop_message().data_words[0]));
    TEST_ASSERT_EQUAL_INT(0, payload_words(sc2_link_bye_message().data_words[0]));
}

/* Each HELLO word is an undefined CONTROL header with toggle 0. */
void test_hello_words(void) {
    static const uint16_t expected[] = {0x0102, 0x0201, 0x0103, 0x0301};
    TEST_ASSERT_EQUAL_HEX16_ARRAY(expected, sc2_link_hello_words, SC2_LINK_HELLO_WORD_COUNT);
    for (int index = 0; index < SC2_LINK_HELLO_WORD_COUNT; index++) {
        TEST_ASSERT_EQUAL_INT(-1, payload_words(sc2_link_hello_words[index]));
        TEST_ASSERT_EQUAL_UINT8(0, sc2_link_word_toggle(sc2_link_hello_words[index]));
    }
}

/* test_sc2link.py's Encoder.test_stamp. */
void test_stamp(void) {
    sc2_link_message_t message = {.data_words = {0x0400, 0x0400, 0x0200}, .word_count = 3};
    uint16_t words[SC2_LINK_MESSAGE_WORDS_MAX];
    TEST_ASSERT_EQUAL_UINT8(1, sc2_link_stamp_message(&message, 0, words));
    static const uint16_t expected[] = {0x8400, 0x0400, 0x8200};
    TEST_ASSERT_EQUAL_HEX16_ARRAY(expected, words, 3);
    TEST_ASSERT_EQUAL_UINT8(0, sc2_link_stamp_message(&message, 1, words));
    static const uint16_t after_toggle_one[] = {0x0400, 0x8400, 0x0200};
    TEST_ASSERT_EQUAL_HEX16_ARRAY(after_toggle_one, words, 3);
}

/* BYE with toggle 1 is 0x8200, which as plain rumble drives the left motor at 0x82. */
void test_session_word(void) {
    TEST_ASSERT_EQUAL_HEX16(0x8200, sc2_link_session_word(0x0200, 1));
    TEST_ASSERT_EQUAL_HEX16(0x0200, sc2_link_session_word(0x0200, 0));
    TEST_ASSERT_EQUAL_HEX16(0x7FFF, sc2_link_session_word(0xFFFF, 0));
    TEST_ASSERT_EQUAL_UINT8(1, sc2_link_word_toggle(0x8200));
    TEST_ASSERT_EQUAL_UINT8(0, sc2_link_word_toggle(0x7FFF));
}

/* test_sc2link.py's Encoder.test_reports_at_full_range, through the dongle's parser. */
void test_reports_at_full_range(void) {
    ASSERT_REPORT(sc2_link_rumble_message(0xFFFF, 0x1234), 0x80, 0x04, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x34, 0x12, 0x00);
    ASSERT_REPORT(sc2_link_rumble_message(0, 0), 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    ASSERT_REPORT(sc2_link_pulse_message(3, 0xFFFF, 1, 0x8000), 0x81, 0x03, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x80);
    ASSERT_REPORT(sc2_link_sweep_message(1, -128, 0xFFFF, 2, 3), 0x84, 0x01, 0x80, 0xFF, 0xFF, 0x02, 0x00, 0x03,
                  0x00);
    ASSERT_REPORT(sc2_link_tone_message(2, 127, 0xFFFF, 4, 5, 0xFF), 0x83, 0x02, 0x7F, 0xFF, 0xFF, 0x04, 0x00, 0x05,
                  0x00, 0xFF);
    ASSERT_REPORT(sc2_link_script_message(1, 0xFF, -1), 0x85, 0x01, 0xFF, 0xFF);
}

/*
 * test_sc2link.py's Decoder.test_round_trip_every_boundary: every field at its
 * minimum, middle and maximum, one at a time, the others at their minimum.
 */
typedef struct field_t {
    int minimum;
    int maximum;
} field_t;

typedef enum message_type_t {
    MESSAGE_COMMAND,
    MESSAGE_SCRIPT,
    MESSAGE_RUMBLE,
    MESSAGE_PULSE,
    MESSAGE_SWEEP,
    MESSAGE_TONE,
    MESSAGE_TYPE_COUNT,
} message_type_t;

#define FIELDS_MAX 6
#define SIDE {0, 3}
#define GAIN {-128, 127}
#define WORD {0, 0xFFFF}

static const struct {
    int field_count;
    field_t fields[FIELDS_MAX];
    uint8_t word_count;
} message_types[MESSAGE_TYPE_COUNT] = {
        [MESSAGE_COMMAND] = {3, {SIDE, {0, 7}, GAIN}, 1},
        [MESSAGE_SCRIPT] = {3, {SIDE, {0, 0xFF}, GAIN}, 2},
        [MESSAGE_RUMBLE] = {2, {WORD, WORD}, 3},
        [MESSAGE_PULSE] = {4, {SIDE, WORD, WORD, WORD}, 4},
        [MESSAGE_SWEEP] = {5, {SIDE, GAIN, WORD, WORD, WORD}, 5},
        [MESSAGE_TONE] = {6, {SIDE, GAIN, WORD, WORD, WORD, {0, 0xFF}}, 5},
};

static sc2_link_message_t encode_case(message_type_t type, const int *values) {
    switch (type) {
        case MESSAGE_COMMAND:
            return sc2_link_command_message(values[0], values[1], values[2]);
        case MESSAGE_SCRIPT:
            return sc2_link_script_message(values[0], values[1], values[2]);
        case MESSAGE_RUMBLE:
            return sc2_link_rumble_message(values[0], values[1]);
        case MESSAGE_PULSE:
            return sc2_link_pulse_message(values[0], values[1], values[2], values[3]);
        case MESSAGE_SWEEP:
            return sc2_link_sweep_message(values[0], values[1], values[2], values[3], values[4]);
        default:
            return sc2_link_tone_message(values[0], values[1], values[2], values[3], values[4], values[5]);
    }
}

/* test_sc2link.py's report_* functions. */
static size_t expected_report(message_type_t type, const int *values, uint8_t report[REPORT_LENGTH_MAX]) {
    switch (type) {
        case MESSAGE_COMMAND:
        case MESSAGE_SCRIPT:
            report[0] = type == MESSAGE_COMMAND ? 0x82 : 0x85;
            report[1] = values[0];
            report[2] = values[1];
            report[3] = (uint8_t) values[2];
            return 4;
        case MESSAGE_RUMBLE:
            report[0] = 0x80;
            report[1] = values[0] || values[1] ? HAPTIC_TYPE_RUMBLE : 0;
            put16(report + 2, values[0] > values[1] ? values[0] : values[1]);
            put16(report + 4, values[0]);
            report[6] = 0;
            put16(report + 7, values[1]);
            report[9] = 0;
            return 10;
        case MESSAGE_PULSE:
            report[0] = 0x81;
            report[1] = values[0];
            put16(report + 2, values[1]);
            put16(report + 4, values[2]);
            put16(report + 6, values[3]);
            return 8;
        case MESSAGE_SWEEP:
            report[0] = 0x84;
            report[1] = values[0];
            report[2] = (uint8_t) values[1];
            put16(report + 3, values[2]);
            put16(report + 5, values[3]);
            put16(report + 7, values[4]);
            return 9;
        default:
            report[0] = 0x83;
            report[1] = values[0];
            report[2] = (uint8_t) values[1];
            put16(report + 3, values[2]);
            put16(report + 5, values[3]);
            put16(report + 7, values[4]);
            report[9] = values[5];
            return 10;
    }
}

static void assert_round_trip(message_type_t type, const int *values) {
    sc2_link_message_t message = encode_case(type, values);
    bool rumble_stop = type == MESSAGE_RUMBLE && values[0] == 0 && values[1] == 0;
    TEST_ASSERT_EQUAL_UINT8(rumble_stop ? 1 : message_types[type].word_count, message.word_count);
    TEST_ASSERT_EQUAL(type == MESSAGE_RUMBLE, message.rumble);
    uint8_t expected[REPORT_LENGTH_MAX];
    size_t expected_length = expected_report(type, values, expected);
    assert_report(expected, expected_length, &message);
}

void test_round_trip_every_boundary(void) {
    for (int type = 0; type < MESSAGE_TYPE_COUNT; type++) {
        int field_count = message_types[type].field_count;
        int values[FIELDS_MAX];
        for (int index = 0; index < field_count; index++) {
            values[index] = message_types[type].fields[index].minimum;
        }
        assert_round_trip(type, values);
        for (int index = 0; index < field_count; index++) {
            field_t field = message_types[type].fields[index];
            int boundaries[] = {(field.minimum + field.maximum + 1) / 2, field.maximum};
            for (int boundary = 0; boundary < 2; boundary++) {
                int case_values[FIELDS_MAX];
                memcpy(case_values, values, sizeof(case_values));
                case_values[index] = boundaries[boundary];
                assert_round_trip(type, case_values);
            }
        }
    }
}

/* Out-of-range sides and commands must not reach bits the dongle rejects. */
void test_narrow_fields_stay_in_their_bits(void) {
    ASSERT_REPORT(sc2_link_command_message(7, 0xF2, 0), 0x82, 0x03, 0x02, 0x00);
    ASSERT_REPORT(sc2_link_pulse_message(0xFF, 1, 2, 3), 0x81, 0x03, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00);
}

/*
 * Each host effect must rebuild the report libvirtualhid decoded it from.
 * The effects are what decode_steam_controller_haptic_effect() makes of each
 * report.
 */
void test_command_kinds_rebuild_report_0x82(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(LI_HAPTIC_TARGET_BOTH, LI_HAPTIC_EFFECT_CLICK, -10);
    ASSERT_REPORT(effect_message(&effect), 0x82, 0x03, 0x02, 0xF6);
    effect = command_effect(LI_HAPTIC_TARGET_BOTH, LI_HAPTIC_EFFECT_OFF, 0);
    ASSERT_REPORT(effect_message(&effect), 0x82, 0x03, 0x00, 0x00);
    effect = command_effect(LI_HAPTIC_TARGET_LEFT, LI_HAPTIC_EFFECT_TICK, -128);
    ASSERT_REPORT(effect_message(&effect), 0x82, 0x01, 0x01, 0x80);
    effect = command_effect(LI_HAPTIC_TARGET_RIGHT, LI_HAPTIC_EFFECT_RUMBLE, 127);
    ASSERT_REPORT(effect_message(&effect), 0x82, 0x02, 0x04, 0x7F);
    effect = command_effect(LI_HAPTIC_TARGET_NONE, LI_HAPTIC_EFFECT_NOISE, -1);
    ASSERT_REPORT(effect_message(&effect), 0x82, 0x00, 0x05, 0xFF);
}

/* 81 03 FF FF 01 00 00 80: on, off and repeat are the report's 16-bit fields. */
void test_pulse_rebuilds_report_0x81(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(LI_HAPTIC_TARGET_BOTH, LI_HAPTIC_EFFECT_PULSE, 0);
    effect.durationUs = 0xFFFF;
    effect.intervalUs = 1;
    effect.repeatCount = 0x8000;
    ASSERT_REPORT(effect_message(&effect), 0x81, 0x03, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x80);
}

/* 83 02 7F FF FF 04 00 05 00 FF: the host holds the duration in microseconds. */
void test_tone_rebuilds_report_0x83(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(LI_HAPTIC_TARGET_RIGHT, LI_HAPTIC_EFFECT_TONE, 127);
    effect.frequencyHz = 0xFFFF;
    effect.durationUs = 4 * 1000;
    effect.lfoFrequencyHz = 5;
    effect.lfoDepthPercent = 0xFF;
    ASSERT_REPORT(effect_message(&effect), 0x83, 0x02, 0x7F, 0xFF, 0xFF, 0x04, 0x00, 0x05, 0x00, 0xFF);
}

/* 84 01 80 FF FF 02 00 03 00. */
void test_sweep_rebuilds_report_0x84(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(LI_HAPTIC_TARGET_LEFT, LI_HAPTIC_EFFECT_LOGARITHMIC_SWEEP,
                                                        -128);
    effect.durationUs = 0xFFFF * 1000;
    effect.startFrequencyHz = 2;
    effect.endFrequencyHz = 3;
    ASSERT_REPORT(effect_message(&effect), 0x84, 0x01, 0x80, 0xFF, 0xFF, 0x02, 0x00, 0x03, 0x00);
}

/* 85 01 FF FF. */
void test_script_rebuilds_report_0x85(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(LI_HAPTIC_TARGET_LEFT, LI_HAPTIC_EFFECT_SCRIPT, -1);
    effect.scriptId = 0xFF;
    ASSERT_REPORT(effect_message(&effect), 0x85, 0x01, 0xFF, 0xFF);
}

/* Sunshine's rumble packet carries the left and right speeds of report 0x80. */
void test_host_rumble_rebuilds_report_0x80(void) {
    ASSERT_REPORT(sc2_link_rumble_message(0x1234, 0xFFFF), 0x80, 0x04, 0xFF, 0xFF, 0x34, 0x12, 0x00, 0xFF, 0xFF, 0x00);
    ASSERT_REPORT(sc2_link_rumble_message(0, 1), 0x80, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00);
}

/* Durations beyond the report's fields saturate rather than wrap. */
void test_effect_durations_saturate(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(LI_HAPTIC_TARGET_BOTH, LI_HAPTIC_EFFECT_PULSE, 0);
    effect.durationUs = 70000;
    effect.intervalUs = 0xFFFFFFFFu;
    effect.repeatCount = 2;
    ASSERT_REPORT(effect_message(&effect), 0x81, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0x00);
    effect.durationUs = -5;
    effect.intervalUs = 0;
    ASSERT_REPORT(effect_message(&effect), 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00);

    effect = command_effect(LI_HAPTIC_TARGET_BOTH, LI_HAPTIC_EFFECT_TONE, 0);
    effect.durationUs = 70000 * 1000;
    ASSERT_REPORT(effect_message(&effect), 0x83, 0x03, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00);
    effect.durationUs = 1999;
    ASSERT_REPORT(effect_message(&effect), 0x83, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00);
}

void test_unknown_effect_kind_has_no_message(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(LI_HAPTIC_TARGET_BOTH, LI_HAPTIC_EFFECT_PULSE + 1, 0);
    sc2_link_message_t message;
    TEST_ASSERT_FALSE(sc2_link_haptic_effect_message(&effect, &message));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_protocol_example);
    RUN_TEST(test_layouts);
    RUN_TEST(test_control_words);
    RUN_TEST(test_hello_words);
    RUN_TEST(test_stamp);
    RUN_TEST(test_session_word);
    RUN_TEST(test_reports_at_full_range);
    RUN_TEST(test_round_trip_every_boundary);
    RUN_TEST(test_narrow_fields_stay_in_their_bits);
    RUN_TEST(test_command_kinds_rebuild_report_0x82);
    RUN_TEST(test_pulse_rebuilds_report_0x81);
    RUN_TEST(test_tone_rebuilds_report_0x83);
    RUN_TEST(test_sweep_rebuilds_report_0x84);
    RUN_TEST(test_script_rebuilds_report_0x85);
    RUN_TEST(test_host_rumble_rebuilds_report_0x80);
    RUN_TEST(test_effect_durations_saturate);
    RUN_TEST(test_unknown_effect_kind_has_no_message);
    return UNITY_END();
}
