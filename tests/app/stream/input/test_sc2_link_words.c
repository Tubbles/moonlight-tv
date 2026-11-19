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
#define PROTOCOL_VERSION 2
// Sides 6 and 7 name no actuator.
#define SIDE_MAX 5

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
            {OPCODE_COMMAND, 0x0000, 0},
            {OPCODE_SCRIPT, 0x0000, 1},
            {OPCODE_RUMBLE, 0x07F8, 2},
            {OPCODE_PULSE, 0x00F8, 3},
            {OPCODE_SWEEP, 0x0000, 4},
            {OPCODE_TONE, 0x0000, 4},
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
        if (opcode != OPCODE_RUMBLE && (header >> 8 & 7) > SIDE_MAX) {
            return -1;
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
    uint8_t side = header >> 8 & 7;
    uint8_t gain = header & 0xFF;
    switch (opcode_of(header)) {
        case OPCODE_COMMAND:
            report[0] = 0x82;
            report[1] = side;
            report[2] = header >> 11 & 7;
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

/* What the dongle does with each word, its timeouts and knock aside: the Model of sc2link_test.py. */
typedef struct dongle_model_t {
    // The protocol version the acknowledgements show.
    uint8_t version;
    uint8_t counter;
    bool active;
    int hello;
    uint16_t header;
    uint16_t payload[4];
    int have;
    int need;
    uint8_t toggle;
    // The word that ended the last session, until another word arrives.
    bool ended;
    uint16_t end_word;
    uint16_t echo;
    uint8_t queued[REPORT_LENGTH_MAX];
    size_t queued_length;
} dongle_model_t;

typedef struct model_outcome_t {
    bool accepted;
    bool copy;
} model_outcome_t;

static uint16_t model_state(const dongle_model_t *model) {
    int expected = model->active ? model->need : (model->hello ? SC2_LINK_HELLO_WORD_COUNT - model->hello : 0);
    return (uint16_t) (model->counter << 8 | (model->active ? 0x80 : 0) | (expected & 7) << 4 | model->version);
}

static void model_end_session(dongle_model_t *model) {
    model->active = false;
    model->need = 0;
}

static model_outcome_t model_accept(dongle_model_t *model, uint16_t echo) {
    model->counter++;
    model->echo = echo;
    return (model_outcome_t) {.accepted = true};
}

static bool model_hello_step(dongle_model_t *model, uint16_t word) {
    if (word != sc2_link_hello_words[model->hello]) {
        model->hello = 0;
    }
    if (word != sc2_link_hello_words[model->hello]) {
        return false;
    }
    model->hello++;
    return true;
}

static bool model_copy(const dongle_model_t *model, uint16_t word) {
    if (model->active) {
        return word >> 15 == model->toggle;
    }
    if (model->ended && word == model->end_word) {
        return true;
    }
    return model->hello && word == sc2_link_hello_words[model->hello - 1];
}

static model_outcome_t model_reject(dongle_model_t *model, uint16_t word) {
    model_end_session(model);
    model_hello_step(model, word);
    return model_accept(model, word);
}

static void model_run(dongle_model_t *model) {
    if (opcode_of(model->header) != OPCODE_CONTROL) {
        model->queued_length = build_report(model->header, model->payload, model->queued);
    } else if ((model->header >> 8 & 7) == 0x02) {
        model_end_session(model);
    }
}

static model_outcome_t model_session_word(dongle_model_t *model, uint16_t word) {
    uint16_t data = word & 0x7FFF;
    if (model->need) {
        if (!payload_valid(model->header, model->have, data)) {
            return model_reject(model, word);
        }
        model->payload[model->have++] = data;
        model->toggle = word >> 15;
        if (--model->need == 0) {
            model_run(model);
        }
        return model_accept(model, word);
    }
    int need = payload_words(data);
    if (need < 0) {
        return model_reject(model, word);
    }
    model->header = data;
    model->have = 0;
    model->need = need;
    model->toggle = word >> 15;
    if (need == 0) {
        model_run(model);
    }
    return model_accept(model, word);
}

static model_outcome_t model_word(dongle_model_t *model, uint16_t word) {
    if (model_copy(model, word)) {
        return (model_outcome_t) {.copy = true};
    }
    if (model->active) {
        model_outcome_t outcome = model_session_word(model, word);
        if (!model->active) {
            model->ended = true;
            model->end_word = word;
        }
        return outcome;
    }
    model->ended = false;
    if (!model_hello_step(model, word)) {
        return (model_outcome_t) {.accepted = false};
    }
    if (model->hello == SC2_LINK_HELLO_WORD_COUNT) {
        model->active = true;
        model->hello = 0;
        model->need = 0;
        model->toggle = 0;
    }
    return model_accept(model, word);
}

/*
 * The transport over the model: sc2link_test.py's FakeLink. The session times
 * out before the word sent as number timeout_before, the word sent as number
 * refused_at is never taken, as with a relay that stays full, and right after
 * the word sent as number foreign_after the dongle takes foreign_word from
 * another FF client.
 */
#define SENT_MAX 128
#define NONE (-1)

typedef struct fake_link_t {
    dongle_model_t model;
    uint16_t sent[SENT_MAX];
    int sent_count;
    int timeout_before;
    int refused_at;
    int foreign_after;
    uint16_t foreign_word;
    bool stopping;
    int silences;
} fake_link_t;

static sc2_link_word_result_t fake_send_word(void *context, uint16_t word, bool in_session,
                                             sc2_link_acknowledgement_t *acknowledgement) {
    fake_link_t *link = context;
    int index = link->sent_count;
    TEST_ASSERT_LESS_THAN_INT(SENT_MAX - SC2_LINK_MAX_RESENDS, index);
    if (index == link->timeout_before) {
        model_end_session(&link->model);
    }
    acknowledgement->state_before = model_state(&link->model);
    link->sent[link->sent_count++] = word;
    model_outcome_t outcome = {.accepted = false};
    if (index != link->refused_at) {
        outcome = model_word(&link->model, word);
    }
    if (index == link->foreign_after) {
        model_word(&link->model, link->foreign_word);
    }
    acknowledgement->echo = link->model.echo;
    acknowledgement->state = model_state(&link->model);
    if (outcome.accepted) {
        return SC2_LINK_WORD_ACKNOWLEDGED;
    }
    // Each resend is a copy or refused again.
    if (!(in_session && !link->model.active)) {
        for (int resend = 0; resend < SC2_LINK_MAX_RESENDS; resend++) {
            link->sent[link->sent_count++] = word;
        }
    }
    return SC2_LINK_WORD_UNACKNOWLEDGED;
}

static bool fake_read_state(void *context, uint16_t *state) {
    fake_link_t *link = context;
    *state = model_state(&link->model);
    return true;
}

static bool fake_wait_silence(void *context, uint32_t milliseconds) {
    fake_link_t *link = context;
    if (milliseconds >= 500 && link->model.active) {
        model_end_session(&link->model);
    }
    link->model.hello = 0;
    link->silences++;
    return !link->stopping;
}

static bool fake_stopping(void *context) {
    fake_link_t *link = context;
    return link->stopping;
}

static fake_link_t fake_link;
static sc2_link_transport_t fake_transport;
static sc2_link_client_t client;

static void assert_sent(int from, const uint16_t *expected, int count) {
    TEST_ASSERT_GREATER_OR_EQUAL_INT(from + count, fake_link.sent_count);
    TEST_ASSERT_EQUAL_HEX16_ARRAY(expected, fake_link.sent + from, count);
}

static int sent_times(uint16_t word) {
    int times = 0;
    for (int index = 0; index < fake_link.sent_count; index++) {
        times += fake_link.sent[index] == word;
    }
    return times;
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

/*
 * What libvirtualhid makes of report 0x82 and what Sunshine passes on. The
 * target is the report's side byte for 1 to 3, whatever LI_HAPTIC_TARGET_*
 * calls them.
 */
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
    memset(&fake_link, 0, sizeof(fake_link));
    fake_link.model.version = PROTOCOL_VERSION;
    fake_link.timeout_before = NONE;
    fake_link.refused_at = NONE;
    fake_link.foreign_after = NONE;
    fake_transport = (sc2_link_transport_t) {
            .context = &fake_link,
            .send_word = fake_send_word,
            .read_state = fake_read_state,
            .wait_silence = fake_wait_silence,
            .stopping = fake_stopping,
    };
    memset(&client, 0, sizeof(client));
}

void tearDown(void) {
}

/* docs/PROTOCOL.md: a click on both pads at -10 dB is COMMAND data 0x52F6, report 82 02 02 F6. */
void test_protocol_example(void) {
    ASSERT_WORDS(sc2_link_command_message(2, 2, -10), 0x52F6);
    ASSERT_REPORT(sc2_link_command_message(2, 2, -10), 0x82, 0x02, 0x02, 0xF6);
}

/* The vectors of test_sc2link.py's Encoder.test_layouts. */
void test_layouts(void) {
    ASSERT_WORDS(sc2_link_command_message(5, 7, -1), 0x7DFF);
    ASSERT_WORDS(sc2_link_script_message(1, 0xFF, -1), 0x09FF, 0x00FF);
    ASSERT_WORDS(sc2_link_script_message(5, 0, -128), 0x0D80, 0x0000);
    ASSERT_WORDS(sc2_link_rumble_message(0xFFFF, 0x1234), 0x1002, 0x7FFF, 0x1234);
    ASSERT_WORDS(sc2_link_pulse_message(3, 0xFFFF, 1, 0x8000), 0x1B05, 0x7FFF, 0x0001, 0x0000);
    ASSERT_WORDS(sc2_link_pulse_message(5, 0, 0, 0), 0x1D00, 0x0000, 0x0000, 0x0000);
    ASSERT_WORDS(sc2_link_sweep_message(1, -128, 0xFFFF, 2, 0x8003), 0x2180, 0x7FFF, 0x0002, 0x0003, 0x0005);
    ASSERT_WORDS(sc2_link_sweep_message(4, 0, 0, 0, 0), 0x2400, 0x0000, 0x0000, 0x0000, 0x0000);
    ASSERT_WORDS(sc2_link_tone_message(2, 127, 0xFFFF, 4, 0x8005, 0xFF), 0x2A7F, 0x7FFF, 0x0004, 0x0005, 0x05FF);
    ASSERT_WORDS(sc2_link_tone_message(5, 0, 0, 0, 0, 0), 0x2D00, 0x0000, 0x0000, 0x0000, 0x0000);
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
#define SIDE {0, SIDE_MAX}
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

/* test_sc2link.py's Encoder.test_every_side: each side, the other fields at their minimum and at their maximum. */
void test_every_side(void) {
    for (int type = 0; type < MESSAGE_TYPE_COUNT; type++) {
        if (type == MESSAGE_RUMBLE) {
            continue;
        }
        for (int side = 0; side <= SIDE_MAX; side++) {
            for (int at_maximum = 0; at_maximum < 2; at_maximum++) {
                int values[FIELDS_MAX];
                values[0] = side;
                for (int index = 1; index < message_types[type].field_count; index++) {
                    field_t field = message_types[type].fields[index];
                    values[index] = at_maximum ? field.maximum : field.minimum;
                }
                assert_round_trip(type, values);
            }
        }
    }
}

/* An out-of-range command must not reach the side's bits. */
void test_command_stays_in_its_bits(void) {
    ASSERT_REPORT(sc2_link_command_message(5, 0xF2, 0), 0x82, 0x05, 0x02, 0x00);
}

/*
 * Each host effect must rebuild the report libvirtualhid decoded it from.
 * The effects are what decode_steam_controller_haptic_effect() makes of each
 * report.
 */
void test_command_kinds_rebuild_report_0x82(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(3, LI_HAPTIC_EFFECT_CLICK, -10);
    ASSERT_REPORT(effect_message(&effect), 0x82, 0x03, 0x02, 0xF6);
    effect = command_effect(3, LI_HAPTIC_EFFECT_OFF, 0);
    ASSERT_REPORT(effect_message(&effect), 0x82, 0x03, 0x00, 0x00);
    effect = command_effect(1, LI_HAPTIC_EFFECT_TICK, -128);
    ASSERT_REPORT(effect_message(&effect), 0x82, 0x01, 0x01, 0x80);
    effect = command_effect(2, LI_HAPTIC_EFFECT_RUMBLE, 127);
    ASSERT_REPORT(effect_message(&effect), 0x82, 0x02, 0x04, 0x7F);
    effect = command_effect(0, LI_HAPTIC_EFFECT_NOISE, -1);
    ASSERT_REPORT(effect_message(&effect), 0x82, 0x00, 0x05, 0xFF);
}

/* 81 03 FF FF 01 00 00 80: on, off and repeat are the report's 16-bit fields. */
void test_pulse_rebuilds_report_0x81(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(3, LI_HAPTIC_EFFECT_PULSE, 0);
    effect.durationUs = 0xFFFF;
    effect.intervalUs = 1;
    effect.repeatCount = 0x8000;
    ASSERT_REPORT(effect_message(&effect), 0x81, 0x03, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x80);
}

/* 83 02 7F FF FF 04 00 05 00 FF: the host holds the duration in microseconds. */
void test_tone_rebuilds_report_0x83(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(2, LI_HAPTIC_EFFECT_TONE, 127);
    effect.frequencyHz = 0xFFFF;
    effect.durationUs = 4 * 1000;
    effect.lfoFrequencyHz = 5;
    effect.lfoDepthPercent = 0xFF;
    ASSERT_REPORT(effect_message(&effect), 0x83, 0x02, 0x7F, 0xFF, 0xFF, 0x04, 0x00, 0x05, 0x00, 0xFF);
}

/* 84 01 80 FF FF 02 00 03 00. */
void test_sweep_rebuilds_report_0x84(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(1, LI_HAPTIC_EFFECT_LOGARITHMIC_SWEEP, -128);
    effect.durationUs = 0xFFFF * 1000;
    effect.startFrequencyHz = 2;
    effect.endFrequencyHz = 3;
    ASSERT_REPORT(effect_message(&effect), 0x84, 0x01, 0x80, 0xFF, 0xFF, 0x02, 0x00, 0x03, 0x00);
}

/* 85 01 FF FF. */
void test_script_rebuilds_report_0x85(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(1, LI_HAPTIC_EFFECT_SCRIPT, -1);
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
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(3, LI_HAPTIC_EFFECT_PULSE, 0);
    effect.durationUs = 70000;
    effect.intervalUs = 0xFFFFFFFFu;
    effect.repeatCount = 2;
    ASSERT_REPORT(effect_message(&effect), 0x81, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0x00);
    effect.durationUs = -5;
    effect.intervalUs = 0;
    ASSERT_REPORT(effect_message(&effect), 0x81, 0x03, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00);

    effect = command_effect(3, LI_HAPTIC_EFFECT_TONE, 0);
    effect.durationUs = 70000 * 1000;
    ASSERT_REPORT(effect_message(&effect), 0x83, 0x03, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00);
    effect.durationUs = 1999;
    ASSERT_REPORT(effect_message(&effect), 0x83, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00);
}

void test_unknown_effect_kind_has_no_message(void) {
    LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(3, LI_HAPTIC_EFFECT_PULSE + 1, 0);
    sc2_link_message_t message;
    TEST_ASSERT_FALSE(sc2_link_haptic_effect_message(&effect, &message));
}

/*
 * The target is Steam's side byte, so every kind sends it as it came. 0 also
 * stands for the right grip and both grips, which libvirtualhid turns into 0.
 */
void test_targets_go_out_as_the_side(void) {
    for (uint8_t kind = LI_HAPTIC_EFFECT_OFF; kind <= LI_HAPTIC_EFFECT_PULSE; kind++) {
        for (uint8_t target = 0; target <= SC2_LINK_SIDE_MAX; target++) {
            LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(target, kind, 0);
            sc2_link_message_t message = effect_message(&effect);
            uint8_t report[REPORT_LENGTH_MAX];
            report_of(&message, report);
            TEST_ASSERT_EQUAL_UINT8(target, report[1]);
        }
    }
}

/* Sides 6 and 7 end the session, larger ones do not fit the field. */
void test_targets_above_side_max_have_no_message(void) {
    static const uint8_t targets[] = {6, 7, 8, 0xFF};
    for (uint8_t kind = LI_HAPTIC_EFFECT_OFF; kind <= LI_HAPTIC_EFFECT_PULSE; kind++) {
        for (size_t index = 0; index < sizeof(targets); index++) {
            LI_CONTROLLER_HAPTIC_EFFECT effect = command_effect(targets[index], kind, 0);
            sc2_link_message_t message;
            TEST_ASSERT_FALSE(sc2_link_haptic_effect_message(&effect, &message));
        }
    }
}

/* xpad sends the high byte of each magnitude, strong first. */
void test_magnitudes(void) {
    TEST_ASSERT_EQUAL_HEX16(0x1200, sc2_link_strong_magnitude(0x1234));
    TEST_ASSERT_EQUAL_HEX16(0x3400, sc2_link_weak_magnitude(0x1234));
    TEST_ASSERT_EQUAL_HEX16(0xFF00, sc2_link_strong_magnitude(0xFFFF));
    TEST_ASSERT_EQUAL_HEX16(0xFF00, sc2_link_weak_magnitude(0xFFFF));
    TEST_ASSERT_EQUAL_HEX16(0x0000, sc2_link_weak_magnitude(0x0100));
}

/* test_sc2link.py's State.test_ack_state_fields. */
void test_ack_state_fields(void) {
    TEST_ASSERT_EQUAL_UINT8(0xFF, sc2_link_accepted_count(0xFFC2));
    TEST_ASSERT_TRUE(sc2_link_session_active(0xFFC2));
    TEST_ASSERT_EQUAL_UINT8(4, sc2_link_expected_words(0xFFC2));
    TEST_ASSERT_EQUAL_UINT8(2, sc2_link_protocol_version(0xFFC2));
    TEST_ASSERT_FALSE(sc2_link_session_active(0x0231));
    TEST_ASSERT_EQUAL_UINT8(3, sc2_link_expected_words(0x0231));
}

void test_state_advanced(void) {
    TEST_ASSERT_FALSE(sc2_link_state_advanced(0x0581, 0x0581));
    TEST_ASSERT_FALSE(sc2_link_state_advanced(0x0581, 0x0501));
    TEST_ASSERT_TRUE(sc2_link_state_advanced(0x0581, 0x0681));
    TEST_ASSERT_TRUE(sc2_link_state_advanced(0xFF81, 0x0081));
    TEST_ASSERT_TRUE(sc2_link_state_advanced(0x0081, 0x7F81));
    // A count that went back is an older frame.
    TEST_ASSERT_FALSE(sc2_link_state_advanced(0x0581, 0x0481));
    TEST_ASSERT_FALSE(sc2_link_state_advanced(0x0081, 0x8081));
}

void test_word_acknowledged(void) {
    sc2_link_acknowledgement_t acknowledgement = {.state_before = 0xFF81, .echo = 0x8400, .state = 0x0081};
    TEST_ASSERT_TRUE(sc2_link_word_acknowledged(&acknowledgement, 0x8400));
    TEST_ASSERT_FALSE(sc2_link_word_acknowledged(&acknowledgement, 0x0400));
    // Another client's word was taken as well.
    acknowledgement.state = 0x0181;
    TEST_ASSERT_FALSE(sc2_link_word_acknowledged(&acknowledgement, 0x8400));
    acknowledgement.state = 0xFF81;
    TEST_ASSERT_FALSE(sc2_link_word_acknowledged(&acknowledgement, 0x8400));
}

void test_hello_needs_silence(void) {
    TEST_ASSERT_FALSE(sc2_link_hello_needs_silence(0x0001));
    TEST_ASSERT_FALSE(sc2_link_hello_needs_silence(0x0000));
    TEST_ASSERT_TRUE(sc2_link_hello_needs_silence(0x0081));
    TEST_ASSERT_TRUE(sc2_link_hello_needs_silence(0x0021));
}

void test_queue_keeps_order(void) {
    sc2_link_message_queue_t queue = {.count = 0};
    sc2_link_message_t message;
    TEST_ASSERT_FALSE(sc2_link_queue_pop(&queue, &message));
    sc2_link_message_t tick = sc2_link_command_message(1, 1, 0);
    sc2_link_message_t click = sc2_link_command_message(2, 2, 0);
    sc2_link_queue_push(&queue, &tick);
    sc2_link_queue_push(&queue, &click);
    TEST_ASSERT_TRUE(sc2_link_queue_pop(&queue, &message));
    TEST_ASSERT_EQUAL_HEX16(tick.data_words[0], message.data_words[0]);
    TEST_ASSERT_TRUE(sc2_link_queue_pop(&queue, &message));
    TEST_ASSERT_EQUAL_HEX16(click.data_words[0], message.data_words[0]);
    TEST_ASSERT_FALSE(sc2_link_queue_pop(&queue, &message));
}

/* A RUMBLE that has not gone out yet takes the newer level, and effects keep their order. */
void test_queue_replaces_waiting_rumble(void) {
    sc2_link_message_queue_t queue = {.count = 0};
    sc2_link_message_t tick = sc2_link_command_message(1, 1, 0);
    sc2_link_message_t first = sc2_link_rumble_message(0x1000, 0x2000);
    sc2_link_message_t stop = sc2_link_rumble_message(0, 0);
    sc2_link_queue_push(&queue, &first);
    sc2_link_queue_push(&queue, &tick);
    sc2_link_queue_push(&queue, &stop);
    TEST_ASSERT_EQUAL_UINT8(2, queue.count);
    sc2_link_message_t message;
    sc2_link_queue_pop(&queue, &message);
    TEST_ASSERT_EQUAL_UINT8(stop.word_count, message.word_count);
    TEST_ASSERT_EQUAL_HEX16(stop.data_words[0], message.data_words[0]);
    sc2_link_queue_pop(&queue, &message);
    TEST_ASSERT_EQUAL_HEX16(tick.data_words[0], message.data_words[0]);
}

/* A full queue drops the oldest effect and keeps the RUMBLE. */
void test_full_queue_drops_oldest_effect(void) {
    sc2_link_message_queue_t queue = {.count = 0};
    sc2_link_message_t rumble = sc2_link_rumble_message(0, 0);
    sc2_link_queue_push(&queue, &rumble);
    for (int index = 0; index < SC2_LINK_MESSAGE_QUEUE_CAPACITY; index++) {
        sc2_link_message_t script = sc2_link_script_message(3, (uint8_t) index, 0);
        sc2_link_queue_push(&queue, &script);
    }
    TEST_ASSERT_EQUAL_UINT8(SC2_LINK_MESSAGE_QUEUE_CAPACITY, queue.count);
    sc2_link_message_t message;
    sc2_link_queue_pop(&queue, &message);
    TEST_ASSERT_TRUE(message.rumble);
    sc2_link_queue_pop(&queue, &message);
    TEST_ASSERT_EQUAL_HEX16(1, message.data_words[1]);
    for (int index = 2; index < SC2_LINK_MESSAGE_QUEUE_CAPACITY; index++) {
        sc2_link_queue_pop(&queue, &message);
        TEST_ASSERT_EQUAL_HEX16(index, message.data_words[1]);
    }
    TEST_ASSERT_FALSE(sc2_link_queue_pop(&queue, &message));
}

void test_hello_opens_a_session(void) {
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_hello(&client, &fake_transport));
    TEST_ASSERT_TRUE(client.in_session);
    TEST_ASSERT_EQUAL_UINT8(0, client.toggle);
    TEST_ASSERT_TRUE(fake_link.model.active);
    assert_sent(0, sc2_link_hello_words, SC2_LINK_HELLO_WORD_COUNT);
    TEST_ASSERT_EQUAL_INT(0, fake_link.silences);
}

/* Another session, or a HELLO in progress, must time out first. */
void test_hello_waits_for_silence_while_a_session_runs(void) {
    for (int index = 0; index < SC2_LINK_HELLO_WORD_COUNT; index++) {
        model_word(&fake_link.model, sc2_link_hello_words[index]);
    }
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_hello(&client, &fake_transport));
    TEST_ASSERT_EQUAL_INT(1, fake_link.silences);
    assert_sent(0, sc2_link_hello_words, SC2_LINK_HELLO_WORD_COUNT);
}

/* A dongle of another protocol version reads the words differently, so it gets no message. */
void test_hello_refuses_another_protocol_version(void) {
    fake_link.model.version = 1;
    sc2_link_message_t nop = sc2_link_nop_message();
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_FAILED, sc2_link_client_deliver(&client, &fake_transport, &nop));
    TEST_ASSERT_FALSE(client.in_session);
    TEST_ASSERT_EQUAL_INT(SC2_LINK_HELLO_WORD_COUNT, fake_link.sent_count);
}

void test_hello_without_acknowledgement_fails(void) {
    fake_link.refused_at = 0;
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_FAILED, sc2_link_client_hello(&client, &fake_transport));
    TEST_ASSERT_FALSE(client.in_session);
    TEST_ASSERT_EQUAL_INT(1 + SC2_LINK_MAX_RESENDS, fake_link.sent_count);
}

void test_toggles_alternate_across_messages(void) {
    sc2_link_message_t nop = sc2_link_nop_message();
    for (int index = 0; index < 3; index++) {
        TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_deliver(&client, &fake_transport, &nop));
    }
    static const uint16_t expected[] = {0x8000, 0x0000, 0x8000};
    assert_sent(SC2_LINK_HELLO_WORD_COUNT, expected, 3);
    TEST_ASSERT_EQUAL_INT(SC2_LINK_HELLO_WORD_COUNT + 3, fake_link.sent_count);
}

/*
 * test_sc2link.py's Client.test_a_session_lost_mid_message_restarts_it: the
 * session times out before the third pulse word, which the dongle then sees
 * as plain rumble. It is not resent, and the whole message follows a new
 * HELLO.
 */
void test_a_session_lost_mid_message_restarts_it(void) {
    sc2_link_message_t pulse = sc2_link_pulse_message(3, 1000, 1000, 3);
    uint16_t words[SC2_LINK_MESSAGE_WORDS_MAX];
    sc2_link_stamp_message(&pulse, 0, words);
    fake_link.timeout_before = 6;
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_hello(&client, &fake_transport));
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_deliver(&client, &fake_transport, &pulse));
    assert_sent(4, words, 3);
    assert_sent(7, sc2_link_hello_words, SC2_LINK_HELLO_WORD_COUNT);
    assert_sent(11, words, 4);
    TEST_ASSERT_EQUAL_INT(15, fake_link.sent_count);
    static const uint8_t report[] = {0x81, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0x03, 0x00};
    TEST_ASSERT_EQUAL_size_t(sizeof(report), fake_link.model.queued_length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(report, fake_link.model.queued, sizeof(report));
}

/* test_sc2link.py's Client.test_a_session_found_gone_before_a_message. */
void test_a_session_found_gone_before_a_message(void) {
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_hello(&client, &fake_transport));
    fake_wait_silence(&fake_link, SC2_LINK_SILENCE_MS);
    sc2_link_message_t nop = sc2_link_nop_message();
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_deliver(&client, &fake_transport, &nop));
    assert_sent(4, sc2_link_hello_words, SC2_LINK_HELLO_WORD_COUNT);
    static const uint16_t nop_word[] = {0x8000};
    assert_sent(8, nop_word, 1);
    TEST_ASSERT_EQUAL_INT(9, fake_link.sent_count);
}

/*
 * test_sc2link.py's Client.test_a_missing_acknowledgement_stops_the_message:
 * the refused word goes out with its resends and nothing after it. The next
 * message waits for the session to time out before its HELLO.
 */
void test_a_missing_acknowledgement_drops_the_message(void) {
    sc2_link_message_t pulse = sc2_link_pulse_message(3, 1000, 1000, 3);
    fake_link.refused_at = 5;
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_hello(&client, &fake_transport));
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DROPPED, sc2_link_client_deliver(&client, &fake_transport, &pulse));
    TEST_ASSERT_EQUAL_INT(4 + 2 + SC2_LINK_MAX_RESENDS, fake_link.sent_count);
    TEST_ASSERT_FALSE(client.in_session);

    sc2_link_message_t nop = sc2_link_nop_message();
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_deliver(&client, &fake_transport, &nop));
    TEST_ASSERT_EQUAL_INT(1, fake_link.silences);
    assert_sent(16, sc2_link_hello_words, SC2_LINK_HELLO_WORD_COUNT);
}

/* test_sc2link.py's Client.test_a_rejected_message_is_tried_twice, with a reserved opcode. */
void test_a_rejected_message_is_tried_twice(void) {
    sc2_link_message_t reserved = {.data_words = {0x3800}, .word_count = 1};
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DROPPED, sc2_link_client_deliver(&client, &fake_transport, &reserved));
    TEST_ASSERT_EQUAL_INT(2, sent_times(0xB800));
}

/* test_sc2link.py's Client.test_bye_ends_the_session_without_a_restart. */
void test_bye_ends_the_session_without_a_restart(void) {
    sc2_link_message_t bye = sc2_link_bye_message();
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_deliver(&client, &fake_transport, &bye));
    TEST_ASSERT_EQUAL_INT(5, fake_link.sent_count);
    TEST_ASSERT_FALSE(fake_link.model.active);
}

void test_client_bye(void) {
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_hello(&client, &fake_transport));
    sc2_link_client_bye(&client, &fake_transport);
    static const uint16_t bye_word[] = {0x8200};
    assert_sent(4, bye_word, 1);
    TEST_ASSERT_FALSE(fake_link.model.active);
    TEST_ASSERT_FALSE(client.in_session);
    // Nothing to end without a session.
    sc2_link_client_bye(&client, &fake_transport);
    TEST_ASSERT_EQUAL_INT(5, fake_link.sent_count);
}

/* A session the dongle already ended needs no BYE. */
void test_client_bye_after_timeout_sends_nothing(void) {
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_hello(&client, &fake_transport));
    fake_wait_silence(&fake_link, SC2_LINK_SILENCE_MS);
    sc2_link_client_bye(&client, &fake_transport);
    TEST_ASSERT_EQUAL_INT(4, fake_link.sent_count);
}

/*
 * Another FF client's word, taken right after this client's, leaves the
 * toggles out of step: the message starts over after the session timed out.
 */
void test_another_clients_word_starts_over(void) {
    sc2_link_message_t pulse = sc2_link_pulse_message(3, 1000, 1000, 3);
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_hello(&client, &fake_transport));
    fake_link.foreign_after = 5;
    fake_link.foreign_word = 0x8123;
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_deliver(&client, &fake_transport, &pulse));
    TEST_ASSERT_EQUAL_INT(1, fake_link.silences);
    uint16_t words[SC2_LINK_MESSAGE_WORDS_MAX];
    sc2_link_stamp_message(&pulse, 0, words);
    assert_sent(6, sc2_link_hello_words, SC2_LINK_HELLO_WORD_COUNT);
    assert_sent(10, words, 4);
    static const uint8_t report[] = {0x81, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0x03, 0x00};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(report, fake_link.model.queued, sizeof(report));
}

/* Stopping opens no session, also not to resend a message whose session got lost. */
void test_stopping_opens_no_session(void) {
    sc2_link_message_t nop = sc2_link_nop_message();
    fake_link.stopping = true;
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_STOPPED, sc2_link_client_deliver(&client, &fake_transport, &nop));
    TEST_ASSERT_EQUAL_INT(0, fake_link.sent_count);

    fake_link.stopping = false;
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_DELIVERED, sc2_link_client_hello(&client, &fake_transport));
    fake_link.stopping = true;
    fake_link.timeout_before = 4;
    TEST_ASSERT_EQUAL(SC2_LINK_CLIENT_STOPPED, sc2_link_client_deliver(&client, &fake_transport, &nop));
    TEST_ASSERT_EQUAL_INT(5, fake_link.sent_count);
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
    RUN_TEST(test_every_side);
    RUN_TEST(test_command_stays_in_its_bits);
    RUN_TEST(test_command_kinds_rebuild_report_0x82);
    RUN_TEST(test_pulse_rebuilds_report_0x81);
    RUN_TEST(test_tone_rebuilds_report_0x83);
    RUN_TEST(test_sweep_rebuilds_report_0x84);
    RUN_TEST(test_script_rebuilds_report_0x85);
    RUN_TEST(test_host_rumble_rebuilds_report_0x80);
    RUN_TEST(test_effect_durations_saturate);
    RUN_TEST(test_unknown_effect_kind_has_no_message);
    RUN_TEST(test_targets_go_out_as_the_side);
    RUN_TEST(test_targets_above_side_max_have_no_message);
    RUN_TEST(test_magnitudes);
    RUN_TEST(test_ack_state_fields);
    RUN_TEST(test_state_advanced);
    RUN_TEST(test_word_acknowledged);
    RUN_TEST(test_hello_needs_silence);
    RUN_TEST(test_queue_keeps_order);
    RUN_TEST(test_queue_replaces_waiting_rumble);
    RUN_TEST(test_full_queue_drops_oldest_effect);
    RUN_TEST(test_hello_opens_a_session);
    RUN_TEST(test_hello_waits_for_silence_while_a_session_runs);
    RUN_TEST(test_hello_refuses_another_protocol_version);
    RUN_TEST(test_hello_without_acknowledgement_fails);
    RUN_TEST(test_toggles_alternate_across_messages);
    RUN_TEST(test_a_session_lost_mid_message_restarts_it);
    RUN_TEST(test_a_session_found_gone_before_a_message);
    RUN_TEST(test_a_missing_acknowledgement_drops_the_message);
    RUN_TEST(test_a_rejected_message_is_tried_twice);
    RUN_TEST(test_bye_ends_the_session_without_a_restart);
    RUN_TEST(test_client_bye);
    RUN_TEST(test_client_bye_after_timeout_sends_nothing);
    RUN_TEST(test_another_clients_word_starts_over);
    RUN_TEST(test_stopping_opens_no_session);
    return UNITY_END();
}
