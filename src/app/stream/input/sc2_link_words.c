#include "sc2_link_words.h"

#include <string.h>

// In a session bit 15 of a word is a toggle that flips with each new word, so
// the dongle tells a copy of the last word from the next one.
#define TOGGLE_SHIFT 15
#define DATA_MASK 0x7FFFu

// A header with data bit 14 set is a COMMAND, whose 14 bits fill the word and
// leave no room for an opcode. Otherwise bits 13 to 11 hold the opcode.
#define COMMAND_BIT 0x4000u
#define COMMAND_SHIFT 11
#define COMMAND_MASK 0x7u
#define OPCODE_SHIFT 11
#define OPCODE_CONTROL 0u
#define OPCODE_SCRIPT 1u
#define OPCODE_RUMBLE 2u
#define OPCODE_PULSE 3u
#define OPCODE_SWEEP 4u
#define OPCODE_TONE 5u

#define CONTROL_SUB_SHIFT 8
#define CONTROL_NOP 0x00u
#define CONTROL_BYE 0x02u

#define SIDE_SHIFT 8
#define SIDE_MASK 0x7u
// Both speeds zero, no payload.
#define RUMBLE_STOP_BIT 0x0004u

#define MICROSECONDS_PER_MILLISECOND 1000

#define STATE_COUNT_SHIFT 8
#define STATE_SESSION_BIT 0x0080u
#define STATE_EXPECTED_SHIFT 4
#define STATE_EXPECTED_MASK 0x7u
#define STATE_VERSION_MASK 0xFu
// A count further ahead than this is an older one.
#define STATE_ADVANCE_MAX 127u

// A message whose session got lost goes out once more in a new one.
#define DELIVERY_ATTEMPTS 2

typedef enum words_result_t {
    WORDS_DELIVERED,
    // The session ended under the message, or another client's word turned up.
    WORDS_LOST,
    // A word stayed without acknowledgement in a session that is still open.
    WORDS_UNACKNOWLEDGED,
    WORDS_FAILED,
} words_result_t;

/*
 * Each is an undefined CONTROL header, so no message starts with one. Their
 * toggle is 0, so the first word of a session carries 1.
 */
const uint16_t sc2_link_hello_words[SC2_LINK_HELLO_WORD_COUNT] = {0x0102, 0x0201, 0x0103, 0x0301};

static sc2_link_message_t message_of(const uint16_t *data_words, uint8_t word_count, bool rumble);

static uint16_t opcode_header(unsigned int opcode, unsigned int bits);

static unsigned int side_and_gain(uint8_t side, int8_t gain_db);

static unsigned int high_bit(uint16_t value, unsigned int at);

static uint16_t low_bits(uint16_t value);

static uint16_t clamp_to_word(int64_t value);

static uint8_t count_advance(uint16_t state_before, uint16_t state);

static void queue_remove(sc2_link_message_queue_t *queue, uint8_t index);

static words_result_t send_words(sc2_link_client_t *client, const sc2_link_transport_t *transport,
                                 const sc2_link_message_t *message);

static bool is_bye(const sc2_link_message_t *message);

sc2_link_message_t sc2_link_command_message(uint8_t side, uint8_t command, int8_t gain_db) {
    uint16_t data_words[] = {
            (uint16_t) (COMMAND_BIT | (command & COMMAND_MASK) << COMMAND_SHIFT | side_and_gain(side, gain_db)),
    };
    return message_of(data_words, 1, false);
}

sc2_link_message_t sc2_link_script_message(uint8_t side, uint8_t script_id, int8_t gain_db) {
    uint16_t data_words[] = {opcode_header(OPCODE_SCRIPT, side_and_gain(side, gain_db)), script_id};
    return message_of(data_words, 2, false);
}

sc2_link_message_t sc2_link_rumble_message(uint16_t left_speed, uint16_t right_speed) {
    if (left_speed == 0 && right_speed == 0) {
        uint16_t data_words[] = {opcode_header(OPCODE_RUMBLE, RUMBLE_STOP_BIT)};
        return message_of(data_words, 1, true);
    }
    uint16_t data_words[] = {
            opcode_header(OPCODE_RUMBLE, high_bit(left_speed, 1) | high_bit(right_speed, 0)),
            low_bits(left_speed),
            low_bits(right_speed),
    };
    return message_of(data_words, 3, true);
}

sc2_link_message_t sc2_link_pulse_message(uint8_t side, uint16_t on_us, uint16_t off_us, uint16_t repeat_count) {
    unsigned int high_bits = high_bit(on_us, 2) | high_bit(off_us, 1) | high_bit(repeat_count, 0);
    uint16_t data_words[] = {
            opcode_header(OPCODE_PULSE, (side & SIDE_MASK) << SIDE_SHIFT | high_bits),
            low_bits(on_us),
            low_bits(off_us),
            low_bits(repeat_count),
    };
    return message_of(data_words, 4, false);
}

/* Its 59 bits do not fit a header and three payload words, so the high bits get a fifth. */
sc2_link_message_t sc2_link_sweep_message(uint8_t side, int8_t gain_db, uint16_t duration_ms, uint16_t start_hz,
                                          uint16_t end_hz) {
    unsigned int high_bits = high_bit(duration_ms, 2) | high_bit(start_hz, 1) | high_bit(end_hz, 0);
    uint16_t data_words[] = {
            opcode_header(OPCODE_SWEEP, side_and_gain(side, gain_db)),
            low_bits(duration_ms),
            low_bits(start_hz),
            low_bits(end_hz),
            (uint16_t) high_bits,
    };
    return message_of(data_words, 5, false);
}

sc2_link_message_t sc2_link_tone_message(uint8_t side, int8_t gain_db, uint16_t frequency_hz, uint16_t duration_ms,
                                         uint16_t lfo_hz, uint8_t lfo_depth) {
    unsigned int high_bits = high_bit(frequency_hz, 10) | high_bit(duration_ms, 9) | high_bit(lfo_hz, 8);
    uint16_t data_words[] = {
            opcode_header(OPCODE_TONE, side_and_gain(side, gain_db)),
            low_bits(frequency_hz),
            low_bits(duration_ms),
            low_bits(lfo_hz),
            (uint16_t) (high_bits | lfo_depth),
    };
    return message_of(data_words, 5, false);
}

sc2_link_message_t sc2_link_nop_message(void) {
    uint16_t data_words[] = {opcode_header(OPCODE_CONTROL, CONTROL_NOP << CONTROL_SUB_SHIFT)};
    return message_of(data_words, 1, false);
}

sc2_link_message_t sc2_link_bye_message(void) {
    uint16_t data_words[] = {opcode_header(OPCODE_CONTROL, CONTROL_BYE << CONTROL_SUB_SHIFT)};
    return message_of(data_words, 1, false);
}

/*
 * The inverse of libvirtualhid's decode_steam_controller_haptic_effect()
 * (src/core/report.cpp, e9ede639), whose fields Sunshine passes on unchanged.
 * Its target is the report's side byte for 1 to 3, under the names left, right
 * and both, and 0 for every other side byte. So the target goes out as the
 * side unchanged, which is the side Steam sent except for the right grip (4)
 * and both grips (5) of 0x82 to 0x85: those arrive as 0 and play on side 0. A
 * target above SC2_LINK_SIDE_MAX names no actuator.
 * Report 0x82 becomes the kind named by its command, so OFF, TICK, CLICK,
 * RUMBLE and NOISE are commands. Its commands 3, 6 and 7 decode to the kinds
 * of reports 0x83, 0x85 and 0x84 with every other field zero, which this
 * cannot tell apart, so those kinds become their own reports. The durations of
 * 0x83 and 0x84 are whole milliseconds on the host, those of 0x81 16-bit
 * microseconds.
 */
bool sc2_link_haptic_effect_message(const LI_CONTROLLER_HAPTIC_EFFECT *effect, sc2_link_message_t *message) {
    uint8_t side = effect->target;
    if (side > SC2_LINK_SIDE_MAX) {
        return false;
    }
    switch (effect->kind) {
        case LI_HAPTIC_EFFECT_OFF:
        case LI_HAPTIC_EFFECT_TICK:
        case LI_HAPTIC_EFFECT_CLICK:
        case LI_HAPTIC_EFFECT_RUMBLE:
        case LI_HAPTIC_EFFECT_NOISE:
            *message = sc2_link_command_message(side, effect->kind, effect->gainDb);
            return true;
        case LI_HAPTIC_EFFECT_TONE:
            *message = sc2_link_tone_message(side, effect->gainDb, effect->frequencyHz,
                                             clamp_to_word(effect->durationUs / MICROSECONDS_PER_MILLISECOND),
                                             effect->lfoFrequencyHz, effect->lfoDepthPercent);
            return true;
        case LI_HAPTIC_EFFECT_SCRIPT:
            *message = sc2_link_script_message(side, effect->scriptId, effect->gainDb);
            return true;
        case LI_HAPTIC_EFFECT_LOGARITHMIC_SWEEP:
            *message = sc2_link_sweep_message(side, effect->gainDb,
                                              clamp_to_word(effect->durationUs / MICROSECONDS_PER_MILLISECOND),
                                              effect->startFrequencyHz, effect->endFrequencyHz);
            return true;
        case LI_HAPTIC_EFFECT_PULSE:
            *message = sc2_link_pulse_message(side, clamp_to_word(effect->durationUs),
                                              clamp_to_word(effect->intervalUs), effect->repeatCount);
            return true;
        default:
            return false;
    }
}

uint16_t sc2_link_session_word(uint16_t data_word, uint8_t toggle) {
    return (uint16_t) ((toggle & 1u) << TOGGLE_SHIFT | (data_word & DATA_MASK));
}

uint8_t sc2_link_word_toggle(uint16_t word) {
    return (uint8_t) (word >> TOGGLE_SHIFT);
}

uint8_t sc2_link_stamp_message(const sc2_link_message_t *message, uint8_t toggle,
                               uint16_t words[SC2_LINK_MESSAGE_WORDS_MAX]) {
    for (uint8_t index = 0; index < message->word_count; index++) {
        toggle ^= 1u;
        words[index] = sc2_link_session_word(message->data_words[index], toggle);
    }
    return toggle;
}

/* xpad sends the high bytes of the magnitudes that ff-memless scaled by the gain. */
uint16_t sc2_link_strong_magnitude(uint16_t word) {
    return (uint16_t) (word & 0xFF00u);
}

uint16_t sc2_link_weak_magnitude(uint16_t word) {
    return (uint16_t) ((word & 0x00FFu) << 8);
}

uint8_t sc2_link_accepted_count(uint16_t state) {
    return (uint8_t) (state >> STATE_COUNT_SHIFT);
}

bool sc2_link_session_active(uint16_t state) {
    return (state & STATE_SESSION_BIT) != 0;
}

uint8_t sc2_link_expected_words(uint16_t state) {
    return (uint8_t) ((state >> STATE_EXPECTED_SHIFT) & STATE_EXPECTED_MASK);
}

uint8_t sc2_link_protocol_version(uint16_t state) {
    return (uint8_t) (state & STATE_VERSION_MASK);
}

bool sc2_link_state_advanced(uint16_t state_before, uint16_t state) {
    uint8_t advance = count_advance(state_before, state);
    return advance != 0 && advance <= STATE_ADVANCE_MAX;
}

bool sc2_link_word_acknowledged(const sc2_link_acknowledgement_t *acknowledgement, uint16_t word) {
    return count_advance(acknowledgement->state_before, acknowledgement->state) == 1 && acknowledgement->echo == word;
}

bool sc2_link_hello_needs_silence(uint16_t state) {
    return sc2_link_session_active(state) || sc2_link_expected_words(state) != 0;
}

/* With one RUMBLE at most in the queue, the oldest other message is the first or the second. */
void sc2_link_queue_push(sc2_link_message_queue_t *queue, const sc2_link_message_t *message) {
    if (message->rumble) {
        for (uint8_t index = 0; index < queue->count; index++) {
            if (queue->messages[index].rumble) {
                queue->messages[index] = *message;
                return;
            }
        }
    }
    if (queue->count == SC2_LINK_MESSAGE_QUEUE_CAPACITY) {
        queue_remove(queue, queue->messages[0].rumble ? 1 : 0);
    }
    queue->messages[queue->count++] = *message;
}

bool sc2_link_queue_pop(sc2_link_message_queue_t *queue, sc2_link_message_t *message) {
    if (queue->count == 0) {
        return false;
    }
    *message = queue->messages[0];
    queue_remove(queue, 0);
    return true;
}

/*
 * The client of docs/PROTOCOL.md, section 9.5.1, as sc2link_test.py's Client
 * keeps a session. One difference: a word acknowledged with another echo or
 * count, from another FF client on the node, starts the message over instead
 * of ending it, as the protocol asks.
 */
sc2_link_client_result_t sc2_link_client_hello(sc2_link_client_t *client, const sc2_link_transport_t *transport) {
    client->in_session = false;
    if (transport->stopping(transport->context)) {
        return SC2_LINK_CLIENT_STOPPED;
    }
    uint16_t state;
    if (!transport->read_state(transport->context, &state)) {
        return SC2_LINK_CLIENT_FAILED;
    }
    if (sc2_link_hello_needs_silence(state) && !transport->wait_silence(transport->context, SC2_LINK_SILENCE_MS)) {
        return SC2_LINK_CLIENT_STOPPED;
    }
    sc2_link_acknowledgement_t acknowledgement;
    for (int index = 0; index < SC2_LINK_HELLO_WORD_COUNT; index++) {
        uint16_t word = sc2_link_hello_words[index];
        if (transport->send_word(transport->context, word, false, &acknowledgement) != SC2_LINK_WORD_ACKNOWLEDGED ||
            !sc2_link_word_acknowledged(&acknowledgement, word)) {
            return SC2_LINK_CLIENT_FAILED;
        }
    }
    if (!sc2_link_session_active(acknowledgement.state) ||
        sc2_link_protocol_version(acknowledgement.state) != SC2_LINK_PROTOCOL_VERSION) {
        return SC2_LINK_CLIENT_FAILED;
    }
    client->in_session = true;
    client->toggle = sc2_link_word_toggle(sc2_link_hello_words[SC2_LINK_HELLO_WORD_COUNT - 1]);
    return SC2_LINK_CLIENT_DELIVERED;
}

sc2_link_client_result_t sc2_link_client_deliver(sc2_link_client_t *client, const sc2_link_transport_t *transport,
                                                 const sc2_link_message_t *message) {
    for (int attempt = 0; attempt < DELIVERY_ATTEMPTS; attempt++) {
        uint16_t state;
        if (!transport->read_state(transport->context, &state)) {
            return SC2_LINK_CLIENT_FAILED;
        }
        if (!client->in_session || !sc2_link_session_active(state)) {
            sc2_link_client_result_t result = sc2_link_client_hello(client, transport);
            if (result != SC2_LINK_CLIENT_DELIVERED) {
                return result;
            }
        }
        words_result_t result = send_words(client, transport, message);
        if (result == WORDS_DELIVERED) {
            return SC2_LINK_CLIENT_DELIVERED;
        }
        client->in_session = false;
        if (result == WORDS_FAILED) {
            return SC2_LINK_CLIENT_FAILED;
        }
        if (result == WORDS_UNACKNOWLEDGED) {
            // No further word until the session timed out, which the next HELLO waits for.
            return SC2_LINK_CLIENT_DROPPED;
        }
    }
    return SC2_LINK_CLIENT_DROPPED;
}

void sc2_link_client_bye(sc2_link_client_t *client, const sc2_link_transport_t *transport) {
    uint16_t state;
    if (client->in_session && transport->read_state(transport->context, &state) && sc2_link_session_active(state)) {
        sc2_link_message_t bye = sc2_link_bye_message();
        send_words(client, transport, &bye);
    }
    client->in_session = false;
}

static sc2_link_message_t message_of(const uint16_t *data_words, uint8_t word_count, bool rumble) {
    sc2_link_message_t message;
    memset(&message, 0, sizeof(message));
    memcpy(message.data_words, data_words, word_count * sizeof(data_words[0]));
    message.word_count = word_count;
    message.rumble = rumble;
    return message;
}

static uint16_t opcode_header(unsigned int opcode, unsigned int bits) {
    return (uint16_t) (opcode << OPCODE_SHIFT | bits);
}

/* Gains are s8 dB and travel as their bit pattern. */
static unsigned int side_and_gain(uint8_t side, int8_t gain_db) {
    return (side & SIDE_MASK) << SIDE_SHIFT | (uint8_t) gain_db;
}

/* Bit 15 of a 16-bit field, moved to bit `at` of another word: a word carries 15 data bits. */
static unsigned int high_bit(uint16_t value, unsigned int at) {
    return (unsigned int) (value >> 15) << at;
}

static uint16_t low_bits(uint16_t value) {
    return (uint16_t) (value & DATA_MASK);
}

static uint16_t clamp_to_word(int64_t value) {
    if (value < 0) {
        return 0;
    }
    return value > UINT16_MAX ? UINT16_MAX : (uint16_t) value;
}

static uint8_t count_advance(uint16_t state_before, uint16_t state) {
    return (uint8_t) (sc2_link_accepted_count(state) - sc2_link_accepted_count(state_before));
}

static void queue_remove(sc2_link_message_queue_t *queue, uint8_t index) {
    memmove(&queue->messages[index], &queue->messages[index + 1],
            (size_t) (queue->count - index - 1) * sizeof(queue->messages[0]));
    queue->count--;
}

/*
 * A session that ended shows in an acknowledgement without the session bit,
 * after BYE as expected, after any other word because the dongle rejected it.
 */
static words_result_t send_words(sc2_link_client_t *client, const sc2_link_transport_t *transport,
                                 const sc2_link_message_t *message) {
    uint16_t words[SC2_LINK_MESSAGE_WORDS_MAX];
    sc2_link_stamp_message(message, client->toggle, words);
    for (uint8_t index = 0; index < message->word_count; index++) {
        sc2_link_acknowledgement_t acknowledgement;
        sc2_link_word_result_t result = transport->send_word(transport->context, words[index], true,
                                                             &acknowledgement);
        if (result == SC2_LINK_WORD_FAILED) {
            return WORDS_FAILED;
        }
        if (result == SC2_LINK_WORD_UNACKNOWLEDGED) {
            return sc2_link_session_active(acknowledgement.state) ? WORDS_UNACKNOWLEDGED : WORDS_LOST;
        }
        if (!sc2_link_word_acknowledged(&acknowledgement, words[index])) {
            return WORDS_LOST;
        }
        client->toggle = sc2_link_word_toggle(words[index]);
        if (!sc2_link_session_active(acknowledgement.state) && !is_bye(message)) {
            return WORDS_LOST;
        }
    }
    return WORDS_DELIVERED;
}

/* Compared as a whole message, since a payload word can hold the same bits. */
static bool is_bye(const sc2_link_message_t *message) {
    return message->word_count == 1 && message->data_words[0] == sc2_link_bye_message().data_words[0];
}
