#pragma once

/*
 * The word protocol of an OpenPuck dongle in SC2 Link mode, which carries the
 * Steam Controller's haptic reports 0x80 to 0x85 from a client that only has
 * evdev nodes: each 16-bit word goes out as one xpad rumble packet and is
 * acknowledged on the pads node. Reference: docs/PROTOCOL.md, section 9.5.1,
 * of the OpenPuck sources, and the sender in its tools/sc2link/sc2link_test.py,
 * whose encoders these follow.
 *
 * Everything here is free of I/O so it can be tested on any Linux host.
 */

#include <stdbool.h>
#include <stdint.h>

#include <Limelight.h>

#define SC2_LINK_PROTOCOL_VERSION 2
// The highest side byte: the dongle ends the session on a header with side 6 or 7.
#define SC2_LINK_SIDE_MAX 5
#define SC2_LINK_HELLO_WORD_COUNT 4
// SWEEP and TONE: a header and four payload words.
#define SC2_LINK_MESSAGE_WORDS_MAX 5
#define SC2_LINK_MESSAGE_QUEUE_CAPACITY 32

// Stop-and-wait: with no acknowledgement after this, read ACK_STATE and resend.
#define SC2_LINK_RESEND_AFTER_MS 30
#define SC2_LINK_MAX_RESENDS 10
// A session ends 500 ms after the last word, so an idle client sends NOP this often.
#define SC2_LINK_KEEP_ALIVE_MS 200
// Waited before a HELLO while a session or a HELLO may be running, so that it ends first.
#define SC2_LINK_SILENCE_MS 600

/** A message as data words, bits 14 to 0, before sc2_link_stamp_message() adds the toggles. */
typedef struct sc2_link_message_t {
    uint16_t data_words[SC2_LINK_MESSAGE_WORDS_MAX];
    uint8_t word_count;
    // RUMBLE sets a level, so a newer one makes an older one that has not gone out yet obsolete.
    bool rumble;
} sc2_link_message_t;

/** Messages waiting for the writer, oldest first. */
typedef struct sc2_link_message_queue_t {
    sc2_link_message_t messages[SC2_LINK_MESSAGE_QUEUE_CAPACITY];
    uint8_t count;
} sc2_link_message_queue_t;

/** What the pads node showed in ACK_ECHO and ACK_STATE after a word, and ACK_STATE before it. */
typedef struct sc2_link_acknowledgement_t {
    uint16_t state_before;
    uint16_t echo;
    uint16_t state;
} sc2_link_acknowledgement_t;

typedef enum sc2_link_word_result_t {
    SC2_LINK_WORD_ACKNOWLEDGED,
    // Not even after the resends, or a session word found the session ended.
    SC2_LINK_WORD_UNACKNOWLEDGED,
    // The node could not be written or read.
    SC2_LINK_WORD_FAILED,
} sc2_link_word_result_t;

/**
 * The I/O the client needs, so that its rules can be tested without a dongle.
 * Each function gets the context.
 */
typedef struct sc2_link_transport_t {
    void *context;

    /**
     * Sends one word and waits for its acknowledgement: the counter in
     * ACK_STATE moved on. Resends the word while none comes, up to
     * SC2_LINK_MAX_RESENDS times, but not a session word once the session bit
     * is clear, since outside a session it is rumble. Fills in what the pads
     * node showed last either way.
     */
    sc2_link_word_result_t (*send_word)(void *context, uint16_t word, bool in_session,
                                        sc2_link_acknowledgement_t *acknowledgement);

    /** ACK_STATE as the kernel holds it now. False when the node could not be read. */
    bool (*read_state)(void *context, uint16_t *state);

    /** Sends nothing for this long. False when the client must stop instead. */
    bool (*wait_silence)(void *context, uint32_t milliseconds);

    /** Whether the client must stop instead of opening a session. */
    bool (*stopping)(void *context);
} sc2_link_transport_t;

/** The client's side of a session. */
typedef struct sc2_link_client_t {
    // Whether HELLO opened the session that the dongle shows and no word has failed in it since.
    bool in_session;
    // The toggle of the last session word acknowledged.
    uint8_t toggle;
} sc2_link_client_t;

typedef enum sc2_link_client_result_t {
    SC2_LINK_CLIENT_DELIVERED,
    // Not delivered, but sending can go on: the session got lost twice, or a
    // word stayed without acknowledgement, after which the session must time
    // out before the next HELLO.
    SC2_LINK_CLIENT_DROPPED,
    // Not delivered because the transport asked the client to stop.
    SC2_LINK_CLIENT_STOPPED,
    // HELLO got no acknowledgement or the node failed: no output from here on.
    SC2_LINK_CLIENT_FAILED,
} sc2_link_client_result_t;

/** Sent outside a session, in this order and without toggles, to open one. */
extern const uint16_t sc2_link_hello_words[SC2_LINK_HELLO_WORD_COUNT];

/*
 * One message each, named after the report the dongle builds from it. Side is
 * the report's side byte, 0 to SC2_LINK_SIDE_MAX. For 0x82 to 0x85: 0 left
 * pad, 1 right pad, 2 both pads, 3 left grip, 4 right grip, 5 both grips.
 * 0x81 swaps the pads: 0 right, 1 left, 2 both.
 */

/** Report 0x82, command as in SDL's haptic_type_t: 0 off, 1 tick, 2 click, 4 rumble, 5 noise. */
sc2_link_message_t sc2_link_command_message(uint8_t side, uint8_t command, int8_t gain_db);

/** Report 0x85. */
sc2_link_message_t sc2_link_script_message(uint8_t side, uint8_t script_id, int8_t gain_db);

/**
 * Report 0x80. Both speeds zero make the one-word stop, from which the dongle
 * builds the same report as from three words.
 */
sc2_link_message_t sc2_link_rumble_message(uint16_t left_speed, uint16_t right_speed);

/** Report 0x81. */
sc2_link_message_t sc2_link_pulse_message(uint8_t side, uint16_t on_us, uint16_t off_us, uint16_t repeat_count);

/** Report 0x84. */
sc2_link_message_t sc2_link_sweep_message(uint8_t side, int8_t gain_db, uint16_t duration_ms, uint16_t start_hz,
                                          uint16_t end_hz);

/** Report 0x83. */
sc2_link_message_t sc2_link_tone_message(uint8_t side, int8_t gain_db, uint16_t frequency_hz, uint16_t duration_ms,
                                         uint16_t lfo_hz, uint8_t lfo_depth);

/** Keeps a session open while nothing else is sent. */
sc2_link_message_t sc2_link_nop_message(void);

/** Ends a session, which stops every haptic on the controller. */
sc2_link_message_t sc2_link_bye_message(void);

/**
 * The message from which the dongle rebuilds the report that libvirtualhid
 * decoded into this effect on the host. False for a kind it has none for and
 * for a target above SC2_LINK_SIDE_MAX.
 */
bool sc2_link_haptic_effect_message(const LI_CONTROLLER_HAPTIC_EFFECT *effect, sc2_link_message_t *message);

/** A session word: the data with the toggle in bit 15. */
uint16_t sc2_link_session_word(uint16_t data_word, uint8_t toggle);

/** The toggle of a session word. */
uint8_t sc2_link_word_toggle(uint16_t word);

/**
 * The session words of a message sent after a word with this toggle, each
 * with the toggle flipped. Returns the toggle of the last one.
 */
uint8_t sc2_link_stamp_message(const sc2_link_message_t *message, uint8_t toggle,
                               uint16_t words[SC2_LINK_MESSAGE_WORDS_MAX]);

/**
 * The FF_RUMBLE magnitudes that make xpad send this word, `00 08 00 SS WW 00
 * 00 00` with SS the high byte, given an FF_GAIN of 0xFFFF.
 */
uint16_t sc2_link_strong_magnitude(uint16_t word);

uint16_t sc2_link_weak_magnitude(uint16_t word);

/** ACK_STATE bits 15 to 8: accepted words, mod 256. */
uint8_t sc2_link_accepted_count(uint16_t state);

/** ACK_STATE bit 7. */
bool sc2_link_session_active(uint16_t state);

/** ACK_STATE bits 6 to 4: payload words the parser still expects, outside a session HELLO words still to come. */
uint8_t sc2_link_expected_words(uint16_t state);

/** ACK_STATE bits 3 to 0, read from an acknowledgement: before the first word the node can show 0. */
uint8_t sc2_link_protocol_version(uint16_t state);

/**
 * Whether ACK_STATE shows words accepted since state_before. A count that went
 * back is an older frame read late, not a wrap: 128 words cannot be accepted
 * during one word's wait.
 */
bool sc2_link_state_advanced(uint16_t state_before, uint16_t state);

/**
 * Whether the dongle took this word: the count moved on by exactly one and
 * ACK_ECHO is the word. Anything else means it took a word this client did not
 * send, from another FF client on the node.
 */
bool sc2_link_word_acknowledged(const sc2_link_acknowledgement_t *acknowledgement, uint16_t word);

/** Whether a HELLO must wait for SC2_LINK_SILENCE_MS: a session or another HELLO may still be running. */
bool sc2_link_hello_needs_silence(uint16_t state);

/**
 * Adds a message at the end. A RUMBLE replaces one still waiting, in its
 * place. A full queue drops its oldest message other than the RUMBLE, which
 * holds the level the controller must end up at.
 */
void sc2_link_queue_push(sc2_link_message_queue_t *queue, const sc2_link_message_t *message);

/** Takes the oldest message. False when there is none. */
bool sc2_link_queue_pop(sc2_link_message_queue_t *queue, sc2_link_message_t *message);

/** Opens a session, after silence when one may still be running. */
sc2_link_client_result_t sc2_link_client_hello(sc2_link_client_t *client, const sc2_link_transport_t *transport);

/**
 * Sends a message in the session, opening one first when there is none. A
 * session that turns out lost, before the message or in an acknowledgement,
 * is opened again and the message sent again from its start, once.
 */
sc2_link_client_result_t sc2_link_client_deliver(sc2_link_client_t *client, const sc2_link_transport_t *transport,
                                                 const sc2_link_message_t *message);

/** Ends the session with BYE, which stops every haptic. Does nothing without one. */
void sc2_link_client_bye(sc2_link_client_t *client, const sc2_link_transport_t *transport);
