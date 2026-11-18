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

#define SC2_LINK_HELLO_WORD_COUNT 4
// SWEEP and TONE: a header and four payload words.
#define SC2_LINK_MESSAGE_WORDS_MAX 5

/** A message as data words, bits 14 to 0, before sc2_link_stamp_message() adds the toggles. */
typedef struct sc2_link_message_t {
    uint16_t data_words[SC2_LINK_MESSAGE_WORDS_MAX];
    uint8_t word_count;
    // RUMBLE sets a level, so a newer one makes an older one that has not gone out yet obsolete.
    bool rumble;
} sc2_link_message_t;

/** Sent outside a session, in this order and without toggles, to open one. */
extern const uint16_t sc2_link_hello_words[SC2_LINK_HELLO_WORD_COUNT];

/*
 * One message each, named after the report the dongle builds from it. Side is
 * 0 none, 1 left, 2 right, 3 both, as in the reports.
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
 * decoded into this effect on the host. False for a kind it has none for.
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
