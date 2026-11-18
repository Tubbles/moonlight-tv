#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * The output half of an SC2 Link dongle: a thread that keeps a word protocol
 * session open on the dongle's xpad node and sends the queued messages through
 * it, one word at a time. The acknowledgements arrive on the pads node, which
 * the reader thread reads and passes on.
 */
typedef struct sc2_link_writer_t sc2_link_writer_t;

typedef enum sc2_link_output_t {
    // No session wanted, and the xpad node is closed.
    SC2_LINK_OUTPUT_IDLE,
    // Opening the xpad node and the session.
    SC2_LINK_OUTPUT_OPENING,
    SC2_LINK_OUTPUT_OPEN,
    // The xpad node cannot be written or the dongle did not answer: nothing is sent until the next start.
    SC2_LINK_OUTPUT_UNAVAILABLE,
} sc2_link_output_t;

/**
 * Starts the thread, which opens the xpad node at xpad_path once output is
 * wanted. pads_fd must stay open until sc2_link_writer_stop() returns.
 *
 * @return NULL when the thread could not be started.
 */
sc2_link_writer_t *sc2_link_writer_start(const char *xpad_path, int pads_fd);

/** Ends the session and the thread, and closes the xpad node. */
void sc2_link_writer_stop(sc2_link_writer_t *writer);

/** Frees a stopped writer. */
void sc2_link_writer_destroy(sc2_link_writer_t *writer);

/** Asks for a session to be opened. */
void sc2_link_writer_want_output(sc2_link_writer_t *writer);

/** ACK_ECHO and ACK_STATE from a frame of the pads node. */
void sc2_link_writer_note_acknowledgement(sc2_link_writer_t *writer, uint16_t echo, uint16_t state);
