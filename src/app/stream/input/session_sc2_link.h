#pragma once

#include <stdbool.h>
#include <SDL_atomic.h>
#include <SDL_thread.h>

typedef struct app_gamepad_state_t app_gamepad_state_t;
typedef struct stream_input_t stream_input_t;
typedef struct sc2_link_writer_t sc2_link_writer_t;

/**
 * A Steam Controller (2nd generation) behind an OpenPuck dongle in SC2 Link mode.
 *
 * SDL opens the dongle's xpad node as an Xbox 360 pad and cannot open its HID
 * nodes, which carry the full controller state. A reader thread sends that
 * state to the host as a Steam Controller, and a writer thread plays the
 * host's rumble and haptics through the xpad node.
 */
typedef struct session_sc2_link_t {
    stream_input_t *input;
    SDL_Thread *thread;
    SDL_atomic_t stop_requested;
    int controls_fd;
    int pads_fd;
    int motion_fd;
    // NULL while no reader runs, or when the xpad node was not found or the thread did not start.
    sc2_link_writer_t *writer;
    // Controller number of the xpad node whose slot the dongle took, -1 for none.
    short controller_number;
} session_sc2_link_t;

void session_sc2_link_init(session_sc2_link_t *link, stream_input_t *input);

/**
 * Called for every SDL controller that joins a stream. Looks for the dongle's
 * HID nodes when the controller is the dongle's xpad node.
 *
 * @return true when the dongle takes over this controller's slot.
 */
bool session_sc2_link_take_over(session_sc2_link_t *link, const app_gamepad_state_t *gamepad);

/** Whether the dongle holds this controller number, so its SDL controller must not be forwarded. */
bool session_sc2_link_owns(const session_sc2_link_t *link, short controller_number);

void session_sc2_link_stop(session_sc2_link_t *link);
