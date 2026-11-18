#pragma once

#include <stdbool.h>

typedef struct app_gamepad_state_t app_gamepad_state_t;

/**
 * A Steam Controller (2nd generation) behind an OpenPuck dongle in SC2 Link mode.
 *
 * SDL opens the dongle's xpad node as an Xbox 360 pad and cannot open its HID
 * nodes, which carry the full controller state.
 */
typedef struct session_sc2_link_t {
    short controller_number;
} session_sc2_link_t;

void session_sc2_link_init(session_sc2_link_t *link);

/**
 * Called for every SDL controller that joins a stream. Looks for the dongle's
 * HID nodes when the controller is the dongle's xpad node.
 *
 * @return true when the dongle takes over this controller's slot.
 */
bool session_sc2_link_take_over(session_sc2_link_t *link, const app_gamepad_state_t *gamepad);

void session_sc2_link_stop(session_sc2_link_t *link);
