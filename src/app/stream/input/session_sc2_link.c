#include "session_sc2_link.h"
#include "session_input.h"
#include "sc2_link_protocol.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/input.h>

#include <SDL_gamecontroller.h>
#include <Limelight.h>

#include "input/app_input.h"
#include "stream/session.h"
#include "logging.h"

#define DEV_INPUT_ROOT "/dev/input"
#define NODE_PATH_MAX 64
#define NODE_TEXT_MAX 128

// Bounds how long stopping the reader blocks the caller.
#define POLL_TIMEOUT_MS 50
#define EVENT_BATCH_SIZE 64

// No motion, battery or haptics yet: nothing reads or plays them.
#define ARRIVAL_CAPABILITIES (LI_CCAP_ANALOG_TRIGGERS | LI_CCAP_TOUCHPAD | LI_CCAP_DUAL_TOUCHPAD)

typedef struct node_location_t {
    char path[NODE_PATH_MAX];
    char phys[NODE_TEXT_MAX];
} node_location_t;

typedef struct node_reader_t {
    int fd;
    int32_t axis_values[ABS_CNT];
    // Set by SYN_DROPPED until the next SYN_REPORT.
    bool dropped;
} node_reader_t;

typedef struct controller_reader_t {
    session_sc2_link_t *link;
    node_reader_t controls;
    node_reader_t pads;
    bool arrived;
    // What the host was last sent, so only changes go out.
    sc2_link_gamepad_state_t sent_state;
    sc2_link_pads_t sent_pads;
} controller_reader_t;

typedef void (*frame_handler_t)(controller_reader_t *reader);

static bool is_sc2_link_xpad(const app_gamepad_state_t *gamepad);

static bool find_hid_nodes(node_location_t *controls, node_location_t *pads);

static bool find_node(sc2_link_node_role_t role, const char *dongle_phys, node_location_t *location);

static sc2_link_node_role_t node_role(int fd, char phys[NODE_TEXT_MAX]);

static bool start_reader(session_sc2_link_t *link, short controller_number, const node_location_t *controls,
                         const node_location_t *pads);

static int open_node(const char *path);

static void close_nodes(session_sc2_link_t *link);

static int reader_main(session_sc2_link_t *link);

static bool read_node(controller_reader_t *reader, node_reader_t *node, short revents, frame_handler_t handle_frame);

static bool apply_event(node_reader_t *node, const struct input_event *event);

static void read_axis_values(node_reader_t *node);

static void handle_controls_frame(controller_reader_t *reader);

static void handle_pads_frame(controller_reader_t *reader);

static void send_arrival(controller_reader_t *reader);

static void send_touch(controller_reader_t *reader, uint8_t pad_index, const sc2_link_pad_t *pad);

static bool accepting_input(const controller_reader_t *reader);

void session_sc2_link_init(session_sc2_link_t *link, stream_input_t *input) {
    link->input = input;
    link->thread = NULL;
    SDL_AtomicSet(&link->stop_requested, 0);
    link->controls_fd = -1;
    link->pads_fd = -1;
    link->controller_number = -1;
}

bool session_sc2_link_take_over(session_sc2_link_t *link, const app_gamepad_state_t *gamepad) {
    if (!is_sc2_link_xpad(gamepad)) {
        if (session_sc2_link_owns(link, gamepad->gs_id)) {
            // The dongle was unplugged and another controller got its number.
            session_sc2_link_stop(link);
        }
        return false;
    }
    if (link->controller_number >= 0 && link->controller_number != gamepad->gs_id) {
        commons_log_warn("SC2Link", "Controller #%d is a second SC2 Link dongle, forwarded as an Xbox 360 pad",
                         gamepad->gs_id);
        return false;
    }
    session_sc2_link_stop(link);
    node_location_t controls, pads;
    if (!find_hid_nodes(&controls, &pads)) {
        commons_log_warn("SC2Link", "Controller #%d is an SC2 Link dongle, but its HID nodes were not found",
                         gamepad->gs_id);
        return false;
    }
    commons_log_info("SC2Link", "Controller #%d is an SC2 Link dongle: controls %s, pads %s", gamepad->gs_id,
                     controls.path, pads.path);
    return start_reader(link, gamepad->gs_id, &controls, &pads);
}

bool session_sc2_link_owns(const session_sc2_link_t *link, short controller_number) {
    return link->controller_number >= 0 && link->controller_number == controller_number;
}

void session_sc2_link_stop(session_sc2_link_t *link) {
    if (link->thread != NULL) {
        SDL_AtomicSet(&link->stop_requested, 1);
        SDL_WaitThread(link->thread, NULL);
        link->thread = NULL;
    }
    close_nodes(link);
    link->controller_number = -1;
}

static bool is_sc2_link_xpad(const app_gamepad_state_t *gamepad) {
    SDL_GameController *controller = gamepad->controller;
    return controller != NULL &&
           sc2_link_is_xpad(SDL_GameControllerGetVendor(controller), SDL_GameControllerGetProduct(controller),
                            SDL_GameControllerGetProductVersion(controller));
}

/**
 * Supports one dongle: with several, the first controls node found wins, and
 * its pads node must share its USB device.
 */
static bool find_hid_nodes(node_location_t *controls, node_location_t *pads) {
    return find_node(SC2_LINK_NODE_CONTROLS, NULL, controls) &&
           find_node(SC2_LINK_NODE_PADS, controls->phys, pads);
}

static bool find_node(sc2_link_node_role_t role, const char *dongle_phys, node_location_t *location) {
    DIR *dir = opendir(DEV_INPUT_ROOT);
    if (dir == NULL) {
        return false;
    }
    bool found = false;
    struct dirent *entry;
    while (!found && (entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "event", 5) != 0 ||
            snprintf(location->path, sizeof(location->path), "%s/%s", DEV_INPUT_ROOT, entry->d_name) >=
            (int) sizeof(location->path)) {
            continue;
        }
        int fd = open(location->path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            continue;
        }
        found = node_role(fd, location->phys) == role &&
                (dongle_phys == NULL || sc2_link_same_dongle(location->phys, dongle_phys));
        close(fd);
    }
    closedir(dir);
    return found;
}

static sc2_link_node_role_t node_role(int fd, char phys[NODE_TEXT_MAX]) {
    struct input_id id;
    char name[NODE_TEXT_MAX];
    memset(name, 0, sizeof(name));
    memset(phys, 0, NODE_TEXT_MAX);
    // One byte short of the buffers, so a truncated string stays terminated.
    if (ioctl(fd, EVIOCGID, &id) < 0 || ioctl(fd, EVIOCGNAME(NODE_TEXT_MAX - 1), name) < 0 ||
        ioctl(fd, EVIOCGPHYS(NODE_TEXT_MAX - 1), phys) < 0) {
        return SC2_LINK_NODE_OTHER;
    }
    return sc2_link_node_role(id.vendor, id.product, name, phys);
}

static bool start_reader(session_sc2_link_t *link, short controller_number, const node_location_t *controls,
                         const node_location_t *pads) {
    link->controls_fd = open_node(controls->path);
    link->pads_fd = open_node(pads->path);
    if (link->controls_fd < 0 || link->pads_fd < 0) {
        close_nodes(link);
        return false;
    }
    // Set before the thread starts, which reads it.
    link->controller_number = controller_number;
    SDL_AtomicSet(&link->stop_requested, 0);
    link->thread = SDL_CreateThread((SDL_ThreadFunction) reader_main, "sc2link", link);
    if (link->thread == NULL) {
        commons_log_error("SC2Link", "Can't start the reader: %s", SDL_GetError());
        close_nodes(link);
        link->controller_number = -1;
        return false;
    }
    return true;
}

/** Grabbed, so nothing else on the TV acts on the node while the stream reads it. */
static int open_node(const char *path) {
    int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        commons_log_error("SC2Link", "Can't open %s: %s", path, strerror(errno));
        return -1;
    }
    if (ioctl(fd, EVIOCGRAB, 1) < 0) {
        commons_log_warn("SC2Link", "Can't grab %s: %s", path, strerror(errno));
    }
    return fd;
}

/** Closing a node also releases its grab. */
static void close_nodes(session_sc2_link_t *link) {
    if (link->controls_fd >= 0) {
        close(link->controls_fd);
        link->controls_fd = -1;
    }
    if (link->pads_fd >= 0) {
        close(link->pads_fd);
        link->pads_fd = -1;
    }
}

static int reader_main(session_sc2_link_t *link) {
    controller_reader_t reader;
    memset(&reader, 0, sizeof(reader));
    reader.link = link;
    reader.controls.fd = link->controls_fd;
    reader.pads.fd = link->pads_fd;
    read_axis_values(&reader.controls);
    read_axis_values(&reader.pads);
    handle_controls_frame(&reader);
    handle_pads_frame(&reader);
    while (!SDL_AtomicGet(&link->stop_requested)) {
        struct pollfd fds[2] = {
                {.fd = reader.controls.fd, .events = POLLIN},
                {.fd = reader.pads.fd, .events = POLLIN},
        };
        int ready = poll(fds, 2, POLL_TIMEOUT_MS);
        if (ready < 0 && errno != EINTR) {
            commons_log_error("SC2Link", "poll failed: %s", strerror(errno));
            break;
        }
        if (ready <= 0) {
            continue;
        }
        if (!read_node(&reader, &reader.controls, fds[0].revents, handle_controls_frame) ||
            !read_node(&reader, &reader.pads, fds[1].revents, handle_pads_frame)) {
            break;
        }
    }
    commons_log_info("SC2Link", "Reader of controller #%d stopped", link->controller_number);
    return 0;
}

/** @return false when the node is gone, for example because the dongle was unplugged. */
static bool read_node(controller_reader_t *reader, node_reader_t *node, short revents, frame_handler_t handle_frame) {
    if (revents & (POLLERR | POLLHUP | POLLNVAL)) {
        return false;
    }
    if (!(revents & POLLIN)) {
        return true;
    }
    struct input_event events[EVENT_BATCH_SIZE];
    for (;;) {
        ssize_t length = read(node->fd, events, sizeof(events));
        if (length < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                return true;
            }
            commons_log_warn("SC2Link", "Reading a node failed: %s", strerror(errno));
            return false;
        }
        size_t count = (size_t) length / sizeof(events[0]);
        for (size_t index = 0; index < count; index++) {
            if (apply_event(node, &events[index])) {
                handle_frame(reader);
            }
        }
        if (count < EVENT_BATCH_SIZE) {
            return true;
        }
    }
}

/** @return true when the event completes a frame. */
static bool apply_event(node_reader_t *node, const struct input_event *event) {
    if (event->type == EV_SYN && event->code == SYN_DROPPED) {
        node->dropped = true;
        return false;
    }
    if (event->type == EV_SYN && event->code == SYN_REPORT) {
        if (node->dropped) {
            // Events were lost, so read every value again.
            read_axis_values(node);
            node->dropped = false;
        }
        return true;
    }
    if (event->type == EV_ABS && event->code < ABS_CNT && !node->dropped) {
        node->axis_values[event->code] = event->value;
    }
    return false;
}

static void read_axis_values(node_reader_t *node) {
    for (unsigned int code = 0; code < ABS_CNT; code++) {
        struct input_absinfo info;
        if (ioctl(node->fd, EVIOCGABS(code), &info) == 0) {
            node->axis_values[code] = info.value;
        }
    }
}

static void handle_controls_frame(controller_reader_t *reader) {
    sc2_link_controls_t controls = sc2_link_decode_controls(reader->controls.axis_values);
    if (!accepting_input(reader)) {
        return;
    }
    if (!reader->arrived) {
        // Any state sent before the arrival makes the host create its default pad for this number.
        if (!sc2_link_controller_connected(&controls)) {
            return;
        }
        send_arrival(reader);
    }
    sc2_link_gamepad_state_t state = sc2_link_gamepad_state(&controls);
    if (sc2_link_gamepad_state_equal(&state, &reader->sent_state)) {
        return;
    }
    LiSendMultiControllerEvent(reader->link->controller_number, app_input_gamepads_mask(reader->link->input->input),
                               state.buttons, state.left_trigger, state.right_trigger, state.left_stick_x,
                               state.left_stick_y, state.right_stick_x, state.right_stick_y);
    reader->sent_state = state;
}

static void handle_pads_frame(controller_reader_t *reader) {
    if (!reader->arrived || !accepting_input(reader)) {
        return;
    }
    sc2_link_pads_t pads = sc2_link_decode_pads(reader->pads.axis_values);
    for (uint8_t pad_index = 0; pad_index < SC2_LINK_PAD_COUNT; pad_index++) {
        send_touch(reader, pad_index, &pads.pads[pad_index]);
    }
}

/** The arrival also sends a neutral state, which is where sent_state starts. */
static void send_arrival(controller_reader_t *reader) {
    short controller_number = reader->link->controller_number;
    short active_gamepad_mask = app_input_gamepads_mask(reader->link->input->input);
    commons_log_info("SC2Link", "Steam Controller arrived as controller #%d", controller_number);
    LiSendControllerArrivalEvent((uint8_t) controller_number, (uint16_t) active_gamepad_mask, LI_CTYPE_STEAM,
                                 sc2_link_supported_button_flags(), ARRIVAL_CAPABILITIES);
    reader->arrived = true;
}

static void send_touch(controller_reader_t *reader, uint8_t pad_index, const sc2_link_pad_t *pad) {
    sc2_link_pad_t *sent = &reader->sent_pads.pads[pad_index];
    int event_type = sc2_link_touch_event_type(sent, pad);
    if (event_type == SC2_LINK_NO_TOUCH_EVENT) {
        return;
    }
    // One finger per pad, so the pad index doubles as the pointer id.
    LiSendControllerTouchEvent2((uint8_t) reader->link->controller_number, (uint8_t) event_type, pad_index,
                                pad_index, sc2_link_pad_x(pad->x), sc2_link_pad_y(pad->y),
                                sc2_link_pad_pressure(pad->pressure));
    *sent = *pad;
}

static bool accepting_input(const controller_reader_t *reader) {
    return session_accepting_input(reader->link->input->session);
}
