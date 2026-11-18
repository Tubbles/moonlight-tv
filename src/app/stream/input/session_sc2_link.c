#include "session_sc2_link.h"
#include "sc2_link_protocol.h"

#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/input.h>

#include <SDL_gamecontroller.h>

#include "input/app_input.h"
#include "logging.h"

#define DEV_INPUT_ROOT "/dev/input"
#define NODE_PATH_MAX 64
#define NODE_TEXT_MAX 128

typedef struct node_location_t {
    char path[NODE_PATH_MAX];
    char phys[NODE_TEXT_MAX];
} node_location_t;

static bool is_sc2_link_xpad(const app_gamepad_state_t *gamepad);

static bool find_hid_nodes(node_location_t *controls, node_location_t *pads);

static bool find_node(sc2_link_node_role_t role, const char *dongle_phys, node_location_t *location);

static sc2_link_node_role_t node_role(int fd, char phys[NODE_TEXT_MAX]);

void session_sc2_link_init(session_sc2_link_t *link) {
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
    link->controller_number = gamepad->gs_id;
    return true;
}

bool session_sc2_link_owns(const session_sc2_link_t *link, short controller_number) {
    return link->controller_number >= 0 && link->controller_number == controller_number;
}

void session_sc2_link_stop(session_sc2_link_t *link) {
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
