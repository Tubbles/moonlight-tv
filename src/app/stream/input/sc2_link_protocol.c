#include "sc2_link_protocol.h"

#include <string.h>

#define CONTROLS_INTERFACE_SUFFIX "/input1"
#define PADS_INTERFACE_SUFFIX "/input2"

static bool ends_with(const char *text, const char *suffix);

sc2_link_node_role_t sc2_link_node_role(uint16_t vendor, uint16_t product, const char *name, const char *phys) {
    if (vendor != SC2_LINK_VENDOR_ID || product != SC2_LINK_PRODUCT_ID ||
        strcmp(name, SC2_LINK_HID_NODE_NAME) != 0) {
        return SC2_LINK_NODE_OTHER;
    }
    if (ends_with(phys, CONTROLS_INTERFACE_SUFFIX)) {
        return SC2_LINK_NODE_CONTROLS;
    }
    if (ends_with(phys, PADS_INTERFACE_SUFFIX)) {
        return SC2_LINK_NODE_PADS;
    }
    return SC2_LINK_NODE_OTHER;
}

bool sc2_link_same_dongle(const char *phys, const char *other_phys) {
    const char *slash = strrchr(phys, '/');
    const char *other_slash = strrchr(other_phys, '/');
    if (slash == NULL || other_slash == NULL) {
        return false;
    }
    size_t prefix_length = (size_t) (slash - phys);
    return prefix_length == (size_t) (other_slash - other_phys) && strncmp(phys, other_phys, prefix_length) == 0;
}

bool sc2_link_is_xpad(uint16_t vendor, uint16_t product, uint16_t version) {
    return vendor == SC2_LINK_VENDOR_ID && product == SC2_LINK_PRODUCT_ID && version == SC2_LINK_XPAD_VERSION;
}

static bool ends_with(const char *text, const char *suffix) {
    size_t text_length = strlen(text);
    size_t suffix_length = strlen(suffix);
    return text_length >= suffix_length && strcmp(text + text_length - suffix_length, suffix) == 0;
}
