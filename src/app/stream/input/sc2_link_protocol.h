#pragma once

/*
 * OpenPuck dongle in its "SC2 Link" mode, which carries one Steam Controller
 * (2nd generation) as evdev nodes: xpad binds USB interface 0 as an ordinary
 * Xbox 360 pad, and hid-generic makes one input node of each HID interface 1
 * to 3 (controls, pads, motion). Reference: docs/PROTOCOL.md, section 9.5, of
 * the OpenPuck sources.
 *
 * Everything here is free of I/O so it can be tested on any Linux host.
 */

#include <stdbool.h>
#include <stdint.h>

#define SC2_LINK_VENDOR_ID 0x045E
#define SC2_LINK_PRODUCT_ID 0x028E
// The mode's bcdDevice, which xpad reports as the version of its node.
#define SC2_LINK_XPAD_VERSION 0x5C21
// usbhid names the HID nodes after the USB manufacturer and product strings.
#define SC2_LINK_HID_NODE_NAME "OpenPuck SC2 Link"

typedef enum sc2_link_node_role_t {
    SC2_LINK_NODE_OTHER = 0,
    SC2_LINK_NODE_CONTROLS,
    SC2_LINK_NODE_PADS,
} sc2_link_node_role_t;

/**
 * Which of the dongle's HID nodes an evdev node is, from its EVIOCGID vendor
 * and product, its name and its phys string. The phys string ends in
 * "/input" and the USB interface number.
 */
sc2_link_node_role_t sc2_link_node_role(uint16_t vendor, uint16_t product, const char *name, const char *phys);

/** Whether two nodes belong to the same USB device: their phys strings differ only after the last '/'. */
bool sc2_link_same_dongle(const char *phys, const char *other_phys);

/** Whether a controller with this vendor, product and version is the dongle's xpad node. */
bool sc2_link_is_xpad(uint16_t vendor, uint16_t product, uint16_t version);
