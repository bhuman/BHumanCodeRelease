#ifndef SPLCOMMSCHALLENGE_H
#define SPLCOMMSCHALLENGE_H

/**
 * For the No-Wifi challenge the expected setup is as follows:
 *
 *
 *          TransmitterRobot [T]  --> non wifi ---->  ReceiverRobot [R]
 *                 ^                                         |
 *                 |                                         |
 *            wired ethernet                          wired ethernet
 *                 |                                         v
 *                 +<---------- CommsTester (on PC) <--------+
 *
 *
 * There are 2 types of SPLNoWifi message which use a common header structure,
 * namely the location message and the data message.
 *
 * When communicated to/from the comms tester, a location message consists
 * of an SPLNoWifiHeader followed immediately by an SPLNoWifiLocationPayload.
 *
 * Similarly a data message consists of an SPLNoWifiHeader followed by
 * an SPLNoWifiDataPayloadHeader followed by the actual bytes of data.
 *
 * When writing a receiver for these packets (e.g. the transmitter robot
 * in the no-wifi challenge) the general approach is:
 *   - read the header to decide what type of message it is
 *   - if it is a location message, just read the fixed size location payload
 *   - if it is a data message, then read the fixed size data payload header
 *     to decide how much data there is. Then read the actual data bytes.
 *   - The SPLNoWifiDataPayload and SPLNoWifiPacket are convenience
 *     structures to keep the various elements together.
 *
 * When writing a transmitter for these packets (e.g. the receiver robot in
 * the no-wifi challenge) the general approach is:
 *   - write the header
 *   - if it is a location message write the fixed length location payload
 *   - if it is a data message, write the fixed length data payload header
 *     followed by the appropriate number of data bytes
 *
 * Refer to the comms_robot.cpp test code for examples of reading and writing
 * these packets.
 *
 * Note that the comms_robot.cpp test code uses blocking calls and is not
 * implemented in a particularly robust way so effort is likely required to turn
 * it into working robot code for the challenge (in addition to implementing
 * the non-wifi communications scheme itself).
 */

#include <stdint.h>

#define SPL_NO_WIFI_COMMS_TESTER_PORT               38000
#define SPL_NO_WIFI_DEFAULT_TRANSMITTER_ROBOT_PORT  10100

#define SPL_NO_WIFI_TIMEOUT_MS              15000

#define SPL_NO_WIFI_DATA_PAYLOAD_MAX_LEN    10000

#define SPL_NO_WIFI_PAYLOAD_TYPE_LOCATION   1
#define SPL_NO_WIFI_PAYLOAD_TYPE_DATA       2

#define SPL_NO_WIFI_LOCATION_TOLERANCE_MM   50

/**
 * This structure is the payload sent from the communications tester to the
 * transmitting robot, T, to indicate the location to be communicated.
 * T then sends the data to the receiving robot R by some non-WiFi
 * communications mechanism.
 * Finally, R populates a new structure and sends this back to the
 * communications tester which verifies that the data was communicated
 * accurately (to within tolerances allowed by the challenge).
 */
struct SPLNoWifiLocationPayload {
    // position on the field in standard SPL coordinates with millimeter units
    // 0,0 is in center of field
    // +ve x-axis points towards the goal that the receiving robot faces
    // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis

    int16_t    x;
    int16_t    y;
};

/**
 * This structure is used to send the data payload from the communications
 * tester to robot T. The fragment offset will always be 0 and the
 * fragmentLength will always be SPL_DATA_PAYLOAD_MAX_LEN in this initial
 * payload.
 * T will then decide how much of the data length it can reasonably
 * expect to send in a short time (specified in the challenge) using its
 * non-WiFi communications scheme and will then attempt to transmit this data
 * to robot R.
 * Robot R forwards the data as it is received back to the communications
 * tester. If the data was fragmented for transmission, then R should populate
 * a separate payload for each fragment.
 */
struct SPLNoWifiDataPayloadHeader {
    // Offset of this fragment relative to original data.
    // (This is used for fragmentation and reassembly of data)
    uint16_t    fragmentOffset;

    // Length of this data fragment. The receiver of this message
    // should not read more than this number of octets from the
    // data array.
    uint16_t    fragmentLength;
};

struct SPLNoWifiDataPayload {
    struct SPLNoWifiDataPayloadHeader header;

    // the actual binary data being communicated
    uint8_t     data[SPL_NO_WIFI_DATA_PAYLOAD_MAX_LEN];
};


struct SPLNoWifiHeader {
    uint16_t     type; // what type of message is this
};

struct SPLNoWifiPacket {
    struct SPLNoWifiHeader header;
    union {
        struct SPLNoWifiLocationPayload   location;
        struct SPLNoWifiDataPayload       data;
    } payload;
};



#endif // SPLCOMMSCHALLENGE_H