/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#ifndef RF24_RADIO_H
#define RF24_RADIO_H


namespace codal
{
    class RF24Radio;
    struct FrameBuffer;
}

#include "CodalConfig.h"
#include "CodalComponent.h"
#include "ManagedBuffer.h"
#include "RP2040Pin.h"
#include "RF24RadioDatagram.h"
#include "RF24RadioEvent.h"
#include "Radio.h"
#include "RF24.h"

#define CE_PIN  7
#define CSN_PIN 8
#define IRQ_PIN 6

// Status Flags
#define RF24_RADIO_STATUS_INITIALISED       0x0001

// Default configuration values
#define RF24_RADIO_DEFAULT_GROUP            0
#define RF24_RADIO_DEFAULT_TX_POWER         RF24_PA_HIGH
#define RF24_RADIO_DEFAULT_FREQUENCY        76
#define RF24_RADIO_MAX_PACKET_SIZE          32
#define RF24_RADIO_HEADER_SIZE              4
#define RF24_RADIO_MAXIMUM_RX_BUFFERS       4

// Known Protocol Numbers
#define RF24_RADIO_PROTOCOL_DATAGRAM        1       // A simple, single frame datagram. a little like UDP but with smaller packets. :-)
#define RF24_RADIO_PROTOCOL_EVENTBUS        2       // Transparent propogation of events from one micro:bit to another.

// Events
#define RF24_RADIO_EVT_DATAGRAM             1       // Event to signal that a new datagram has been received.

#define RF24_BLE_POWER_LEVELS                 8

namespace codal
{
struct FrameBuffer
{
    uint8_t         length;                             // The length of the remaining bytes in the packet. includes protocol/version/group fields, excluding the length field itself.
    uint8_t         version;                            // Protocol version code.
    uint8_t         group;                              // ID of the group to which this packet belongs.
    uint8_t         protocol;                           // Inner protocol number c.f. those issued by IANA for IP protocols

    uint8_t         payload[RF24_RADIO_MAX_PACKET_SIZE];    // User / higher layer protocol data
    FrameBuffer     *next;                              // Linkage, to allow this and other protocols to queue packets pending processing.
    int             rssi;                               // Received signal strength of this frame.
};


class RF24Radio : public Radio
{
    uint8_t                 group;      // The radio group to which this micro:bit belongs.
    uint8_t                 queueDepth; // The number of packets in the receiver queue.
    int                     rssi;
    FrameBuffer             *rxQueue;   // A linear list of incoming packets, queued awaiting processing.
    FrameBuffer             *rxBuf;     // A pointer to the buffer being actively used by the RADIO hardware.

    public:
    RF24RadioDatagram       datagram;   // A simple datagram service.
    RF24RadioEvent          event;      // A simple event handling service.
    RF24                    radio;      // The RF24 instance.
    RP2040Pin               irq;        // irq pin for events/interrupts

    static RF24Radio        *instance;  // A singleton reference, used purely by the interrupt service routine.

    /**
      * Constructor.
      *
      * Initialise the RF24Radio.
      *
      * @note This class is demand activated, as a result most resources are only
      *       committed if send/recv or event registrations calls are made.
      */
    RF24Radio(uint16_t id = DEVICE_ID_RADIO);

    int recvFrame();

    /**
      * Change the output power level of the transmitter to the given value.
      *
      * @param power a value in the range 0..7, where 0 is the lowest power and 7 is the highest.
      *
      * @return DEVICE_OK on success, or DEVICE_INVALID_PARAMETER if the value is out of range.
      */
    int setTransmitPower(int power);

    /**
      * Change the transmission and reception band of the radio to the given channel
      *
      * @param band a frequency band in the range 0 - 100. Each step is 1MHz wide, based at 2400MHz.
      *
      * @return DEVICE_OK on success, or DEVICE_INVALID_PARAMETER if the value is out of range,
      *         or DEVICE_NOT_SUPPORTED if the BLE stack is running.
      */
    int setFrequencyBand(int band);

    /**
      * Retrieve a pointer to the currently allocated receive buffer. This is the area of memory
      * actively being used by the radio hardware to store incoming data.
      *
      * @return a pointer to the current receive buffer.
      */
    FrameBuffer * getRxBuf();

    /**
      * Attempt to queue a buffer received by the radio hardware, if sufficient space is available.
      *
      * @return DEVICE_OK on success, or DEVICE_NO_RESOURCES if a replacement receiver buffer
      *         could not be allocated (either by policy or memory exhaustion).
      */
    int queueRxBuf();

    /**
      * Sets the RSSI for the most recent packet.
      * The value is measured in -dbm. The higher the value, the stronger the signal.
      * Typical values are in the range -42 to -128.
      *
      * @param rssi the new rssi value.
      *
      * @note should only be called from RADIO_IRQHandler...
      */
    int setRSSI(int rssi);

    /**
      * Retrieves the current RSSI for the most recent packet.
      * The return value is measured in -dbm. The higher the value, the stronger the signal.
      * Typical values are in the range -42 to -128.
      *
      * @return the most recent RSSI value or DEVICE_NOT_SUPPORTED if the BLE stack is running.
      */
    int getRSSI();

    /**
      * Initialises the radio for use as a multipoint sender/receiver
      *
      * @return DEVICE_OK on success, DEVICE_NOT_SUPPORTED if the BLE stack is running.
      */
    virtual int enable();

    /**
      * Disables the radio for use as a multipoint sender/receiver.
      *
      * @return DEVICE_OK on success, DEVICE_NOT_SUPPORTED if the BLE stack is running.
      */
    virtual int disable();

    /**
      * Sets the radio to listen to packets sent with the given group id.
      *
      * @param group The group to join. A micro:bit can only listen to one group ID at any time.
      *
      * @return DEVICE_OK on success, or DEVICE_NOT_SUPPORTED if the BLE stack is running.
      */
    int setGroup(uint8_t group);

    /**
      * A background, low priority callback that is triggered whenever the processor is idle.
      * Here, we empty our queue of received packets, and pass them onto higher level protocol handlers.
      */
    virtual void idleCallback();

    /**
      * Determines the number of packets ready to be processed.
      *
      * @return The number of packets in the receive buffer.
      */
    int dataReady();

    /**
      * Retrieves the next packet from the receive buffer.
      * If a data packet is available, then it will be returned immediately to
      * the caller. This call will also dequeue the buffer.
      *
      * @return The buffer containing the the packet. If no data is available, NULL is returned.
      *
      * @note Once recv() has been called, it is the callers responsibility to
      *       delete the buffer when appropriate.
      */
    FrameBuffer* recv();

    /**
      * Transmits the given buffer onto the broadcast radio.
      * The call will wait until the transmission of the packet has completed before returning.
      *
      * @param data The packet contents to transmit.
      *
      * @return DEVICE_OK on success, or DEVICE_NOT_SUPPORTED if the BLE stack is running.
      */
    int send(FrameBuffer *buffer);

    /**
      * Retrieves the next packet from the receive buffer.
      * If a data packet is available, then it will be returned immediately to
      * the caller. This call will also dequeue the buffer.
      *
      * @return The buffer containing the the packet. If no data is available, NULL is returned.
      *
      * @note Once recv() has been called, it is the callers responsibility to
      *       delete the buffer when appropriate.
      */
    virtual ManagedBuffer recvBuffer();

    /**
      * Transmits the given buffer onto the broadcast radio.
      * The call will wait until the transmission of the packet has completed before returning.
      *
      * @param data The packet contents to transmit.
      *
      * @return DEVICE_OK on success, or DEVICE_NOT_SUPPORTED if the BLE stack is running.
      */
    virtual int sendBuffer(ManagedBuffer b);
};
}

#endif
