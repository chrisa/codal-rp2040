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

#include "CodalDmesg.h"
#include "RF24Radio.h"
#include "Radio.h"
#include "EventModel.h"
#include "Event.h"
#include "ErrorNo.h"

#include "ram.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

const int8_t RF24_BLE_POWER_LEVEL[] = {-30, -20, -16, -12, -8, -4, 0, 4};
uint8_t ADDRESS[] = { 0x00, 0x75, 0x62, 0x69, 0x74 };

using namespace codal;

RF24Radio* RF24Radio::instance = NULL;

extern "C" void interruptHandler(uint gpio, uint32_t events)
{
    DMESG("in irq handler");
    RF24Radio::instance->recvFrame();
}

/**
  * Constructor.
  *
  * Initialise the RF24Radio.
  *
  * @note This class is demand activated, as a result most resources are only
  *       committed if send/recv or event registrations calls are made.
  */
RF24Radio::RF24Radio(uint16_t id) : 
    datagram(*this),
    event(*this),
    radio(CE_PIN, CSN_PIN),
    irq(DEVICE_ID_IO_P0 + IRQ_PIN, (PinNumber) IRQ_PIN, PIN_CAPABILITY_DIGITAL)
{
    this->id = id;
    this->status = 0;
	this->group = RF24_RADIO_DEFAULT_GROUP;
	this->queueDepth = 0;
    this->rssi = 0;
    this->rxQueue = NULL;
    this->rxBuf = NULL;

    DMESG("creating RF24Radio instance...");

    // this->radio = RF24(CE_PIN, CSN_PIN);
    // this->irq = RP2040Pin();

    instance = this;
}

int RF24Radio::recvFrame()
{
    DMESG("recvFrame");
    // Associate this packet's rssi value with the data just
    // transferred by DMA receive
    setRSSI(-42);

    uint8_t pipe;
    if (radio.available(&pipe)) {
        rxBuf->group = group;
        rxBuf->length = radio.getPayloadSize();
        radio.read(&rxBuf->payload, rxBuf->length);
    }

    // Now move on to the next buffer, if possible.
    // The queued packet will get the rssi value set above.
    RF24Radio::instance->queueRxBuf();
}

/**
  * Change the output power level of the transmitter to the given value.
  *
  * @param power a value in the range 0..7, where 0 is the lowest power and 7 is the highest.
  *
  * @return DEVICE_OK on success, or DEVICE_INVALID_PARAMETER if the value is out of range.
  */
int RF24Radio::setTransmitPower(int power)
{
    if (power < 0 || power >= RF24_BLE_POWER_LEVELS)
        return DEVICE_INVALID_PARAMETER;

    DMESG("setTransmitPower");

    switch(power) {
        case 0:
            radio.setPALevel(RF24_PA_MIN);
            break;
        case 1:
        case 2:
        case 3:
            radio.setPALevel(RF24_PA_LOW);
            break;
        case 4:
        case 5:
        case 6:
            radio.setPALevel(RF24_PA_HIGH);
            break;
        case 7:
            radio.setPALevel(RF24_PA_MAX);
            break;
        default:
            radio.setPALevel(RF24_PA_MIN);
            break;
    }

    return DEVICE_OK;
}

/**
  * Change the transmission and reception band of the radio to the given channel
  *
  * @param band a frequency band in the range 0 - 100. Each step is 1MHz wide, based at 2400MHz.
  *
  * @return DEVICE_OK on success, or DEVICE_INVALID_PARAMETER if the value is out of range,
  *         or DEVICE_NOT_SUPPORTED if the BLE stack is running.
  */
int RF24Radio::setFrequencyBand(int band)
{
    if (band < 0 || band > 125)
        return DEVICE_INVALID_PARAMETER;

    DMESG("setFrequencyBand");

    radio.setChannel(band);

    return DEVICE_OK;
}

/**
  * Retrieve a pointer to the currently allocated receive buffer. This is the area of memory
  * actively being used by the radio hardware to store incoming data.
  *
  * @return a pointer to the current receive buffer.
  */
FrameBuffer* RF24Radio::getRxBuf()
{
    return rxBuf;
}

/**
  * Attempt to queue a buffer received by the radio hardware, if sufficient space is available.
  *
  * @return DEVICE_OK on success, or DEVICE_NO_RESOURCES if a replacement receiver buffer
  *         could not be allocated (either by policy or memory exhaustion).
  */
int RF24Radio::queueRxBuf()
{
    DMESG("queueRxBuf");

    if (rxBuf == NULL)
        return DEVICE_INVALID_PARAMETER;

    if (queueDepth >= RF24_RADIO_MAXIMUM_RX_BUFFERS)
        return DEVICE_NO_RESOURCES;

    // Store the received RSSI value in the frame
    rxBuf->rssi = getRSSI();

    // Ensure that a replacement buffer is available before queuing.
    FrameBuffer *newRxBuf = new FrameBuffer();

    if (newRxBuf == NULL)
        return DEVICE_NO_RESOURCES;

    // We add to the tail of the queue to preserve causal ordering.
    rxBuf->next = NULL;

    if (rxQueue == NULL)
    {
        rxQueue = rxBuf;
    }
    else
    {
        FrameBuffer *p = rxQueue;
        while (p->next != NULL)
            p = p->next;

        p->next = rxBuf;
    }

    // Increase our received packet count
    queueDepth++;

    // Allocate a new buffer for the receiver hardware to use. the old on will be passed on to higher layer protocols/apps.
    rxBuf = newRxBuf;

    return DEVICE_OK;
}

/**
  * Sets the RSSI for the most recent packet.
  * The value is measured in -dbm. The higher the value, the stronger the signal.
  * Typical values are in the range -42 to -128.
  *
  * @param rssi the new rssi value.
  *
  * @note should only be called from RADIO_IRQHandler...
  */
int RF24Radio::setRSSI(int rssi)
{
    if (!(status & RF24_RADIO_STATUS_INITIALISED))
        return DEVICE_NOT_SUPPORTED;

    this->rssi = rssi;

    return DEVICE_OK;
}

/**
  * Retrieves the current RSSI for the most recent packet.
  * The return value is measured in -dbm. The higher the value, the stronger the signal.
  * Typical values are in the range -42 to -128.
  *
  * @return the most recent RSSI value or DEVICE_NOT_SUPPORTED if the BLE stack is running.
  */
int RF24Radio::getRSSI()
{
    //

    return this->rssi;
}

/**
  * Initialises the radio for use as a multipoint sender/receiver
  *
  * @return DEVICE_OK on success, DEVICE_NOT_SUPPORTED if the BLE stack is running.
  */
int RF24Radio::enable()
{
    // If the device is already initialised, then there's nothing to do.
    if (status & RF24_RADIO_STATUS_INITIALISED)
        return DEVICE_OK;

    DMESG("starting enable");

    // If this is the first time we've been enable, allocate out receive buffers.
    if (rxBuf == NULL)
        rxBuf = new FrameBuffer();

    if (rxBuf == NULL)
        return DEVICE_NO_RESOURCES;

    // initialize the transceiver on the SPI bus
    if (!radio.begin()) {
        DMESG("radio hardware is not responding!!\n");
        return false;
    }

    if (radio.isChipConnected())
    {
        DMESG("chip appears connected");
    }
    else
    {
        DMESG("chip not connected");
    }

    setGroup(this->group);

    radio.setPALevel(RF24_RADIO_DEFAULT_TX_POWER);
    radio.setChannel(RF24_RADIO_DEFAULT_FREQUENCY);

    // radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

    ADDRESS[0] = this->group;
    radio.setAddressWidth(5);
    radio.openReadingPipe(0, ADDRESS);
    radio.startListening();

    // register ourselves for a callback event, in order to empty the receive queue.
    status |= DEVICE_COMPONENT_STATUS_IDLE_TICK;

    // Done. Record that our RADIO is configured.
    status |= RF24_RADIO_STATUS_INITIALISED;

    //gpio_set_irq_enabled_with_callback(IRQ_PIN, GPIO_IRQ_EDGE_FALL, true, &interruptHandler);

    DMESG("finished enable");

    return DEVICE_OK;
}

/**
  * Disables the radio for use as a multipoint sender/receiver.
  *
  * @return DEVICE_OK on success, DEVICE_NOT_SUPPORTED if the BLE stack is running.
  */
int RF24Radio::disable()
{
    DMESG("disable");

    // Only attempt to enable.disable the radio if the protocol is alreayd running.
    if (status & RF24_RADIO_STATUS_INITIALISED)
        return DEVICE_OK;

    radio.stopListening();
    
    // deregister ourselves from the callback event used to empty the receive queue.
    status &= ~DEVICE_COMPONENT_STATUS_IDLE_TICK;

    // record that the radio is now disabled
    status &= ~RF24_RADIO_STATUS_INITIALISED;

    return DEVICE_OK;
}

/**
  * Sets the radio to listen to packets sent with the given group id.
  *
  * @param group The group to join. A micro:bit can only listen to one group ID at any time.
  *
  * @return DEVICE_OK on success, or DEVICE_NOT_SUPPORTED if the BLE stack is running.
  */
int RF24Radio::setGroup(uint8_t group)
{
    DMESG("setGroup");

    // Record our group id locally
    this->group = group;

    // and that's it; address is used when initially opening a listen pipe

    return DEVICE_OK;
}

/**
  * A background, low priority callback that is triggered whenever the processor is idle.
  * Here, we empty our queue of received packets, and pass them onto higher level protocol handlers.
  */

void RF24Radio::idleCallback()
{
    DMESG("idleCallback");

    // Walk the list of packets and process each one.
    while(rxQueue)
    {
        FrameBuffer *p = rxQueue;

        switch (p->protocol)
        {
            case RF24_RADIO_PROTOCOL_DATAGRAM:
                datagram.packetReceived();
                break;

            case RF24_RADIO_PROTOCOL_EVENTBUS:
                event.packetReceived();
                break;

            default:
                Event(DEVICE_ID_RADIO_DATA_READY, p->protocol);
        }

        // If the packet was processed, it will have been recv'd, and taken from the queue.
        // If this was a packet for an unknown protocol, it will still be there, so simply free it.
        if (p == rxQueue)
        {
            recv();
            delete p;
        }

        DMESG("POORECV");
        // this is perhaps the wrong event to fire... will do for now.
        Event(this->id, RADIO_EVT_DATA_READY);
    }
}

/**
  * Determines the number of packets ready to be processed.
  *
  * @return The number of packets in the receive buffer.
  */
int RF24Radio::dataReady()
{
    return queueDepth;
}

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
FrameBuffer* RF24Radio::recv()
{
    FrameBuffer *p = rxQueue;

    DMESG("recv");

    if (p)
    {
        gpio_set_irq_enabled(IRQ_PIN, GPIO_IRQ_EDGE_FALL, false);

        rxQueue = rxQueue->next;
        queueDepth--;

        gpio_set_irq_enabled_with_callback(IRQ_PIN, GPIO_IRQ_EDGE_FALL, true, &interruptHandler);
    }

    return p;
}

/**
  * Transmits the given buffer onto the broadcast radio.
  * The call will wait until the transmission of the packet has completed before returning.
  *
  * @param data The packet contents to transmit.
  *
  * @return DEVICE_OK on success, or DEVICE_NOT_SUPPORTED if the BLE stack is running.
  */
int RF24Radio::send(FrameBuffer *buffer)
{
    DMESG("send");

    if (buffer == NULL)
        return DEVICE_INVALID_PARAMETER;

    if (buffer->length > RF24_RADIO_MAX_PACKET_SIZE + RF24_RADIO_HEADER_SIZE - 1)
        return DEVICE_INVALID_PARAMETER;

    // Firstly, disable the Radio interrupt. We want to wait until the transmission completes.
    gpio_set_irq_enabled(IRQ_PIN, GPIO_IRQ_EDGE_FALL, false);

    // Turn off the transceiver.
    radio.stopListening();

    // Configure the radio to send the buffer provided.
    ADDRESS[0] = buffer->group;
    radio.openWritingPipe(ADDRESS);

    // Start transmission and wait for end of packet.
    bool report = radio.write(buffer->payload, buffer->length, false);

    // Return the radio to using the default receive buffer
    ADDRESS[0] = this->group;
    radio.openReadingPipe(0, ADDRESS);
    radio.startListening();

    // Re-enable the Radio interrupt.
    gpio_set_irq_enabled_with_callback(IRQ_PIN, GPIO_IRQ_EDGE_FALL, true, &interruptHandler);

    return DEVICE_OK;
}

ManagedBuffer RF24Radio::recvBuffer()
{
    return datagram.recv();
}

int RF24Radio::sendBuffer(ManagedBuffer b)
{
    return datagram.send(b);
}
