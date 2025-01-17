/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include "usb_desc.h"
#ifdef AUDIO_INTERFACE

#define FEATURE_MAX_VOLUME 0xFF  // volume accepted from 0 to 0xFF

// Forward declarations for C++ classes
#ifdef __cplusplus
class AudioInputUSB;
class AudioOutputUSB;
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Shared configuration and buffers
extern void usb_audio_configure(void);
extern uint32_t usb_audio_sync_feedback;
extern uint8_t usb_audio_receive_setting;
extern uint8_t usb_audio_transmit_setting;
extern uint32_t feedback_accumulator;
extern volatile uint32_t usb_audio_underrun_count;
extern volatile uint32_t usb_audio_overrun_count;

// Buffer declarations
extern uint16_t usb_audio_transmit_buffer[AUDIO_TX_SIZE/2];
extern uint8_t usb_audio_sync_nbytes;
extern uint8_t usb_audio_sync_rshift;

// Transfer buffers
extern uint8_t rx_buffer[AUDIO_RX_SIZE];
extern uint8_t tx_buffer[AUDIO_TX_SIZE];

// Input/Output callbacks
extern void usb_audio_receive_callback(unsigned int len);
extern unsigned int usb_audio_transmit_callback(void);

// Feature control functions
extern int usb_audio_set_feature(void *stp, uint8_t *buf);
extern int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen);

#ifdef __cplusplus
}

// C++ structures and classes
struct setup_struct {
    union {
        struct {
            uint8_t bmRequestType;
            uint8_t bRequest;
            union {
                struct {
                    uint8_t bChannel;  // 0=main, 1=left, 2=right
                    uint8_t bCS;       // Control Selector
                };
                uint16_t wValue;
            };
            union {
                struct {
                    uint8_t bIfEp;     // type of entity
                    uint8_t bEntityId; // UnitID, TerminalID, etc.
                };
                uint16_t wIndex;
            };
            uint16_t wLength;
        };
    };
};

// Audio features structure
struct usb_audio_features_struct {
    int change;  // set to 1 when any value is changed
    int mute;    // 1=mute, 0=unmute
    int volume;  // volume from 0 to FEATURE_MAX_VOLUME, maybe should be float from 0.0 to 1.0
};

#include "AudioStream.h"

#define MAX_USB_CHANNELS 8  // Maximum supported channels

class AudioInputUSB : public AudioStream {
public:
    AudioInputUSB(void) : AudioStream(0, NULL) { begin(); }
    virtual void update(void);
    void begin(void);
    friend void usb_audio_receive_callback(unsigned int len);
    friend int usb_audio_set_feature(void *stp, uint8_t *buf);
    friend int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen);
    static struct usb_audio_features_struct features;
    float volume(void) {
        if (features.mute) return 0.0;
        return (float)(features.volume) * (1.0 / (float)FEATURE_MAX_VOLUME);
    }
private:
    static bool update_responsibility;
    static audio_block_t *incoming_left;
    static audio_block_t *incoming_right;
    static audio_block_t *ready_left;
    static audio_block_t *ready_right;
    static uint16_t incoming_count;
    static uint8_t receive_flag;
};

class AudioOutputUSB : public AudioStream {
public:
    AudioOutputUSB(uint8_t channels = AUDIO_CHANNELS) : AudioStream(channels, inputQueueArray) {
        begin(channels); 
    }
    virtual void update(void);
    void begin(uint8_t channels = AUDIO_CHANNELS);
    friend unsigned int usb_audio_transmit_callback(void);

    static uint8_t getChannelCount() { return num_channels; }
    
private:
    static bool update_responsibility;
    static const uint8_t BUFFER_COUNT = 4;
    static audio_block_t *buffer_channels[MAX_USB_CHANNELS][BUFFER_COUNT];
    static uint8_t num_channels;
    static volatile uint8_t write_index;
    static volatile uint8_t read_index;
    static volatile uint16_t buffer_offset;
    static volatile uint32_t underflow_count;
    static volatile uint32_t overflow_count;
    static volatile uint32_t last_log_ms;
    audio_block_t *inputQueueArray[MAX_USB_CHANNELS];
};

#endif // __cplusplus
#endif // AUDIO_INTERFACE
