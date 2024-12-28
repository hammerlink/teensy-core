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

#include <Arduino.h>
#include "usb_dev.h"
#include "usb_audio.h"
#include "AudioStream.h"

// Static member definitions for AudioOutputUSB
bool AudioOutputUSB::update_responsibility = false;
audio_block_t * AudioOutputUSB::buffer_channels[MAX_USB_CHANNELS][AudioOutputUSB::BUFFER_COUNT];
uint8_t AudioOutputUSB::num_channels = 2;
volatile uint8_t AudioOutputUSB::write_index = 0;
volatile uint8_t AudioOutputUSB::read_index = 0;
volatile uint16_t AudioOutputUSB::buffer_offset = 0;
volatile uint32_t AudioOutputUSB::underflow_count = 0;
volatile uint32_t AudioOutputUSB::overflow_count = 0;
volatile uint32_t AudioOutputUSB::last_log_ms = 0;

// Output event handler
static void tx_event(transfer_t *t)
{
    if (t) {
        int len = AUDIO_TX_SIZE - ((tx_transfer.status >> 16) & 0x7FFF);
        printf("tx %u\n", len);
        usb_audio_transmit_callback();
    }
    usb_prepare_transfer(&tx_transfer, tx_buffer, AUDIO_TX_SIZE, 0);
    arm_dcache_flush(&tx_buffer, AUDIO_TX_SIZE);
    usb_transmit(AUDIO_TX_ENDPOINT, &tx_transfer);
}

// Helper function for copying data from output buffers
static void copy_from_buffers(uint32_t *dst, const int16_t *left, const int16_t *right, unsigned int len)
{
    const uint32_t *end = dst + len;
    while (dst < end) {
        *dst++ = (*left++) | (((uint32_t)(*right++)) << 16);
    }
}
#include "debug/printf.h"

#ifdef AUDIO_INTERFACE

void AudioOutputUSB::begin(uint8_t channels)
{
	update_responsibility = false;
	write_index = 0;
	read_index = 0;
	buffer_offset = 0;
	num_channels = (channels > 0 && channels <= MAX_USB_CHANNELS && (channels % 2 == 0)) ? channels : 2;
	
	for (int ch = 0; ch < MAX_USB_CHANNELS; ch++) {
		for (int i = 0; i < BUFFER_COUNT; i++) {
			buffer_channels[ch][i] = NULL;
		}
	}
}

void AudioOutputUSB::update(void)
{
	audio_block_t *blocks[MAX_USB_CHANNELS];

	// Receive blocks for all channels
	for (int ch = 0; ch < num_channels; ch++) {
		blocks[ch] = receiveWritable(ch);
	}
	
	if (usb_audio_transmit_setting == 0) {
		for (int ch = 0; ch < num_channels; ch++) {
			if (blocks[ch]) release(blocks[ch]);
		}
		__disable_irq();
		for (int ch = 0; ch < num_channels; ch++) {
			for (int i = 0; i < BUFFER_COUNT; i++) {
				if (buffer_channels[ch][i]) {
					release(buffer_channels[ch][i]);
					buffer_channels[ch][i] = NULL;
				}
			}
		}
		write_index = 0;
		read_index = 0;
		buffer_offset = 0;
		__enable_irq();
		return;
	}

	// Allocate missing blocks and initialize with silence
	for (int ch = 0; ch < num_channels; ch++) {
		if (blocks[ch] == NULL) {
			blocks[ch] = allocate();
			if (blocks[ch] == NULL) {
				// If allocation fails, release all previously allocated blocks
				for (int j = 0; j < ch; j++) {
					if (blocks[j]) release(blocks[j]);
				}
				return;
			}
			memset(blocks[ch]->data, 0, sizeof(blocks[ch]->data));
		}
	}

	__disable_irq();
	uint8_t next_write = (write_index + 1) % BUFFER_COUNT;
	
	if (next_write == read_index) {
		// Buffer full - overrun
		AudioOutputUSB::overflow_count++;
		// Release all channel buffers at read_index
		for (int ch = 0; ch < num_channels; ch++) {
			if (buffer_channels[ch][read_index]) {
				release(buffer_channels[ch][read_index]);
				buffer_channels[ch][read_index] = NULL;
			}
		}
		read_index = (read_index + 1) % BUFFER_COUNT;
		buffer_offset = 0;
	}
	
	// Store all channel buffers at write_index
	for (int ch = 0; ch < num_channels; ch++) {
		buffer_channels[ch][write_index] = blocks[ch];
	}
	write_index = next_write;
	__enable_irq();
}

unsigned int usb_audio_transmit_callback(void)
{
	static uint32_t count=5;
	uint32_t avail, num, target, len=0;
	uint32_t current_ms = millis();
	
	// Log stats every second
	if (current_ms - AudioOutputUSB::last_log_ms >= 1000) {
		Serial.print("USB Audio Stats - Underflows: ");
		Serial.print(AudioOutputUSB::underflow_count);
		Serial.print(", Overflows: ");
		Serial.println(AudioOutputUSB::overflow_count);
		AudioOutputUSB::underflow_count = 0;
		AudioOutputUSB::overflow_count = 0;
		AudioOutputUSB::last_log_ms = current_ms;
	}

	if (++count < 10) {   // TODO: dynamic adjust to match USB rate
		target = 44;
	} else {
		count = 0;
		target = 45;
	}

	while (len < target) {
		num = target - len;
		
		if (AudioOutputUSB::read_index == AudioOutputUSB::write_index) {
			// Buffer underrun - no data available
			AudioOutputUSB::underflow_count++;
			memset(usb_audio_transmit_buffer + len, 0, num * 4);
			break;
		}

		// Check if all channels have valid buffers
		bool valid_buffers = true;
		int16_t *channel_ptrs[MAX_USB_CHANNELS];
		for (int ch = 0; ch < AudioOutputUSB::num_channels; ch++) {
			audio_block_t *block = AudioOutputUSB::buffer_channels[ch][AudioOutputUSB::read_index];
			if (!block) {
				valid_buffers = false;
				break;
			}
			channel_ptrs[ch] = block->data + AudioOutputUSB::buffer_offset;
		}

		if (!valid_buffers) {
			// Invalid state - should never happen
			memset(usb_audio_transmit_buffer + len, 0, num * 4);
			break;
		}

		avail = AUDIO_BLOCK_SAMPLES - AudioOutputUSB::buffer_offset;
		if (num > avail) num = avail;

		copy_from_buffers((uint32_t *)usb_audio_transmit_buffer + len,
			channel_ptrs, AudioOutputUSB::num_channels, num);
		
		len += num;
		AudioOutputUSB::buffer_offset += num;

		if (AudioOutputUSB::buffer_offset >= AUDIO_BLOCK_SAMPLES) {
			// Release all channel buffers
			for (int ch = 0; ch < AudioOutputUSB::num_channels; ch++) {
				audio_block_t *block = AudioOutputUSB::buffer_channels[ch][AudioOutputUSB::read_index];
				if (block) {
					AudioStream::release(block);
					AudioOutputUSB::buffer_channels[ch][AudioOutputUSB::read_index] = NULL;
				}
			}
			AudioOutputUSB::read_index = (AudioOutputUSB::read_index + 1) % AudioOutputUSB::BUFFER_COUNT;
			AudioOutputUSB::buffer_offset = 0;
		}
	}
	return target * 4;
}

#endif
