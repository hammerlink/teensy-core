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
#include "debug/printf.h"

#ifdef AUDIO_INTERFACE

// Shared variables
uint8_t usb_audio_receive_setting = 0;
uint8_t usb_audio_transmit_setting = 0;
uint8_t usb_audio_sync_nbytes;
uint8_t usb_audio_sync_rshift;
uint32_t feedback_accumulator = 739875226; // 44.1 * 2^24
uint32_t usb_audio_sync_feedback __attribute__ ((aligned(32)));
volatile uint32_t usb_audio_underrun_count = 0;
volatile uint32_t usb_audio_overrun_count = 0;

// Transfer structures and buffers
static transfer_t rx_transfer __attribute__ ((used, aligned(32)));
static transfer_t sync_transfer __attribute__ ((used, aligned(32)));
static transfer_t tx_transfer __attribute__ ((used, aligned(32)));
DMAMEM uint8_t rx_buffer[AUDIO_RX_SIZE] __attribute__ ((aligned(32)));
DMAMEM uint8_t tx_buffer[AUDIO_TX_SIZE] __attribute__ ((aligned(32)));

// External declarations
extern volatile uint8_t usb_high_speed;


// Forward declarations
static void rx_event(transfer_t *t);
static void tx_event(transfer_t *t);
static void sync_event(transfer_t *t);

// Shared event handler for sync feedback
static void sync_event(transfer_t *t)
{
    usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
    usb_prepare_transfer(&sync_transfer, &usb_audio_sync_feedback, usb_audio_sync_nbytes, 0);
    arm_dcache_flush(&usb_audio_sync_feedback, usb_audio_sync_nbytes);
    usb_transmit(AUDIO_SYNC_ENDPOINT, &sync_transfer);
}

// Shared USB audio configuration
void usb_audio_configure(void)
{
    printf("usb_audio_configure\n");
    usb_audio_underrun_count = 0;
    usb_audio_overrun_count = 0;
    feedback_accumulator = 739875226; // 44.1 * 2^24
    
    // Configure based on USB speed
    if (usb_high_speed) {
        usb_audio_sync_nbytes = 4;
        usb_audio_sync_rshift = 8;
    } else {
        usb_audio_sync_nbytes = 3;
        usb_audio_sync_rshift = 10;
    }
    
    // Initialize transfers
    memset(&rx_transfer, 0, sizeof(rx_transfer));
    memset(&sync_transfer, 0, sizeof(sync_transfer));
    memset(&tx_transfer, 0, sizeof(tx_transfer));
    
    // Configure endpoints
    usb_config_rx_iso(AUDIO_RX_ENDPOINT, AUDIO_RX_SIZE, 1, rx_event);
    usb_config_tx_iso(AUDIO_SYNC_ENDPOINT, usb_audio_sync_nbytes, 1, sync_event);
    usb_config_tx_iso(AUDIO_TX_ENDPOINT, AUDIO_TX_SIZE, 1, tx_event);
    
    // Initialize events
    rx_event(NULL);
    sync_event(NULL);
    tx_event(NULL);
}

void AudioInputUSB::begin(void)
{
	incoming_count = 0;
	incoming_left = NULL;
	incoming_right = NULL;
	ready_left = NULL;
	ready_right = NULL;
	receive_flag = 0;
	// update_responsibility = update_setup();
	// TODO: update responsibility is tough, partly because the USB
	// interrupts aren't sychronous to the audio library block size,
	// but also because the PC may stop transmitting data, which
	// means we no longer get receive callbacks from usb.c
	update_responsibility = false;
}

static void copy_to_buffers(const uint32_t *src, int16_t *left, int16_t *right, unsigned int len)
{
	uint32_t *target = (uint32_t*) src + len; 
	while ((src < target) && (((uintptr_t) left & 0x02) != 0)) {
		uint32_t n = *src++;
		*left++ = n & 0xFFFF;
		*right++ = n >> 16;
	}

	while ((src < target - 2)) {
		uint32_t n1 = *src++;
		uint32_t n = *src++;
		*(uint32_t *)left = (n1 & 0xFFFF) | ((n & 0xFFFF) << 16);
		left+=2;
		*(uint32_t *)right = (n1 >> 16) | ((n & 0xFFFF0000)) ;
		right+=2;
	}

	while ((src < target)) {
		uint32_t n = *src++;
		*left++ = n & 0xFFFF;
		*right++ = n >> 16;
	}
}

// Called from the USB interrupt when an isochronous packet arrives
// we must completely remove it from the receive buffer before returning
//
#if 1
void usb_audio_receive_callback(unsigned int len)
{
	unsigned int count, avail;
	audio_block_t *left, *right;
	const uint32_t *data;

	AudioInputUSB::receive_flag = 1;
	len >>= 2; // 1 sample = 4 bytes: 2 left, 2 right
	data = (const uint32_t *)rx_buffer;

	count = AudioInputUSB::incoming_count;
	left = AudioInputUSB::incoming_left;
	right = AudioInputUSB::incoming_right;
	if (left == NULL) {
		left = AudioStream::allocate();
		if (left == NULL) return;
		AudioInputUSB::incoming_left = left;
	}
	if (right == NULL) {
		right = AudioStream::allocate();
		if (right == NULL) return;
		AudioInputUSB::incoming_right = right;
	}
	while (len > 0) {
		avail = AUDIO_BLOCK_SAMPLES - count;
		if (len < avail) {
			copy_to_buffers(data, left->data + count, right->data + count, len);
			AudioInputUSB::incoming_count = count + len;
			return;
		} else if (avail > 0) {
			copy_to_buffers(data, left->data + count, right->data + count, avail);
			data += avail;
			len -= avail;
			if (AudioInputUSB::ready_left || AudioInputUSB::ready_right) {
				// buffer overrun, PC sending too fast
				AudioInputUSB::incoming_count = count + avail;
				if (len > 0) {
					usb_audio_overrun_count++;
					printf("!");
					//serial_phex(len);
				}
				return;
			}
			send:
			AudioInputUSB::ready_left = left;
			AudioInputUSB::ready_right = right;
			//if (AudioInputUSB::update_responsibility) AudioStream::update_all();
			left = AudioStream::allocate();
			if (left == NULL) {
				AudioInputUSB::incoming_left = NULL;
				AudioInputUSB::incoming_right = NULL;
				AudioInputUSB::incoming_count = 0;
				return;
			}
			right = AudioStream::allocate();
			if (right == NULL) {
				AudioStream::release(left);
				AudioInputUSB::incoming_left = NULL;
				AudioInputUSB::incoming_right = NULL;
				AudioInputUSB::incoming_count = 0;
				return;
			}
			AudioInputUSB::incoming_left = left;
			AudioInputUSB::incoming_right = right;
			count = 0;
		} else {
			if (AudioInputUSB::ready_left || AudioInputUSB::ready_right) return;
			goto send; // recover from buffer overrun
		}
	}
	AudioInputUSB::incoming_count = count;
}
#endif

void AudioInputUSB::update(void)
{
	audio_block_t *left, *right;

	__disable_irq();
	left = ready_left;
	ready_left = NULL;
	right = ready_right;
	ready_right = NULL;
	uint16_t c = incoming_count;
	uint8_t f = receive_flag;
	receive_flag = 0;
	__enable_irq();
	if (f) {
		int diff = AUDIO_BLOCK_SAMPLES/2 - (int)c;
		feedback_accumulator += diff * 1;
		//uint32_t feedback = (feedback_accumulator >> 8) + diff * 100;
		//usb_audio_sync_feedback = feedback;

		//printf(diff >= 0 ? "." : "^");
	}
	//serial_phex(c);
	//serial_print(".");
	if (!left || !right) {
		usb_audio_underrun_count++;
		//printf("#"); // buffer underrun - PC sending too slow
		if (f) feedback_accumulator += 3500;
	}
	if (left) {
		transmit(left, 0);
		release(left);
	}
	if (right) {
		transmit(right, 1);
		release(right);
	}
}

















#if 1
#define MAX_USB_CHANNELS 8  // Maximum supported channels

bool AudioOutputUSB::update_responsibility;
audio_block_t * AudioOutputUSB::buffer_channels[MAX_USB_CHANNELS][BUFFER_COUNT];
uint8_t AudioOutputUSB::num_channels = 2; // Default to stereo, can be 2,4,6,8
volatile uint8_t AudioOutputUSB::write_index = 0;
volatile uint8_t AudioOutputUSB::read_index = 0;
volatile uint16_t AudioOutputUSB::buffer_offset = 0;
volatile uint32_t AudioOutputUSB::underflow_count = 0;
volatile uint32_t AudioOutputUSB::overflow_count = 0;
volatile uint32_t AudioOutputUSB::last_log_ms = 0;

/*DMAMEM*/ uint16_t usb_audio_transmit_buffer[AUDIO_TX_SIZE/2] __attribute__ ((used, aligned(32)));


static void tx_event(transfer_t *t)
{
	int len = usb_audio_transmit_callback();
	usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
	usb_prepare_transfer(&tx_transfer, usb_audio_transmit_buffer, len, 0);
	arm_dcache_flush_delete(usb_audio_transmit_buffer, len);
	usb_transmit(AUDIO_TX_ENDPOINT, &tx_transfer);
}


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

static void copy_from_buffers(uint32_t *dst, int16_t **channel_data, uint8_t num_channels, unsigned int len)
{
	
	while (len > 0) {
		uint32_t sample = 0;
		for (int ch = 0; ch < num_channels; ch += 2) {
			// Pack two channels into each 32-bit word
			if (ch == 0) {
				sample = (channel_data[ch+1][0] << 16) | (channel_data[ch][0] & 0xFFFF);
				*dst++ = sample;
			} else {
				sample = (channel_data[ch+1][0] << 16) | (channel_data[ch][0] & 0xFFFF);
				*dst++ = sample;
			}
			// Advance the channel data pointers
			channel_data[ch]++;
			channel_data[ch+1]++;
		}
		len--;
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


// Called from the USB interrupt when ready to transmit another
// isochronous packet.  If we place data into the transmit buffer,
// the return is the number of bytes.  Otherwise, return 0 means
// no data to transmit
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

int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen)
{
	struct setup_struct setup = *((struct setup_struct *)stp);
	if (setup.bmRequestType==0xA1) { // should check bRequest, bChannel, and UnitID
			if (setup.bCS==0x01) { // mute
				data[0] = AudioInputUSB::features.mute;  // 1=mute, 0=unmute
				*datalen = 1;
				return 1;
			}
			else if (setup.bCS==0x02) { // volume
				if (setup.bRequest==0x81) { // GET_CURR
					data[0] = AudioInputUSB::features.volume & 0xFF;
					data[1] = (AudioInputUSB::features.volume>>8) & 0xFF;
				}
				else if (setup.bRequest==0x82) { // GET_MIN
					//serial_print("vol get_min\n");
					data[0] = 0;     // min level is 0
					data[1] = 0;
				}
				else if (setup.bRequest==0x83) { // GET_MAX
					data[0] = FEATURE_MAX_VOLUME;  // max level, for range of 0 to MAX
					data[1] = 0;
				}
				else if (setup.bRequest==0x84) { // GET_RES
					data[0] = 1; // increment vol by by 1
					data[1] = 0;
				}
				else { // pass over SET_MEM, etc.
					return 0;
				}
				*datalen = 2;
				return 1;
			}
	}
	return 0;
}

int usb_audio_set_feature(void *stp, uint8_t *buf) 
{
	struct setup_struct setup = *((struct setup_struct *)stp);
	if (setup.bmRequestType==0x21) { // should check bRequest, bChannel and UnitID
			if (setup.bCS==0x01) { // mute
				if (setup.bRequest==0x01) { // SET_CUR
					AudioInputUSB::features.mute = buf[0]; // 1=mute,0=unmute
					AudioInputUSB::features.change = 1;
					return 1;
				}
			}
			else if (setup.bCS==0x02) { // volume
				if (setup.bRequest==0x01) { // SET_CUR
					AudioInputUSB::features.volume = buf[0];
					AudioInputUSB::features.change = 1;
					return 1;
				}
			}
	}
	return 0;
}


#endif // AUDIO_INTERFACE
