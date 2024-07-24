/*
 Mark-David Hosale
 July 24 2024

  * The MIT License (MIT)
  * Based on code by Reinhard Panhuber, in pico_sdk/lib/tinyusb/examples/device/audio_test
  * On code by Hunter Adams (for the ADC DMA code),
  * Hunter-Adams-RP2040-Demos/Audio/f_Audio_FFT/fft.c (https://github.com/vha3/Hunter-Adams-RP2040-Demos/blob/master/Audio/f_Audio_FFT/fft.c)
  * And for a guide on the USB MIDI Code - see https://diyelectromusic.com/2022/10/04/getting-started-with-the-raspberry-pi-pico-c-c-sdk-and-tinyusb-midi/
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  * THE SOFTWARE.
  *
  */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "usb_audio_midi.h"

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500
};


static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void audio_task(void);
void midi_task(void);

/*------------- MAIN -------------*/
int main(void)
{
    stdio_init_all();
    
    configure_adc_capture();
    
    configure_dma_capture();
    
    initialize_USB();
    
    start_dma();
    
    while (1)
    {
        tud_task(); // tinyusb device task
        audio_task();
        midi_task();
        led_blinking_task();
    }
    return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
    blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    blink_interval_ms = BLINK_MOUNTED;
}


//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
    static uint32_t start_ms = 0;
    static bool led_state = false;
    
    // Blink every interval ms
    if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
    start_ms += blink_interval_ms;
    
    board_led_write(led_state);
    led_state = 1 - led_state; // toggle
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void)
{
    // Yet to be filled - e.g. put meas data into TX FIFOs etc.
    // asm("nop");
    //    for (size_t cnt = 0; cnt < CFG_TUD_AUDIO_EP_SZ_IN/2; cnt++)
    //    {
    //        usb_audio_buffer[cnt] = startVal++;
    //    }
    
    
    // Wait for NUM_SAMPLES samples to be gathered
    // Measure wait time with timer. THIS IS BLOCKING
    dma_channel_wait_for_finish_blocking(sample_chan);
    adc_run(false) ;
    adc_fifo_drain() ;

    
    for (int i = 0; i<NUM_DMA_SAMPLES; i++) {
        usb_audio_buffer[cnt] = sample_array[0];
        if(cnt < CFG_TUD_AUDIO_EP_SZ_IN/2){
            cnt++;
        } else {
            cnt = 0;
        }
    }
    
    // Restart the sample channel, now that we have our copy of the samples
    dma_channel_start(control_chan) ;
    
    adc_run(true) ;
}

//--------------------------------------------------------------------+
// MIDI Task
//--------------------------------------------------------------------+

// Variable that holds the current position in the sequence.
uint32_t note_pos = 0;

// Store example melody as an array of note values
uint8_t note_sequence[] =
{
  74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85,88,92,97,100,97,92,88,85,81,78,
  74,69,66,62,57,62,66,69,74,78,81,86,90,93,97,102,97,93,90,85,81,78,73,68,64,61,
  56,61,64,68,74,78,81,86,90,93,98,102
};

void midi_task(void)
{
  static uint32_t start_ms = 0;

  uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
  uint8_t const channel   = 0; // 0 for channel 1

  // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be read
  // (possibly just discarded) to avoid the sender blocking in IO
  uint8_t packet[4];
    while ( tud_midi_available() ){
        tud_midi_packet_read(packet);
        //echo
        tud_midi_stream_write(cable_num, packet, 3);
    }
    
  // send note periodically
  if (board_millis() - start_ms < 286) return; // not enough time
  start_ms += 286;

  // Previous positions in the note sequence.
  int previous = (int) (note_pos - 1);

  // If we currently are at position 0, set the
  // previous position to the last note in the sequence.
  if (previous < 0) previous = sizeof(note_sequence) - 1;

  // Send Note On for current position at full velocity (127) on channel 1.
  uint8_t note_on[3] = { 0x90 | channel, note_sequence[note_pos], 127 };
  tud_midi_stream_write(cable_num, note_on, 3);

  // Send Note Off for previous note.
  uint8_t note_off[3] = { 0x80 | channel, note_sequence[previous], 0};
  tud_midi_stream_write(cable_num, note_off, 3);

  // Increment position
  note_pos++;

  // If we are at the end of the sequence, start over.
  if (note_pos >= sizeof(note_sequence)) note_pos = 0;
}
