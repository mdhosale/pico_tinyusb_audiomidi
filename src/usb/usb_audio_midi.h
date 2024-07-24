/*
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

/*
 for tiny USB - see https://diyelectromusic.com/2022/10/04/getting-started-with-the-raspberry-pi-pico-c-c-sdk-and-tinyusb-midi/
 for DMA see:https://github.com/vha3/Hunter-Adams-RP2040-Demos/blob/master/Audio/f_Audio_FFT/fft.c
 GPIO 26/ADC0 (pin 31)-> Analog in
 3.3v (pin 36) -> VCC on microphone board
 GND (pin 38)  -> GND on microphone board
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "pico/binary_info.h"

#include "bsp/board.h"
#include "tusb.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

#ifndef AUDIO_SAMPLE_RATE
#define AUDIO_SAMPLE_RATE   48000
#endif

//ADC
#define ADC_NUM 0
#define ADC_PIN (26 + ADC_NUM)
//#define ADC_VREF 3.3
//#define ADC_RANGE (1 << 12)
//#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))

// ADC clock rate (unmutable!)
#define ADCCLK 48000000.0

#define NUM_DMA_SAMPLES 8//CFG_TUD_AUDIO_EP_SZ_IN/2

// DMA channels for sampling ADC
int sample_chan ;
int control_chan ;

// Here's where we'll have the DMA channel put ADC samples
uint16_t sample_array[NUM_DMA_SAMPLES] ;//same as USB

// Pointer to address of start of sample buffer
uint16_t * sample_address_pointer = &sample_array[0] ;

// Audio controls
// Current states
bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; 				          // +1 for master channel 0
uint16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; 					// +1 for master channel 0
uint32_t sampFreq;
uint8_t clkValid;

// Range states
audio_control_range_2_n_t(1) volumeRng[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX+1]; 			// Volume range state
audio_control_range_4_n_t(1) sampleFreqRng; 						// Sample frequency range state

// Audio test data
uint16_t usb_audio_buffer[CFG_TUD_AUDIO_EP_SZ_IN/2];
uint16_t startVal = 0;
size_t cnt = 0;

//ADC value
uint16_t adc_raw;

void configure_adc_capture();
void configure_dma_capture();
void initialize_USB();
void start_dma();

void audio_task(void);
void midi_task(void);

//--------------------------------------------------------------------+
// Initializers
//--------------------------------------------------------------------+
void configure_adc_capture(){
    //ADC
    ///////////////////////////////////////////////////////////////////////////////
    // ============================== ADC CONFIGURATION ==========================
    //////////////////////////////////////////////////////////////////////////////
    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(ADC_PIN);
    
    // Initialize the ADC harware
    // (resets it, enables the clock, spins until the hardware is ready)
    adc_init() ;
    
    // Select analog mux input (0...3 are GPIO 26, 27, 28, 29; 4 is temp sensor)
    adc_select_input(ADC_NUM) ;
    
    // Setup the FIFO
    adc_fifo_setup(
                   true,    // Write each completed conversion to the sample FIFO
                   true,    // Enable DMA data request (DREQ)
                   1,       // DREQ (and IRQ) asserted when at least 1 sample present
                   false,   // We won't see the ERR bit because of 8 bit reads; disable.
                   true     // Shift each sample to 8 bits when pushing to FIFO
                   );
    
    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock. This is setup
    // to grab a sample at 10kHz (48Mhz/10kHz - 1)
    adc_set_clkdiv(ADCCLK/AUDIO_SAMPLE_RATE);
}

void configure_dma_capture(){
    /////////////////////////////////////////////////////////////////////////////////
    // ============================== ADC DMA CONFIGURATION =========================
    /////////////////////////////////////////////////////////////////////////////////
    
    sample_chan = dma_claim_unused_channel(true);
    control_chan = dma_claim_unused_channel(true);
    
    // Channel configurations
    dma_channel_config c2 = dma_channel_get_default_config(sample_chan);
    dma_channel_config c3 = dma_channel_get_default_config(control_chan);
    
    
    // ADC SAMPLE CHANNEL
    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);
    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&c2, DREQ_ADC);
    // Configure the channel
    dma_channel_configure(sample_chan,
                          &c2,            // channel config
                          sample_array,   // dst
                          &adc_hw->fifo,  // src
                          NUM_DMA_SAMPLES,    // transfer count
                          false            // don't start immediately
                          );
    
    // CONTROL CHANNEL
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);      // 32-bit txfers
    channel_config_set_read_increment(&c3, false);                // no read incrementing
    channel_config_set_write_increment(&c3, false);               // no write incrementing
    channel_config_set_chain_to(&c3, sample_chan);                // chain to sample chan
    
    dma_channel_configure(
                          control_chan,                         // Channel to be configured
                          &c3,                                // The configuration we just created
                          &dma_hw->ch[sample_chan].write_addr,  // Write address (channel 0 read address)
                          &sample_address_pointer,                   // Read address (POINTER TO AN ADDRESS)
                          1,                                  // Number of transfers, in this case each is 4 byte
                          false                               // Don't start immediately.
                          );
    
}
void initialize_USB(){
    //USB
    board_init();
    
    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);
    
    // Init values
    sampFreq = AUDIO_SAMPLE_RATE;
    clkValid = 1;
    
    sampleFreqRng.wNumSubRanges = 1;
    sampleFreqRng.subrange[0].bMin = AUDIO_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bMax = AUDIO_SAMPLE_RATE;
    sampleFreqRng.subrange[0].bRes = 0;
}

void start_dma(){
    //DMA
    // Start the ADC channel
    dma_start_channel_mask((1u << sample_chan)) ;
    
    // Start the ADC
    adc_run(true) ;
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
    (void) rhport;
    (void) pBuff;
    
    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);
    
    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t ep = TU_U16_LOW(p_request->wIndex);
    
    (void) channelNum; (void) ctrlSel; (void) ep;
    
    return false; 	// Yet not implemented
}

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
    (void) rhport;
    (void) pBuff;
    
    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);
    
    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);
    
    (void) channelNum; (void) ctrlSel; (void) itf;
    
    return false; 	// Yet not implemented
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
    (void) rhport;
    
    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    
    (void) itf;
    
    // We do not support any set range requests here, only current value requests
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);
    
    // If request is for our feature unit
    if ( entityID == 2 )
    {
        switch ( ctrlSel )
        {
            case AUDIO_FU_CTRL_MUTE:
                // Request uses format layout 1
                TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));
                
                mute[channelNum] = ((audio_control_cur_1_t*) pBuff)->bCur;
                
                TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
                return true;
                
            case AUDIO_FU_CTRL_VOLUME:
                // Request uses format layout 2
                TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));
                
                volume[channelNum] = (uint16_t) ((audio_control_cur_2_t*) pBuff)->bCur;
                
                TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
                return true;
                
                // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
        }
    }
    return false;    // Yet not implemented
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
    (void) rhport;
    
    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t ep = TU_U16_LOW(p_request->wIndex);
    
    (void) channelNum; (void) ctrlSel; (void) ep;
    
    //	return tud_control_xfer(rhport, p_request, &tmp, 1);
    
    return false; 	// Yet not implemented
}

// Invoked when audio class specific get request received for an interface
bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
    (void) rhport;
    
    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t itf = TU_U16_LOW(p_request->wIndex);
    
    (void) channelNum; (void) ctrlSel; (void) itf;
    
    return false; 	// Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
    (void) rhport;
    
    // Page 91 in UAC2 specification
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    // uint8_t itf = TU_U16_LOW(p_request->wIndex); 			// Since we have only one audio function implemented, we do not need the itf value
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    
    // Input terminal (Microphone input)
    if (entityID == 1)
    {
        switch ( ctrlSel )
        {
            case AUDIO_TE_CTRL_CONNECTOR:
            {
                // The terminal connector control only has a get request with only the CUR attribute.
                audio_desc_channel_cluster_t ret;
                
                // Those are dummy values for now
                ret.bNrChannels = 1;
                ret.bmChannelConfig = 0;
                ret.iChannelNames = 0;
                
                TU_LOG2("    Get terminal connector\r\n");
                
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*) &ret, sizeof(ret));
            }
                break;
                
                // Unknown/Unsupported control selector
            default:
                TU_BREAKPOINT();
                return false;
        }
    }
    
    // Feature unit
    if (entityID == 2)
    {
        switch ( ctrlSel )
        {
            case AUDIO_FU_CTRL_MUTE:
                // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
                // There does not exist a range parameter block for mute
                TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
                return tud_control_xfer(rhport, p_request, &mute[channelNum], 1);
                
            case AUDIO_FU_CTRL_VOLUME:
                switch ( p_request->bRequest )
                {
                    case AUDIO_CS_REQ_CUR:
                        TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
                        return tud_control_xfer(rhport, p_request, &volume[channelNum], sizeof(volume[channelNum]));
                        
                    case AUDIO_CS_REQ_RANGE:
                        TU_LOG2("    Get Volume range of channel: %u\r\n", channelNum);
                        
                        // Copy values - only for testing - better is version below
                        audio_control_range_2_n_t(1)
                        ret;
                        
                        ret.wNumSubRanges = 1;
                        ret.subrange[0].bMin = -90;    // -90 dB
                        ret.subrange[0].bMax = 90;		// +90 dB
                        ret.subrange[0].bRes = 1; 		// 1 dB steps
                        
                        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*) &ret, sizeof(ret));
                        
                        // Unknown/Unsupported control
                    default:
                        TU_BREAKPOINT();
                        return false;
                }
                break;
                
                // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
        }
    }
    
    // Clock Source unit
    if ( entityID == 4 )
    {
        switch ( ctrlSel )
        {
            case AUDIO_CS_CTRL_SAM_FREQ:
                // channelNum is always zero in this case
                switch ( p_request->bRequest )
                {
                    case AUDIO_CS_REQ_CUR:
                        TU_LOG2("    Get Sample Freq.\r\n");
                        return tud_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));
                        
                    case AUDIO_CS_REQ_RANGE:
                        TU_LOG2("    Get Sample Freq. range\r\n");
                        return tud_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));
                        
                        // Unknown/Unsupported control
                    default:
                        TU_BREAKPOINT();
                        return false;
                }
                break;
                
            case AUDIO_CS_CTRL_CLK_VALID:
                // Only cur attribute exists for this request
                TU_LOG2("    Get Sample Freq. valid\r\n");
                return tud_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));
                
                // Unknown/Unsupported control
            default:
                TU_BREAKPOINT();
                return false;
        }
    }
    
    TU_LOG2("  Unsupported entity: %d\r\n", entityID);
    return false; 	// Yet not implemented
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) itf;
    (void) ep_in;
    (void) cur_alt_setting;
    
    tud_audio_write ((uint8_t *)usb_audio_buffer, CFG_TUD_AUDIO_EP_SZ_IN);
    
    return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void) rhport;
    (void) n_bytes_copied;
    (void) itf;
    (void) ep_in;
    (void) cur_alt_setting;
    
    return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
    (void) rhport;
    (void) p_request;
    startVal = 0;
    
    return true;
}
