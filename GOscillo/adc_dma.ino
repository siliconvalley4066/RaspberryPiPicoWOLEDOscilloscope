// Sample from the ADC continuously at a particular sample rate
// and then compute an FFT over the data
//
// much of this code is from pico-examples/adc/dma_capture/dma_capture.c
// the rest is written by Alex Wulff (www.AlexWulff.com)

#include "hardware/adc.h"
#include "hardware/dma.h"

// set this to determine sample rate
// 0     = 500,000 Hz
// 120   = 400,000 Hz
// 192   = 250,000 Hz
// 240   = 200,000 Hz
// 480   = 100,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define CLOCK_DIV 0
#define FSAMP 500000

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL0 0
#define CAPTURE_CHANNEL1 1

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly

// globals
dma_channel_config cfg;
uint dma_chan;

void sample_2us() {
  byte ch;
  uint16_t *p;
  if (orate > RATE_DMA)
    dmaadc_setup(0);
  else if (orate != rate)
    adc_set_clkdiv(0);
  orate = rate;
  if (ch0_mode == MODE_OFF && ch1_mode != MODE_OFF) {
    ch = ad_ch1;
    p = cap_buf1;
    adc_select_input(CAPTURE_CHANNEL1);
  } else {
    ch = ad_ch0;
    p = cap_buf;
    adc_select_input(CAPTURE_CHANNEL0);
  }
  adc_set_round_robin(0);
  sample_dma(p);
  scaleDataArray(ch, trigger_point());
}

void sample_4us() {
  int t;
  float clock_div;
  if (rate == 4) {        // dual channel 10us sampling (100ksps)
    clock_div = 240;
  } else {                // dual channel 4us sampling (250ksps)
    clock_div = 0;
  }
  if (orate > RATE_DMA)
    dmaadc_setup(clock_div);
  else if (orate != rate)
    adc_set_clkdiv(clock_div);
  orate = rate;
  adc_select_input(CAPTURE_CHANNEL0);
  adc_set_round_robin(0x3);
  sample_dma(cap_buf);
  split_capture();
  t = trigger_point();
  scaleDataArray(ad_ch0, t);
  scaleDataArray(ad_ch1, t);
}

void sample_dma(uint16_t *capture_buf) {
//  adc_run(false);
      
  dma_channel_configure(dma_chan, &cfg,
      capture_buf,    // dst
      &adc_hw->fifo,  // src
      NSAMP,          // transfer count
      true            // start immediately
      );

  adc_run(true);

  // Once DMA finishes, stop any new conversions from starting, and clean up
  // the FIFO in case the ADC was still mid-conversion.
  dma_channel_wait_for_finish_blocking(dma_chan);
  adc_run(false);
  adc_fifo_drain();
}

void dmaadc_setup(float clock_div) {
  // Make sure GPIO is high-impedance, no pullups etc
  adc_gpio_init(26 + CAPTURE_CHANNEL0);
  adc_gpio_init(26 + CAPTURE_CHANNEL1);

  adc_init();
  adc_select_input(CAPTURE_CHANNEL0);
  adc_fifo_setup(
     true,    // Write each completed conversion to the sample FIFO
     true,    // Enable DMA data request (DREQ)
     1,       // DREQ (and IRQ) asserted when at least 1 sample present
     false,   // We won't see the ERR bit because of 8 bit reads; disable.
     false    // do not Shift each sample to 8 bits when pushing to FIFO
     );

  // set sample rate
  adc_set_clkdiv(clock_div);

  sleep_ms(100);
  // Set up the DMA to start transferring data as soon as it appears in FIFO
  dma_chan = dma_claim_unused_channel(true);
  cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);
}

void split_capture() {
  for (int i = 0; i < NSAMP/2; ++i){
    cap_buf1[i] = cap_buf[i+i+1];
    cap_buf[i] = cap_buf[i+i];
  }
}

int trigger_point() {
  int trigger_ad, i;
  uint16_t *cap;

  if (trig_ch == ad_ch1) {
    trigger_ad = advalue(trig_lv, VREF[range1], ch1_mode, ch1_off);
    cap = cap_buf1;
  } else {
    trigger_ad = advalue(trig_lv, VREF[range0], ch0_mode, ch0_off);
    cap = cap_buf;
  }
  for (i = 0; i < (NSAMP/2 - SAMPLES); ++i) {
    if (trig_edge == TRIG_E_UP) {
      if (cap[i] < trigger_ad && cap[i+1] > trigger_ad)
        break;
    } else {
      if (cap[i] > trigger_ad && cap[i+1] < trigger_ad)
        break;
    }
  }
  return i;
}
