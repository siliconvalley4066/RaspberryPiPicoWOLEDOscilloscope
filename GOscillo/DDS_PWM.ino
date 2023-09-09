/*
 * DDS Sine Generator mit ATMEGS 168
 * Timer2 generates the  31250 KHz Clock Interrupt
 *
 * KHM 2009 /  Martin Nawrath
 * Kunsthochschule fuer Medien Koeln
 * Academy of Media Arts Cologne
 */

#include "hardware/pwm.h"

extern const unsigned char sine256[], saw256[], revsaw256[], triangle[], rect256[];
extern const unsigned char pulse20[], pulse10[], pulse05[], delta[], noise[];
extern const unsigned char gaussian_noise[], ecg[], sinc5[], sinc10[], sinc20[];
extern const unsigned char sine2harmonic[], sine3harmonic[], choppedsine[];
extern const unsigned char sinabs[], trapezoid[], step2[], step4[], chainsaw[];
unsigned char *wp;
const unsigned char * wavetable[] PROGMEM = {sine256, saw256, revsaw256, triangle, rect256,
  pulse20, pulse10, pulse05, delta, noise, gaussian_noise, ecg, sinc5, sinc10, sinc20,
  sine2harmonic, sine3harmonic, choppedsine, sinabs, trapezoid, step2, step4, chainsaw};
const char Wavename[][5] PROGMEM = {"Sine", "Saw", "RSaw", "Tri", "Rect",
  "PL20", "PL10", "PL05", "Dlta", "Nois", "GNoi", "ECG", "Snc1", "Snc2", "Snc3",
  "Sin2", "Sin3", "CSin", "Sabs", "Trpz", "Stp2", "Stp4", "Csaw"};
const byte wave_num = (sizeof(wavetable) / sizeof(&sine256));
long ifreq = 23841; // frequency * 100 for 0.01Hz resolution
byte wave_id = 0;
static word slice_num2; // GP2 PWM slice number
#define DDSPin 2

// const double refclk=61035.15625; // 61.03515625kHz at 125MHz
double refclk=61035.15625;          // System clock is typically 125MHz, eventually 133MHz

// variables used inside interrupt service declared as voilatile
volatile byte icnt;             // var inside interrupt
volatile unsigned long phaccu;  // pahse accumulator
volatile unsigned long tword_m; // dds tuning word m

void dds_setup() {
//  pinMode(DDSPin, OUTPUT);      // GPIO2= PWM  output / frequency output
  refclk = sys_clk / (double) (8*256);
  Setup_timer2();
  tword_m=pow(2,32)*ifreq*0.01/refclk; // calulate DDS new tuning word
  wp = (unsigned char *) wavetable[wave_id];
}

void dds_close() {
//  pinMode(DDSPin, INPUT_PULLUP);  // GPIO2= PWM output / frequency output
  pwm_set_enabled(slice_num2, false);
}

void dds_set_freq() {
  double dfreq;
  dfreq = (double)ifreq*0.01;     // adjust output frequency
  tword_m=pow(2,32)*dfreq/refclk; // calulate DDS new tuning word
}

void rotate_wave(bool fwd) {
  if (fwd) {
    wave_id = (wave_id + 1) % wave_num;
  } else {
    if (wave_id > 0) --wave_id;
    else wave_id = wave_num - 1;
  }
  wp = (unsigned char *) wavetable[wave_id];
}

void set_wave(int id) {
  wave_id = id;
  wp = (unsigned char *) wavetable[wave_id];
}

//******************************************************************
// timer2 setup
// set prscaler to 8, PWM mode, 125000000/8/256 = 61035.15625 Hz clock
void Setup_timer2() {
    gpio_set_function(2, GPIO_FUNC_PWM);    // GPIO2 PWM
    slice_num2  = pwm_gpio_to_slice_num(2); // PWM slice number
    pwm_clear_irq(slice_num2);
    pwm_set_irq_enabled(slice_num2, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwmISR);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    pwm_set_clkdiv(slice_num2, 8);          // 15.625MHz
    pwm_set_wrap(slice_num2, 255);          // max count 61.035kHz sampling
    pwm_set_chan_level(slice_num2, PWM_CHAN_A, 128);  // duty50%
    pwm_set_enabled(slice_num2, true);      // PWM start
}

//******************************************************************
// Timer2 Interrupt Service at 61.035 KHz = 16uSec
// this is the timebase REFCLOCK for the DDS generator
// FOUT = (M (REFCLK)) / (2 exp 32)
// runtime : ? microseconds ( inclusive push and pop)
void pwmISR(void) {
  pwm_clear_irq(slice_num2);
  phaccu=phaccu+tword_m;  // soft DDS, phase accu with 32 bits
  icnt=phaccu >> 24;      // use upper 8 bits for phase accu as frequency information
                          // read value fron ROM sine table and send to PWM DAC
  pwm_set_chan_level(slice_num2, PWM_CHAN_A, wp[icnt]); // set level
}

void update_ifrq(long diff) {
  long newFreq;
  int fast;
  if (diff != 0) {
    if (abs(diff) > 3) {
      fast = ifreq / 40;
    } else if (abs(diff) > 2) {
      fast = ifreq / 300;
    } else if (abs(diff) > 1) {
      fast = 25;
    } else {
      fast = 1;
    }
    if (fast < 1) fast = 1;
    newFreq = ifreq + fast * diff;
  } else {
    newFreq = ifreq;
  }
  newFreq = constrain(newFreq, 1, 999999);
  if (newFreq != ifreq) {
    ifreq = newFreq;
    dds_set_freq();
  }
}

void disp_dds_freq(void) {
  display.setTextColor(TXTCOLOR, BGCOLOR);
  display.setCursor(72, 56);
  display.print((double)ifreq * 0.01, 2); display.print("Hz");
}

void disp_dds_wave(void) {
  display.print(Wavename[wave_id]); 
}
