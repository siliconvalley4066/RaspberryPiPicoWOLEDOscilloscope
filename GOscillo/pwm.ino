#define PWMPin 0

byte duty = 128; // duty ratio = duty/256
byte p_range = 0;
unsigned short count;
const long range_min[9] PROGMEM = {1, 32767, 32767, 32767, 32767, 32767, 32767, 32767, 32896};
const int range_div[9] PROGMEM = {1, 2, 4, 8, 16, 32, 64, 128, 255};
static uint slice_num;

#if 0
void pulse_init() {
  // Tell GPIO 0 and 1 they are allocated to the PWM
  gpio_set_function(0, GPIO_FUNC_PWM);
  gpio_set_function(1, GPIO_FUNC_PWM);
  
  // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
  slice_num = pwm_gpio_to_slice_num(0);
  
  // Set frequency of 10kHz 
  pwm_set_wrap(slice_num, 12499);
  pwm_set_clkdiv(slice_num, 1.0);
  // Set channel A Duty
  pwm_set_chan_level(slice_num, PWM_CHAN_A, 6250);
  // Set initial B Duty
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 3125);
  // Set the PWM running
  pwm_set_enabled(slice_num, true);
}

#else
double pulse_frq(void) {          // 7.48Hz <= freq <= 62.5MHz
  int divide = range_div[p_range];
  return(sys_clk / ((double)((long)count + 1) * (double)divide));
}

void set_pulse_frq(float freq) {  // 7.48Hz <= freq <= 62.5MHz
  if (freq > (float)(sys_clk / 2)) freq = sys_clk / 2;
  p_range = constrain(9 - int(10.0 - log(sys_clk / 32768.0 / freq)/log(2)), 0, 8);
  int divide = range_div[p_range];
  setCounter(divide);
  count = (float)sys_clk/freq/(float)divide - 1;
  pwm_set_wrap(slice_num, count);
  setduty();
}

void pulse_init() {
  int divide;
  p_range = constrain(p_range, 0, 8);
  divide = range_div[p_range];

  // Tell GPIO 0 and 1 they are allocated to the PWM
  gpio_set_function(0, GPIO_FUNC_PWM);
  // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
  slice_num = pwm_gpio_to_slice_num(0);
  setCounter(divide);           // set divider
  pwm_set_wrap(slice_num, count);
  pwm_set_chan_level(slice_num, PWM_CHAN_A, (unsigned int)(((long)count * duty) >> 8));
  // Set the PWM running
  pwm_set_enabled(slice_num, true);
}
#endif

void update_frq(int diff) {
  int divide, fast;
  long newCount;

  if (abs(diff) > 3) {
    fast = 512;
  } else if (abs(diff) > 2) {
    fast = 128;
  } else if (abs(diff) > 1) {
    fast = 25;
  } else {
    fast = 1;
  }
  newCount = (long)count + fast * diff;

  if (newCount < range_min[p_range]) {
    if (p_range < 1) {
      newCount = 1;
    } else {
      --p_range;
      newCount = 65535;
    }
  } else if (newCount > 65535) {
    if (p_range < 8) {
      ++p_range;
      newCount = range_min[p_range];
    } else {
      newCount = 65535;
    }
  }
  divide = range_div[p_range];
  setCounter(divide);
  count = newCount;
  pwm_set_wrap(slice_num, count);
  setduty();
}

#ifndef NOLCD
void disp_pulse_frq(void) {
  float freq = pulse_frq();
  if (freq < 10.0) {
    display.print(freq, 5);
  } else if (freq < 100.0) {
    display.print(freq, 4);
  } else if (freq < 1000.0) {
    display.print(freq, 3);
  } else if (freq < 10000.0) {
    display.print(freq, 2);
  } else if (freq < 100000.0) {
    display.print(freq, 1);
  } else if (freq < 1000000.0) {
    display.print(freq * 1e-3, 2); display.print('k');
  } else if (freq < 10000000.0) {
    display.print(freq * 1e-6, 4); display.print('M');
  } else {
    display.print(freq * 1e-6, 3); display.print('M');
  }
  display.print("Hz");
}

void disp_pulse_dty(void) {
  static bool sp = true;
  float fduty = duty*100.0/256.0;
  display.print(fduty, 1); display.print('%');
  if (fduty < 9.95) {
    if (sp) {
      display.print(' ');
      sp = false;
    }
  } else {
    sp = true;
  }
}
#endif

void setCounter(int divide) {
  if (divide == 1) {
    pwm_set_clkdiv(slice_num, 1.0);   // ck/1 > 1907.3Hz
  } else if (divide == 2) {
    pwm_set_clkdiv(slice_num, 2.0);   // ck/2 > 953.7Hz
  } else if (divide == 4) {
    pwm_set_clkdiv(slice_num, 4.0);   // ck/4 > 476.8Hz
  } else if (divide == 8) {
    pwm_set_clkdiv(slice_num, 8.0);   // ck/8 > 238.4Hz
  } else if (divide == 16) {
    pwm_set_clkdiv(slice_num, 16.0);  // ck/16 > 119.2Hz
  } else if (divide == 32) {
    pwm_set_clkdiv(slice_num, 32.0);  // ck/32 > 59.6Hz
  } else if (divide == 64) {
    pwm_set_clkdiv(slice_num, 64.0);  // ck/64 > 29.8Hz
  } else if (divide == 128) {
    pwm_set_clkdiv(slice_num, 128.0); // ck/128 > 14.9Hz
  } else {
    pwm_set_clkdiv(slice_num, 255.0); // slow clock > 7.48Hz
  }
}

void pulse_start(void) {
  pwm_set_enabled(slice_num, true);
  setCounter(range_div[p_range]); // start clock of pulse generator
}

void pulse_close(void) {
  setCounter(0);              // stop clock of pulse generator
  pwm_set_enabled(slice_num, false);
}

void setduty(void) {
  pwm_set_chan_level(slice_num, PWM_CHAN_A, (unsigned int)(((long)count * duty + 128) >> 8));
}
