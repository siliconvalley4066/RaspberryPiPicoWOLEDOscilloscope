/*
 * Raspberry Pi Pico Oscilloscope using a 128x64 OLED Version 1.25
 * The max realtime sampling rates are 250ksps with 2 channels and 500ksps with a channel.
 * + Pulse Generator
 * + PWM DDS Function Generator (23 waveforms)
 * + Frequency Counter (kHz)
 * Copyright (c) 2023, Siliconvalley4066
 */
/*
 * Arduino Oscilloscope using a graphic LCD
 * The max sampling rates are 4.3ksps with 2 channels and 8.6ksps with a channel.
 * Copyright (c) 2009, Noriaki Mitsunaga
 */

#include <Adafruit_GFX.h>
#include "hardware/clocks.h"
#include "hardware/adc.h"
#define GPIN1 (22)

#define BUTTON5DIR
#define DISPLAY_IS_SSD1306
#define SCREEN_WIDTH   128              // OLED display width
#define SCREEN_HEIGHT   64              // OLED display height
#define OLED_RESET     -1     // Reset pin # (or -1 if sharing Arduino reset pin)
#ifdef DISPLAY_IS_SSD1306
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#else
#include <Adafruit_SH110X.h>
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define WHITE 1
#define BLACK 0
#endif
#ifndef ARDUINO_ARCH_MBED_RP2040
#define EEPROM_START 0
#endif
#ifdef EEPROM_START
#include <EEPROM.h>
#endif
#include "arduinoFFT.h"
#define FFT_N 128
arduinoFFT FFT = arduinoFFT();  // Create FFT object

#define txtLINE0   0
#define txtLINE1   8
#define txtLINE2   16
#define txtLINE3   24
#define txtLINE4   32

float waveFreq;                // frequency (Hz)
float waveDuty;                // duty ratio (%)
int dataMin;                   // buffer minimum value (smallest=0)
int dataMax;                   //        maximum value (largest=1023)
int dataAve;                   // 10 x average value (use 10x value to keep accuracy. so, max=10230)
int saveTimer;                 // remaining time for saving EEPROM
int timeExec;                  // approx. execution time of current range setting (ms)
extern byte duty;
extern byte p_range;
extern unsigned short count;
extern long ifreq;
extern byte wave_id;

const int LCD_WIDTH = 128;
const int LCD_HEIGHT = 64;
const int LCD_YMAX = 60;
const int SAMPLES = 128;
const int NSAMP = 512;
const int DISPLNG = 100;
const int DOTS_DIV = 10;
const byte ad_ch0 = 26;                 // Analog pin for channel 0
const byte ad_ch1 = 27;                 // Analog pin for channel 1
const long VREF[] = {32, 64, 161, 322, 645}; // reference voltage 3.3V ->  32 :   1V/div range (100mV/dot)
                                        //                        ->  64 : 0.5V/div
                                        //                        -> 161 : 0.2V/div
                                        //                        -> 322 : 100mV/div
                                        //                        -> 644 :  50mV/div
//const int MILLIVOL_per_dot[] = {100, 50, 20, 10, 5}; // mV/dot
const int ac_offset[] PROGMEM = {104, -204, -388, -450, -481};
const int MODE_ON = 0;
const int MODE_INV = 1;
const int MODE_OFF = 2;
const char Modes[3][4] PROGMEM = {"ON", "INV", "OFF"};
const int TRIG_AUTO = 0;
const int TRIG_NORM = 1;
const int TRIG_SCAN = 2;
const int TRIG_ONE  = 3;
const char TRIG_Modes[4][5] PROGMEM = {"Auto", "Norm", "Scan", "One"};
const int TRIG_E_UP = 0;
const int TRIG_E_DN = 1;
#define RATE_MIN 0
#define RATE_MAX 19
#define RATE_NUM 20
#define RATE_DMA 3
#define RATE_DUAL 1
#define RATE_ROLL 15
#define ITEM_MAX 29
const char Rates[RATE_NUM][5] PROGMEM = {"20us", "40us", "50us", "100u", "120u", "200u", "500u", " 1ms", " 2ms", " 5ms", "10ms", "20ms", "50ms", "0.1s", "0.2s", "0.5s", " 1s ", " 2s ", " 5s ", " 10s"};
const unsigned long HREF[] PROGMEM = {20, 40, 50, 100, 120, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000, 2000000, 5000000, 10000000};
#define RANGE_MIN 0
#define RANGE_MAX 4
#define VRF 3.3
const char Ranges[5][5] PROGMEM = {" 1V ", "0.5V", "0.2V", "0.1V", "50mV"};
byte range0 = RANGE_MIN;
byte range1 = RANGE_MIN;
byte ch0_mode = MODE_ON, ch1_mode = MODE_ON, rate = 0, orate, wrate = 0;
byte trig_mode = TRIG_AUTO, trig_lv = 10, trig_edge = TRIG_E_UP, trig_ch = ad_ch0;
bool Start = true;  // Start sampling
byte item = 0;      // Default item
byte menu = 0;      // Default menu
short ch0_off = 0, ch1_off = 400;
byte data[2][SAMPLES];                  // keep the number of channels buffer
uint16_t cap_buf[NSAMP], cap_buf1[NSAMP/2];
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
uint16_t payload[SAMPLES*2];
#endif
byte odat00, odat01, odat10, odat11;    // old data buffer for erase
byte sample=0;                          // index for double buffer
bool fft_mode = false, pulse_mode = false, dds_mode = false, fcount_mode = false;
bool full_screen = false;
byte info_mode = 3; // Text information display mode
int trigger_ad;
float sys_clk;      // System clock is typically 125MHz, eventually 133MHz
volatile bool wfft, wdds;

#define LEFTPIN   18  // LEFT
#define RIGHTPIN  19  // RIGHT
#define UPPIN     20  // UP
#define DOWNPIN   21  // DOWN
#define CALPIN    0
#define CH0DCSW   16  // DC/AC switch ch0
#define CH1DCSW   17  // DC/AC switch ch1
//#define I2CSDA    4   // I2C SDA
//#define I2CSCL    5   // I2C SCL
#define BGCOLOR   BLACK
#define GRIDCOLOR WHITE
#define CH1COLOR  WHITE
#define CH2COLOR  WHITE
#define TXTCOLOR  WHITE

void setup(){
  pinMode(CH0DCSW, INPUT_PULLUP);   // CH1 DC/AC
  pinMode(CH1DCSW, INPUT_PULLUP);   // CH2 DC/AC
  pinMode(UPPIN, INPUT_PULLUP);     // up
  pinMode(DOWNPIN, INPUT_PULLUP);   // down
  pinMode(RIGHTPIN, INPUT_PULLUP);  // right
  pinMode(LEFTPIN, INPUT_PULLUP);   // left
  pinMode(LED_BUILTIN, OUTPUT);     // sets the digital pin as output
//  pinMode(CALPIN, OUTPUT);          // PWM out
//  pinMode(2, OUTPUT);               // DDS out
#ifdef DISPLAY_IS_SSD1306
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // select 3C or 3D (set your OLED I2C address)
#else
  display.begin(0x3c, true);                  // initialise the library
#endif

//  Serial.begin(115200);
#ifdef EEPROM_START
  EEPROM.begin(512);                    // set EEPROM size. Necessary for Raspberry Pi Pico
  loadEEPROM();                         // read last settings from EEPROM
#else
  set_default();
#endif
  menu = item >> 3;
  wfft = fft_mode;
  wdds = dds_mode;
  display.clearDisplay();
//  DrawGrid();
//  DrawText();
//  display.display();
  sys_clk = (float) clock_get_hz(clk_sys);  // identify 125MHz or 133MHz or else
  if (pulse_mode)
    pulse_init();                       // calibration pulse output
  if (dds_mode)
    dds_setup();
  orate = RATE_DMA + 1;                 // old rate befor change
  dmaadc_setup(0);                      // initial setup for DMA adc
  clock_configure_gpin(clk_gpout0, GPIN1, 1000, 1000);  // frequency count on GPIO22
}

byte lastsw = 255;
unsigned long vtime;

void CheckSW() {
  static unsigned long Millis = 0;
  unsigned long ms;
  byte sw;

  ms = millis();
  if ((ms - Millis)<200)
    return;
  Millis = ms;

  if (wrate != 0) {
    updown_rate(wrate);
    wrate = 0;
  }

/* SW10 Menu
 * SW9  CH1 range down
 * SW8  CH2 range down
 * SW7  TIME/DIV slow
 * SW6  TRIG_MODE down
 * SW5  Send
 * SW4  TRIG_MODE up
 * SW3  TIME/DIV fast
 * SW2  CH2 range up
 * SW1  CH1 range up
 * SW0  Start/Hold
 */
#ifdef BUTTON5DIR
  if (digitalRead(DOWNPIN) == LOW && digitalRead(LEFTPIN) == LOW) {
    sw = 11;    // both button press
  } else if (digitalRead(UPPIN) == LOW && digitalRead(RIGHTPIN) == LOW) {
    sw = 12;    // both button press
#else
  if (digitalRead(RIGHTPIN) == LOW && digitalRead(LEFTPIN) == LOW) {
    sw = 11;    // both button press
  } else if (digitalRead(UPPIN) == LOW && digitalRead(DOWNPIN) == LOW) {
    sw = 12;    // both button press
#endif
  } else if (digitalRead(DOWNPIN) == LOW) {
    sw = 10;    // down
  } else if (digitalRead(RIGHTPIN) == LOW) {
    sw = 3;     // right
  } else if (digitalRead(LEFTPIN) == LOW) {
    sw = 7;     // left
  } else if (digitalRead(UPPIN) == LOW) {
    sw = 0;     // up
  } else {
    lastsw = 255;
    return;
  }
  if (sw != lastsw)
    vtime = ms;
  saveTimer = 5000;     // set EEPROM save timer to 5 secnd
  if (sw == 12) {
    full_screen = !full_screen;
    display.fillRect(DISPLNG + 1,0,27,64, BGCOLOR); // clear text area that will be drawn below 
  } else {
    switch (menu) {
    case 0:
      menu0_sw(sw); 
      break;
    case 1:
      menu1_sw(sw); 
      break;
    case 2:
      menu2_sw(sw); 
      break;
    case 3:
      menu3_sw(sw); 
      break;
    default:
      break;
    }
    DrawText();
    display.display();
  }
  lastsw = sw;
}

void updown_ch0range(byte sw) {
  if (sw == 3) {        // CH0 RANGE +
    if (range0 > 0)
      range0 --;
  } else if (sw == 7) { // CH0 RANGE -
    if (range0 < RANGE_MAX)
      range0 ++;
  }
}

void updown_ch1range(byte sw) {
  if (sw == 3) {        // CH1 RANGE +
    if (range1 > 0)
      range1 --;
  } else if (sw == 7) { // CH1 RANGE -
    if (range1 < RANGE_MAX)
      range1 ++;
  }
}

void updown_rate(byte sw) {
  if (sw == 3) {        // RATE FAST
    orate = rate;
    if (rate > 0) rate --;
  } else if (sw == 7) { // RATE SLOW
    orate = rate;
    if (rate < RATE_MAX) rate ++;
    else rate = RATE_MAX;
  }
}

void menu0_sw(byte sw) {  
  switch (item) {
  case 0: // CH0 voltage range
    updown_ch0range(sw);
    break;
  case 1: // CH1 voltage range
    updown_ch1range(sw);
    break;
  case 2: // rate
    updown_rate(sw);
    break;
  case 3: // sampling mode
    break;
  case 4: // trigger mode
    if (sw == 3) {        // TRIG MODE +
      if (trig_mode < TRIG_ONE)
        trig_mode ++;
      else
        trig_mode = 0;
    } else if (sw == 7) { // TRIG MODE -
      if (trig_mode > 0)
        trig_mode --;
      else
        trig_mode = TRIG_ONE;
    }
    if (trig_mode != TRIG_ONE)
        Start = true;
    break;
  case 5: // trigger source and polarity
    if (sw == 3) {        // trigger + edge
      if (trig_edge == TRIG_E_UP)
        trig_edge = TRIG_E_DN;
      else
        trig_edge = TRIG_E_UP;
    } else if (sw == 7) { // trigger - channel
      if (trig_ch == ad_ch0)
        trig_ch = ad_ch1;
      else
        trig_ch = ad_ch0;
      set_trigger_ad();
    }
    break;
  case 6: // trigger level
    if (sw == 3) {        // trigger level +
      if (trig_lv < LCD_YMAX) {
        trig_lv ++;
        set_trigger_ad();
      }
    } else if (sw == 7) { // trigger level -
      if (trig_lv > 0) {
        trig_lv --;
        set_trigger_ad();
      }
    }
    break;
  case 7: // run / hold
    if (sw == 3 || sw == 7) {
      Start = !Start;
    }
    break;
  }
  menu_updown(sw);
}

void menu1_sw(byte sw) {  
  switch (item - 8) {
  case 1: // CH0 mode
    if (sw == 3) {        // CH0 + ON/INV
      if (ch0_mode == MODE_ON)
        ch0_mode = MODE_INV;
      else
        ch0_mode = MODE_ON;
    } else if (sw == 7) { // CH0 - ON/OFF
      if (ch0_mode == MODE_OFF)
        ch0_mode = MODE_ON;
      else
        ch0_mode = MODE_OFF;
    }
    break;
  case 2: // CH0 voltage range
    updown_ch0range(sw);
    break;
  case 3: // CH0 offset
    if (sw == 3) {        // offset +
      if (ch0_off < 4095)
        ch0_off += 4096/VREF[range0];
    } else if (sw == 7) { // offset -
      if (ch0_off > -4095)
        ch0_off -= 4096/VREF[range0];
    } else if (sw == 11) { // offset reset
      if (digitalRead(CH0DCSW) == LOW)    // DC/AC input
        ch0_off = ac_offset[range0];
      else
        ch0_off = 0;
    }
    break;
  case 5: // CH1 mode
    if (sw == 3) {        // CH1 + ON/INV
      if (ch1_mode == MODE_ON)
        ch1_mode = MODE_INV;
      else
        ch1_mode = MODE_ON;
    } else if (sw == 7) { // CH1 - ON/OFF
      if (ch1_mode == MODE_OFF)
        ch1_mode = MODE_ON;
      else
        ch1_mode = MODE_OFF;
    }
    break;
  case 6: // CH1 voltage range
    updown_ch1range(sw);
    break;
  case 7: // CH1 offset
    if (sw == 3) {        // offset +
      if (ch1_off < 4095)
        ch1_off += 4096/VREF[range1];
    } else if (sw == 7) { // offset -
      if (ch1_off > -4095)
        ch1_off -= 4096/VREF[range1];
    } else if (sw == 11) { // offset reset
      if (digitalRead(CH1DCSW) == LOW)    // DC/AC input
        ch1_off = ac_offset[range1];
      else
        ch1_off = 0;
    }
    break;
  }
  menu_updown(sw);
}

void menu2_sw(byte sw) {
  int diff;
  switch (item - 16) {
  case 0: // CH0 voltage range
    updown_ch0range(sw);
    break;
  case 1: // rate
    updown_rate(sw);
    break;
  case 2: // FFT mode
    if (sw == 3) {        // ON
      wfft = true;
    } else if (sw == 7) { // OFF
      wfft = false;
    }
    break;
  case 3: // Frequency and Duty display
    if (sw == 3) {        // ON
      info_mode |= 1;
    } else if (sw == 7) { // OFF
      info_mode &= ~1;
    }
    break;
  case 4: // Voltage display
    if (sw == 3) {        // ON
      info_mode |= 2;
    } else if (sw == 7) { // OFF
      info_mode &= ~2;
    }
    break;
  case 5: // PWM
    if (sw == 3) {        // +
      update_frq(0);
      pulse_start();
      pulse_mode = true;
    } else if (sw == 7) { // -
      pulse_close();
      pulse_mode = false;
    }
    break;
  case 6: // PWM Duty ratio
    diff = 1;
    if (sw == lastsw) {
      if (millis() - vtime > 5000) diff = 8;
    }
    if (sw == 3) {        // +
      if (pulse_mode) {
        if ((256 - duty) > diff) duty += diff;
      } else {
        pulse_start();
      }
      update_frq(0);
      pulse_mode = true;
    } else if (sw == 7) { // -
      if (pulse_mode) {
        if (duty > diff) duty -= diff;
      } else {
        pulse_start();
      }
      update_frq(0);
      pulse_mode = true;
    }
    break;
  case 7: // PWM Frequency
    diff = sw_accel(sw);
    if (sw == 3) {        // +
      if (pulse_mode)
        update_frq(-diff);
      else {
        update_frq(0);
        pulse_start();
      }
      pulse_mode = true;
    } else if (sw == 7) { // -
      if (pulse_mode)
        update_frq(diff);
      else {
        update_frq(0);
        pulse_start();
      }
      pulse_mode = true;
    }
    break;
  }
  menu_updown(sw);
}

void menu3_sw(byte sw) {
  char diff;
  switch (item - 24) {
  case 0: // CH0 voltage range
    updown_ch0range(sw);
    break;
  case 1: // rate
    updown_rate(sw);
    break;
  case 2: // DDS
    if (sw == 3) {        // +
      dds_setup();
      dds_mode = wdds = true;
    } else if (sw == 7) { // -
      dds_close();
      dds_mode = wdds = false;
    }
    break;
  case 3: // WAVE
    if (sw == 3) {        // +
      rotate_wave(true);
    } else if (sw == 7) { // -
      rotate_wave(false);
    }
    break;
  case 4: // FREQ
    diff = sw_accel(sw);
    if (sw == 3) {        // +
      update_ifrq(diff);
    } else if (sw == 7) { // -
      update_ifrq(-diff);
    }
    break;
  case 5: // Frequency Counter
    if (sw == 3 && rate <= RATE_MAX) {  // on
      fcount_mode = true;
    } else if (sw == 7) {               // off
      fcount_mode = false;
    }
    break;
  }
  menu_updown(sw);
}

void menu_updown(byte sw) {
  if (sw == 10) {       // MENU down SW
    increment_item();
  } else if (sw == 0) { // Menu up SW
    decrement_item();
  }
}

void increment_item() {
  ++item;
  if (item > ITEM_MAX) item = 0;
  if (menu == 0 && item == 3) item = 4;
  if (item < 16 || item > 18) wfft = false; // exit FFT mode
  menu = item >> 3;
}

void decrement_item() {
  if (item > 0) --item;
  else item = ITEM_MAX;
  if (menu == 0 && item == 3) item = 2;
  if (item < 16 || item > 18) wfft = false; // exit FFT mode
  menu = item >> 3;
}

byte sw_accel(byte sw) {
  char diff = 1;
  if (sw == lastsw) {
    unsigned long curtime = millis();
    if (curtime - vtime > 6000) diff = 4;
    else if (curtime - vtime > 4000) diff = 3;
    else if (curtime - vtime > 2000) diff = 2;
  }
  return (diff);
}

void DrawGrid() {
  int disp_leng;
  if (full_screen) disp_leng = SAMPLES;
  else disp_leng = DISPLNG;
  for (int x=0; x<=disp_leng; x += 2) { // Horizontal Line
    for (int y=0; y<=LCD_YMAX; y += DOTS_DIV) {
      display.drawPixel(x, y, GRIDCOLOR);
      CheckSW();
    }
  }
  for (int x=0; x<=disp_leng; x += DOTS_DIV ) { // Vertical Line
    for (int y=0; y<=LCD_YMAX; y += 2) {
      display.drawPixel(x, y, GRIDCOLOR);
      CheckSW();
    }
  }
}

void DrawText() {
  display.fillRect(DISPLNG+1,0,27,64, BGCOLOR); // clear text area that will be drawn below 

  switch (menu) {
  case 0:
    set_line_color(0);
    if (ch0_mode != MODE_OFF) {
      display_range(range0);
    } else {
      display.print("CH2"); display_ac(CH1DCSW);
    }
    set_line_color(1);
    if (ch1_mode != MODE_OFF && rate >= RATE_DUAL) {
      display_range(range1);
    } else {
      display.print("CH1"); display_ac(CH0DCSW);
    }
    set_line_color(2);
    display_rate();
    set_line_color(3);
    if (rate > RATE_DMA) display.print("real");
    else display.print("DMA");
    set_line_color(4);
    display_trig_mode();
    set_line_color(5);
    display.print(trig_ch == ad_ch0 ? "TG1" : "TG2"); 
    display.print(trig_edge == TRIG_E_UP ? char(0x18) : char(0x19)); 
    set_line_color(6);
    display.print("Tlev"); 
    set_line_color(7);
    display.print(Start ? "RUN" : "HOLD"); 
    break;
  case 1:
    set_line_color(0);
    display.print("CH1"); display_ac(CH0DCSW);
    set_line_color(1);
    display_mode(ch0_mode);
    set_line_color(2);
    display_range(range0);
    set_line_color(3);
    display.print("OFS1"); 
    set_line_color(4);
    display.print("CH2"); display_ac(CH1DCSW);
    set_line_color(5);
    display_mode(ch1_mode);
    set_line_color(6);
    display_range(range1);
    set_line_color(7);
    display.print("OFS2");
    break;
  case 2:
    set_line_color(0);
    display_range(range0);
    set_line_color(1);
    display_rate();
    set_line_color(2);
    if (!fft_mode) {
      display.print("FFT"); 
      set_line_color(3);
      display.print("FREQ"); 
      set_line_color(4);
      display.print("VOLT"); 
      set_line_color(5);
      display.print("PWM"); 
      set_line_color(6);
      display.print("DUTY"); 
      set_line_color(7);
      display.print("FREQ");
      if (pulse_mode && (item > 20 && item < 24))
        disp_pulse_frq();
    }
    break;
  case 3:
    set_line_color(0);
    display_range(range0);
    set_line_color(1);
    display_rate();
    set_line_color(2);
    display.print("DDS");
    set_line_color(3);
    disp_dds_wave();
    set_line_color(4);
    display.print("FREQ");
    if (dds_mode) disp_dds_freq();
    set_line_color(5);
    display.print("FCNT");
    fcount_disp();
    break;
  }
//  if (info_mode && Start) {
  if (info_mode) {
    dataAnalize();
    if (info_mode & 1)
      measure_frequency();
    if (info_mode & 2)
      measure_voltage();
  }
  if (!full_screen && !fft_mode)
    display.drawFastHLine(DISPLNG, LCD_YMAX - trig_lv, 3, GRIDCOLOR); // draw trig_lv tic
}

unsigned long fcount = 0;
//const double freq_ratio = 20000.0 / 19987.0;

void fcount_disp() {
  char s[10];
  if (!fcount_mode) return;
  fcount = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLKSRC_GPIN1);
//  fcount = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
//  fcount = fcount * freq_ratio; // compensate the ceramic osc
  display.setTextColor(TXTCOLOR, BGCOLOR);
  display.setCursor(127-6*sprintf(s, "%lukHz", fcount), 48);
  display.print(s);
}

void display_range(byte rng) {
  display.print(Ranges[rng]);
}

void display_rate(void) {
  display.print(Rates[rate]);
}

void display_mode(byte chmode) {
  display.print(Modes[chmode]); 
}

void display_trig_mode(void) {
  display.print(TRIG_Modes[trig_mode]); 
}

void display_ac(byte pin) {
  if (digitalRead(pin) == LOW) display.print('~');
}

void set_line_color(byte line) {
  if ((item & 0x7) == line) display.setTextColor(BGCOLOR, TXTCOLOR);  // highlight
  else display.setTextColor(TXTCOLOR, BGCOLOR);           // normal
  display.setCursor(DISPLNG + 3, 8 * line); // locate curser for printing text
}

void DrawGrid(int x) {
  if ((x % DOTS_DIV) == 0) {
    for (int y=0; y<=LCD_YMAX; y += 2)
      display.drawPixel(x, y, GRIDCOLOR);
  } else if ((x % 2) == 0)
    for (int y=0; y<=LCD_YMAX; y += DOTS_DIV)
      display.drawPixel(x, y, GRIDCOLOR);
}

void ClearAndDrawGraph() {
  int disp_leng;
  if (full_screen) disp_leng = SAMPLES-1;
  else disp_leng = DISPLNG-1;
  bool ch1_active = ch1_mode != MODE_OFF && rate >= RATE_DUAL;
#if 0
  for (int x=0; x<DISPLNG; x++) {
    display.drawPixel(x, LCD_YMAX-data[sample+0][x], CH1COLOR);
    display.drawPixel(x, LCD_YMAX-data[sample+1][x], CH2COLOR);
  }
#else
  for (int x=0; x<disp_leng; x++) {
    if (ch0_mode != MODE_OFF)
      display.drawLine(x, LCD_YMAX-data[sample+0][x], x+1, LCD_YMAX-data[sample+0][x+1], CH1COLOR);
    if (ch1_active)
      display.drawLine(x, LCD_YMAX-data[sample+1][x], x+1, LCD_YMAX-data[sample+1][x+1], CH2COLOR);
    CheckSW();
  }
#endif
}

void ClearAndDrawDot(int i) {
#if 0
  for (int x=0; x<DISPLNG; x++) {
    display.drawPixel(i, LCD_YMAX-odat01, BGCOLOR);
    display.drawPixel(i, LCD_YMAX-odat11, BGCOLOR);
    display.drawPixel(i, LCD_YMAX-data[sample+0][i], CH1COLOR);
    display.drawPixel(i, LCD_YMAX-data[sample+1][i], CH2COLOR);
  }
#else
  if (i < 1)
    return;
  if (ch0_mode != MODE_OFF) {
    display.drawLine(i-1, LCD_YMAX-odat00,   i, LCD_YMAX-odat01, BGCOLOR);
    display.drawLine(i-1, LCD_YMAX-data[0][i-1], i, LCD_YMAX-data[0][i], CH1COLOR);
  }
  if (ch1_mode != MODE_OFF) {
    display.drawLine(i-1, LCD_YMAX-odat10,   i, LCD_YMAX-odat11, BGCOLOR);
    display.drawLine(i-1, LCD_YMAX-data[1][i-1], i, LCD_YMAX-data[1][i], CH2COLOR);
  }
#endif
  DrawGrid(i);
}

void scaleDataArray(byte ad_ch, int trig_point)
{
  byte *pdata, ch_mode, range;
  short ch_off;
  uint16_t *idata, *qdata;
  long a, b;

  if (ad_ch == ad_ch1) {
    ch_off = ch1_off;
    ch_mode = ch1_mode;
    range = range1;
    pdata = data[1];
    idata = &cap_buf1[trig_point];
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
    qdata = payload+SAMPLES;
#endif
  } else {
    ch_off = ch0_off;
    ch_mode = ch0_mode;
    range = range0;
    pdata = data[0];
    idata = &cap_buf[trig_point];
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
    qdata = payload;
#endif
  }
  for (int i = 0; i < SAMPLES; i++) {
    a = ((*idata + ch_off) * VREF[range] + 2048) >> 12;
    if (a > LCD_YMAX) a = LCD_YMAX;
    else if (a < 0) a = 0;
    if (ch_mode == MODE_INV)
      a = LCD_YMAX - a;
    *pdata++ = (byte) a;
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
    b = ((*idata++ + ch_off) * VREF[range] + 40) / 80;
    if (b > 4095) b = 4095;
    else if (b < 0) b = 0;
    if (ch_mode == MODE_INV)
      b = 4095 - b;
    *qdata++ = (int16_t) b;
#else
    ++idata;
#endif
  }
}

byte adRead(byte ch, byte mode, int off, int i)
{
  if (ch == ad_ch1) {
    adc_select_input(1);
  } else {
    adc_select_input(0);
  }
  int16_t aa = adc_read();
  long a = (((long)aa+off)*VREF[ch == ad_ch0 ? range0 : range1]+2048) >> 12;
  if (a > LCD_YMAX) a = LCD_YMAX;
  else if (a < 0) a = 0;
  if (mode == MODE_INV)
    a = LCD_YMAX - a;
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
  long b = (((long)aa+off)*VREF[ch == ad_ch0 ? range0 : range1] + 40) / 80;
  if (b > 4095) b = 4095;
  else if (b < 0) b = 0;
  if (mode == MODE_INV)
    b = 4095 - b;
#endif
  if (ch == ad_ch1) {
    cap_buf1[i] = aa;
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
    payload[i+SAMPLES] = b;
#endif
  } else {
    cap_buf[i] = aa;
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
    payload[i] = b;
#endif
  }
  return a;
}

int advalue(int value, long vref, byte mode, int offs) {
  if (mode == MODE_INV)
    value = LCD_YMAX - value;
  return ((long)value << 12) / vref - offs;
}

void set_trigger_ad() {
  if (trig_ch == ad_ch0) {
    trigger_ad = advalue(trig_lv, VREF[range0], ch0_mode, ch0_off);
    adc_select_input(0);
  } else {
    trigger_ad = advalue(trig_lv, VREF[range1], ch1_mode, ch1_off);
    adc_select_input(1);
  }
}

void loop() {
  int oad, ad;
  unsigned long auto_time;

  timeExec = 100;
  digitalWrite(LED_BUILTIN, HIGH);
  if (rate > RATE_DMA) {
    adc_set_round_robin(0); // de-activate round robin
    set_trigger_ad();
    auto_time = pow(10, rate / 3);
    if (trig_mode != TRIG_SCAN) {
      unsigned long st = millis();
      oad = adc_read();
      for (;;) {
        ad = adc_read();

        if (trig_edge == TRIG_E_UP) {
          if (ad > trigger_ad && trigger_ad > oad)
            break;
        } else {
          if (ad < trigger_ad && trigger_ad < oad)
            break;
        }
        oad = ad;

        if (rate > 9)
          CheckSW();      // no need for fast sampling
        if (trig_mode == TRIG_SCAN)
          break;
        if (trig_mode == TRIG_AUTO && (millis() - st) > auto_time)
          break; 
      }
    }
  }
  
  // sample and draw depending on the sampling rate
  if (rate < RATE_ROLL && Start) {

    if (rate == 0) {        // DMA, channel 0 only 2us sampling (500ksps)
      sample_2us(ad_ch0);
    } else if (rate <= RATE_DMA) {  // DMA, dual channel 4us,5us,10us sampling (250ksps,200ksps,100ksps)
      sample_4us();
    } else if (rate > RATE_DMA && rate <= 8) {  // dual channel 12us, 20us, 50us, 100us, 200us sampling
      sample_dual_us(HREF[rate] / 10);
    } else {                // dual channel .5ms, 1ms, 2ms, 5ms, 10ms, 20ms sampling
      sample_dual_ms(HREF[rate] / 10);
    }
    digitalWrite(LED_BUILTIN, LOW);
    draw_screen();
  } else if (Start) { // 50ms - 1000ms sampling
    timeExec = 5000;
    static const unsigned long r_[] PROGMEM = {50000, 100000, 200000, 500000, 1000000};
    unsigned long r;
    int disp_leng;
    if (full_screen) disp_leng = SAMPLES;
    else disp_leng = DISPLNG;
//    unsigned long st0 = millis();
    unsigned long st = micros();
    for (int i=0; i<disp_leng; i ++) {
      r = r_[rate - RATE_ROLL];  // rate may be changed in loop
      while((st - micros())<r) {
        CheckSW();
        if (rate<RATE_ROLL)
          break;
      }
      if (rate<RATE_ROLL) { // sampling rate has been changed
        display.clearDisplay();
        break;
      }
      st += r;
      if (st - micros()>r)
          st = micros(); // sampling rate has been changed to shorter interval
      if (!Start) {
         i --;
         continue;
      }
      odat00 = odat01;      // save next previous data ch0
      odat10 = odat11;      // save next previous data ch1
      odat01 = data[0][i];  // save previous data ch0
      odat11 = data[1][i];  // save previous data ch1
      if (ch0_mode != MODE_OFF) data[0][i] = adRead(ad_ch0, ch0_mode, ch0_off, i);
      if (ch1_mode != MODE_OFF) data[1][i] = adRead(ad_ch1, ch1_mode, ch1_off, i);
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
      if (ch0_mode == MODE_OFF) payload[0] = -1;
      if (ch1_mode == MODE_OFF) payload[SAMPLES] = -1;
      rp2040.fifo.push_nb(1);   // notify Websocket server core
#endif
      ClearAndDrawDot(i);     
      display.display();  // 42ms
    }
    // Serial.println(millis()-st0);
    digitalWrite(LED_BUILTIN, LOW);
    DrawGrid();
    if (!full_screen) DrawText();
  } else {
    DrawText();
  }
  if (trig_mode == TRIG_ONE)
    Start = false;
  CheckSW();
#ifdef EEPROM_START
  saveEEPROM();                         // save settings to EEPROM if necessary
#endif
  if (wdds != dds_mode) {
    dds_mode = wdds;
    if (dds_mode) {
      dds_setup();
    } else {
      dds_close();
    }
  }
}

void draw_screen() {
  display.clearDisplay();
  if (wfft != fft_mode) {
    fft_mode = wfft;
  }
  if (fft_mode) {
    DrawText();
    plotFFT();
  } else {
    DrawGrid();
    ClearAndDrawGraph();
    if (!full_screen) DrawText();
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
    if (ch0_mode == MODE_OFF) payload[0] = -1;
    if (ch1_mode == MODE_OFF) payload[SAMPLES] = -1;
#endif
  }
  display.display();
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
  rp2040.fifo.push_nb(1);   // notify Websocket server core
  delay(10);    // wait Web task to send it (adhoc fix)
#endif
}

#define textINFO 54
void measure_frequency() {
  int x1, x2;
  freqDuty();
  display.setTextColor(TXTCOLOR, BGCOLOR);
  display.setCursor(textINFO, txtLINE0);
  if (waveFreq < 999.5)
    display.print(waveFreq);
  else if (waveFreq < 999999.5)
    display.print(waveFreq, 0);
  else {
    display.print(waveFreq/1000.0, 0);
    display.print('k');
  }
  display.print("Hz");
  if (fft_mode) return;
  display.setCursor(textINFO + 12, txtLINE1);
  display.print(waveDuty);  display.print('%');
}

void measure_voltage() {
  int x, dave, dmax, dmin;
  if (fft_mode) return;
  if (ch0_mode == MODE_INV) {
    dave = (LCD_YMAX) * 10 - dataAve;
    dmax = dataMin;
    dmin = dataMax;
  } else {
    dave = dataAve;
    dmax = dataMax;
    dmin = dataMin;
  }
  float vavr = VRF * ((dave * 409.6) / VREF[range0] - ch0_off) / 4096.0;
  float vmax = VRF * advalue(dmax, VREF[range0], ch0_mode, ch0_off) / 4096.0;
  float vmin = VRF * advalue(dmin, VREF[range0], ch0_mode, ch0_off) / 4096.0;
  display.setCursor(textINFO, txtLINE2);
  display.print("max");  display.print(vmax); if (vmax >= 0.0) display.print('V');
  display.setCursor(textINFO, txtLINE3);
  display.print("avr");  display.print(vavr); if (vavr >= 0.0) display.print('V');
  display.setCursor(textINFO, txtLINE4);
  display.print("min");  display.print(vmin); if (vmin >= 0.0) display.print('V');
}

void sample_dual_us(unsigned int r) { // dual channel. r > 67
  if (ch0_mode != MODE_OFF && ch1_mode == MODE_OFF) {
    adc_select_input(0);
    unsigned long st = micros();
    for (int i=0; i<SAMPLES; i ++) {
      while(micros() - st < r) ;
      cap_buf[i] = adc_read();
      st += r;
    }
    scaleDataArray(ad_ch0, 0);
  } else if (ch0_mode == MODE_OFF && ch1_mode != MODE_OFF) {
    adc_select_input(1);
    unsigned long st = micros();
    for (int i=0; i<SAMPLES; i ++) {
      while(micros() - st < r) ;
      cap_buf1[i] = adc_read();
      st += r;
    }
    scaleDataArray(ad_ch1, 0);
  } else {
    unsigned long st = micros();
    for (int i=0; i<SAMPLES; i ++) {
      while(micros() - st < r) ;
      adc_select_input(0);
      cap_buf[i] = adc_read();
      adc_select_input(1);
      cap_buf1[i] = adc_read();
      st += r;
    }
    scaleDataArray(ad_ch0, 0);
    scaleDataArray(ad_ch1, 0);
  }
}

void sample_dual_ms(unsigned int r) { // dual channel. r > 500
// .5ms, 1ms or 2ms sampling
  unsigned long st = micros();
  for (int i=0; i<SAMPLES; i ++) {
    while(micros() - st < r) ;
    st += r;
    if (ch0_mode != MODE_OFF) {
      adc_select_input(0);
      cap_buf[i] = adc_read();
    }
    if (ch1_mode != MODE_OFF) {
      adc_select_input(1);
      cap_buf1[i] = adc_read();
    }
  }
//  if (ch0_mode == MODE_OFF) memset(data[0], 0, SAMPLES);
//  if (ch1_mode == MODE_OFF) memset(data[1], 0, SAMPLES);
  scaleDataArray(ad_ch0, 0);
  scaleDataArray(ad_ch1, 0);
}

double vReal[FFT_N]; // Real part array, actually float type
double vImag[FFT_N]; // Imaginary part array

void plotFFT() {
  int ylim = 56;

  for (int i = 0; i < FFT_N; i++) {
    vReal[i] = cap_buf[i];
    vImag[i] = 0.0;
  }
  FFT.DCRemoval(vReal, FFT_N);
  FFT.Windowing(vReal, FFT_N, FFT_WIN_TYP_HANN, FFT_FORWARD); // Weigh data
  FFT.Compute(vReal, vImag, FFT_N, FFT_FORWARD);          // Compute FFT
  FFT.ComplexToMagnitude(vReal, vImag, FFT_N);            // Compute magnitudes
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
  payload[0] = 0;
#endif
  for (int i = 1; i < FFT_N/2; i++) {
    float db = log10(vReal[i]);
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
    payload[i] = constrain((int)(1024.0 * (db - 1.6)), 0, 4095);
#endif
    int dat = constrain((int)(15.0 * db - 20), 0, ylim);
    display.drawFastVLine(i * 2, ylim - dat, dat, TXTCOLOR);
  }
  draw_scale();
}

void draw_scale() {
  int ylim = 56;
  float fhref, nyquist;
  display.setTextColor(TXTCOLOR);
  display.setCursor(0, ylim); display.print("0Hz"); 
  fhref = (float)HREF[rate];
  nyquist = 5.0e6 / fhref; // Nyquist frequency
#ifdef ARDUINO_RASPBERRY_PI_PICO_W
  long inyquist = nyquist;
  payload[FFT_N/2] = (short) (inyquist / 1000);
  payload[FFT_N/2+1] = (short) (inyquist % 1000);
#endif
  if (nyquist > 999.0) {
    nyquist = nyquist / 1000.0;
    if (nyquist > 99.5) {
      display.setCursor(58, ylim); display.print(nyquist/2,0);display.print('k');
      display.setCursor(104, ylim); display.print(nyquist,0);
    } else if (nyquist > 9.95) {
      display.setCursor(58, ylim); display.print(nyquist/2,0);display.print('k');
      display.setCursor(110, ylim); display.print(nyquist,0);
    } else {
      display.setCursor(52, ylim); display.print(nyquist/2,1);display.print('k');
      display.setCursor(104, ylim); display.print(nyquist,1);
    }
    display.print('k');
  } else {
    display.setCursor(58, ylim); display.print(nyquist/2,0);
    display.setCursor(110, ylim); display.print(nyquist,0);
  }
}

#ifdef EEPROM_START
void saveEEPROM() {                   // Save the setting value in EEPROM after waiting a while after the button operation.
  int p = EEPROM_START;
  if (saveTimer > 0) {                // If the timer value is positive
    saveTimer = saveTimer - timeExec; // Timer subtraction
    if (saveTimer <= 0) {             // if time up
      EEPROM.write(p++, range0);      // save current status to EEPROM
      EEPROM.write(p++, ch0_mode);
      EEPROM.write(p++, lowByte(ch0_off));  // save as Little endian
      EEPROM.write(p++, highByte(ch0_off));
      EEPROM.write(p++, range1);
      EEPROM.write(p++, ch1_mode);
      EEPROM.write(p++, lowByte(ch1_off));  // save as Little endian
      EEPROM.write(p++, highByte(ch1_off));
      EEPROM.write(p++, rate);
      EEPROM.write(p++, trig_mode);
      EEPROM.write(p++, trig_lv);
      EEPROM.write(p++, trig_edge);
      EEPROM.write(p++, trig_ch);
      EEPROM.write(p++, fft_mode);
      EEPROM.write(p++, info_mode);
      EEPROM.write(p++, item);
      EEPROM.write(p++, pulse_mode);
      EEPROM.write(p++, duty);
      EEPROM.write(p++, p_range);
      EEPROM.write(p++, lowByte(count));  // save as Little endian
      EEPROM.write(p++, highByte(count));
      EEPROM.write(p++, dds_mode);
      EEPROM.write(p++, wave_id);
      EEPROM.write(p++, ifreq & 0xff);
      EEPROM.write(p++, (ifreq >> 8) & 0xff);
      EEPROM.write(p++, (ifreq >> 16) & 0xff);
      EEPROM.write(p++, (ifreq >> 24) & 0xff);
      EEPROM.commit();    // actually write EEPROM. Necessary for Raspberry Pi Pico
    }
  }
}
#endif

void set_default() {
  range0 = RANGE_MIN;
  ch0_mode = MODE_ON;
  ch0_off = 0;
  range1 = RANGE_MIN;
  ch1_mode = MODE_ON;
  ch1_off = 2048;
  rate = 6;
  trig_mode = TRIG_AUTO;
  trig_lv = 15;
  trig_edge = TRIG_E_UP;
  trig_ch = ad_ch0;
  fft_mode = false;
  info_mode = 1;  // display frequency and duty.  Voltage display is off
  item = 2;       // menu item
  pulse_mode = true;
  duty = 128;     // PWM 50%
  p_range = 0;    // PWM range
  count = 12499;  // PWM 10kHz
  dds_mode = false;
  wave_id = 0;    // sine wave
  ifreq = 23841;  // 238.41Hz
}

extern const byte wave_num;

#ifdef EEPROM_START
void loadEEPROM() { // Read setting values from EEPROM (abnormal values will be corrected to default)
  int p = EEPROM_START, error = 0;

  range0 = EEPROM.read(p++);                // range0
  if ((range0 < RANGE_MIN) || (range0 > RANGE_MAX)) ++error;
  ch0_mode = EEPROM.read(p++);              // ch0_mode
  if (ch0_mode > 2) ++error;
  *((byte *)&ch0_off) = EEPROM.read(p++);     // ch0_off low
  *((byte *)&ch0_off + 1) = EEPROM.read(p++); // ch0_off high
  if ((ch0_off < -4096) || (ch0_off > 4095)) ++error;

  range1 = EEPROM.read(p++);                // range1
  if ((range1 < RANGE_MIN) || (range1 > RANGE_MAX)) ++error;
  ch1_mode = EEPROM.read(p++);              // ch1_mode
  if (ch1_mode > 2) ++error;
  *((byte *)&ch1_off) = EEPROM.read(p++);     // ch1_off low
  *((byte *)&ch1_off + 1) = EEPROM.read(p++); // ch1_off high
  if ((ch1_off < -4096) || (ch1_off > 4095)) ++error;

  rate = EEPROM.read(p++);                  // rate
  if ((rate < RATE_MIN) || (rate > RATE_MAX)) ++error;
//  if (ch0_mode == MODE_OFF && rate < 5) ++error;  // correct ch0_mode
  trig_mode = EEPROM.read(p++);             // trig_mode
  if (trig_mode > TRIG_SCAN) ++error;
  trig_lv = EEPROM.read(p++);               // trig_lv
  if (trig_lv > LCD_YMAX) ++error;
  trig_edge = EEPROM.read(p++);             // trig_edge
  if (trig_edge > 1) ++error;
  trig_ch = EEPROM.read(p++);               // trig_ch
  if (trig_ch != ad_ch0 && trig_ch != ad_ch1) ++error;
  fft_mode = EEPROM.read(p++);              // fft_mode
  info_mode = EEPROM.read(p++);             // info_mode
  if (info_mode > 3) ++error;
  item = EEPROM.read(p++);                  // item
  if (item > ITEM_MAX) ++error;
  pulse_mode = EEPROM.read(p++);            // pulse_mode
  duty = EEPROM.read(p++);                  // duty
  p_range = EEPROM.read(p++);               // p_range
  if (p_range > 8) ++error;
  *((byte *)&count) = EEPROM.read(p++);     // count low
  *((byte *)&count + 1) = EEPROM.read(p++); // count high
  dds_mode = EEPROM.read(p++);              // DDS wave id
  wave_id = EEPROM.read(p++);               // DDS wave id
  if (wave_id >= wave_num) ++error;
  *((byte *)&ifreq) = EEPROM.read(p++);     // ifreq low
  *((byte *)&ifreq + 1) = EEPROM.read(p++); // ifreq
  *((byte *)&ifreq + 2) = EEPROM.read(p++); // ifreq
  *((byte *)&ifreq + 3) = EEPROM.read(p++); // ifreq high
  if (ifreq > 999999L) ++error;
  if (error > 0)
    set_default();
}
#endif
