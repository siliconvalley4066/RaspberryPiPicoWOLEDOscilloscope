# RaspberryPiPicoWOLEDOscilloscope
Raspberry Pi Pico W dual channel Oscilloscope for OLED and wireless WEB display<br>
The 80x160 IPS TFT version is available at IPS80x160 branch.

<img src="DSC00020.jpg">
<img src="RPPicoWEBOLED.png">

This displays an oscilloscope screen on a 128x64 OLED and also on the WEB page simultaneusly.
The settings are controled by the 5 direction switch and also on the WEB page.
You can view the oscilloscope screen on the WEB browser of the PC or the tablet or the smartphone.
It contains Pulse Generator, DDS Function Generator and Frequency Counter.

Specifications:
<li>Dual input channel</li>
<li>Input voltage range 0 to 3.3V</li>
<li>12 bit ADC 500 ksps single channel, 250 ksps dual channel</li>
<li>Measures minimum, maximum and average values</li>
<li>Measures frequency and duty cycle</li>
<li>Spectrum FFT analysis</li>
<li>Sampling rate selection</li>
<li>Built in Pulse Generator</li>
<li>Built in DDS Function Generator</li>
<br>

The source codes can be compiled for Raspberry Pi Pico (not W) without WEB functions.

For WEB operations, edit the source code WebTask.ino and replace your Access Point and the password.
<pre>
Edit:
const char* ssid = "XXXX";
const char* pass = "YYYY";
To:
const char* ssid = "Your Access Point";
const char* pass = "Your Password";
</pre>

Develop environment is:<br>
Arduino IDE 1.8.19<br>
Raspberry Pi Pico/RP2040 by Earle F. Philhower, III version 3.4.0<br>
CPU speed 125MHz<br>

Libraries:<br>
Adafruit_SSD1306<br>
Adafruit_SH110X<br>
arduinoFFT by Enrique Condes 2.0.0<br>
arduinoWebSockets from https://github.com/Links2004/arduinoWebSockets

2usec/div range is 10 times magnification at 500ksps.<br>
4usec/div range is 5 times magnification at 500ksps.<br>
The magnification applies sin(x)/x interpolation.

Schematics:<br>
<img src="RPPicoGOscillo.png"><br>
There is no software support for input attenuation yet.

Schematics for RP2040-zero:<br>
Select the board "Waveshare RP2040 Zero" in the Arduino IDE<br>
<img src="RP2040ZeroGOscillo.png"><br>
There is no software support for input attenuation yet.

Description is here, although it is written in Japanese language:
https://ss1.xrea.com/harahore.g2.xrea.com/RaspberryPiPico/RPPicoGOscillo.html
