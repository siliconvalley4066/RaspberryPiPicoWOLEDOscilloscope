# RaspberryPiPicoWIPSOscilloscope
Raspberry Pi Pico W dual channel Oscilloscope for 80x160 IPS TFT and wireless WEB display

<img src="DSC03505.jpg">
<img src="RPPicoWEBIPS.png">

This displays an oscilloscope screen on a 80x160 IPS TFT and also on the WEB page simultaneusly.
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

The source codes can be compiled for Raspberry Pi Pico (not W) without WEB functions.<br>
Also it can be compiled for RP2040-Zero without WEB functions.

<img src="DSC03508.jpg">

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
TFT_eSPI 2.5.43<br>
arduinoFFT by Enrique Condes 2.0.0<br>
arduinoWebSockets from https://github.com/Links2004/arduinoWebSockets

You need to customize the TFT_espi library by referring to the TFT_espi folder here.

2usec/div range is 10 times magnification at 500ksps.<br>
4usec/div range is 5 times magnification at 500ksps.<br>
The magnification applies sin(x)/x interpolation.

Schematics:<br>
<img src="RPPicoIPSOscillo.png"><br>
There is no software support for input attenuation yet.

Schematics for RP2040-zero:<br>
Select the board "Waveshare RP2040 Zero" in the Arduino IDE<br>
<img src="RP2040ZeroIPSOscillo.png"><br>
There is no software support for input attenuation yet.

Description is here, although it is written in Japanese language:
http://harahore.g2.xrea.com/RaspberryPiPico/RPPicoIPSOscillo.html
