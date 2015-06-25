# Gertboard-CNC-Motion
Raspberrypi+Gertboard provide CNC motion control with encoder position feedback and H bridge DC Motor drive. See Youtube video for a demonstration.

Reference Gertboard R2 manual http://farnell.com/datasheets/1683444.pdf for programming the ATMega 328, and the use of Minicom.

It is easiest to build up the CNC controller in stages:

1. Two directional motor control. Test with PWMmotortest2DGB. A pot is temporarly required to provide an input for the speed control.
2. Read the rotary position encoder. Test with GBRotary_enc_read3 and display the results on the screen with Minicom.
3. Implement the full closed loop control with _34567RPolymotioncontrolDF_GB for a pole Zero controller (Lead /Lag) or _34567RPolymotioncontrol_GB for a PID controller.

Before experimenting conduct a risk assessment, and implement approriate safety measures. Local Guarding, Safety Glasses, Torque limiting -Over current Fusing. Out of control motors can cause damage.

MIT License (MIT)

Copyright (c) [2015]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
