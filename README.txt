EE30186 2019 Coursework - Embedded Control System for a Variable Speed Fan

TURNING THE SYSTEM ON & OFF:

The system will begin in an off state which is indicated on the seven segment display with "OFF" being displayed accross segments 2 to 0.In order to switch the system on and off, the user can press Key0. The system will start with fan speed 0 which can be changed via the rotary encoder.

CHANGING THE SPEED OF THE FAN:

In order to increase the speed of the fan, the rotary encoder (found on the extension board) can be rotated clockwise and anti-clockwise in order to reduce it. The maximum speed of the fan is 2500rpm and will only rotate in one direction. 

SWITCHING BETWEEN OPEN & CLOSED LOOP CONTROL:

When in Closed Loop Control, the segments 4 and 5 of the SSD will display "CL". Otherwise, it will display "OL".

SWITCH0: When this switch is turned on, the system will operate in closed loop control and when it is off the system will be open loop. 

KEY2: When this key is pressed, the desired speed produced by the PID controller will be displayed on the seven segment display

CHANGING THE SENSITIVITY OF THE ROTARY ENCODER:

Switches 5 through 9 which all differ the speed change from a single rotation of the rotary encoder.

Switch 5: 10rpm
Switch 6: 20rpm
Switch 7: 50rpm
Switch 8: 100rpm
Switch 9: 200rpm

OUTPUTS:

SEVEN SEGMENT DISPLAY:

On the left hand section of the display, the system will display the control type the system is operating in. C represents closed loop and O, open loop. The second digit displays the sensitivity of the rotary encoder.

The right 4 digits will display one of speed values. By default when no keys are pressed, this will be the user chosen speed. By pressing key1, this will instead display the measured Speed and by pressing Key2 the desired Speed will be displayed.

TACHOMETER:

KEY1: By pressing this, the Seven Segment Display will show the speed measured by the tachometer.


