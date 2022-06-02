# Project MCU
# Update : 02/06/22
# version : 0.1
To make a binary counter we need a counter variable, whose value increases by 1 each time the USER button (PA0) is pressed. 
We also want the value to be shown on the display, which in our case is represented by the 8 LEDs.

Example:
cnt = 1 -> blue led PE8 on
cnt = 2 -> red led PE9 on
cnt = 3 -> blue and red led (PE8 and PE9) on

