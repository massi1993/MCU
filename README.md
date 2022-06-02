# Project MCU
# Update : 02/06/22 , version : 0.1

# CONFIGURATION IDE IAR EMBEDDED : Follow these steps to configure project options
1. In the Project Editor, right-click on the project name and select Options... to display the Options dialog box:
In the Options dialog box, select the General Options category, open the Target tab and select Device - ST -STM32F303:
![config1](https://user-images.githubusercontent.com/83538787/171648935-19605a9c-bc42-47fe-afc5-051d73a3ed70.png)

2. If your source files include header files, select the C/C++ Compiler category, open the Preprocessor tab, and specify their paths. The path of the
include directory is a relative path. In this project, the library to include is stored in path /Library/ as shown:
![config2](https://user-images.githubusercontent.com/83538787/171651442-1ca16b80-0eb6-4209-97c2-196c8ea41944.PNG)

3. To set up the ST-Link embedded debug tool interface, select the Debugger category, open the Setup tab and, from the drop-down Driver menu, select ST-Link. Then,
open the Debugger tab and select Use flash loader(s):
![config3](https://user-images.githubusercontent.com/83538787/171648974-84341f7e-4eb4-470f-9b3f-03217b024df4.PNG)

4. Select the ST-Link category, open the ST-Link tab and select SWD as the connection protocol
![config4](https://user-images.githubusercontent.com/83538787/171649073-55a4ca78-45a0-476a-93d6-c6c35de240b2.PNG)

5. Done: click OK to save the project settings.

# Description
## Project binary counter
To make a binary counter we need a counter variable, whose value increases by 1 each time the USER button (PA0) is pressed. 
We also want the value to be shown on the display, which in our case is represented by the 8 LEDs.

Example:

- cnt = 1 -> blue led PE8 on
- cnt = 2 -> red led PE9 on
- cnt = 3 -> blue and red led (PE8 and PE9) on

## Project LED FLASHING EVERY x SECONDS
The clock frequency provided by the board is 8 MHz, which is 8 million counts per second. 
For example, if we want the LEDs to change state every 0.5 second, the counter must reach 4 million.
fck = 8 MHz
Tck = 1 / fck = 125 ns
N = Δt / Tck = 4 000 000

