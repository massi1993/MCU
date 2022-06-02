# Project MCU
# Update : 02/06/22 , version : 0.1

# CONFIGURATION IDE IAR EMBEDDED : Follow these steps to configure project options
1. In the Project Editor, right-click on the project name and select Options... to display the Options dialog box:
![Alt text](https://github.com/massiAvg/MCU/blob/develop/config_ide/config1.png)
2. In the Options dialog box, select the General Options category, open the Target tab and select Device - ST -STM32F303:
![Alt text2](https://github.com/massiAvg/MCU/blob/develop/config_ide/config2.png)
3. If your source files include header files, select the C/C++ Compiler category, open the Preprocessor tab, and specify their paths. The path of the
include directory is a relative path, and always starts with the project directory location referenced by $PROJ_DIR$\..\inc
4. To set up the ST-Link embedded debug tool interface, select the Debugger category, open the Setup tab and, from the drop-down Driver menu, select ST-Link. Then,
open the Debugger tab and select Use flash loader(s):
![Alt text3](https://github.com/massiAvg/MCU/blob/develop/config_ide/config3.png)
5. Select the ST-Link category, open the ST-Link tab and select SWD as the connection protocol
![Alt text4](https://github.com/massiAvg/MCU/blob/develop/config_ide/config4.png)
6. Done: click OK to save the project settings.

# Description

To make a binary counter we need a counter variable, whose value increases by 1 each time the USER button (PA0) is pressed. 
We also want the value to be shown on the display, which in our case is represented by the 8 LEDs.

Example:

- cnt = 1 -> blue led PE8 on
- cnt = 2 -> red led PE9 on
- cnt = 3 -> blue and red led (PE8 and PE9) on

