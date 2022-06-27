# Project MCU
# Update : 13/06/22 , version : 0.5

# Table of contents

0. [IDE_Configuration](#confIDE)
1. [Binary counter](#bin-count) 
2. [LED FLASHING EVERY x SECONDS](#ledOnSeconds)     
3. [EXTERNAL INTERRUPT (PA0)](#PA0exti)         
4. [INTERNAL INTERRUPT (TIMER3)](#TIM3int)      
5. [FLASHING LED WITH SPEED CHANGE](#ledOnSpeed)   
6. [STOPWATCH/NOT_STOPWATCH](#StopWatchOrNo)          
7. [VOLTAGE CONVERTER](#V_converter)
8. [DAC-ADC](#DAC-ADC)     
9. [DAC-ADC with USART](#DAC-ADC-USART)     
10. [Flashing leds with USART](#led-USART)     

# CONFIGURATION IDE IAR EMBEDDED :  <a name="confIDE"></a>
## Follow these steps to configure project options
1. In the Project Editor, right-click on the project name and select Options... to display the Options dialog box:
In the Options dialog box, select the General Options category, open the Target tab and select Device - ST -STM32F303:
![config1](https://user-images.githubusercontent.com/83538787/171648935-19605a9c-bc42-47fe-afc5-051d73a3ed70.png)

2. If your source files include header files, select the C/C++ Compiler category, open the Preprocessor tab, and specify their paths. The path of the
include directory is a relative path. In this project, the library to include is stored in path /libraries/ as shown:

![config2](https://user-images.githubusercontent.com/83538787/171996102-4ffdc9a5-c428-4759-a331-e0912adb2bca.PNG)

3. To set up the ST-Link embedded debug tool interface, select the Debugger category, open the Setup tab and, from the drop-down Driver menu, select ST-Link. Then,
open the Debugger tab and select Use flash loader(s):
![config3](https://user-images.githubusercontent.com/83538787/171648974-84341f7e-4eb4-470f-9b3f-03217b024df4.PNG)

4. Select the ST-Link category, open the ST-Link tab and select SWD as the connection protocol
![config4](https://user-images.githubusercontent.com/83538787/171649073-55a4ca78-45a0-476a-93d6-c6c35de240b2.PNG)

5. Done: click OK to save the project settings.

6. For this project, the workspace has been saved in the "binary_counter" folder and is renamed "workspace_project".
          

# Description
## 1. Project binary counter (folder binary counter) <a name="bin-count"></a>
To make a binary counter we need a counter variable, whose value increases by 1 each time the USER button (PA0) is pressed. 
We also want the value to be shown on the display, which in our case is represented by the 8 LEDs.

Example:

- cnt = 1 -> blue led PE8 on
- cnt = 2 -> red led PE9 on
- cnt = 3 -> blue and red led (PE8 and PE9) on

However, we can use also BSRR to set/reset the corrispondent ODRx bit.

## 2. Project LED FLASHING EVERY x SECONDS (folder led_on_timer) <a name="ledOnSeconds"></a>
The clock frequency provided by the board is 8 MHz, which is 8 million counts per second. 
For example, if we want the LEDs to change state every 0.5 second, the counter must reach 4 million.

$ f_{ck} = 8 MHz $

$ T_{ck} = 1 / f_{ck} = 125 ns$

$ N = Δt / T_{ck} = 4 000 000$


## 3. Project EXTERNAL INTERRUPT (PA0) (folder PA0_EXTI) <a name="PA0exti"></a>
Management external interrupt when PA0 is pressed.
For this project, is added a new c source file (system_stm32f30x.c)


## 4. Project INTERNAL INTERRUPT (TIMER3) (folder Timer3_internal_interrupt) <a name="TIM3int"></a>
For this project, is added a new c source file (system_stm32f30x.c)
Management INTERNAL interrupt WITH LED ON / OFF EVERY SECOND. 
Inasmuch as TIM3 is a 16-bit timer, it can count to 65,535 before rolling over, which means we can measure events no longer than about 819 microseconds if Fck = 80 MHz!
As a matter of fact:

$ f_{ck} = 80 MHz$

$ Δt_{max} = (2^{16})/f_{ck} = 819,2 us $

$ N = [Δt_{max} * f_{ck} - 1]= 65 535 $

If we wish to measure longer events, we need to use a prescaler, which is a piece of hardware that divides the clock source. 
For example, a prescaler of 80 would turn an 80 MHz clock into a 1 MHz clock.

![1a01c1b8-ed69-4d86-8c9e-67388366a487](https://user-images.githubusercontent.com/83538787/172060373-4970bbf8-c6ca-4e81-8eb0-47ae446bf3de.jpg)


## 5. Project FLASHING LED WITH SPEED CHANGE (folder led_on_velocity_change) <a name="ledOnSpeed"></a>
The speed is varied by decreasing the maximum count, stored in the ARR (AutoReloadRegister).

## 6. Project STOPWATCH/NOT_STOPWATCH (folder Measurement_time_usingPA0) <a name="StopWatchOrNo"></a>
Start time measurement when the PA0 is pressed; Stop time when PA0 is pressed again. Otherwise, if the mode is NOT_STOP_WATCH, it measure the time betwen two pressing of PA0. Basically, we are implementing a stopwatch or not_stop_watch TIMER.

[MODE_STOPWATCH]

![image](https://user-images.githubusercontent.com/83538787/173041289-64b4213a-c3e8-4aab-a95f-83aa3fc1a8e6.png)
 
## 7. Project VOLTAGE CONVERTER (folder Voltage_converter) <a name="V_converter"></a>

The converted digital data is found on the RDATA register (Regular DATA which then goes to the AHB interface) and represents the number of samples counted, each of which is  $ V_{in} =R_{DATA} * (V_{DD} * 2^{n}) $  volts (this difference represents precisely the quantization and $ 2^{n} $ are the quantization levels -> in the case of 12 bits these quantization levels are $ 2^{12} = 4096$ ). 

So, the voltage is:

$ V_{in} =R_{DATA} * (V_{DD} * 2^{12})$  (1)  

with $ V_{DD} = 3 V $ or $ 3.3 V $ respectively if the microcontroller is powered via USB or battery.

In the project, we use the ADC to obtain and elaborate the analog signal from PA0 (set as analog input) in order to get voltage's value (on PA0) , using formula (1)

[PA0 NOT PRESSED]

![image](https://user-images.githubusercontent.com/83538787/173183914-16214e62-49ea-4e82-8a58-9f671f408cbf.png)

[PA0 PRESSED]

![image](https://user-images.githubusercontent.com/83538787/173183945-aa556b50-7764-4ad1-b4c3-aa87896f101a.png)

## 8. Project DAC-ADC IN SERIES (folder DAC_ADC) <a name="DAC-ADC"></a>

![DAC-ADC](https://user-images.githubusercontent.com/83538787/173192086-aec7c769-2c91-4fb4-86e5-5344fb71de2f.PNG)

Given an input code (from 0 to 4095) to the DAC, the relative output voltage from the ADC is obtained. 

From the datasheet we have chosen to use pin PA4 for the DAC (DAC1_OUT1) and pin PA5 for the ADC (ADC2_IN2).

![image](https://user-images.githubusercontent.com/83538787/173192289-3261a279-5553-49ec-bd6f-03b0999842cf.png)

## 9. Project DAC-ADC with USART (folder DAC_ADC_USART) <a name="DAC-ADC-USART"></a>

A code is sent, via USART, to the DAC which then converts into an analog signal, which is converted back into a digital signal via ADC.
For this project, is added a c sourc file system_stm32f30x.c
![dac-adc-usart](https://user-images.githubusercontent.com/83538787/173396173-81055e52-a89c-4f06-a5ba-11aa3c58527f.PNG)

![teraTerm](https://user-images.githubusercontent.com/83538787/173401386-0db9d5ac-aa10-49e3-9d31-b0ad9e915ed8.PNG)

## 10. Project Flashing leds  with USART (folder Usart_flashing_led) <a name="led-USART"></a>

Let's use Timer2 to turn on, every 0.5 seconds, the red or blue leds: in this case it's need sending 'B' or 'R' via USART.
Let's use Timer3 to turn on, every second, the green or orange leds: here, we have to send 'O' or 'G' via USART.

