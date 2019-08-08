# DFRobot_MCP23017
This is a 16-bit digital IO expansion board that communicates with main-controller via IIC to read and set Level value of the pins. <br>
The board supports 8 IIC addresses. One main-controller board can be connected with at most 8 modules in parallel to expand 128 IO ports. <br>

这里需要显示拍照图片，可以一张图片，可以多张图片（不要用SVG图）

![正反面svg效果图](https://github.com/Arya11111/DFRobot_MCP23017/blob/master/resources/images/SEN0245svg1.png)


## Product Link（链接到英文商城）
    DFR0626：Gravity: MCP23017 IIC to 16 digital IO expansion module
   
## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary

1. Set pin mode: input, output, pull-up input(internal 100KΩ pull-up resistor); <br>
2. Read and set pin Level value; <br>
3. Support 5 interruption modes: high-level interrupt, low-level interrupt, rising edge interrupt, falling edge interrupt, double edge interrupts; <br>
4. Support 2-way interrupt signal output: when an interrupt occurs on some pins of portA, pin INTA output High level, when an interrupt on some pins of PortB, INTB output High level; <br>
5. Polled interrupt: detect if an interrupt occurs on the pins via polled interrupt function, and run the relevant interrupt service function; <br>

## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
/**
 * @brief Init function
 * @return Return 0 if initialization succeeds, otherwise return non-zero. 
 */
int begin(void);

/**
 * @brief Set the pin mode to  input, output or pull-up input (internal 100KΩ pull-up resistor)
 * @param pin Pin number, it could be all enumeration values (eGPA0-eGPB7/ 0-15) included in ePin_t. 
 * @param mode Mode, it can be set to Input, Output, Pull-up Input (internal 100KΩ pull-up resistor)
 * @return Return 0 if the setting is successful, otherwise return non-zero. 
 */
int pinMode(ePin_t pin, uint8_t mode);

/**
 * @brief Write digtial pin. The pin needs to be set to output mode before writing. 
 * @param pin Pin number, it could be all enumeration values (eGPA0-eGPB7/ 0-15) inlcuded in ePin_t.
 * @param level High level 1 or Low level 0
 * @return Return 0 if the writing is successful, otherwise return non-zero. 
 */
int digitalWrite(ePin_t pin, uint8_t level);

/**
 * @brief Read digital pin. The pin needs to be set to output mode before reading. 
 * @param pin Pin number, it could be all enumeration values (eGPA0-eGPB7/ 0-15) included in ePin_t.
 * @return Return High or Low
 */
int digitalRead(ePin_t pin);

/**
 * @brief Set a pin 将某个引脚设置为中断模式
 * @param pin 引脚编号，可填ePin_t包含的所有枚举值（eGPA0-eGPB7/ 0-15）
 * @param mode 中断方式：可填eInterruptMode_t包含的所有枚举值
 * @param cb 中断服务函数，由用户外部定义函数传参，原型为void func(int)
 */
void pinModeInterrupt(ePin_t pin, eInterruptMode_t mode,  MCP23017_INT_CB cb);

/**
 * @brief 轮询某组端口是否发生中断
 * @param group 端口组，可填eGPIOGrout_t包含的所有枚举值GPIO A组（eGPIOA）、GPIO B组（eGPIOB）A+B组（eGPIOALL）
 * @n 填eGPIOA，则轮询A组端口是否发生中断
 * @n 填eGPIOB，则轮询B组端口是否发生中断
 * @n 填eGPIOALL，则轮询A组和B组端口是否发生中断
 * @n 不填，默认轮询A组和B组所有端口是否发生中断
 */
void pollInterrupts(eGPIOGrout_t group=eGPIOALL);

/**
 * @brief 将引脚转为字符串描述
 * @param pin 引脚编号，可填ePin_t包含的所有枚举值（eGPA0-eGPB7/ 0-15）
 * @return 返回引脚描述字符串
 * @n 如"GPIOA0" "GPIOA1" "GPIOA2" "GPIOA3" "GPIOA4" "GPIOA5" "GPIOA6" "GPIOA7"
 * @n   "GPIOB0" "GPIOB1" "GPIOB2" "GPIOB3" "GPIOB4" "GPIOB5" "GPIOB6" "GPIOB7"
 */
String pinDescription(ePin_t pin);

/**
 * @brief 将引脚转为字符串描述
 * @param pin 引脚编号，范围0~15
 * @return 返回引脚描述字符串,
 * @n 如"GPIOA0" "GPIOA1" "GPIOA2" "GPIOA3" "GPIOA4" "GPIOA5" "GPIOA6" "GPIOA7"
 * @n   "GPIOB0" "GPIOB1" "GPIOB2" "GPIOB3" "GPIOB4" "GPIOB5" "GPIOB6" "GPIOB7"
 */
String pinDescription(int pin);
```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino Uno        |      √       |              |             | 
Mega2560        |      √       |              |             | 
Leonardo        |      √       |              |             | 
ESP32         |      √       |              |             | 
micro:bit        |      √       |              |             | 

## History

- data 2019-7-18
- version V1.0

## Credits

Written by(xue.peng@dfrobot.com), 2019. (Welcome to our [website](https://www.dfrobot.com/))





