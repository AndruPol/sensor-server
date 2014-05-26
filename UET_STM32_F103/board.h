/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the UET STM32-F103Z board.
 */

/*
 * Board identifier.
 */
#define BOARD_UET_STM32_F103Z
#define BOARD_NAME              "UET STM32-F103Z"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            8000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F10X_HD

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0              0
#define GPIOA_PIN1              1
#define GPIOA_PIN2              2
#define GPIOA_PIN3              3
#define GPIOA_PIN4              4
#define GPIOA_PIN5              5
#define GPIOA_PIN6              6
#define GPIOA_PIN7              7
#define GPIOA_PIN8              8
#define GPIOA_USART1_TX         9
#define GPIOA_USART1_RX         10
#define GPIOA_USB_DM            11
#define GPIOA_USB_DP            12
#define GPIOA_JTMS              13
#define GPIOA_JTCK              14
#define GPIOA_JTDI              15

#define GPIOB_PIN0              0
#define GPIOB_PIN1              1
#define GPIOB_BOOT1             2
#define GPIOB_JTDO              3
#define GPIOB_JNTRST            4
#define GPIOB_PIN5              5
#define GPIOB_PIN6              6
#define GPIOB_PIN7              7
#define GPIOB_PIN8              8
#define GPIOB_PIN9              9
#define GPIOB_PIN10             10
#define GPIOB_PIN11             11
#define GPIOB_PIN12             12
#define GPIOB_PIN13             13
#define GPIOB_PIN14             14
#define GPIOB_PIN15             15

#define GPIOC_PIN0              0
#define GPIOC_PIN1              1
#define GPIOC_PIN2              2
#define GPIOC_PIN3              3
#define GPIOC_PIN4              4
#define GPIOC_PIN5              5
#define GPIOC_PIN6              6
#define GPIOC_PIN7              7
#define GPIOC_PIN8              8
#define GPIOC_PIN9              9
#define GPIOC_PIN10             10
#define GPIOC_PIN11             11
#define GPIOC_PIN12             12
#define GPIOC_LED               13
#define GPIOC_OSC32_IN          14
#define GPIOC_OSC32_OUT         15

#define GPIOD_PIN0              0
#define GPIOD_PIN1              1
#define GPIOD_PIN2              2
#define GPIOD_PIN3              3
#define GPIOD_PIN4              4
#define GPIOD_PIN5              5
#define GPIOD_PIN6              6
#define GPIOD_PIN7              7
#define GPIOD_PIN8              8
#define GPIOD_PIN9              9
#define GPIOD_PIN10             10
#define GPIOD_PIN11             11
#define GPIOD_PIN12             12
#define GPIOD_PIN13             13
#define GPIOD_PIN14             14
#define GPIOD_PIN15             15

#define GPIOE_PIN0              0
#define GPIOE_PIN1              1
#define GPIOE_PIN2              2
#define GPIOE_PIN3              3
#define GPIOE_PIN4              4
#define GPIOE_PIN5              5
#define GPIOE_PIN6              6
#define GPIOE_PIN7              7
#define GPIOE_PIN8              8
#define GPIOE_PIN9              9
#define GPIOE_PIN10             10
#define GPIOE_PIN11             11
#define GPIOE_PIN12             12
#define GPIOE_PIN13             13
#define GPIOE_PIN14             14
#define GPIOE_PIN15             15

#define GPIOF_PIN0              0
#define GPIOF_PIN1              1
#define GPIOF_PIN2              2
#define GPIOF_PIN3              3
#define GPIOF_PIN4              4
#define GPIOF_PIN5              5
#define GPIOF_PIN6              6
#define GPIOF_PIN7              7
#define GPIOF_PIN8              8
#define GPIOF_PIN9              9
#define GPIOF_PIN10             10
#define GPIOF_PIN11             11
#define GPIOF_PIN12             12
#define GPIOF_PIN13             13
#define GPIOF_PIN14             14
#define GPIOF_PIN15             15

#define GPIOG_PIN0              0
#define GPIOG_PIN1              1
#define GPIOG_PIN2              2
#define GPIOG_PIN3              3
#define GPIOG_PIN4              4
#define GPIOG_PIN5              5
#define GPIOG_PIN6              6
#define GPIOG_PIN7              7
#define GPIOG_PIN8              8
#define GPIOG_PIN9              9
#define GPIOG_PIN10             10
#define GPIOG_PIN11             11
#define GPIOG_PIN12             12
#define GPIOG_PIN13             13
#define GPIOG_PIN14             14
#define GPIOG_PIN15             15

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything input with pull-up except:
 * PA9  - Alternate output  (USART1 TX).
 * PA10 - Normal input      (USART1 RX).
 * PA11 - Normal input      (USB DM).
 * PA12 - Normal input      (USB DP).
 */
#define VAL_GPIOACRL            0x88888888      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x888444B8      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOBCRL            0x88888888      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x88888888      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 * Everything input with pull-up except:
 * PC13 - Push Pull output (LED).
 * PC14 - Normal input (OSC32_IN).
 * PC15 - Normal input (OSC32_OUT).
 */
#define VAL_GPIOCCRL            0x88888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x44B88888      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIODCRL            0x88888888      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

/*
 * Port F setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOFCRL            0x88888888      /*  PF7...PF0 */
#define VAL_GPIOFCRH            0x88888888      /* PF15...PF8 */
#define VAL_GPIOFODR            0xFFFFFFFF

/*
 * Port G setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOGCRL            0x88888888      /*  PG7...PG0 */
#define VAL_GPIOGCRH            0x88888888      /* PG15...PG8 */
#define VAL_GPIOGODR            0xFFFFFFFF


/*
 * USB bus activation macro, required by the USB driver.
 */
#define usb_lld_connect_bus(usbp) palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_MODE_INPUT);

/*
 * USB bus de-activation macro, required by the USB driver.
 */
#define usb_lld_disconnect_bus(usbp) { \
 palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_MODE_OUTPUT_PUSHPULL); \
 palClearPad(GPIOA, GPIOA_USB_DP); \
}

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
