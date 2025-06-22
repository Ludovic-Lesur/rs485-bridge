/*
 * stm32g4xx_device_flags.h
 *
 *  Created on: 01 feb. 2025
 *      Author: Ludo
 */

#ifndef __STM32G4XX_DEVICE_FLAGS_H__
#define __STM32G4XX_DEVICE_FLAGS_H__

/*** STM32G4XX device compilation flags ***/

#ifndef RS485_BRIDGE
#define STM32G4XX_DEVICE_DISABLE
#endif

#define STM32G4XX_DEVICE_STACK_SIZE     0x00000400
#define STM32G4XX_DEVICE_HEAP_SIZE      0x00000C00

#endif /* __STM32G4XX_DEVICE_FLAGS_H__ */
