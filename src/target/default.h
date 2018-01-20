#pragma once
//springboard header file
#include "includes.h"

#define RCC_ALL_CLK (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2)

#if defined(OMNIBUS)
    #include "omnibus.h"
#elif defined(MOTOLAB)
    #include "motolab.h"
#else

    #define BOARD_COMM_SPI            SPI3
    #define BOARD_COMM_SPI_IRQn       SPI3_IRQn
    #define BOARD_COMM_RX_DMA         DMA1_Channel2
    #define BOARD_COMM_TX_DMA         DMA1_Channel3
    #define BOARD_COMM_CS_HARDWARE    1
    #define BOARD_COMM_CS_PIN         GPIO_PIN_4
    #define BOARD_COMM_CS_PORT        GPIOA
    #define BOARD_COMM_CS_ALTERNATE   GPIO_AF5_SPI3
    #define BOARD_COMM_MISO_PIN       GPIO_PIN_4
    #define BOARD_COMM_MISO_PORT      GPIOB
    #define BOARD_COMM_MISO_ALTERNATE GPIO_AF5_SPI3
    #define BOARD_COMM_MOSI_PIN       GPIO_PIN_5
    #define BOARD_COMM_MOSI_PORT      GPIOB
    #define BOARD_COMM_MOSI_ALTERNATE GPIO_AF5_SPI3
    #define BOARD_COMM_SCK_PIN        GPIO_PIN_3
    #define BOARD_COMM_SCK_PORT       GPIOB
    #define BOARD_COMM_SCK_ALTERNATE  GPIO_AF5_SPI3

    #define GYRO_SPI            SPI2
    #define GYRO_SPI_IRQn       SPI2_IRQn
    #define GYRO_RX_DMA         DMA1_Channel4
    #define GYRO_TX_DMA         DMA1_Channel5
    #define GYRO_CS_HARDWARE    0
    #define GYRO_CS_PIN         GPIO_PIN_9
    #define GYRO_CS_PORT        GPIOA
    #define GYRO_CS_ALTERNATE   GPIO_AF5_SPI2
    #define GYRO_MISO_PIN       GPIO_PIN_10
    #define GYRO_MISO_PORT      GPIOA
    #define GYRO_MISO_ALTERNATE GPIO_AF5_SPI2
    #define GYRO_MOSI_PIN       GPIO_PIN_11
    #define GYRO_MOSI_PORT      GPIOA
    #define GYRO_MOSI_ALTERNATE GPIO_AF5_SPI2
    #define GYRO_SCK_PIN        GPIO_PIN_1
    #define GYRO_SCK_PORT       GPIOF
    #define GYRO_SCK_ALTERNATE  GPIO_AF5_SPI2

    #define BOOTLOADER_CHECK_PORT  GPIOB
    #define BOOTLOADER_CHECK_PIN   GPIO_PIN_5

#endif