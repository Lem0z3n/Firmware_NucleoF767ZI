#ifndef PINDEFSF767ZI
#define PINDEFSF767ZI


// SPI principal
#define mySCK    PB_3   // SPI1_SCK
#define myMOSI   PB_5   // SPI1_MOSI
#define myMISO   PB_4   // SPI1_MISO


//  USART
#define UART_TX  PD_5   
#define UART_RX  PD_6   


// SPI para SRAM
#define SRAM_SCLK  PE_2   // SPI4_SCK
#define SRAM_MOSI  PE_6   // SPI4_MOSI
#define SRAM_MISO  PE_5   // SPI4_MISO

// I2C
#define mySCL  PB_8   // I2C1_SCL
#define mySDA  PB_9   // I2C1_SDA

#endif