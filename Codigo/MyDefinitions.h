#ifndef MY_DEFINITIONS_H
#define MY_DEFINITIONS_H



#include <cstdint>
#if defined(TARGET_NUCLEO_F767ZI)
    #include    "pinDefsF767zi.h"
    #include    "EthernetInterface.h"
    #define TCP_PORT 8080
#endif

#if defined(TARGET_NUCLEO_L552ZE_Q)
    #include    "pinDefsL552ZE.h"
#endif

/*BOARD ID*/
#define BOARD_A
//#define BOARD_B
//#define BOARD_C

#define COMPILATION_TIME                    1733934717
// Got typing date +%s in Linux.

#define DeadMan_message                     '.'
#define DeadMan_time                        30s

/* MISCELLANEOUS */
#define ENTER                               '\n'
#define SRAM_BLOCKSIZE                      65536 // 
#define DO_NOTHING                          c1 = c1

#define HIGHEST_VCC                         3300

/* Definitions of models*/
#define SRAM130nm                   "CY62167DV"
#define TECH130nm                           130
#define SECTIONS16b_130nm                   16
#define SRAM090nm                   "CY62167E"
#define TECH090nm                           90
#define SECTIONS16b_090nm                   16
#define SRAM065nm                   "CY62167G"
#define TECH065nm                           65
#define SECTIONS16b_065nm                   16
#define SRAM040nm                   "IS61WV204816BLL"
#define TECH040nm                           40
#define SECTIONS16b_040nm                   32
#define SRAM110nm                   "RMLV1616AGSA"
#define TECH110nm                           110
#define SECTIONS16b_110nm                   16
#define PSRAM                       "IS66WVS4M8BLL"
#define TECHPSRAM                           0
#define SECTIONS16b_PSRAM                   32
#define PSRAM_SIZE_IN_BYTES                 4194304
#define PSRAM_SIZE_IN_PAGES                 4096
#define PSRAM_PAGE_SIZE                     1024
#define PSRAM_SPI_WRITE_CODE                0x02
#define PSRAM_SPI_READ_CODE                 0x03
#define PSRAM_TIME_TO_POWERUP               200 //us. The manufacturer proposes 150.

#define SRAM_TIME_TO_POWERUP                100 // ms. Probably unnecessary.
#define SRAM_TIME_TO_SETUP                  100 // us. Probably unnecessary.
#define SRAM_TPW                            40 //ns. Time required to write a word in the SRAM. Provided by the manufacturer, probably shorter.

/* Constant for RS232 communicatino */
#define MAX_SERIAL_BUFFER_SIZE              1024
#define SERIAL_BAUD_RATE                    115200//921600


#define TARGET_TX_PIN                       USBTX  // This defines the ports for printf. 
#define TARGET_RX_PIN                       USBRX

#define FLAG_FOR_DATA_START                 "\nBEGIN\n"
#define FLAG_FOR_ACTION_START               "\nSTART\n"
#define FLAG_FOR_ACTION_END                 "DONE\n"
#define FLAG_FOR_DATA_END                   "END\n"

#define SPI_CLK_DAC_SPEED                   100000 // Can go up to 20MHz, but unnecessary and not tested by the manufacturer.
#define SPI_MODE_DAC                        0
#define SPI_DAC_WORD_LENGTH                 16
#define DAC_A_ABOVE_2048                    0x1000
#define DAC_A_BELOW_2048                    0x3000
#define DAC_B_ABOVE_2048                    0x9000
#define DAC_B_BELOW_2048                    0xB000
#define CE_DAC                              DAC_EN  // This sentence allows to synchronize the nomenclature in board and sotware.
#define DAC_TCSSR                           100
#define DAC_TLS                             100
#define DAC_TLD                             100
#define DELAY_FOR_SOFT_APROACH              2000 //microseconds. See void set_output_voltage_softly(int16_t STOP)

/*Definitions for ADC and analog ports*/
#ifdef BOARD_A
   #define ADC_VREF                            3324 // in millivolts.
#endif
#ifdef BOARD_B
   #define ADC_VREF                            3316 // in millivolts.
#endif
#ifdef BOARD_C
   #define ADC_VREF                            3313 // in millivolts.
#endif

#define MEASURE_VCC                         (int32_t)(READ_VCC.read()*ADC_VREF)
#define N_FOR_OVERSAMPLING                  16
#define TIME_FOR_OVERSAMPLING               100 //microseconds.

#define TEMP_SLOPE                          2.5 // mV/K for ADC_TEMP
#define TEMP_30                             760 //mV at 30 C for ADC_TEMP

// Definitions related to the measurement of current. 

#ifdef BOARD_A
   #define CURRSENSE_G                         21.0 // needs calibration.
   #define CURRSENSE_VOS                       0.0 // mV
#endif
#ifdef BOARD_B
   #define CURRSENSE_G                         28.0 // needs calibration.
   #define CURRSENSE_VOS                       0.0 // mV
#endif
#ifdef BOARD_C
   #define CURRSENSE_G                         21.0 // needs calibration.
   #define CURRSENSE_VOS                       0.0 // mV
#endif

#define RQ                                  10000.0 // Ohms. This is R in schematic.

#define TIME_TO_CUT_LATCHUP                 1 // seconds.


#define LM75A_ADDRESS                       0x48  // 0b0100_1000
/* Bits 7-3: 01001 
   Bits 2-0: A2-0: 000 */
#define LM75A_SETUP                         0x00

/*Constants for SRAM*/
#define A20                                 PD_4
#define A19                                 PD_3
#define A18                                 PD_2
#define A17                                 PD_1
#define A16                                 PD_0

#define PATTERN_ALL_0                       0x0000
#define PATTERN_ALL_1                       0xFFFF
#define PATTERN_CB_55                       0x5555
#define PATTERN_CB_AA                       0xAAAA

#define DynTst_MARCHC                       '1'
#define DynTst_STRESS                       '2'
#define DynTst_CLASSC                       '3'
#define DynTst_MATS                         '4'
#define DynTst_mMATS                        '5'
#define DynTst_PSEUD                        '6'

// PARAMETERS FOR SPI PSRAM 

#define SRAM_CE                             CSRAM  // I started with recycling some code with different nomenclature...

#define SPI_CLK_PSRAM_SPEED                 2500000
#define SPI_MODE_PSRAM                      0
#define SPI_PSRAM_WORD_LENGTH               8
#define PSRAM_SIZE_IN_PAGES                 4096
#define PSRAM_PAGE_SIZE                     1024
#define PSRAM_SPI_WRITE_CODE                0x02
#define PSRAM_SPI_READ_CODE                 0x03
//#define PSRAM_TIME_TO_POWERUP               200 //us. The manufacturer proposes 150.
#define TIME_BETWEEN_REDUNDANT_READINGS     10 //us
#define FIRMWARE_VERSION   "3"
/////////////////////////////////////////////
// DECLARATION OF FUNCTIONS.               //
/////////////////////////////////////////////

// COMMON FUNCTIONS FOR PARALLEL AND SPI
void initialize_GPIO(void);  // Puts all the GPIO in default mode.
void display_parameters_on_screen(void);
uint32_t Time_from_EPOCH(void); // Time since EPOCH
void dead_man(void);
//MODULATION FUNCTIONS
void displayParams(char &c1);
void changeParams();
void chooseAction(void);
//MODULATION FUNCTIONS FOR CHOOSEACTION
void _mainWriteSRAM(void);
void _mainReadSRAM(void);
void _mainSleepSRAM(char &c1);
void _mainInjectFailures(void);
void _mainDynTests(char &c1);
//GENERALITATION FUNC FOR DynTests
void _DynTestsRun(char testType);
//void Increase_time_since_last_message(void);
void restart_global_variables(void);
void set_output_voltage_vcc(int16_t BIASVOLTAGE, int16_t LUREF);  // Programs the DAC to show this two values in both outputs. It is impossible to set only one output.
void set_output_voltage_softly_vcc(int16_t BIASVOLTAGE, int16_t LUREF); // A loop with pauses to reach the desired Bias Voltage
float Calculate_QC(void);
int16_t set_latchup_voltage_threshold(int16_t LimitCurrent);
void LatchUpDetected(void);
float Measure_external_temperature(void);

//deadman functions
void send_dot_irq(void);
void enable_deadman(void);
void disable_deadman(void);


//TESTS
void R0W1UP(int32_t round, char phase);
void R1W0UP(int32_t round, char phase);
void R0W1DOWN(int32_t round, char phase);
void R1W0DOWN(int32_t round, char phase);
void R0W1R05UP(int32_t round, char phase);
void R1W0R05UP(int32_t round, char phase);
void R0W1R05DOWN(int32_t round, char phase);
void R1W0R05DOWN(int32_t round, char phase);

//DYNAMICTESTS
void RunDynamicClassic(int32_t round);
void RunMarchTest(int32_t round);
void Run_m_MatsPlus(int32_t round);
void RunMatsPlus(int32_t round);
void RunDynamicStress(int32_t round);

// END OF DECLARATION OF FUNCTIONS



//SRAM HANDLING FUNCTIONS 
void DisableECC_065nm(void);
uint16_t unscramble_content(uint16_t Content);
uint16_t scramble_pattern(uint16_t Pattern);
void write_address(uint8_t MSB_ADDRESS, uint32_t LSB_ADDRESS, uint16_t TARGET_PATTERN); // This address is written.
void write_full_memory(void);
uint16_t read_address(uint8_t MSB_ADDRESS, uint32_t LSB_ADDRESS);
void read_full_memory(void);
void Inject_fails_in_static(void);
void write_page_spi(uint8_t ADDRESS_MSB, uint8_t ADDRESS_INT, uint8_t PATTERN);
void write_full_SRAM_spi(void);
void read_full_SRAM_spi(void);
void write_address_spi(uint32_t ADDRESS, uint8_t tPATTERN);
uint8_t read_address_spi(uint32_t ADDRESS);
void Inject_fails_in_static_spi(void);

//TCP functions
void connectionStartTimer(void);
bool try_TCP_connection(void);

//Wrapper functions for ethernet+usb compatibility
int nprintf(const char* format, ...);
int nscanf(const char* format, ...);
char ngetchar(void);
bool input_readable(void);


#endif