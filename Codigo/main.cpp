
#include "mbed.h"
#include "MyDefinitions.h"
#include <cctype>
#include <cstdio>
//////////////////////////////////////
////
////  PIN DEFINITIONS      ///////////
////
//////////////////////////////////////


AnalogIn  CURR_SENSE(PC_0);          // Measures the output of the INA270 and determines the quiescent current.
AnalogIn  READ_VCC(PC_3);            // Determines current bias voltage of the SRAM.
AnalogIn  READ_VCC2(PC_2);           // Determines current bias volage of the level shifters.

// SRAM Control
PortInOut  DATABUS(PortE, 0xFFFF); // 16-bit DATA BUS
// When words are of 16-bit, the ADDRESS needs 21 (ISSI) or 20 bits (Cypress). 
// The leas significant bits are declared by meanss of a fast port, the MSB with 
// a more flexible but slower Bus.
PortOut    ADDRESS_LSB_PORT(PortF, 0xFFFF); // 
BusOut     ADDRESS_MSB_PORT(A16,A17,A18,A19,A20); // Fixed error. T
//BusOut     ADDRESS_MSB_PORT(A20,A19,A18,A17,A16); 
/// This GPIO pins controll different parts
DigitalOut CSRAM(PG_4);      // Chip Select para SRAM
DigitalOut OE(PG_6);         // Output Enable
DigitalOut WE(PG_0);         // Write Enable
DigitalOut LB(PG_5);         // Lower Byte
DigitalOut UB(PG_14);        // Upper Byte
DigitalIn  ERR(PG_12);       // Pin de error de la SRAM

// COMMUNICATION
// Now, black magic begins... This function is called 3 times at setup in order to override stdIn stdOut and stdErr
BufferedSerial usb_port(USBTX, USBRX, SERIAL_BAUD_RATE); //USB COMMUNICATION

FileHandle *mbed::mbed_override_console(int) {
      return &usb_port;
 }
// Why have I added this after the definition of usb_port?
// usb_port.readable() did not work, but this forum entry gave the clue
// to solve the problem.
// https://forums.mbed.com/t/nonblocing-getchar-or-using-printf-and-serial-read-together/15200
// Better not to remove it.

SPI spi_port(myMOSI, myMISO, mySCK); // comumunication with DAC
SPI sram_spi_port(SRAM_MOSI, SRAM_MISO, SRAM_SCLK);

DigitalOut CAN_RD(PB_8); // If CAN is eventually used. Currently, disabled.
DigitalOut CAN_TD(PB_9);
I2C i2c(mySDA, mySCL);   // Used to measure room temperature.

// SPI DEVICES
DigitalOut SDCARD_EN(PA_14);  // A Flas card.
DigitalOut DAC_EN(PB_2);      // Chip Enable for DAC
DigitalOut LDAC(PB_1);        // Load register in DAC.


/// Interrupts
InterruptIn LU_Warning(PB_0);   // Warning if latchup detected. Currently experimental.

/// ANALOG PORTS

AnalogOut NUC_LU_REF(PA_4);          // Reserved for a DAC output to set the latchup reference. Disabled.
AnalogOut NUC_DAC(PA_5);             // Idem to set the SRAM bias voltage with embedded DAC. Disabled.


////  END OF PIN DEFINITION



//////////////////////////////////////
////
////    GLOBAL VARIABLES
////
//////////////////////////////////////

int16_t VCC_NOMINAL=3250;     // Value at which the SRAM will be written or read. Expressed in mV.
int16_t VCC_IN_STANDBY=750;   // Value at which the SRAM will be sleep in standby. Expressed in mV.
int16_t VOLT_REF_LATCHUP = 3000;  // Reference voltage for the comparator that detects latchup. Must be deduced from  THRESHOLD_FOR_LATCH_UP

uint16_t PATTERN = PATTERN_CB_55;  // The pattern to be written and compared to the read content in the word in static mode.
uint8_t  PATTERN_8 = 0x55;
uint8_t  N_SECTIONS16b = SECTIONS16b_040nm; // Number of blocks of 2^16 words that are addressable with fixed ADDRESS_MSB_PORT and variable ADDRESS_ÑSB_PORT

char MODEL_NAME[] = SRAM040nm;  // The SRAM model.
char MODEL_TECH = TECH040nm;    // The technology.

char MODEL_LABEL='A';           // The label for the tested sample. A letter o one number.

bool ENABLE_CURRENT_SURVEY_IN_SBY=true;
bool ENABLE_LATCHUP_DETECTION_IN_SBY=true;
bool COMING_FROM_LATCHUP = false;
int16_t THRESHOLD_FOR_LATCH_UP = 100;    
int16_t TIME_CURRENT_SAMPLES_in_ms = 1000; // Time between different readings of the current in static mode.
int16_t TIME_PSEUDO_STATIC_TESTS_in_seconds = 60;  // time between different readings in pseudostatic mode.
int32_t NBF = 0;
char DynamicTest = DynTst_CLASSC;

Ticker   periodicDotTicker;

//Eth variables
volatile bool    tcpConnection=false;
#ifdef TARGET_NUCLEO_F767ZI
    EthernetInterface net;
    TCPSocket server;
    TCPSocket* client;
#endif
////  END OF DEFINITION OF GLOBAL VARIABLES

/////////////////////////////////////////////
// STATES FOR MACHINE AND GLOBAL VARIABLES //
/////////////////////////////////////////////

typedef enum{
    s_START,
    s_DISPLAY_PARAMETERS,
    s_CHANGE_PARAMETERS,
    s_CHOOSE_ACTION
}state_machine;

state_machine STATE = s_START;

///////////////////////////////////////////////////
///MAIN or LVL 1 State Machine. ////
///////////////////////////////////////////////////


// main() runs in its own thread in the OS
int main()
{
    char    c1 = 'x'; 

    set_time(COMPILATION_TIME); 
    usb_port.set_baud(SERIAL_BAUD_RATE);
    ThisThread::sleep_for(100ms);

    
    #if defined(TARGET_NUCLEO_F767ZI)
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        __DSB();
        tcpConnection= try_TCP_connection();
    #endif
    enable_deadman();

      
     nprintf("ahoi\n");
    
    while (true) {

        switch(STATE){

            case s_START:
                
                    nprintf("state: start\n"); 
                
                initialize_GPIO();
                STATE = s_DISPLAY_PARAMETERS;
                break;

            case s_DISPLAY_PARAMETERS:
                
                    nprintf("state: display params\n"); 
                
                displayParams(c1);
                break;
            
            case s_CHANGE_PARAMETERS:
                
                    nprintf("state: change params\n"); 
                
                changeParams();
                break;
                
            case s_CHOOSE_ACTION:
                
                
                    nprintf("state: choose action\n"); 
                
                chooseAction();
                break;            
            /////////////
            default: 
                break;

        }

    }
}

///////////////////////////////////////////////////
/// FUNCTIONS OTHER THAN MAIN //
///////////////////////////////////////////////////

void initialize_GPIO(void){

    spi_port.format(8, SPI_MODE_DAC);
    spi_port.frequency(SPI_CLK_DAC_SPEED);

    set_output_voltage_vcc(0, 0);

    CAN_RD = 0;
    CAN_TD = 0;

    SDCARD_EN = 1;
    DAC_EN = 1;
    LDAC = 1;

    CSRAM = 1;
    WE = 1;
    OE = 1;
    UB = 0;
    LB = 0;

    DATABUS.output();
    DATABUS = PATTERN_ALL_0;

    ADDRESS_MSB_PORT = 0b00000;
    ADDRESS_LSB_PORT = PATTERN_ALL_0;

    NUC_DAC = 0.0f;
    NUC_LU_REF = 0.0f;
    set_output_voltage_vcc(0, 3200);

    nprintf("Firmware Version: %s\n",FIRMWARE_VERSION); 
    nprintf("Ready...\n"); 
    

}

void displayParams(char &c1){
    while((c1 = ngetchar()) <0){};
    if (((c1>='A')&&(c1<='L'))||((c1>='a')&&(c1<='l'))){
        STATE = s_CHANGE_PARAMETERS;
        return;
    }else{
        switch (c1) {
            case 'n':
            case 'N':
                STATE = s_CHOOSE_ACTION;
                break;
            case 'r':
            case 'R':
                restart_global_variables();
                STATE = s_START;
                break;
            case 'v':
            case 'V':
                STATE = s_DISPLAY_PARAMETERS;
                display_parameters_on_screen();
                break;
            case 'z':
            case 'Z':
                STATE = s_DISPLAY_PARAMETERS;
                break;
            default:
                STATE = s_DISPLAY_PARAMETERS;
                
        nprintf("\nUnknown.\n"); 
    
                
        }
    }
}

void changeParams(){
    char buff;
    int16_t ibuff16;
    int32_t ibuff32;
    char c1;
    while((c1 = ngetchar()) <0){};
    switch(c1){
        case 'a':
        case 'A':
            
                nprintf("choose sram tech\n"); 
            
            while((c1 = ngetchar()) <0){};;
            switch(c1){
                case '1': MODEL_TECH = TECH130nm; strcpy(MODEL_NAME, SRAM130nm); N_SECTIONS16b = SECTIONS16b_130nm; break;
                case '2': MODEL_TECH = TECH090nm; strcpy(MODEL_NAME, SRAM090nm); N_SECTIONS16b = SECTIONS16b_090nm; break;
                case '3': MODEL_TECH = TECH065nm; strcpy(MODEL_NAME, SRAM065nm); N_SECTIONS16b = SECTIONS16b_065nm; break;
                case '4': MODEL_TECH = TECH040nm; strcpy(MODEL_NAME, SRAM040nm); N_SECTIONS16b = SECTIONS16b_040nm; break;
                case '5': MODEL_TECH = TECH110nm; strcpy(MODEL_NAME, SRAM110nm); N_SECTIONS16b = SECTIONS16b_110nm; break;
                case '6': MODEL_TECH = TECHPSRAM; strcpy(MODEL_NAME, PSRAM)    ; N_SECTIONS16b = SECTIONS16b_PSRAM; break;
                default:  nprintf("Unknown\n"); 
   
            }
            STATE = s_DISPLAY_PARAMETERS;
            break;

        case 'b':
        case 'B':
            
                nprintf("choose label\n"); 
            
            nscanf("%c", &buff);
            buff = toupper(buff);
            if (((buff >='A') && (buff <= 'Z'))||((buff >='0') && (buff <= '9'))){
                MODEL_LABEL = buff;
            }
            STATE = s_DISPLAY_PARAMETERS;
            break;

        case 'c':
        case 'C':
            
                nprintf("change nominal Vcc\n"); 
            
            //input new  vcc
            nscanf("%i\n", (int*) &ibuff16);
            // Error checks:
            VCC_NOMINAL=VCC_NOMINAL;
            if(ibuff16 != -1) 
                VCC_NOMINAL = ibuff16;
            if(VCC_NOMINAL < 0)  
                VCC_NOMINAL = 0;
            // If too high, set to highest possible value.
            if(VCC_NOMINAL > HIGHEST_VCC)
               VCC_NOMINAL = HIGHEST_VCC;  
            
            STATE = s_DISPLAY_PARAMETERS;
            break;
        
        case 'd':
        case 'D':
            
                nprintf("change standby Vcc\n"); 
            
            nscanf("%i\n", (int*) &ibuff16); 
            //ERROR CHECKS:
            VCC_IN_STANDBY=VCC_IN_STANDBY;
            if(ibuff16 != -1 )
                VCC_IN_STANDBY = ibuff16;
            if(VCC_IN_STANDBY < 0)
                VCC_IN_STANDBY = 0;
            // If too high, set to highest possible value.
            if(VCC_IN_STANDBY > HIGHEST_VCC)
                VCC_IN_STANDBY = HIGHEST_VCC;
            
            STATE = s_DISPLAY_PARAMETERS;
            break;
        
        case 'e':
        case 'E':
            
            nprintf("change pattern\n"); 
            
            while((c1 = ngetchar()) <0){};
            switch(c1){
                case '1': PATTERN = PATTERN_ALL_0; break;
                case '2': PATTERN = PATTERN_ALL_1; break;
                case '3': PATTERN = PATTERN_CB_55; break;
                case '4': PATTERN = PATTERN_CB_AA; break;
                default: nprintf("Unknown\n");
            }
            PATTERN_8 = (uint8_t) (PATTERN & 0x00FF);
            STATE = s_DISPLAY_PARAMETERS;
            break;
        
        case 'f':
        case 'F':
            
                nprintf("enable current survey\n"); 
            
            while((c1 = ngetchar()) <0){};;
            if (c1 == '1') {
                ENABLE_CURRENT_SURVEY_IN_SBY = true;
            } else if (c1 == '0') {
                ENABLE_CURRENT_SURVEY_IN_SBY = false;
            } else {
                nprintf("Unknown\n");
            }
            STATE = s_DISPLAY_PARAMETERS;
            break;     
        
        case 'g':
        case 'G':
            
                nprintf("change testing\n"); 
            
            while((c1 = ngetchar()) <0){};;
            switch(c1){
                case DynTst_MARCHC: 
                case DynTst_STRESS: 
                case DynTst_CLASSC:
                case DynTst_MATS:
                case DynTst_mMATS: 
                case DynTst_PSEUD: DynamicTest = c1; break;
                default: nprintf("Unknown\n");
            }
            STATE = s_DISPLAY_PARAMETERS;
            break;
        
        case 'h':
        case 'H':
            
                nprintf("change sampling time\n"); 
            
            nscanf("%i\n",(int*) &ibuff16);
            if(ibuff16 <= 0) {
                TIME_CURRENT_SAMPLES_in_ms = TIME_CURRENT_SAMPLES_in_ms;
            } else{
                TIME_CURRENT_SAMPLES_in_ms = ibuff16;
            }
            STATE = s_DISPLAY_PARAMETERS;
            break;

        case 'i':
        case 'I':
            
                nprintf("enable latchup detection\n"); 
            
            while((c1 = ngetchar()) <0){};;
            if (c1 == '1') {
                ENABLE_LATCHUP_DETECTION_IN_SBY = true;
            } else if (c1 == '0') {
                ENABLE_LATCHUP_DETECTION_IN_SBY = false;
            } else {
                nprintf("Unknown\n");
            }
            STATE = s_DISPLAY_PARAMETERS;
            break;

        case 'j':
        case 'J':
            
                nprintf("threshold for latchup detection\n"); 
            
            nscanf("%i\n",(int*) &ibuff16);
            if(ibuff16 <= 0){
                THRESHOLD_FOR_LATCH_UP = THRESHOLD_FOR_LATCH_UP;
            } else{
                THRESHOLD_FOR_LATCH_UP = ibuff16;
            }
                
            VOLT_REF_LATCHUP = set_latchup_voltage_threshold(THRESHOLD_FOR_LATCH_UP);
            STATE = s_DISPLAY_PARAMETERS;
            break;    

        case 'k':
        case 'K':
            
                nprintf("threshold for latchup detection\n"); 
            
            nscanf("%i\n",(int*) &ibuff16);
            //ERROR CHECKS:
            TIME_PSEUDO_STATIC_TESTS_in_seconds=TIME_PSEUDO_STATIC_TESTS_in_seconds;
            if(ibuff16 != -1)
                TIME_PSEUDO_STATIC_TESTS_in_seconds = ibuff16;
            if(TIME_PSEUDO_STATIC_TESTS_in_seconds < 0) 
                TIME_PSEUDO_STATIC_TESTS_in_seconds = 0 ;

            
            STATE = s_DISPLAY_PARAMETERS;
            break;

        case 'l':
        case 'L':
            
                nprintf("set time\n"); 
            
            nscanf("%i\n",(int*) &ibuff32);
            if(ibuff32 > 0){ 
                set_time(ibuff32);
            }
            else{
                nprintf("Unknown\n");
            }
            STATE = s_DISPLAY_PARAMETERS;
            break;
        
        case 'n':
        case 'N':
            STATE = s_CHOOSE_ACTION;
            break;
        
        case 'r':
        case 'R':
            restart_global_variables();
            STATE = s_START;
            break;
        
        case 'v':
        case 'V':
            display_parameters_on_screen();
            break;
        
        default:
            STATE = s_CHANGE_PARAMETERS;
            break;
    }
}

void _mainWriteSRAM(void){
    if (MODEL_TECH!=TECHPSRAM){

        nprintf("\nACTION  \t:Writing - 0x%04X - %i - Time:%i\n", PATTERN, VCC_NOMINAL, (int) Time_from_EPOCH());
        
        display_parameters_on_screen();
        nprintf(FLAG_FOR_ACTION_START);  
        set_output_voltage_softly_vcc(VCC_NOMINAL, VOLT_REF_LATCHUP);
        ThisThread::sleep_for(chrono::milliseconds(SRAM_TIME_TO_POWERUP));
        if (MODEL_TECH == TECH065nm) { DisableECC_065nm(); }

        write_full_memory();
    }else{

        PATTERN_8 = (uint8_t)(PATTERN & 0x00FF);
        nprintf("\nACTION  \t:Writing - 0x%02X - %i - Time:%i\n", PATTERN_8, VCC_NOMINAL, (int) Time_from_EPOCH());
        
        display_parameters_on_screen();
        nprintf(FLAG_FOR_ACTION_START); 
        set_output_voltage_softly_vcc(VCC_NOMINAL, VOLT_REF_LATCHUP);
        ThisThread::sleep_for(chrono::milliseconds(SRAM_TIME_TO_POWERUP));
        write_full_SRAM_spi();
    }
    nprintf(FLAG_FOR_ACTION_END);
    STATE = s_CHOOSE_ACTION;
}

void _mainReadSRAM(void){
    NBF = 0;
    if (MODEL_TECH!=TECHPSRAM){
        nprintf("\nACTION  \t:Reading - 0x%04X - %i\n", PATTERN, VCC_NOMINAL);
        display_parameters_on_screen();
        set_output_voltage_softly_vcc(VCC_NOMINAL, VOLT_REF_LATCHUP);
        ThisThread::sleep_for(chrono::milliseconds(SRAM_TIME_TO_POWERUP));
        nprintf(FLAG_FOR_ACTION_START);
        nprintf(FLAG_FOR_DATA_START);
        nprintf("#MOD:%s-%c,TEST:STAT,VCC:%i,IQ:%.2f,T:%.1f,PATTERN:0x%04X,TIME:%i\n", 
            MODEL_NAME, 
            MODEL_LABEL, 
            (int) MEASURE_VCC, 
            Calculate_QC(),
            Measure_external_temperature(),
            PATTERN, 
            (int) Time_from_EPOCH());
        
        read_full_memory();
    }else{
        PATTERN_8 = (uint8_t)(PATTERN & 0x00FF);
        nprintf("\nACTION  \t:Reading - 0x%02X - %i, TIME:%i\n", PATTERN_8, VCC_NOMINAL,(int) Time_from_EPOCH());
        
        display_parameters_on_screen();
        set_output_voltage_softly_vcc(VCC_NOMINAL, VOLT_REF_LATCHUP);
        ThisThread::sleep_for(chrono::milliseconds(SRAM_TIME_TO_POWERUP));
        nprintf(FLAG_FOR_ACTION_START); 
        nprintf(FLAG_FOR_DATA_START); 
        nprintf("#MOD:%s-%c,TEST:STAT,VCC:%i,IQ:%.2f,T:%.1f,PATTERN:0x%04X,TIME:%i\n", 
            MODEL_NAME, 
            MODEL_LABEL, 
            (int)MEASURE_VCC, 
            Calculate_QC(),
            Measure_external_temperature(),
            PATTERN, 
            (int)Time_from_EPOCH());
        
        read_full_SRAM_spi();
    }
    nprintf(FLAG_FOR_DATA_END);
    nprintf(FLAG_FOR_ACTION_END);
    nprintf("NBF:%i\n", (int) NBF);
    NBF = 0;
    STATE = s_CHOOSE_ACTION;
}



void _mainInjectFailures(void){

    nprintf("\nACTION  \t:INJECTING FAILS");
    nprintf(FLAG_FOR_ACTION_START);
    set_output_voltage_softly_vcc(VCC_NOMINAL, VOLT_REF_LATCHUP);

    ThisThread::sleep_for(chrono::milliseconds(SRAM_TIME_TO_POWERUP));

    if(MODEL_TECH!=TECHPSRAM){
        if (MODEL_TECH == TECH065nm) { DisableECC_065nm(); }
        Inject_fails_in_static();
    }else{
        Inject_fails_in_static_spi();
    }

    nprintf(FLAG_FOR_ACTION_END);
    STATE = s_CHOOSE_ACTION;
}

void _DynTestsRun(char testType){
    int32_t round = 1;
    while (!(input_readable())){
        nprintf("Round:%i,Time:%i,VCC:%i,IQ:%.2f,T:%.1f,NBF:%i\n", 
                (int)round,
                (int)Time_from_EPOCH(), 
                (int)MEASURE_VCC,
                Calculate_QC(), 
                Measure_external_temperature(),
                (int)NBF);      
        switch(testType){
            case DynTst_MARCHC: 
                RunMarchTest(round);
                break;
            case DynTst_STRESS: 
                RunDynamicStress(round);
                break;
            case DynTst_CLASSC:
                RunDynamicClassic(round);
                break;
            case DynTst_MATS:
                RunMatsPlus(round);
                break;
            case DynTst_mMATS: 
                Run_m_MatsPlus(round);
                break;
            case DynTst_PSEUD: // Preferiría refactorizar esto pero preciso verlo en accion.
                ThisThread::sleep_for(chrono::seconds(TIME_PSEUDO_STATIC_TESTS_in_seconds));
                set_output_voltage_softly_vcc(VCC_NOMINAL, VOLT_REF_LATCHUP);
                
                if (MODEL_TECH != TECHPSRAM){
                    if (MODEL_TECH == TECH065nm) { DisableECC_065nm(); }
                    read_full_memory();                                    
                    DATABUS.output();
                    DATABUS = PATTERN_ALL_0;
                    ADDRESS_MSB_PORT = 0b00000;
                    ADDRESS_LSB_PORT = PATTERN_ALL_0;     
                    wait_us(SRAM_TIME_TO_SETUP);           
                    CSRAM = 0;
                    wait_us(SRAM_TIME_TO_SETUP);           
                    WE = 0;
                    OE = 0;
                }else{
                    read_full_SRAM_spi();
                    CSRAM = 0;
                }
                break;
            default: 
             break;
        }        
        RunMarchTest(round);
        round++;
    }
}

void _mainDynTests(char &c1){

    int16_t ibuff16 = 0;

    nprintf("\nACTION  \t:Run Dynamic Test\n");
    display_parameters_on_screen();
    nprintf(FLAG_FOR_ACTION_START);
    set_output_voltage_softly_vcc(VCC_NOMINAL, VOLT_REF_LATCHUP);
    if (MODEL_TECH == TECH065nm) { 
        DisableECC_065nm(); 
    }
    nprintf(FLAG_FOR_DATA_START);
    nprintf("#MOD:%s-%c,TEST:DYN%c,VCC:%i,IQ:%.2f,T:%.1f,PATTERN:0x%04X,TIME:%i\n", 
                        MODEL_NAME, MODEL_LABEL, DynamicTest, 
                        (int)MEASURE_VCC, Calculate_QC(), Measure_external_temperature(), PATTERN, (int)Time_from_EPOCH());
    
    NBF = 0; //number of bit flips.                          
    switch (DynamicTest) {
        case DynTst_MARCHC:
            if (MODEL_TECH!=TECHPSRAM){
                ibuff16 = PATTERN; 
                PATTERN = PATTERN_ALL_0;

                if (MODEL_TECH == TECH065nm) { 
                    DisableECC_065nm(); 
                }
                write_full_memory();
                PATTERN = ibuff16;
                _DynTestsRun(DynTst_MARCHC);
                while((c1 = ngetchar()) <0){};;
            }else{
                nprintf("WARNING: NOT IMPLEMENTED\n");
            }
            break;
        case DynTst_STRESS:
            if (MODEL_TECH!=TECHPSRAM){
                ibuff16 = PATTERN; 
                PATTERN = PATTERN_ALL_1;
                if (MODEL_TECH == TECH065nm) {
                    DisableECC_065nm(); 
                }
                write_full_memory();
                PATTERN = ibuff16;
                _DynTestsRun(DynTst_STRESS);
                while((c1 = ngetchar()) <0){};;
            }else{
                nprintf("WARNING: NOT IMPLEMENTED\n");
            }                          
            break;

        case DynTst_CLASSC:
            if (MODEL_TECH!=TECHPSRAM){
                if (MODEL_TECH == TECH065nm) { DisableECC_065nm(); }
                _DynTestsRun(DynTst_CLASSC);
                while((c1 = ngetchar()) <0){};;
            }else{
                nprintf("WARNING: NOT IMPLEMENTED\n");
            }
            break;

        case DynTst_MATS: 
            if (MODEL_TECH!=TECHPSRAM){
                ibuff16 = PATTERN; 
                PATTERN = PATTERN_ALL_0;
                if (MODEL_TECH == TECH065nm) { DisableECC_065nm(); }
                write_full_memory();
                PATTERN = ibuff16;

                _DynTestsRun(DynTst_MATS);
                while((c1 = ngetchar()) <0){};;
            }else{
                nprintf("WARNING: NOT IMPLEMENTED");
            }
            break;

        case DynTst_mMATS: 
            if (MODEL_TECH!=TECHPSRAM){
                ibuff16 = PATTERN; 
                PATTERN = PATTERN_ALL_0;
                if (MODEL_TECH == TECH065nm) { 
                    DisableECC_065nm(); 
                }
                write_full_memory();
                PATTERN = ibuff16;

                _DynTestsRun(DynTst_mMATS);
                while((c1 = ngetchar()) <0){};;
            }else{
                nprintf("WARNING: NOT IMPLEMENTED\n");
            }
            break;

        case DynTst_PSEUD: 
            if (MODEL_TECH != TECHPSRAM){
                if (MODEL_TECH == TECH065nm) { 
                    DisableECC_065nm(); 
                }
                write_full_memory();
                DATABUS.output();
                DATABUS = PATTERN_ALL_0;
                ADDRESS_MSB_PORT = 0b00000;
                ADDRESS_LSB_PORT = PATTERN_ALL_0;     
                wait_us(SRAM_TIME_TO_SETUP);           
                CSRAM = 0;
                wait_us(SRAM_TIME_TO_SETUP);           
                WE = 0;
                OE = 0;
            }else{
                write_full_SRAM_spi(); 
                CSRAM = 0;
            }           
            nprintf("\n");
            _DynTestsRun(DynTst_PSEUD);
            break;

        default: 
            break;
    }
    
    nprintf(FLAG_FOR_DATA_END);
    nprintf(FLAG_FOR_ACTION_END);
    STATE = s_CHOOSE_ACTION;
}

void chooseAction(void){

    disable_deadman();
    char c1;
    while((c1 = ngetchar()) <0){};
    switch (c1) {
        case 'a':
        case 'A':
            _mainWriteSRAM();
            break;

        case 'b':
        case 'B':
            _mainReadSRAM();                 
            break;

        case 'c':
        case 'C':
            _mainSleepSRAM(c1);
            break;

        case 'd':
        case 'D':
            _mainInjectFailures();
            break;

        case 'e':
        case 'E':
            _mainDynTests(c1);
            break;

        case 'z':
        case 'Z':
            STATE = s_DISPLAY_PARAMETERS;
            break;
        case 'r':
        case 'R':
            restart_global_variables();
            STATE = s_START;
            break;
        case 'v':
        case 'V':
            display_parameters_on_screen();
            STATE = s_CHOOSE_ACTION;
            break;
        default:
            nprintf("\n\n\tUnknown\n\n");
            STATE = s_CHOOSE_ACTION;
            break;                   
        
    }
    enable_deadman();
}

void set_output_voltage_vcc(int16_t BIASVOLTAGE, int16_t LUREF){

    uint16_t CODE_FOR_DAC_A = 0;
    uint16_t CODE_FOR_DAC_B = 0;

    if (BIASVOLTAGE > 0){

        if (BIASVOLTAGE < 2049){

            CODE_FOR_DAC_A = (uint16_t) (2*BIASVOLTAGE & 0x0FFF)|DAC_A_BELOW_2048;
            // In this configuration, the VLSB = 0.5 mV

        } else {
            if (BIASVOLTAGE < HIGHEST_VCC){

                CODE_FOR_DAC_A = (uint16_t) ((BIASVOLTAGE) & 0x0FFF)|DAC_A_ABOVE_2048;
            }else{

                CODE_FOR_DAC_A = (uint16_t) (HIGHEST_VCC & 0x0FFF)|DAC_A_ABOVE_2048;
            }
        }
    }else{

        CODE_FOR_DAC_A = (uint16_t) DAC_A_BELOW_2048;

    }

    if (LUREF > 0){

        if (LUREF < 2049){

            CODE_FOR_DAC_B = (uint16_t) (2*LUREF & 0x0FFF)|DAC_B_BELOW_2048;
            // In this configuration, the VLSB = 0.5 mV.

        } else { 
            if (BIASVOLTAGE < HIGHEST_VCC){

                CODE_FOR_DAC_B = (uint16_t) ((LUREF) & 0x0FFF)|DAC_B_ABOVE_2048;
            }else{

                CODE_FOR_DAC_B = (uint16_t) (HIGHEST_VCC & 0x0FFF)|DAC_B_ABOVE_2048;
            }
        }
    }else{
        CODE_FOR_DAC_B = (uint16_t) DAC_B_ABOVE_2048;
    }
    spi_port.format(SPI_DAC_WORD_LENGTH, SPI_MODE_DAC);
    spi_port.frequency(SPI_CLK_DAC_SPEED);

    CE_DAC = 0; wait_ns(DAC_TCSSR);
    spi_port.write(CODE_FOR_DAC_A); wait_ns(DAC_TCSSR);
    CE_DAC = 1; wait_ns(DAC_TCSSR);
    
    CE_DAC = 0; wait_ns(DAC_TCSSR);
    spi_port.write(CODE_FOR_DAC_B); wait_ns(DAC_TCSSR);
    CE_DAC = 1; wait_ns(DAC_TLS);
    LDAC = 0;
    wait_ns(DAC_TLD);
    LDAC = 1;
}

void set_output_voltage_softly_vcc(int16_t BIASVOLTAGE, int16_t LUREF){

    int16_t START, STOP, i;

    START = MEASURE_VCC;

    STOP = BIASVOLTAGE; 

    STOP > HIGHEST_VCC ? STOP = HIGHEST_VCC : STOP = STOP;
    STOP < 0    ? STOP = 0 :    STOP = STOP; 

    if (START>=STOP){

        for (i = START; i>STOP; i--){
            set_output_voltage_vcc(i, LUREF);
            wait_us(DELAY_FOR_SOFT_APROACH);
        }
    }else{

        for (i = START; i<STOP; i++){
            set_output_voltage_vcc(i, LUREF);
            wait_us(DELAY_FOR_SOFT_APROACH);
        }

    }
    
}

void display_parameters_on_screen(void){

    time_t seconds = time(NULL);

    nprintf("\tMODEL   :%s", MODEL_NAME);
    nprintf("\n\tLABEL   :%c", MODEL_LABEL);
    nprintf("\n\tVCCNOM  :%i", VCC_NOMINAL);
    nprintf("\n\tVCCSBY  :%i", VCC_IN_STANDBY);
    nprintf("\n\tCUR.VCC :%i",(int) MEASURE_VCC);
    MODEL_TECH == TECHPSRAM ?
        nprintf("\n\tPATTERN :0x%02X", PATTERN_8) :
        nprintf("\n\tPATTERN :0x%04X", PATTERN);
    nprintf("\n\tCUR.MEA.:"); (ENABLE_CURRENT_SURVEY_IN_SBY) ? nprintf("yes") : nprintf("no");
    nprintf("\n\tCUR.Time:%i", TIME_CURRENT_SAMPLES_in_ms);
    nprintf("\n\tLU MITI.:"); (ENABLE_LATCHUP_DETECTION_IN_SBY) ? nprintf("yes") : nprintf("no");
    nprintf("\n\tLU THR. :%i", THRESHOLD_FOR_LATCH_UP);
    nprintf("\n\tDYN. TST:");
        switch (DynamicTest) {
        case DynTst_MARCHC: nprintf("1-March C-"); break;
        case DynTst_STRESS: nprintf("2-Dynamic Stress"); break;
        case DynTst_CLASSC: nprintf("3-Dynamic Classic"); break;
        case DynTst_MATS: nprintf("4-Mats+"); break;
        case DynTst_mMATS: nprintf("5-mMats+"); break;
        case DynTst_PSEUD: nprintf("6-PSEUDO / %i s", TIME_PSEUDO_STATIC_TESTS_in_seconds);break;
        default: nprintf("?");
        }
    nprintf("\n\tTime    :%s", ctime(&seconds));
    nprintf("\tSTATE   :%i", STATE);
    nprintf("\n\t----------\n");
    
}

uint32_t Time_from_EPOCH(void){
    time_t seconds = time(NULL);
    return (uint32_t) seconds;
}

void restart_global_variables(void){

    VCC_NOMINAL=3000;     // Value at which the SRAM will be written or read. Expressed in mV.
    VCC_IN_STANDBY=750;   // Value at which the SRAM will be sleep in standby. Expressed in mV.
    VOLT_REF_LATCHUP = 3000;  // Reference voltage for the comparator that detects latchup. Must be deduced from  THRESHOLD_FOR_LATCH_UP

    PATTERN = PATTERN_CB_55;  // The pattern to be written and compared to the read content in the word in static mode.
    N_SECTIONS16b = SECTIONS16b_040nm; // Number of blocks of 2^16 words that are addressable with fixed ADDRESS_MSB_PORT and variable ADDRESS_ÑSB_PORT

    strcpy(MODEL_NAME, SRAM040nm);  // The SRAM model.
    MODEL_TECH = TECH040nm;    // The technology.

    MODEL_LABEL='A';           // The label for the tested sample. A letter o one number.

    ENABLE_CURRENT_SURVEY_IN_SBY=true;
    ENABLE_LATCHUP_DETECTION_IN_SBY=true;
    COMING_FROM_LATCHUP = false;
    THRESHOLD_FOR_LATCH_UP = 100;    
    TIME_CURRENT_SAMPLES_in_ms = 1000; // Time between different readings of the current in static mode.
    TIME_PSEUDO_STATIC_TESTS_in_seconds = 60;  // time between different readings in pseudostatic mode.

    DynamicTest = DynTst_CLASSC;
}

float Calculate_QC(void){

    float Measurement;

    Measurement = 0.0f;

    for (uint8_t i=0; i<N_FOR_OVERSAMPLING;i++){

        Measurement += CURR_SENSE.read(); // We are oversampling to increase resolution.
        wait_us(TIME_FOR_OVERSAMPLING);
    }

    Measurement = Measurement / N_FOR_OVERSAMPLING;

    return (Measurement*ADC_VREF-CURRSENSE_VOS)/CURRSENSE_G - READ_VCC.read()*ADC_VREF/RQ; // Current expressed in mA.

}

float Measure_external_temperature(void){

    float temperature;

    char cmd[2];

    cmd[0] = LM75A_SETUP;
    cmd[1] = 0x02;

    i2c.write(LM75A_ADDRESS<<1, cmd, 2);;

    cmd[0] = LM75A_SETUP;

    i2c.write(LM75A_ADDRESS<<1, cmd, 1, 1);
    i2c.read(LM75A_ADDRESS<<1, cmd, 2, 0);

    int16_t temperature_index = cmd[0]*8+((cmd[1]>>5)&0x07);

    if (temperature_index < 1023){
        // the temperature is positive.
        temperature = ((float) temperature_index)*0.125;
    }else{
        // the temperature is negative.
        temperature = ((float) (1024-(temperature_index & 0x3F)))*0.125;
    }

    // If the previous if does not work, comment it and use the following.
    // temperature = (float((cmd[0]*256) + cmd[1]) / 256.0);

    return temperature;

}

void R0W1UP(int32_t round, char phase){

    uint16_t READ_WORD;

    CSRAM = 0; WE = 1; OE = 1; UB = 0; LB = 0;

    for (uint8_t k_MSB = 0; k_MSB < N_SECTIONS16b; k_MSB++){
        
        if (input_readable()){break;}
        ADDRESS_MSB_PORT = k_MSB;

        for (uint32_t k_LSB=0; k_LSB < SRAM_BLOCKSIZE; k_LSB++){

            if (input_readable()){break;}

            ADDRESS_LSB_PORT = (uint16_t)k_LSB; 
            DATABUS.input();
            OE = 0; 
            READ_WORD = DATABUS.read();
            OE = 1;            
            if (READ_WORD != PATTERN_ALL_0){
                nprintf("0x%02X%04X,0x%04X,0x%04X,R%i%c\n", k_MSB, (unsigned int)k_LSB, unscramble_content(READ_WORD), PATTERN_ALL_0, (int)round, phase);
                
                NBF+=1;
            }          
            DATABUS.output();
            DATABUS = PATTERN_ALL_1;
            WE = 0; WE = 1;
        }
    }
}

void R1W0UP(int32_t round, char phase){

    uint16_t READ_WORD;

    CSRAM = 0; WE = 1; OE = 1; UB = 0; LB = 0;

    for (uint8_t k_MSB = 0; k_MSB < N_SECTIONS16b; k_MSB++){

        if (input_readable()){break;}
        ADDRESS_MSB_PORT = k_MSB;

        for (uint32_t k_LSB=0; k_LSB < SRAM_BLOCKSIZE; k_LSB++){
            
            if (input_readable()){break;}
            ADDRESS_LSB_PORT = k_LSB; 
            DATABUS.input();
            OE = 0; 
            READ_WORD = DATABUS.read();
            OE = 1;            
            if (READ_WORD != PATTERN_ALL_1){
                nprintf("0x%02X%04X,0x%04X,0x%04X,R%i%c\n", k_MSB, (unsigned int)k_LSB, unscramble_content(READ_WORD), PATTERN_ALL_1, (int)round, phase);
                
                NBF+=1;
            }
            DATABUS.output();
            DATABUS = PATTERN_ALL_0;
            WE = 0; WE = 1;            
        }
    }
}

void R0W1DOWN(int32_t round, char phase){

    uint16_t READ_WORD;

    CSRAM = 0; WE = 1; OE = 1;  UB = 0; LB = 0;

    for (int8_t k_MSB = N_SECTIONS16b-1; k_MSB >=0; k_MSB--){

        if (input_readable()){break;}
        ADDRESS_MSB_PORT = (uint8_t)(k_MSB);

        for (int32_t k_LSB=SRAM_BLOCKSIZE-1; k_LSB >=0; k_LSB--){
            
            if (input_readable()){break;}
            ADDRESS_LSB_PORT = (uint16_t)(k_LSB); 
            DATABUS.input();

            OE = 0; 
            READ_WORD = DATABUS.read();
            OE = 1;            
            if (READ_WORD != PATTERN_ALL_0){
                nprintf("0x%02X%04X,0x%04X,0x%04X,R%i%c\n", k_MSB, (unsigned int)k_LSB, unscramble_content(READ_WORD), PATTERN_ALL_0, (int)round, phase);
                            
                NBF+=1;
            }           
            DATABUS.output();
            DATABUS = PATTERN_ALL_1;
            WE = 0; WE = 1;      
        }
    }
}

void R1W0DOWN(int32_t round, char phase){

    uint16_t READ_WORD;

    CSRAM = 0; WE = 1; OE = 1; UB = 0; LB = 0;

    for (int8_t k_MSB = N_SECTIONS16b-1; k_MSB >=0; k_MSB--){

        if (input_readable()){break;}

        ADDRESS_MSB_PORT = (uint8_t)(k_MSB);

        for (int32_t k_LSB=SRAM_BLOCKSIZE-1; k_LSB >=0; k_LSB--){
            if (input_readable()){break;}

            ADDRESS_LSB_PORT = (uint16_t)(k_LSB); 
            DATABUS.input();
            OE = 0; 
            READ_WORD = DATABUS.read();
            OE = 1;            
            if (READ_WORD != PATTERN_ALL_1){
                nprintf("0x%02X%04X,0x%04X,0x%04X,R%i%c\n", k_MSB, (unsigned int)k_LSB, unscramble_content(READ_WORD), PATTERN_ALL_1, (int)round, phase);
                
                NBF +=1;
            }           
            DATABUS.output();
            DATABUS = PATTERN_ALL_0;
            WE = 0; WE = 1;      
        }
    }
}

void R1W0R05UP(int32_t round, char phase){

    uint16_t READ_WORD;

    CSRAM = 0; WE = 1; OE = 1; UB = 0; LB = 0;

    for (uint8_t k_MSB = 0; k_MSB < N_SECTIONS16b; k_MSB++){

        if (input_readable()){break;} 
        ADDRESS_MSB_PORT = k_MSB;

        for (uint32_t k_LSB=0; k_LSB < SRAM_BLOCKSIZE; k_LSB++){
            if (input_readable()){break;}

            ADDRESS_LSB_PORT = k_LSB; 
            DATABUS.input();
            OE = 0; 
            READ_WORD = DATABUS.read();
            OE = 1;            
            if (READ_WORD != PATTERN_ALL_1){
                nprintf("0x%02X%04X,0x%04X,0x%04X,R%i%c\n", k_MSB, (unsigned int)k_LSB, unscramble_content(READ_WORD), PATTERN_ALL_1, (int)round, phase);
                
                NBF+=1;
            }           
            DATABUS.output();
            DATABUS = PATTERN_ALL_0;
            WE = 0; WE = 1;            

            DATABUS.input();
            for (uint8_t kread = 0; kread<5; kread++){
                OE = 0; 
                OE = 1;
            }
        }
    }
}

void R0W1R05UP(int32_t round, char phase){

    uint16_t READ_WORD;

    CSRAM = 0; WE = 1; OE = 1; UB = 0; LB = 0;

    for (uint8_t k_MSB = 0; k_MSB < N_SECTIONS16b; k_MSB++){

        if (input_readable()){break;}
        ADDRESS_MSB_PORT = k_MSB;

        for (uint32_t k_LSB=0; k_LSB < SRAM_BLOCKSIZE; k_LSB++){
            if (input_readable()){break;}

            ADDRESS_LSB_PORT = k_LSB; 
            DATABUS.input();
            OE = 0; 
            READ_WORD = DATABUS.read();
            OE = 1;            
            if (READ_WORD != PATTERN_ALL_0){
                nprintf("0x%02X%04X,0x%04X,0x%04X,R%i%c\n", k_MSB, (unsigned int)k_LSB, unscramble_content(READ_WORD), PATTERN_ALL_0, (int)round, phase);
                
                NBF+=1;
            }           
            DATABUS.output();
            DATABUS = PATTERN_ALL_1;
            WE = 0; WE = 1;            

            DATABUS.input();
            for (uint8_t kread = 0; kread<5; kread++){
                OE = 0; 
                OE = 1;
            }
        }
    }
}

void R0W1R05DOWN(int32_t round, char phase){

    uint16_t READ_WORD;

    CSRAM = 0; WE = 1; OE = 1;  UB = 0; LB = 0;

    for (int8_t k_MSB = N_SECTIONS16b-1; k_MSB >=0; k_MSB--){

        if (input_readable()){break;}
        ADDRESS_MSB_PORT = (uint8_t)(k_MSB);

        for (int32_t k_LSB=SRAM_BLOCKSIZE-1; k_LSB >=0; k_LSB--){
            
            if (input_readable()){break;}

            ADDRESS_LSB_PORT = (uint16_t)(k_LSB); 
            DATABUS.input();

            OE = 0; 
            READ_WORD = DATABUS.read();
            OE = 1;            
            if (READ_WORD != PATTERN_ALL_0){
                nprintf("0x%02X%04X,0x%04X,0x%04X,R%i%c\n", k_MSB, (unsigned int)k_LSB, unscramble_content(READ_WORD), PATTERN_ALL_0,(int) round, phase);
                
                NBF+=1;
            }           
            DATABUS.output();
            DATABUS = PATTERN_ALL_1;
            WE = 0; WE = 1;      

            DATABUS.input();
            for (uint8_t kread = 0; kread<5; kread++){
                OE = 0; 
                OE = 1;
            }
        }
    }
}

void R1W0R05DOWN(int32_t round, char phase){

    uint16_t READ_WORD;

    CSRAM = 0; WE = 1; OE = 1; UB = 0; LB = 0;

    for (int8_t k_MSB = N_SECTIONS16b-1; k_MSB >=0; k_MSB--){

        if (input_readable()){break;}
        ADDRESS_MSB_PORT = (uint8_t)(k_MSB);

        for (int32_t k_LSB=SRAM_BLOCKSIZE-1; k_LSB >=0; k_LSB--){

            if (input_readable()){break;}
            ADDRESS_LSB_PORT = (uint16_t)(k_LSB); 
            DATABUS.input();
            OE = 0; 
            READ_WORD = DATABUS.read();
            OE = 1;            
            if (READ_WORD != PATTERN_ALL_1){
                nprintf("0x%02X%04X,0x%04X,0x%04X,R%i%c\n", k_MSB,(unsigned int) k_LSB, unscramble_content(READ_WORD), PATTERN_ALL_1,(int) round, phase);
                
                NBF+=1;
            }                       
            DATABUS.output();
            DATABUS = PATTERN_ALL_0;
            WE = 0; WE = 1;     
            DATABUS.input();
            for (uint8_t kread = 0; kread<5; kread++){
                OE = 0; 
                OE = 1;
            } 
        }
    }
}

void RunDynamicClassic(int32_t round){

    uint16_t READ_WORD;

    char phase;

    DATABUS.output();
    UB = 0;
    LB = 0;
    WE = 1;
    OE = 1;
    CSRAM = 0;

    DATABUS = PATTERN_ALL_0;

    for (uint8_t k_MSB = 0; k_MSB < N_SECTIONS16b; k_MSB++){

        if (input_readable()){break;}

        ADDRESS_MSB_PORT = k_MSB;

        for (uint32_t k_LSB=0; k_LSB < SRAM_BLOCKSIZE; k_LSB++){

            ADDRESS_LSB_PORT = k_LSB; 

            WE = 0; WE = 1;          
        }

    }
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE 
        write_address(0x12, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif

    //nprintf("\tR0++\n");
    phase = 'A';

    DATABUS.input();

    for (uint8_t k_MSB = 0; k_MSB < N_SECTIONS16b; k_MSB++){

        if (input_readable()){break;}

        ADDRESS_MSB_PORT = k_MSB;

        for (uint32_t k_LSB=0; k_LSB < SRAM_BLOCKSIZE; k_LSB++){

            ADDRESS_LSB_PORT = k_LSB; 

            OE = 0; 
            READ_WORD = DATABUS.read();
            OE = 1;            
            if (READ_WORD != PATTERN_ALL_0){
                nprintf("0x%02X%04X,0x%04X,0x%04X,R%i%c\n", k_MSB, (unsigned int)k_LSB, unscramble_content(READ_WORD), PATTERN_ALL_0,(int) round, phase);
                
                NBF+=1;
            }
            
        }
    }

    DATABUS.output();

    DATABUS = PATTERN_ALL_1;

    for (uint8_t k_MSB = 0; k_MSB < N_SECTIONS16b; k_MSB++){

        if (input_readable()){break;}

        ADDRESS_MSB_PORT = k_MSB;

        for (uint32_t k_LSB=0; k_LSB < SRAM_BLOCKSIZE; k_LSB++){

            ADDRESS_LSB_PORT = k_LSB; 

            WE = 0; WE = 1;            
        }
    }
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x12, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif

    DATABUS.input();
    phase++;

    for (int8_t k_MSB = N_SECTIONS16b-1; k_MSB >=0; k_MSB--){

        if (input_readable()){break;}

        ADDRESS_MSB_PORT = (uint8_t)k_MSB;

        for (int32_t k_LSB=SRAM_BLOCKSIZE-1; k_LSB >=0; k_LSB--){

            ADDRESS_LSB_PORT = (uint16_t)(k_LSB); 

            OE = 0; 
            READ_WORD = DATABUS.read();
            OE = 1;            
            if (READ_WORD != PATTERN_ALL_1){
                //nprintf("\tROUND %i ADDRESS: 0x%02X%04X CONTENT: 0x%04X PATTERN 0x%04X\n", round, k_MSB, k_LSB, READ_WORD, PATTERN_ALL_1);
                nprintf("0x%02X%04X,0x%04X,0x%04X,R%i%c\n", k_MSB, (unsigned int)k_LSB, unscramble_content(READ_WORD), PATTERN_ALL_1, (int)round, phase);
                
                NBF+=1;
            }           
        }
    }

    CSRAM = 1;
    WE = 1;
    OE = 1;
}

void RunMarchTest(int32_t round){

    char phase;

    UB = 0;
    LB = 0;
    CSRAM = 0;
    WE = 1;
    OE = 1;

    phase = 'A';
    R0W1UP(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x12, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    phase++;
    R1W0UP(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x01, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    phase++;
    R0W1DOWN(round, phase);    
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x01, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    phase++;
    R1W0DOWN(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x01, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    CSRAM = 1;
    WE = 1;
    OE = 1;
}

void RunMatsPlus(int32_t round){

    char phase;
   
    WE = 1; OE = 1; UB = 0; LB = 0; CSRAM = 0; 
    phase = 'A';
    R0W1UP(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x01, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    phase++;
    R1W0DOWN(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x10, 0x6534, 0x789A); CSRAM=0;
        write_address(0x08, 0x7654, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    CSRAM = 1;
    WE = 1;
    OE = 1;

}

void Run_Tst_Pseud(){
    
}

void Run_m_MatsPlus(int32_t round){

    char phase;

    UB = 0;
    LB = 0;
    CSRAM = 0;
    WE = 1;
    OE = 1;
    phase = 'A';
    R0W1UP(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x01, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    phase++;
    R1W0UP(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x10, 0x6534, 0x789A); CSRAM=0;
        write_address(0x08, 0x7654, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    CSRAM = 1;
    WE = 1;
    OE = 1;

}

void RunDynamicStress(int32_t round){

    char phase;

    UB = 0;
    LB = 0;
    CSRAM = 0;
    WE = 1;
    OE = 1;

    phase = 'A';
    R1W0R05UP(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x12, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    phase++;
    R0W1R05UP(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x12, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    phase++;
    R1W0R05UP(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x12, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    phase++;
    R0W1R05DOWN(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x12, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    phase++;
    R1W0R05DOWN(round, phase);
    #ifdef INJECT_FAILS_IN_DYNAMIC_MODE
        write_address(0x12, 0x3456, 0x789A); CSRAM=0;
        write_address(0x03, 0x4567, 0x89AB); CSRAM=0;
        write_address(0x1F, 0xFFFF, 0x1000); CSRAM=0;
        write_address(0x00, 0x0000, 0x0001); CSRAM=0;
    #endif
    phase++;
    R0W1R05DOWN(round, phase);
    CSRAM = 1;
    WE = 1;
    OE = 1;
}

int16_t set_latchup_voltage_threshold(int16_t LimitCurrent){

    // Limit current is expressed in mA. Using the equations from the schematic.

    return (int16_t) (LimitCurrent-(int16_t) (VCC_NOMINAL/RQ))*CURRSENSE_G;
    
}

void LatchUpDetected(void){ //function not used.

    if (ENABLE_LATCHUP_DETECTION_IN_SBY==false)
        return;

    /*if (MODEL_TECH!=TECHPSRAM){
        DATABUS.output();
        DATABUS = PATTERN_ALL_0;
        ADDRESS_MSB_PORT = 0b00000;
        ADDRESS_LSB_PORT = PATTERN_ALL_0;            
        CSRAM = 0;      
        WE = 0;
        OE = 0;
    }else{ 
        CSRAM = 0;                        
    }

    set_output_voltage_vcc(0, 3300);*/
    usb_port.write("\nLATCHUP\n", 9);
    system_reset();
    /*ThisThread::sleep_for(1s);
    set_output_voltage_vcc(0, VOLT_REF_LATCHUP);
    STATE = s_START;*/
    
}



void write_full_SRAM_spi(void){

    uint16_t page_index;

    uint8_t  ADDRESS_MSB_temp, ADDRESS_INT_temp;

    sram_spi_port.format(SPI_PSRAM_WORD_LENGTH, SPI_MODE_PSRAM);
    sram_spi_port.frequency(SPI_CLK_PSRAM_SPEED);

    for (page_index=0; page_index<PSRAM_SIZE_IN_PAGES;page_index++){

        ADDRESS_INT_temp = (uint8_t) ((page_index << 2) & 0x00FC);
        ADDRESS_MSB_temp = (uint8_t) ((page_index >> 6) & 0x003F);

        write_page_spi(ADDRESS_MSB_temp, ADDRESS_INT_temp, PATTERN_8);
        wait_us(1);
    }
}

void _mainSleepSRAM(char &c1){
    float   QuiescentCurrent = 0.0;

    nprintf("\nACTION  \t:Sleeping - 0x%04X - %i - TIME:%i\n", PATTERN, VCC_IN_STANDBY, (int)Time_from_EPOCH());
    
    nprintf(FLAG_FOR_ACTION_START);                        
    if (MODEL_TECH!=TECHPSRAM){
        DATABUS.output();
        DATABUS = PATTERN_ALL_0;
        ADDRESS_MSB_PORT = 0b00000;
        ADDRESS_LSB_PORT = PATTERN_ALL_0;     
        wait_us(SRAM_TIME_TO_SETUP);           
        CSRAM = 0;
        wait_us(SRAM_TIME_TO_SETUP);           
        WE = 0;
        OE = 0;
    }else{ 
        CSRAM = 0;
    }
    set_output_voltage_softly_vcc(VCC_IN_STANDBY, VOLT_REF_LATCHUP);
    nprintf(FLAG_FOR_ACTION_END);

    while (!(input_readable())) {

        if (ENABLE_CURRENT_SURVEY_IN_SBY){
            QuiescentCurrent = Calculate_QC(); // In mA.                                
            nprintf("VCC:%i,IQ:%.2f,T:%.1f\n", (int)MEASURE_VCC,QuiescentCurrent, Measure_external_temperature());
            
            ThisThread::sleep_for(chrono::milliseconds(TIME_CURRENT_SAMPLES_in_ms));
        }
    }
    if(input_readable()){
        while((c1 = ngetchar()) <0){};;
    }; // This action cleans the buffer.
    nprintf("\nWAKING - TIME:%i", (int)Time_from_EPOCH());
    
    nprintf(FLAG_FOR_ACTION_START);
    
    set_output_voltage_softly_vcc(VCC_NOMINAL, VOLT_REF_LATCHUP);
    wait_us(SRAM_TIME_TO_SETUP);           
    CSRAM = 1;
    nprintf(FLAG_FOR_ACTION_END);
    
    STATE = s_CHOOSE_ACTION;
}

void DisableECC_065nm(void){

    const uint16_t keyLSB[25]={ 
        0x2000, 0x0004, 0x0002, 0x2002, 0x0004,
        0x0002, 0x2004, 0x2000, 0x2006, 0x2002,
        0x0000, 0x0000, 0x2000, 0x0006, 0x2002, 
        0x0004, 0x0006, 0x2000, 0x2004, 0x2002,
        0x0000, 0x2006, 0x0002, 0x2006, 0x2000
    };
    const uint8_t keyMSB[25]={
        0x00, 0x01, 0x00, 0x01, 0x00,
        0x01, 0x01, 0x01, 0x00, 0x00,
        0x00, 0x01, 0x01, 0x01, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x01, 0x01, 0x00, 0x00
    };

    //nprintf("\nDisabling ECC...\n");
    DATABUS.output();
    DATABUS = PATTERN_ALL_0;
    OE = 0;
    WE = 0;
    CSRAM = 0;
    UB = 0;
    LB = 0;

    for (int k=0; k<25; k++){

        ADDRESS_MSB_PORT = keyMSB[k];
        ADDRESS_LSB_PORT = keyLSB[k];
        wait_us(1);
        OE = 1;
        wait_us(1);
        OE = 0;
        wait_us(1);
    }

    CSRAM = 1;
    OE = 1;
    WE = 1;

    // The erasing has corrupted the original values in 25 addresses. Fixing to avoid annoying noisy text.

    for (int k=0; k<25; k++){

        write_address(keyMSB[k], keyLSB[k], PATTERN);

        wait_us(1);
    }

}

uint16_t unscramble_content(uint16_t Content){

    const uint16_t LookupTable[16]={
        0b0000000000000001, // Bit  0 bus <-- bit  0 Cypress
        0b0000000000000100, // Bit  2 bus <-- bit  1 Cypress
        0b0000000010000000, // Bit  7 bus <-- bit  2 Cypress
        0b0000000000100000, // Bit  5 bus <-- bit  3 Cypress
        0b0001000000000000, // Bit 12 bus <-- bit  4 Cypress
        0b0100000000000000, // Bit 14 bus <-- bit  5 Cypress
        0b0000100000000000, // Bit 11 bus <-- bit  6 Cypress
        0b0000001000000000, // Bit  9 bus <-- bit  7 Cypress
        0b0000000000000010, // Bit  1 bus <-- bit  8 Cypress
        0b0000000000001000, // Bit  3 bus <-- bit  9 Cypress
        0b0000000001000000, // Bit  6 bus <-- bit 10 Cypress
        0b0000000000010000, // Bit  4 bus <-- bit 11 Cypress
        0b0010000000000000, // Bit 13 bus <-- bit 12 Cypress
        0b1000000000000000, // Bit 15 bus <-- bit 13 Cypress
        0b0000010000000000, // Bit 10 bus <-- bit 14 Cypress
        0b0000000100000000, // Bit  8 bus <-- bit 15 Cypress
    };

    uint16_t unscrambled_CONTENT = 0x0000;
    uint16_t copy_of_CONTENT = Content;

    if(MODEL_TECH == TECH040nm){
        unscrambled_CONTENT = Content;
    }else{

        unscrambled_CONTENT = 0x0000;

        for (size_t k=0; k<16; k++){

            if((copy_of_CONTENT & 0x0001)==0x0001){
                unscrambled_CONTENT = unscrambled_CONTENT | LookupTable[k];
            }
            copy_of_CONTENT = copy_of_CONTENT >>1;
        }
    }

    return unscrambled_CONTENT;
}

uint16_t scramble_pattern(uint16_t Pattern){

    const uint16_t LookupTable[16]={
        0b0000000000000001, // Bit  0 bus --> bit  0 Cypress
        0b0000000100000000, // Bit  1 bus --> bit  8 Cypress
        0b0000000000000010, // Bit  2 bus --> bit  1 Cypress
        0b0000001000000000, // Bit  3 bus --> bit  9 Cypress
        0b0000100000000000, // Bit  4 bus --> bit 11 Cypress
        0b0000000000001000, // Bit  5 bus --> bit  3 Cypress
        0b0000010000000000, // Bit  6 bus --> bit 10 Cypress
        0b0000000000000100, // Bit  7 bus --> bit  2 Cypress
        0b1000000000000000, // Bit  8 bus --> bit 15 Cypress
        0b0000000010000000, // Bit  9 bus --> bit  7 Cypress
        0b0100000000000000, // Bit 10 bus --> bit 14 Cypress
        0b0000000001000000, // Bit 11 bus --> bit  6 Cypress
        0b0000000000010000, // Bit 12 bus --> bit  4 Cypress
        0b0001000000000000, // Bit 13 bus --> bit 12 Cypress
        0b0000000000100000, // Bit 14 bus --> bit  5 Cypress
        0b0010000000000000, // Bit 15 bus --> bit 13 Cypress
    };

    uint16_t scrambled_PATTERN = 0x0000;
    uint16_t copy_of_PATTERN = Pattern;

    if(MODEL_TECH == TECH040nm){
        scrambled_PATTERN = Pattern;
    }else{

        scrambled_PATTERN = 0x0000;

        for (size_t k=0; k<16; k++){

            if((copy_of_PATTERN & 0x0001)==0x0001){
                scrambled_PATTERN = scrambled_PATTERN | LookupTable[k];
            }
            copy_of_PATTERN = copy_of_PATTERN >> 1;
        }
    }

    return scrambled_PATTERN;
}

void write_address(uint8_t MSB_ADDRESS, uint32_t LSB_ADDRESS, uint16_t TARGET_PATTERN){

    OE = 1;
    
    ADDRESS_LSB_PORT.write((uint16_t) LSB_ADDRESS);

    ADDRESS_MSB_PORT.write(MSB_ADDRESS);

    DATABUS.output();
    DATABUS.write(scramble_pattern(TARGET_PATTERN));
//    nprintf("PATTERN: 0x%04X\t -->0x%04X\n", TARGET_PATTERN, scramble_pattern(TARGET_PATTERN));
    wait_ns(SRAM_TPW);

    CSRAM = 0;
    wait_ns(SRAM_TPW);

    WE = 0;
    wait_ns(200);
    WE = 1;
    wait_ns(SRAM_TPW);

    CSRAM = 1;
    DATABUS.input();
}

void write_full_memory(void){

    LB = 0;
    UB = 0;
    OE = 1;
    WE = 1;
    wait_us(SRAM_TIME_TO_SETUP);
    CSRAM = 0;
    DATABUS.output();
    DATABUS = scramble_pattern(PATTERN);
//    nprintf("PATTERN: 0x%04X\t -->0x%04X\n", PATTERN, scramble_pattern(PATTERN));
    

    for (uint8_t k_MSB = 0; k_MSB < N_SECTIONS16b; k_MSB++){

        ADDRESS_MSB_PORT = k_MSB;

        for (uint32_t k_LSB=0; k_LSB < SRAM_BLOCKSIZE; k_LSB++){

            ADDRESS_LSB_PORT = (uint16_t) k_LSB;
            WE = 0;
            WE = 1;
        }
    }

    CSRAM = 1;
    wait_us(SRAM_TIME_TO_SETUP);
    OE = 1;
    WE = 1;
    wait_us(SRAM_TIME_TO_SETUP);
    DATABUS = PATTERN_ALL_0;
    DATABUS.input();
}

void Inject_fails_in_static(void){
    write_address(0b00000, 0x0000, 0xABCD);
    write_address(0b00000, 0x0001, 0xBCDE);
    write_address(0b00000, 0x0002, 0xCDEF);
    write_address(0b00000, 0x0004, 0xDEF0);
    write_address(0b00000, 0x0008, 0xEF01);
    write_address(0b00000, 0x0010, 0xF012);
    write_address(0b00000, 0x0020, 0x0123);
    write_address(0b00000, 0x0040, 0x1234);
    write_address(0b00000, 0x0080, 0x2345);
    write_address(0b00000, 0x0100, 0x3456);
    write_address(0b00000, 0x0200, 0x4567);
    write_address(0b00000, 0x0400, 0x5678);
    write_address(0b00000, 0x0800, 0x6789);
    write_address(0b00000, 0x1000, 0x789A);
    write_address(0b00000, 0x2000, 0x89AB);
    write_address(0b00000, 0x4000, 0x9ABC);
    write_address(0b00000, 0x8000, 0xFEDC);
    write_address(0b00001, 0x0000, 0xEDCB);
    write_address(0b00010, 0x0000, 0xDCBA);
    write_address(0b00100, 0x0000, 0xCBA9);
    write_address(0b01000, 0x0008, 0xBA98);
    write_address(0b01111, 0xFFFF, 0xA987);
    if(MODEL_TECH==TECH040nm){
        write_address(0b10000, 0x0000, 0x9876);
        write_address(0b11111, 0xFFFF, 0x8765);
    }
}

uint16_t read_address(uint8_t MSB_ADDRESS, uint32_t LSB_ADDRESS){

    uint16_t CONTENT;

    ADDRESS_MSB_PORT = MSB_ADDRESS;
    ADDRESS_LSB_PORT  = (uint16_t) LSB_ADDRESS;

    CSRAM = 0;

    OE = 0;

    DATABUS.input();

    CONTENT = unscramble_content(DATABUS.read());

    OE = 1;
    CSRAM = 1;

    return CONTENT;

}

void read_full_memory(void){

    uint16_t CONTENT, scrambled_PATTERN;

    DATABUS.input();
    LB = 0;
    UB = 0;
    OE = 1;
    WE = 1;
    wait_us(SRAM_TIME_TO_SETUP);
    CSRAM = 0;

    scrambled_PATTERN = scramble_pattern(PATTERN);

    for (uint8_t k_MSB = 0; k_MSB < N_SECTIONS16b; k_MSB++){

        ADDRESS_MSB_PORT = k_MSB;
        for (uint32_t k_LSB=0; k_LSB < SRAM_BLOCKSIZE; k_LSB++){            
            ADDRESS_LSB_PORT = (uint16_t) k_LSB;
            OE = 0;
            CONTENT = DATABUS.read();
            OE = 1;
            if (CONTENT != scrambled_PATTERN){
                nprintf("0x%02X%04X,0x%04X,0x%04X\n", 
                    k_MSB,  (unsigned int)k_LSB,
                    unscramble_content(CONTENT), 
                    PATTERN
                    );                   
                write_address(k_MSB,k_LSB, PATTERN);    
                CSRAM = 0;  // necessary. write_address left CSRAM up.     
                NBF +=1;
            }
        }
    }
    CSRAM = 1;
    wait_us(SRAM_TIME_TO_SETUP);
    OE = 1;
    WE = 1;
    nprintf("\n");

}


void write_page_spi(uint8_t ADDRESS_MSB, uint8_t ADDRESS_INT, uint8_t PATTERN){

    CSRAM = 0;

    sram_spi_port.write(PSRAM_SPI_WRITE_CODE);
    sram_spi_port.write(ADDRESS_MSB);
    sram_spi_port.write(ADDRESS_INT);
    sram_spi_port.write(0x00);

    for (uint16_t word=0;word<PSRAM_PAGE_SIZE;word++){

        sram_spi_port.write(PATTERN_8);

    }
    wait_us(1);
    CSRAM = 1;
}

void read_full_SRAM_spi(void){
    
    uint8_t READ_PATTERN[PSRAM_PAGE_SIZE];

    uint8_t  ADDRESS_MSB_temp=0, ADDRESS_INT_temp=0;

    uint16_t word_index=0;

    NBF = 0;

    sram_spi_port.format(SPI_PSRAM_WORD_LENGTH, SPI_MODE_PSRAM);
    sram_spi_port.frequency(SPI_CLK_PSRAM_SPEED);

    /* The following 4 lines are included to get rid of the odd behaviour of the PSRAM after
    waking up in some circunstances. Somehow, the SPI blocks fails at reading the first page, 
    so a mock read is done before the real one. 
    */    
    /// Starting the mock reading.
    CSRAM = 0;
    sram_spi_port.write(PSRAM_SPI_READ_CODE);
    for (uint16_t k = 0; k < PSRAM_PAGE_SIZE+3;k++){spi_port.write(0x00);}
    CSRAM = 1;
    // ending the mock reading.

    for (uint16_t page_index = 0; page_index < PSRAM_SIZE_IN_PAGES; page_index++){

        // We'll read in page mode.

        ADDRESS_INT_temp = (uint8_t) ((page_index << 2) & 0x00FC);
        ADDRESS_MSB_temp = (uint8_t) ((page_index >> 6) & 0x003F);

        CSRAM = 0;

        // The following lines start the read page mode.
        sram_spi_port.write(PSRAM_SPI_READ_CODE);
        sram_spi_port.write(ADDRESS_MSB_temp);
        sram_spi_port.write(ADDRESS_INT_temp);
        sram_spi_port.write(0x00); 

        // collecting data from the page.

        for (word_index=0;word_index<PSRAM_PAGE_SIZE;word_index++){

            READ_PATTERN[word_index]=sram_spi_port.write(0x00);

        }

        CSRAM = 1;

        // Now, let us look for discrepancies.

        for (word_index=0;word_index<PSRAM_PAGE_SIZE;word_index++){

            if(READ_PATTERN[word_index] !=PATTERN_8){
                // ERROR! but we need to confirm it.
                uint32_t WRONG_ADDRESS = (uint32_t)ADDRESS_MSB_temp;
                WRONG_ADDRESS = (WRONG_ADDRESS*64*1024+ADDRESS_INT_temp*256)+word_index; // I must think about this, but it works!!!!
                                    
                uint8_t CONTENT2 = read_address_spi(WRONG_ADDRESS);
                wait_us(TIME_BETWEEN_REDUNDANT_READINGS);
                uint8_t CONTENT3 = read_address_spi(WRONG_ADDRESS);

                uint8_t TMR_CONTENT = (READ_PATTERN[word_index]&&CONTENT2)||(READ_PATTERN[word_index]&&CONTENT3)||(CONTENT2&&CONTENT3);

                if (TMR_CONTENT != PATTERN){

                    nprintf("0x%06X,0x%02X,0x%02X\n", 
                                            page_index*1024+word_index,  
                                            READ_PATTERN[word_index], 
                                            PATTERN_8
                                        ); 
                      
                    NBF +=1;                                      
                    write_address_spi(WRONG_ADDRESS, PATTERN_8);          
                }              
            }
                            
        }
    }
}

void write_address_spi(uint32_t tADDRESS, uint8_t tPATTERN){

    uint8_t ADDRESS_MSB, ADDRESS_CEN, ADDRESS_LSB;

    ADDRESS_LSB = (uint8_t) (tADDRESS & 0x000000FF);

    ADDRESS_CEN = (uint8_t) ((tADDRESS >> 8)  & 0x000000FF);

    ADDRESS_MSB = (uint8_t) ((tADDRESS >> 16)  & 0x000000FF);

    CSRAM = 0;

    sram_spi_port.write(PSRAM_SPI_WRITE_CODE);

    sram_spi_port.write(ADDRESS_MSB);

    sram_spi_port.write(ADDRESS_CEN);

    sram_spi_port.write(ADDRESS_LSB);

    sram_spi_port.write(tPATTERN);

    CSRAM = 1;

}

uint8_t read_address_spi(uint32_t tADDRESS){

    uint8_t ADDRESS_MSB, ADDRESS_CEN, ADDRESS_LSB, CONTENT;

    ADDRESS_LSB = (uint8_t) (tADDRESS & 0x000000FF);

    ADDRESS_CEN = (uint8_t) ((tADDRESS >> 8)  & 0x000000FF);

    ADDRESS_MSB = (uint8_t) ((tADDRESS >> 16)  & 0x000000FF);

    CSRAM = 0;

    sram_spi_port.write(PSRAM_SPI_READ_CODE);

    sram_spi_port.write(ADDRESS_MSB);

    sram_spi_port.write(ADDRESS_CEN);

    sram_spi_port.write(ADDRESS_LSB);

    CONTENT = spi_port.write(0x00);

    CSRAM = 1;

    return CONTENT;
}

void Inject_fails_in_static_spi(void){

    const int N_INJECTIONS = 8;

    const uint32_t INJECTED_ADDRESSES[N_INJECTIONS]={
        0x000000,
        0x000001,
        0x00ABCD,
        0x038765,
        0x111234,
        0x254321,
        0x3FFF01,
        0x3FFFFF
    };

    const uint8_t INJECTED_VALUES[N_INJECTIONS]={
        0x11,
        0x22,
        0x33,
        0x44,
        0x66,
        0x77,
        0x88,
        0x99
    };

    for (size_t k=0; k<N_INJECTIONS; k++){

        write_address_spi(INJECTED_ADDRESSES[k], INJECTED_VALUES[k]);

    }
}

void disable_deadman(void){
    periodicDotTicker.detach();
}

void enable_deadman(void){
    periodicDotTicker.attach(&send_dot_irq, 30s);
}

void send_dot_irq(void) {
    usb_port.write(".", 1);
}


#ifdef TARGET_NUCLEO_F767ZI

    void connection_start_timer() {
        ThisThread::sleep_for(30s);
        if (!tcpConnection) {
            
                nprintf("Error: No se ha establecido una conexión con ningún cliente después de 30 segundos.\n"); 
            
            server.close();
        }
    }           

    bool try_TCP_connection(void){

        
        nsapi_size_or_error_t result = net.connect();
        if (result != 0) {
            printf("error al conectar\n");
            return false;
        }

        // Mostramos IP
        SocketAddress ip;
        
        result = net.get_ipv6_link_local_address(&ip);
        if (result != 0) {
            return false;
        }
        
        printf("Conectado. IP local: %s\n", ip.get_ip_address());
        
        
        server.open(&net);
        server.bind(TCP_PORT);
        server.listen();

        printf("Servidores TCP escuchando en el puerto %d\n", TCP_PORT); 
        printf("Puedes hacer ping a %s y conectarte vía TCP.\n", ip.get_ip_address());

        Thread hilo_timer;
        hilo_timer.start(connection_start_timer);

        SocketAddress   clientAddress;
        client = server.accept();
        //if server is closed by timeout thread client == nullptr 
        if(client==nullptr){
            
                nprintf("Servidor detenido por timeout sin conexión.\n"); 
            
            net.disconnect();
            return false;
        }
        
        client->getpeername(&clientAddress);
        
            nprintf("Conexión obtenida.\n"); 
            nprintf("Cliente escuchando desde: %s\n", clientAddress.get_ip_address());
        
        tcpConnection = true;
        
        return true;
    }

#endif


//WRAPPER FUNCTIONS

/* I tried overwritting the console, like using dup, with the function at the beggining of the program, however MBED is a picky master
   it proved impossible to overwrite the console mid-program, as we wanted, since it is only possible to do so at startup and
    we don't know at startup if we will have a TCP connection. Therefore we need this functions, which are a wrapper for scanf and nprintf, pretty simple,
    if we have a TCP connection we will use it if not we just use STDout or STDin. 
    ONE BIG DIFFERENCE theese functions are BLOCKING FUNCTIONS as for the nature of the TCP server it is possible to change this.
    obvs this is not a problem for scanf
*/
int nprintf(const char* format, ...) {
    char outputBuffer[512];
    va_list args;
    va_start(args, format);

    int len = vsnprintf(outputBuffer, sizeof(outputBuffer), format, args);

    va_end(args);

    if (len < 0) {
        return -1;
    }
    #ifdef TARGET_NUCLEO_F767ZI
        if (tcpConnection && client != nullptr) {
            int toSend = (len < sizeof(outputBuffer)) ? len : sizeof(outputBuffer) - 1;
            int sent = client->send(outputBuffer, toSend);
            return sent;
        } 
    #endif
    
    return printf("%s", outputBuffer);
    

}


int nscanf(const char* format, ...) {
    char inputBuffer[2] = {0};
    va_list args;
    va_start(args, format);

    int ret = 0;
    #ifdef TARGET_NUCLEO_F767ZI
        if (tcpConnection && client != nullptr) {
            int received = client->recv(inputBuffer, sizeof(inputBuffer) - 1);
            
                nprintf("%s\n",inputBuffer); 
            
            if (received <= 0) {
                va_end(args);
                printf("conexion perdida\n");
                server.close();
                net.disconnect();
                tcpConnection = false;
                return -1;
            }
            inputBuffer[received] = '\0';
            ret = vsscanf(inputBuffer, format, args);
        }
    #endif

    if(!tcpConnection){
        ret = vscanf(format, args);
    }

    va_end(args);
    return ret;
}

char ngetchar(void) {
    
    if(!tcpConnection){
        return getchar();
    }

    #ifdef TARGET_NUCLEO_F767ZI
        char ch='z';
        int received = client->recv(&ch, 1);
        if (received == 1) {
            printf("%c\n",ch);
            return ch;
        } 
        else if(received <= 0){
            printf("conexion perdida\n");
            server.close();
            net.disconnect();
            tcpConnection = false;
        }
    #endif


    return -1;
}

bool input_readable(void){
    #ifdef TARGET_NUCLEO_F767ZI
    char buffer[512] = {0};
    
    if(tcpConnection){
        client->set_blocking(false);
        nsapi_size_or_error_t n = client->recv(buffer, sizeof(buffer));
        printf("%i\n",n);
        client->set_blocking(true);
        return !(n == NSAPI_ERROR_WOULD_BLOCK);
    }
    #endif
    return usb_port.readable();
    
}