/*
          Analog_Devices_AD4020_AD5791_Loopback_Test.ino 
Author: Jarvis Hill (hilljarvis@gmail.com)
Purpose: Use an Arduino UNO (Atmega328) to perform a loopback between a arbitrary waveform generator, Analog Devices EVAL-AD400xFMCZ 20-bit AD4020 ADC demo board and the 
         Analog Devices EVAL-AD5791SDZ 20-bit AD5791 DAC demo board.

Usage:
         1. Function generator is conneced to AN+, AN- inputs of the EVAL-AD400xFMCZ (see datasheet for acceptable input voltages)
         2. Connect AD4020 and AD5791 demo board signals to Arduino UNO per the Loopback Test Connection Diagram in sketch folder
         3. Connect additional +5V and +12V main power supplies to the ADC and DAC demo boards per the connection diagram.
                  
References: ADI4020 and AD5791 datasheets
*/


/*INCLUDES*/
#include <SPI.h>                //Required for SPI 



/*GLOBALS*/
//AD4020 control signals
#define conv     PIN7           //ADC CNV signal (replaces  SPI chip-select signal)
#define conv_ddr DDD7         

//AD5791 DAC control signals
#define nLDAC      PIN6       
#define nLDAC_ddr  DDD6
#define nSYNC      PIN5
#define nSYNC_ddr  DDD5


/*GPIO states*/
#define ON  true
#define OFF false


/*Sprintf buffer used for sending debugging messages to serial terminal*/
//char buf [100];


/*ADC*/
//#define N_conv 200              //Number of conversions/samples
//unsigned long ADC_data[N_conv] = {0};
uint8_t byte2;
unsigned long byte0, ADC_conv_value;
float vin_adc;
uint16_t byte1;


/*SPI Settings*/
SPISettings settingsA(50000000, MSBFIRST, SPI_MODE1); 


/*DAC*/
#define Vrefn -10
#define Vrefp  10
#define Vrefp  10
#define dac_bit_size 20
#define DAC_write_cmd 0x10

uint8_t DAC_byte0,DAC_byte1,DAC_byte2;
unsigned long DAC_code;


/*FUNCTIONS*/

/**********************************************************************
 * init_ADC_DAC_io
 * Purpose: Configure I/O for ADC AD4020 and DAC AD5791 communications
 **********************************************************************/
void init_ADC_DAC_io(){
  //intialize AD4020 and AD5791 control signals (all SPI signals are handled by SPI.begin() function in setup)
  DDRD = (1 << conv_ddr) | (1 << nSYNC) | (1<<nLDAC);                           //Set signals to output

  PORTD = (OFF << conv) | (ON << nSYNC) | (OFF <<nLDAC_ddr);                    //Init pin logic levels, OFF = LOW, ON = HIGH
  
}


/****************************************************************************************************
 * ADC_read_conv
 * Purpose: Triggers AD4020 conversion and reads back the converted analog data
 * Note: EVAL-AD4020 ADC has nanosecond SPI timing requirements. This code + 16 MHz 
 *       Arduino UNO can't run fast enough to cause timing conflicts so that's why 
 *       there's no delays within the SPI transaction.
****************************************************************************************************/
unsigned long ADC_read_conv(){

    //Init conversion value 
    ADC_conv_value = 0; 

    //Read 20-bit ADC sample   
    SPI.beginTransaction(settingsA);             //Start SPI transaction using desired SPI settings and driving Arduino SS pin LOW but this code is not using it
    
    PORTD &= ~(ON << conv);                      //Init conversion by pulsing ADC CNV signal (conversion generated on rising-edge)           
    PORTD |= (ON << conv);
        
    PORTD &= ~(ON << conv);                      //Drive CNV LOW after ADC conversion complete
    byte0 =SPI.transfer(0);                      //CLK out conversion data (8 bits per transfer, need 20-bits so three transfers must be performed which gives 24 bits total)
    byte1 =SPI.transfer(0);
    byte2 =SPI.transfer(0);
    PORTD |= (ON << conv);                       //Set CNV HIGH to end transaction with ADC
  
    SPI.endTransaction();                        //Stops SPI transaction and brings Arduino SS pin HIGH but this code is not using it

    /*Print ADC conversion to serial terminal for debugging*/
    //conv_value = (byte0<<12)+(byte1<<4)+(byte2>>4);
    //Serial.println(byte0<<12);
    //Serial.println(byte1<<4);
    //Serial.println(byte2>>4);
    //Serial.println(conv_value);
    //conv_value = (byte0<<12)+(byte1<<4)+(byte2>>4);
    //vin_adc = (conv_value/(1048576.0))*10.0;
    //vin_adc = 1.5;
    //Serial.println(vin_adc,3);
}
 

/************************************************************************************************************************************
 * DAC_Load
 * Purpose: Wite data to AD5791 DAC.
 * Note: 3 bytes (24-bits) is required to write DAC command.  Command = [R/W (1-bit) | Control Register (3 bits)| DAC code (20-bits)]
 *       Data is shifted into DAC on CLK falling edge.  Tristate must be disabled in DAC control reg before it will output a voltage.
*************************************************************************************************************************************/
void DAC_Load(uint8_t b0, uint8_t b1, uint8_t b2){

    //unsigned long conv_value = 0;   

    //Disable DAC output
    //PORTD |= (ON << nLDAC);         //DAC latch signal HIGH to disable output
    //delayMicroseconds(1);

    //Write data to DAC
    PORTD &= ~(ON << nSYNC);          //DAC mSYNC signal LOW to enable loading (DAC output is update as soon as it receives 24-bits)

    
    //Control reg  settings binary offset coding (32, 0, 18)
    //Control reg  settings  2's comp coding (32, 0, 2)
    //DAC Vo = 3V (26,102,101)
    SPI.beginTransaction(settingsA);  //Start SPI transaction using desired SPI settings and driving Arduino SS pin LOW but this code is not using it  
    SPI.transfer(b0);                 //8 clk cycles, 8 bits, 5V out
    SPI.transfer(b1);                 //8 clk cycles, 8 bits
    SPI.transfer(b2);                 //8 clk cycles, 8 bits

    //End data load
    PORTD |= (ON << nSYNC);           //DAC mSYNC signal HIGH to disable loading

    //Enable DAC output
    //PORTD &= ~(ON << nLDAC);        //DAC latch signal LOW to disable output
    //PORTD |= (ON << nLDAC);         //DAC latch signal LOW to disable output
           
    SPI.endTransaction();             //Stops SPI transaction and brings Arduino SS pin HIGH but this code is not using it

}



/*SETUP*/
void setup() {
 
  /*Init SPI*/
  SPI.begin();
  
  /*Open Serial connection for debbuging*/
  //Serial.begin(2000000);

  /*Init EVAL-AD400xFMCZ and EVAL-AD5791 I/O*/
  init_ADC_DAC_io();

  //Init DAC (ensure output tristate mode is disabled and DAC is in binary offset DAC reg mode)
  DAC_Load(32, 0, 18);
  delay(1000);              //delay

}



/*MAIN PROGRAM*/
void loop() {

  /*Read ADC sample*/
  ADC_read_conv();
  ADC_conv_value = (byte0<<12)+(byte1<<4)+(byte2>>4);                             //Constructs 20-bit ADC value from 24-bit sample received
  //vin_adc = (conv_value/(1048576.0))*20.0;                                      //analog voltage measured by ADC

  /*Write DAC output*/                                                              
  //DAC_code = (((vin_adc - Vrefn)*(1048576 - 1))/float(Vrefp - Vrefn));          //The ADC and DAC has the same voltage reference and # of bits so ADC value can directly pass into DAC 
                                                                                  //to generate the ADC sampled voltage
                                                                                  
  DAC_byte0 = DAC_write_cmd | ((ADC_conv_value & 0b11110000000000000000) >> 16);  //Appends DAC register write command to DAC code
  DAC_byte1 = (ADC_conv_value  & 0b1111111100000000) >> 8;
  DAC_byte2 = (ADC_conv_value &  0b11111111);   
  
  DAC_Load(DAC_byte0, DAC_byte1, DAC_byte2);                                      //Loads DAC with desired code to gerate voltage sampled by ADC       
  
  //Serial.println(vin_adc,3);                                                    //Print data to serial port for debugging

}



