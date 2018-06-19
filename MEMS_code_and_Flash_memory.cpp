
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
/* I2S Header Files */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_spi_flash.h"
#include "driver/i2s.h"
#include "driver/dac.h"
#include <math.h>
#include <stdio.h>


#include<SPIMemory.h>
#include <command_list.h>

uint8_t pageBuffer[256];
String serialCommand;
char printBuffer[128];
uint32_t addr;
uint8_t dataByte;
uint16_t dataInt;
String inputString, outputString;
int counter = 0;


/* i2s variables */
const int sample_rate = 10000;
static const int i2s_num = 0; // i2s port number
uint32_t mic_sample;

i2s_pin_config_t audio_in_pin_config = {  // configue connections from ESP32 to digital microphone
    .bck_io_num = 33, // BCLK pin  17
    .ws_io_num = 26,  // LRCL pin   18
    .data_out_num = I2S_PIN_NO_CHANGE,//I2S_PIN_NO_CHANGE, // Not used   -1
    .data_in_num = 15  // DOUT pin    5
};

i2s_config_t audio_in_i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sample_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, //only one microphone
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_LSB), //(I2S_COMM_FORMAT_I2S |)
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
    .dma_buf_count = 32,
    .dma_buf_len = 64,
    .use_apll = 0
};


//Define a flash memory size (if using non-Winbond memory) according to the list in defines.h
//#define CHIPSIZE MB64



#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
//#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#if defined (SIMBLEE)
#define BAUD_RATE 250000
#define RANDPIN 1
#else
#define BAUD_RATE 115200
#if defined(ARCH_STM32)
#define RANDPIN PA0
#else
#define RANDPIN A0
#endif
#endif

//SPIFlash flash;
SPIFlash flash(5);


//SPIFlash flash(SS1, &SPI1);       //Use this constructor if using an SPI bus other than the default SPI. Only works with chips with more than one hardware SPI bus
void clearprintBuffer()
{
  for (uint8_t i = 0; i < 128; i++) {
    printBuffer[i] = 0;
  }
}

//Reads a string from Serial
bool readSerialStr(String &inputStr) {
  if (!Serial)
    Serial.begin(115200);
  while (Serial.available()) {
    inputStr = Serial.readStringUntil('\n');
    return true;
  }
  return false;
}


void printLine()
{
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------"));
}

void printSplash()
{
  Serial.println(F("                                                        SPIFlash library test                                                     "));
}

void printNextCMD()
{
  Serial.println(F("Please type the next command. Type 0 to get the list of commands"));
}

void printOutputChoice()
{
  Serial.print("Would you like your output in decimal or hexadecimal? Please indicate with '1' for HEX or '2' for DEC: ");
}

void printReadChoice()
{
  Serial.print("Type 1 to read the page you have just modified. Type 0 to continue: ");
}

void writeSuccess()
{
  Serial.println("Data write successful");
}

void writeFail()
{
  Serial.println("Data write failed");
}

void inputAddress(void) {
  Serial.print(F("Please enter the address (0 - CAPACITY) you wish to access: "));
  while (!Serial.available()) {
  }
  addr = Serial.parseInt();
  Serial.println(addr);
}

void save_data(void){
  printLine();
  Serial.println(F("                                                       Function 4 : Write Word                                                    "));
  printSplash();
  printLine();
  addr = counter;
  counter += 1;
  Serial.print(F("Please enter the value of the word (>255) you wish to save: "));

  i2s_pop_sample((i2s_port_t)i2s_num, (char*)&mic_sample, portMAX_DELAY);

  dataInt = mic_sample;
  if (dataInt != 0){
  Serial.println(dataInt);
  if (flash.writeWord(addr, dataInt) ){
    clearprintBuffer();
    sprintf(printBuffer, "%d has been written to address %d", dataInt, addr);
    Serial.println(printBuffer);
  }
}
  else {
    writeFail();
  }
  printf("data saved");
}


void setup() {

  Serial.begin(115200);
  i2s_driver_install((i2s_port_t)i2s_num, &audio_in_i2s_config, 0, NULL);
  i2s_set_pin((i2s_port_t)i2s_num, &audio_in_pin_config);
  pinMode(15, INPUT);
  pinMode(33, OUTPUT);
  pinMode(26, OUTPUT);

  i2s_zero_dma_buffer((i2s_port_t)i2s_num);
#if defined (ARDUINO_ARCH_SAMD) || (__AVR_ATmega32U4__) || defined(ARCH_STM32) || defined(NRF5)
  while (!Serial) ; // Wait for Serial monitor to open
#endif
  delay(50); //Time to terminal get connected
  Serial.print(F("Initialising"));
  for (uint8_t i = 0; i < 10; ++i)
  {
    Serial.print(F("."));
  }
  Serial.println();
  randomSeed(analogRead(RANDPIN));
  flash.begin(MB(128));
  //To use a custom flash memory size (if using memory from manufacturers not officially supported by the library) - declare a size variable according to the list in defines.h
  //flash.begin(MB(1));

}

void loop() {

  while (Serial.available() > 0) {
    uint8_t commandNo = Serial.parseInt();
    if (commandNo == 0) {
      printf("Choose a command");
    }

    else if (commandNo == 4) {
      while(counter < 10001){
        save_data();

}
      counter = 0;
      for (int i = 0; i < 10001 ; i++){
        addr = i;
        clearprintBuffer();
        sprintf(printBuffer, "The unsigned int at address %d is: ", addr);
        Serial.print(printBuffer);
        Serial.println(flash.readWord(addr));

        }
      }



    else if (commandNo == 14) {
      printLine();
      Serial.println(F("                                                      Function 14 : Erase Chip                                                    "));
      printSplash();
      printLine();
      Serial.println(F("This function will erase the entire flash memory."));
      Serial.println(F("Do you wish to continue? (Y/N)"));
      char c;
      while (!Serial.available()) {
      }
      c = (char)Serial.read();
      if (c == 'Y' || c == 'y') {
        if (flash.eraseChip())
          Serial.println(F("Chip erased"));
        else
          Serial.println(F("Error erasing chip"));
      }

    }
  }
}
