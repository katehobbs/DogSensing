/* Arduino Header Files */
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

/* SD card variables */
File audioFile;
char bufFile[14]="/audio020.txt";
char audioDirName[7]="/adlog";
boolean hascard=false;
#define sdCardSSpin 5   // SD Card ss pin
unsigned long starttime = 0;


/* i2s variables */
const int sample_rate = 44100;
static const int i2s_num = 0; // i2s port number
int32_t mic_sample;
int counter = 0;

i2s_pin_config_t audio_in_pin_config = {  // configue connections from ESP32 to digital microphone
    .bck_io_num = 33, // BCLK pin  17
    .ws_io_num = 26,  // LRCL pin   18
    .data_out_num = I2S_PIN_NO_CHANGE,//I2S_PIN_NO_CHANGE, // Not used   -1
    .data_in_num = 15   // DOUT pin    5
};


i2s_config_t audio_in_i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sample_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, //only one microphone
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB), //(I2S_COMM_FORMAT_I2S |)
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = 1
};


boolean isAudioFileExists(const char * dirname, char comparisonText[14]){
  char tempDirName[21] = "";
  strcat(tempDirName,audioDirName);  // joins strings together "" and adlog
  strcat(tempDirName, comparisonText); //joins strings together "" and bufFile
  Serial.println("file name is");
  Serial.println(tempDirName);
//  File root = SD.open(dirname);
  audioFile = SD.open(tempDirName, FILE_WRITE);
  if (audioFile) {
    Serial.print(tempDirName);
    Serial.println("opened");


  } else{
    Serial.println(F("audioFile failed to open Kate "));

  }
   return true;
}


void initiateSDCard(void){
  pinMode(sdCardSSpin, OUTPUT);


  if (SD.begin()) {
    hascard=true;

    if (SD.exists("/adlog") == false) {
      SD.mkdir("/adlog");
    }
  //createAudioFile(bufFile);

}
}


void setup() {
    Serial.begin(115200);
    i2s_driver_install((i2s_port_t)i2s_num, &audio_in_i2s_config, 0, NULL);
    i2s_set_pin((i2s_port_t)i2s_num, &audio_in_pin_config);
    pinMode(15, INPUT);
    pinMode(33, OUTPUT);
    pinMode(26, OUTPUT);

    i2s_zero_dma_buffer((i2s_port_t)i2s_num);
    initiateSDCard();
    isAudioFileExists("/adlog", bufFile);
    Serial.println("logging data...");
    delay(2000);


}

void loop() {
  if (starttime==0){
    starttime = millis();
  }

    i2s_pop_sample((i2s_port_t)i2s_num, (char*)&mic_sample, portMAX_DELAY); // Read samples into buffers one by one
    //mic_sample <<= 14;



    audioFile.println(mic_sample);
    counter++;


    if (counter > 100000*3) {
      audioFile.println(millis()-starttime);
      audioFile.close();
      Serial.println("file closed");
      Serial.println(millis()-starttime);
      while(1);
}
}
