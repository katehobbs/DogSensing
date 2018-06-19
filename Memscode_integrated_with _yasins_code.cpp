#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "driver/i2s.h"

// definitions
#define TIMERPERIOD 125      // Timer period
#define BUF_SIZE 512    // buffer size for adc readings
#define sdCardSSpin 5   // SD Card ss pin

// WiFi and UDP constants and variables
WiFiUDP Udp;
const char *APssid = "guderesearch";
const char *APpassword = "12345678";
const unsigned int udpPort = 3333;
char incomingPacket[255];
int packetSize = 0;
boolean sendit=false;
boolean isWiFiOn=true;
char packetBuffer[255];          // buffer to hold incoming packet


// Timer variables
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

// Accel variables
Adafruit_LIS3DH accelerometer = Adafruit_LIS3DH();
unsigned int accelCount=0;
boolean sendAccel=false;
uint8_t accelData[6];

// SD card variables
File audioFile;
File accelFile;
char accelFileName[14]="/accel000.log";
char bufFile[14]="/audio000.wav";
char audioDirName[7]="/ADLOG";
char accelDirName[10]="/accellog";
boolean hascard=false;
byte wavheader[44];

// variables and checks for audio recording
uint8_t BCLKpin = 33;
uint8_t LRCLpin = 26;
uint8_t DOUTpin = 15;
int readings = 0, bufcount=0, counter=0;
uint8_t bufa[BUF_SIZE];
uint8_t bufb[BUF_SIZE];
boolean aready=true, writeit=false, hasdata=false, stopTimer=false,isData=false;
unsigned long starttime = 0;
unsigned long period, frequency;
/* i2s variables */
const int sample_rate = 10000;
static const int i2s_num = 0; // i2s port number
int32_t mic_sample;

i2s_pin_config_t audio_in_pin_config = {  // configue connections from ESP32 to digital microphone
    .bck_io_num = BCLKpin,
    .ws_io_num = LRCLpin,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = DOUTpin
};

i2s_config_t audio_in_i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sample_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = 1
};

// Timer function
void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
  i2s_pop_sample((i2s_port_t)i2s_num, (char*)&mic_sample, portMAX_DELAY);

  if(aready) { // get the new value from analog port
      bufa[bufcount]=uint8_t((mic_sample)/(-1.098924438879442e+08)*128);
  } else {
      bufb[bufcount]=uint8_t((mic_sample)/(-1.098924438879442e+08)*128);
  }
  readings++; // increment data counter
  bufcount++; // increment buffer counter
  if(bufcount==BUF_SIZE){
      if(writeit==false) {
          bufcount=0;
          aready = ! aready;
          writeit=true; // flag that a write is needed
          //accelCount++;
      } else { // wait for file write to complete
          bufcount--;
          readings--;
      }
  }

  // if (accelCount == 1){
  //     sendAccel = true;
  //     accelCount = 0;
  // }
}

boolean isAudioFileExists(const char * dirname, char comparisonText[14]){
    char tempDirName[21] = "";
    strcat(tempDirName,audioDirName);
    strcat(tempDirName, comparisonText);

    File root = SD.open(dirname);
    File file = root.openNextFile();
    while(file){
        if (strcmp(tempDirName, file.name()) == 0){
            return false;  // if file exists, return false.
        }
        file.close();
        file = root.openNextFile();
    }
    // if file does not exist, open a new file
    audioFile = SD.open(tempDirName, FILE_WRITE);

    if (audioFile) {
        audioFile.write(wavheader, 44); // write wav header
        audioFile.seek(44); //set data start
        Serial.print(tempDirName);
        Serial.println(" opened");
    } else{
        Serial.println(F("audioFile failed to open"));
    }
    return true;
}

void createAudioFile(char* text){
    unsigned int countNumOfFiles = 1;
    String fileNumber;
    while(!isAudioFileExists("/ADLOG", text)){
        fileNumber = String(countNumOfFiles);
        if (countNumOfFiles<10){
            text[8]=fileNumber[0];
        }
        else if (countNumOfFiles<100){
            text[7]=fileNumber[0];
            text[8]=fileNumber[1];
        }
        else if (countNumOfFiles<1000){
            text[6]=fileNumber[0];
            text[7]=fileNumber[1];
            text[8]=fileNumber[2];
        } else {
            Serial.println(F("a lot files!"));
            while(1);
        }
        countNumOfFiles++;
    }
}


boolean isAccelFileExists(const char * dirname, char comparisonText[14]){
    char tempDirName[24] = "";
    strcat(tempDirName,accelDirName);
    strcat(tempDirName, comparisonText);
    //comparisonText = String(dirname) + String(comparisonText);
    File root = SD.open(dirname);
    File file = root.openNextFile();
    while(file){
        if (strcmp(tempDirName, file.name()) == 0){
            return false;
        }
        file.close();
        file = root.openNextFile();
    }
    accelFile = SD.open(tempDirName, FILE_WRITE);

    if (accelFile) {
        Serial.print(tempDirName);
        Serial.println(" opened");
    } else{
        Serial.println(F("accelFile failed to open"));
        while(1);
    }
    return true;
}

void createAccelFile(char* text){
    unsigned int countNumOfFiles = 1;
    String fileNumber;
    while(!isAccelFileExists("/accellog", text)){
        fileNumber = String(countNumOfFiles);
        if (countNumOfFiles<10){
            text[8]=fileNumber[0];
        }
        else if (countNumOfFiles<100){
            text[7]=fileNumber[0];
            text[8]=fileNumber[1];
        }
        else if (countNumOfFiles<1000){
            text[6]=fileNumber[0];
            text[7]=fileNumber[1];
            text[8]=fileNumber[2];
        } else {
            Serial.println(F("a lot files!"));
            while(1);
        }
        countNumOfFiles++;
    }
}

void setupWavHeader(){
    // little endian (lowest byte 1st)
    wavheader[0]='R';
    wavheader[1]='I';
    wavheader[2]='F';
    wavheader[3]='F';
    //wavheader[4] to wavheader[7] size of data + header -8
    wavheader[8]='W';
    wavheader[9]='A';
    wavheader[10]='V';
    wavheader[11]='E';
    wavheader[12]='f';
    wavheader[13]='m';
    wavheader[14]='t';
    wavheader[15]=' ';
    wavheader[16]=16;
    wavheader[17]=0;
    wavheader[18]=0;
    wavheader[19]=0;
    wavheader[20]=1;
    wavheader[21]=0;
    wavheader[22]=1;
    wavheader[23]=0;
    // wavheader[24] to wavheader[27] samplerate hz
    // wavheader[28] to wavheader[31] samplerate*1*1
    // optional bytes can be added here
    wavheader[32]=1;
    wavheader[33]=0;
    wavheader[34]=8;
    wavheader[35]=0;
    wavheader[36]='d';
    wavheader[37]='a';
    wavheader[38]='t';
    wavheader[39]='a';
    //wavheader[40] to wavheader[43] sample number
}

void initiateMEMs(){
    i2s_driver_install((i2s_port_t)i2s_num, &audio_in_i2s_config, 0, NULL);
    i2s_set_pin((i2s_port_t)i2s_num, &audio_in_pin_config);
    pinMode(DOUTpin, INPUT);
    pinMode(BCLKpin, OUTPUT);
    pinMode(LRCLpin, OUTPUT);
    i2s_zero_dma_buffer((i2s_port_t)i2s_num);
}


void initiateSDCard(void){
    pinMode(sdCardSSpin, OUTPUT);
    // wavheader setup
    setupWavHeader();

    if (SD.begin(sdCardSSpin)) {
        hascard=true;
        // Create adlog sub-directory
        if (SD.exists("/adlog") == false) {
            SD.mkdir("/adlog");
        }
        // Create accellog sub-directory
        if (SD.exists("/accellog") == false) {
            SD.mkdir("/accellog");
        }
        createAudioFile(bufFile);
        //createAccelFile(accelFileName);
    } else {
        // display error and enter stasus
        Serial.println(F("SD.begin Problem"));
        while(1);
    }
}

void initiateAccel(void){
    if (! accelerometer.begin()) {
      Serial.println(F("Accel couldnt start"));
      while (1);
    }
    accelerometer.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
    accelerometer.setDataRate( LIS3DH_DATARATE_LOWPOWER_5KHZ);
    Serial.println(F("Accel is OK"));
}

void initiateTimerADC(void){
        // Create semaphore to inform us when the timer has fired
    timerSemaphore = xSemaphoreCreateBinary();
    // Use 1st timer of 4 (counted from zero).
    // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for
    // more info).
    timer = timerBegin(0, 80, true);
    // Attach onTimer function to our timer.
    timerAttachInterrupt(timer, &onTimer, true);
    // Set alarm to call onTimer function every second (value in microseconds).
    // Repeat the alarm (third parameter)
    timerAlarmWrite(timer, TIMERPERIOD, true);
    // Start an alarm
    timerAlarmEnable(timer);
    Serial.println("Timer running");
}

void initiateUDPConnection(void) {
    WiFi.mode(WIFI_AP);    // initialize ESP module in AP mode

    // start access point
    WiFi.softAP(APssid,APpassword);
    Serial.println("Access point started");
    IPAddress ip = WiFi.localIP();
    Serial.print(F("IP Address: "));
    Serial.println(ip);

    // if you get a connection, report back via serial:
    Udp.begin(udpPort);
    Serial.print("Listening on port ");
    Serial.println(udpPort);
}

void listenUdpCommand(void){
    packetSize = Udp.parsePacket();
    if (packetSize) {
        Serial.print(F("Received packet of size "));
        Serial.println(packetSize);
        Serial.print(F("From "));
        IPAddress remoteIp = Udp.remoteIP();
        Serial.print(remoteIp);
        Serial.print(F(", port "));
        Serial.println(Udp.remotePort());
        // read the packet into packetBufffer
        int len = Udp.read(packetBuffer, 255);
        if (len > 0) {
          packetBuffer[len] = 0;
        }
        Serial.println(F("Contents:"));
        Serial.println(packetBuffer);
    } else {
        packetBuffer[0] = '!';
    }
}

void setupNewRecord(void){
    readings = 0, bufcount=0, counter=0;
    aready=true, writeit=false, hasdata=false, stopTimer=false,isData=false;
    starttime = 0;
    isrCounter = 0;
    lastIsrAt = 0;
    accelCount=0;
    sendAccel=false;
    sendit = false;
    hascard=false;
    initiateSDCard();
    //initiateAccel();
    initiateTimerADC();
}

void headmod(long value, byte location){
    // write four bytes for a long
    audioFile.seek(location); // find the location in the file
    byte tbuf[4];
    tbuf[0] = value & 0xFF; // lo byte
    tbuf[1] = (value >> 8) & 0xFF;
    tbuf[2] = (value >> 16) & 0xFF;
    tbuf[3] = (value >> 24) & 0xFF; // hi byte
    audioFile.write(tbuf,4); // write the 4 byte buffer"
}

void endAllOperations(void){
    Serial.println(F("Logging stopped"));
    isData = true;
    sendit = false;

    if (stopTimer == true) {
      // If timer is still running
      if (timer) {
        // Stop and free timer
        timerEnd(timer);
        timer = NULL;
      }
    }

    frequency = float(readings - BUF_SIZE)/period * 1000;
    Serial.print("frequency: ");
    Serial.println(frequency);

    // update wav header
    long datacount=readings;
    long setf=long(frequency);
    headmod(datacount + 36,4); //set size of data +44-8
    headmod(setf, 24); //set sample rate Hz
    headmod(setf, 28); //set sample rate Hz
    headmod(datacount, 40); // set data size

    audioFile.close();
    accelFile.close();
}

void recordAccelData(void){
    sensors_event_t accelEvent;
    accelerometer.getEvent(&accelEvent);

    if (accelEvent.acceleration.x < 0){
        accelData[0] = 100 + abs(accelEvent.acceleration.x);
    } else {
        accelData[0] = accelEvent.acceleration.x + 1;
    }
    if (accelEvent.acceleration.y < 0){
        accelData[2] = 100 + abs(accelEvent.acceleration.y);
    } else {
        accelData[2] = accelEvent.acceleration.y + 1;
    }
    if (accelEvent.acceleration.z < 0){
        accelData[4] = 100 + abs(accelEvent.acceleration.z);
    } else {
        accelData[4] = accelEvent.acceleration.z + 1;
    }

    //float parameter conversion
    accelData[1] = abs(accelEvent.acceleration.x * 100) - uint8_t(abs(accelEvent.acceleration.x)) * 100 + 100;
    accelData[3] = abs(accelEvent.acceleration.y * 100) - uint8_t(abs(accelEvent.acceleration.y)) * 100 + 100;
    accelData[5] = abs(accelEvent.acceleration.z * 100) - uint8_t(abs(accelEvent.acceleration.z)) * 100 + 100;
}

void UDPsend(uint8_t* buf) {
    Udp.beginPacket (Udp.remoteIP(),Udp.remotePort());
    Udp.write(buf, BUF_SIZE);
    Udp.endPacket();
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);

    initiateMEMs();
    initiateSDCard();
    //initiateAccel();
    initiateUDPConnection();
    initiateTimerADC();
}

void loop() {
    listenUdpCommand();

    if (strcmp(packetBuffer, "WiFiOff") == 0) {
        isWiFiOn = false;
    }
    if (strcmp(packetBuffer, "WiFiOn") == 0) {
        isWiFiOn = true;
    }

    if (sendit == false){
        if (strcmp(packetBuffer, "start") == 0) {
            if (starttime==0) {
                sendit = true;
                Serial.println(F("Logging started"));
                starttime=millis();
            }
        }
        if (strcmp(packetBuffer, "new") == 0) {
            setupNewRecord();
        }
    }

    if (sendit) {
        if(writeit){
            sendAccel = false;
            if (sendAccel){
                //recordAccelData();
                //accelFile.write((uint8_t*)accelData,sizeof(accelData));

                if (aready) {
                    audioFile.write(bufb, BUF_SIZE);
                    // for (int i=0;i<6;i++){
                    //     bufb[i]=accelData[i];
                    // }
                    if (isWiFiOn) {
                        UDPsend(bufb);
                    }
                } else {
                    audioFile.write(bufa, BUF_SIZE);
                    // for (int i=0;i<6;i++){
                    //     bufa[i]=accelData[i];
                    // }
                    if (isWiFiOn) {
                        UDPsend(bufa);
                    }
                }
                sendAccel = false;
            } else {
                if (aready){
                    audioFile.write(bufb, BUF_SIZE); // write the data block
                    UDPsend(bufb);
                } else {
                    audioFile.write(bufa, BUF_SIZE); // write the data block
                    UDPsend(bufa);
                }
            }
            writeit=false; // the write is done!
        }
        if (strcmp(packetBuffer, "stop") == 0) {
            if (starttime) {
                period = millis()-starttime;
                sendit = false;
                hasdata=true; // flag the data presence
                writeit=false; // the write is done!
                stopTimer = true;
            }
        }
    }

    if (hasdata) {
        // done
        if (isData == false) {
            endAllOperations();
        }
    }
}
