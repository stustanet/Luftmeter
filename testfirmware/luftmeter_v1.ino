
#define HDC2080
#define AVAIL_SCD30
#define BAROSENSOR
//#define RGB_LEDS_AVAILABLE
#define ADS1119_AVAIL
#define SDCARD_AVAIL

#include <Wire.h>
#include <BaroSensor.h>
#include "paulvha_SCD30.h"
#include <FastLED.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     16000

#define FAN_PIN            17

#define ADS1119_ADDR 0x4c


void writeReg_hdc(uint8_t reg, uint8_t data);
float readHumidity_hdc(void);
float readTemp_hdc(void);


void ads1119_write_reg(uint8_t reg, uint8_t data);
uint8_t ads1119_read_reg(uint8_t reg);
int16_t ads1119_read_data(void);
void ads1119_start_conversion(void);
void ads1119_set_channel(uint8_t chan);
void ads1119_powerdown(void);

void listDir(fs::FS &fs, const char * dirname, uint8_t levels);


#ifdef AVAIL_SCD30
SCD30 airSensor;
#endif


#ifdef RGB_LEDS_AVAILABLE
#define DATA_PIN 14
#define NUM_LEDS 8
CRGB leds[NUM_LEDS];
#endif

void setup() {
  // put your setup code here, to run once:

  #ifdef RGB_LEDS_AVAILABLE
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical
  leds[0] = CRGB::Blue;
  leds[7] = CRGB::Blue;
  FastLED.show();
  #endif

  Serial.begin(115200);
  Wire.begin();

  #ifdef HDC2080
  //setup hdc2080
  writeReg_hdc(0x0E, 0x80);
  delay(100);
  writeReg_hdc(0x0E, 0x50);
  writeReg_hdc(0x0F, 0x01);
  #endif

  #ifdef BAROSENSOR
  //Barosensor
  BaroSensor.begin();
  #endif

  #ifdef AVAIL_SCD30
  //SCD30
  airSensor.begin(Wire); //This will cause readings to occur every two seconds

  //TODO get pressure
  //airSensor.setAmbientPressure((uint16_t)(press_bmp/100));
  #endif

  //fancontrol
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(FAN_PIN, LEDC_CHANNEL_0);

  ledcWrite(LEDC_CHANNEL_0,5000);

  #ifdef ADS1119_AVAIL
  ads1119_start_conversion();
  #endif

  #ifdef SDCARD_AVAIL
    if(!SD.begin()){
      Serial.println("Card Mount Failed");
      return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    listDir(SD, "/", 0);
    #endif

  

}

void loop() {

#ifdef AVAIL_SCD30
  static int i = 1995;
  int n = 0;
  int co2 = 0;

  if (airSensor.dataAvailable())
  {
    co2 = airSensor.getCO2();

    Serial.print("CO2: ");
    Serial.print(co2);
    Serial.print("ppm, rH: ");
    Serial.print(airSensor.getHumidity());
    Serial.print("%, Temperatur (CO2): ");
    Serial.print(airSensor.getTemperature());
    Serial.println("°C");

    #ifdef RGB_LEDS_AVAILABLE

    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    leds[2] = CRGB::Black;
    leds[3] = CRGB::Black;
    leds[4] = CRGB::Black;
    leds[5] = CRGB::Black;
    leds[6] = CRGB::Black;
    leds[7] = CRGB::Black;

    leds[0] = CHSV( HUE_GREEN, 255, 60);
    if (co2 > 600)
      leds[1] = CHSV( HUE_GREEN, 255, 60);
    if (co2 > 750)
      leds[2] = CHSV( HUE_GREEN, 255, 60);
    if (co2 > 900)
      leds[3] = CHSV( HUE_YELLOW, 255, 60);
    if (co2 > 1050)
      leds[4] = CHSV( HUE_YELLOW, 255, 60);
    if (co2 > 1200)
      leds[5] = CHSV( HUE_RED, 255, 60);
    if (co2 > 1500)
      leds[6] = CHSV( HUE_RED, 255, 60);
    if (co2 > 2000)
      leds[7] = CHSV( HUE_RED, 255, 60);
    
    FastLED.show();
    #endif

    
#endif

#ifdef HDC2080
    Serial.print("rel. Feuchtigkeit: ");
    Serial.print(readHumidity_hdc());
    Serial.println("%");
    Serial.print("Temperatur (humi): ");
    Serial.print(readTemp_hdc());
    Serial.println("°C");
#endif

#ifdef BAROSENSOR
    Serial.print("Temperatur (baro): ");
    Serial.print(BaroSensor.getTemperature());
    Serial.println("°C");
  
    Serial.print("Luftdruck: ");
    Serial.print(BaroSensor.getPressure());
    Serial.println("hPa");
#endif


    #ifdef ADS1119_AVAIL
    Serial.print("NO2 raw data: ");
    Serial.print(ads1119_read_data());
    Serial.println(" Arb");
    #endif


    Serial.println();
    Serial.println();

#ifdef SDCARD_AVAIL
    File file = SD.open("/log.txt", FILE_APPEND);
    file.print("lol\n");
    file.close();
#endif


#ifdef AVAIL_SCD30
  }

  i++;
  if (i>2000)
  {
    i= 0;
    airSensor.setAmbientPressure((uint16_t)(BaroSensor.getPressure()));
    Serial.println("Set CO2 air pressure reference");
    Serial.print(BaroSensor.getPressure());
    Serial.println("hPa");
  }
#endif


  #ifdef ADS1119_AVAIL
  ads1119_start_conversion();
  #endif

  delay(1000);

}



/*************************  HDC2080  ********************************************/
void writeReg_hdc(uint8_t reg, uint8_t data)
{
  
  Wire.beginTransmission(0x40);    // Open Device addr 0x40
  Wire.write(reg);            // Point to register
  Wire.write(data);           // Write data to register 
  Wire.endTransmission(1);       // Relinquish bus control
  
}

uint8_t readReg_hdc(uint8_t reg)
{
  Wire.beginTransmission(0x40);     // Connect to HDC2080
  Wire.write(reg);            // point to specified register
  Wire.endTransmission(0);
  //delay(2);
  uint8_t reading=0;          // holds byte of read data
  Wire.requestFrom(0x40, 1, 1);     // Request 1 byte from open register
  //Wire.endTransmission();       // Relinquish bus control
  
  if (1 <= Wire.available())
  {
    reading = (Wire.read());      // Read byte
  } 
  return reading;
}

#define TEMP_LOW 0x00
#define TEMP_HIGH 0x01
#define HUMID_LOW 0x02
#define HUMID_HIGH 0x03

float readHumidity_hdc(void)
{
  uint8_t byte[2];
  uint16_t humidity;
  byte[0] = readReg_hdc(HUMID_LOW);
  byte[1] = readReg_hdc(HUMID_HIGH);
  
  humidity = (unsigned int)byte[1] << 8 | byte[0];
  
  return (float)(humidity)/( 65536 )* 100;
  
}

float readTemp_hdc(void)
{
  uint8_t byte[2];
  uint16_t temp;
  byte[0] = readReg_hdc(TEMP_LOW);
  byte[1] = readReg_hdc(TEMP_HIGH);
  
  temp = (unsigned int)byte[1] << 8 | byte[0];
  
  return (float)(temp) * 165 / 65536 - 40;
  
}




void ads1119_write_reg(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(ADS1119_ADDR);    // Open Device
  Wire.write(0x40+4*reg);            // command register
  Wire.write(data);           // Write data to register 
  Wire.endTransmission(1);       // Relinquish bus control

}


uint8_t ads1119_read_reg(uint8_t reg)
{
  Wire.beginTransmission(ADS1119_ADDR);    // Open Device
  Wire.write(0x20+4*reg);            // command register
  Wire.endTransmission(0);
  //delay(2);
  uint8_t reading=0;          // holds byte of read data
  Wire.requestFrom(0x40, 1, 1);     // Request 1 byte from open register
  //Wire.endTransmission();       // Relinquish bus control
  
  if (1 <= Wire.available())
  {
    reading = (Wire.read());      // Read byte
  } 
  return reading;

}


int16_t ads1119_read_data(void)
{
  Wire.beginTransmission(ADS1119_ADDR);    // Open Device
  Wire.write(0x10);            // command register
  Wire.endTransmission(0);
  //delay(2);
  uint8_t dat0=0;          // holds byte of read data
  uint8_t dat1=0;          // holds byte of read data

  Wire.requestFrom(0x40, 2, 1);     // Request 2 byte from open register
  //Wire.endTransmission();       // Relinquish bus control
  
  if (1 <= Wire.available())
  {
    dat0 = (Wire.read());      // Read byte
    dat1 = (Wire.read());      // Read byte
  } 
  return (int16_t)((((uint16_t)dat0)<<8)|dat1);

}

void ads1119_start_conversion(void)
{
  Wire.beginTransmission(ADS1119_ADDR);    // Open Device
  Wire.write(0x08);            // command register
  Wire.endTransmission(1);       // Relinquish bus control
}

void ads1119_powerdown(void)
{
  Wire.beginTransmission(ADS1119_ADDR);    // Open Device
  Wire.write(0x02);            // command register
  Wire.endTransmission(1);       // Relinquish bus control
}

void ads1119_set_channel(uint8_t chan)
{
  uint8_t reg = ads1119_read_reg(0);
  ads1119_write_reg(0,(reg& 0x1F) | (chan<<5));
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}
