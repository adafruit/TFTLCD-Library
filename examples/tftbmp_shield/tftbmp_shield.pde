#include <SD.h>
#include <SPI.h>
#include "TFTLCD.h"

#if not defined USE_ADAFRUIT_SHIELD_PINOUT 
 #error "For use with the shield, make sure to #define USE_ADAFRUIT_SHIELD_PINOUT in the TFTLCD.h library file"
#endif

// These are the pins as connected in the shield
#define LCD_CS A3    // Chip Select goes to Analog 3
#define LCD_CD A2    // Command/Data goes to Analog 2
#define LCD_WR A1    // LCD Write goes to Analog 1
#define LCD_RD A0    // LCD Read goes to Analog 0

// The chip select pin for the SD card on the shield
#define SD_CS 5 
// In the SD card, place 24 bit color BMP files (be sure they are 24-bit!)
// There are examples in the sketch folder

// our TFT wiring
TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, 0);

// the file itself
File bmpFile;

// information we extract about the bitmap file
int bmpWidth, bmpHeight;
uint8_t bmpDepth, bmpImageoffset;

/************* HARDWARE SPI ENABLE/DISABLE */
// we want to reuse the pins for the SD card and the TFT - to save 2 pins. this means we have to
// enable the SPI hardware interface whenever accessing the SD card and then disable it when done
int8_t saved_spimode;

void disableSPI(void) {
  saved_spimode = SPCR;
  SPCR = 0;
}

void enableSPI(void) {
  SPCR = saved_spimode; 
}
/******************************************/

void setup()
{

  Serial.begin(9600);
  
  tft.reset();
  
  // find the TFT display
  uint16_t identifier = tft.readRegister(0x0);
  if (identifier == 0x9325) {
    Serial.println("Found ILI9325");
  } else if (identifier == 0x9328) {
    Serial.println("Found ILI9328");
  } else {
    Serial.print("Unknown driver chip ");
    Serial.println(identifier, HEX);
    while (1);
  }  
 
  tft.initDisplay();
  // the image is a landscape, so get into landscape mode
  tft.setRotation(1);

  Serial.print("Initializing SD card...");
 
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
    return;
  }
  Serial.println("SD OK!");
  
  bmpFile = SD.open("tiger.bmp");

  if (! bmpFile) {
    Serial.println("didnt find image");
    while (1);
  }
  
  if (! bmpReadHeader(bmpFile)) { 
     Serial.println("bad bmp");
     return;
  }
  
  Serial.print("image size "); 
  Serial.print(bmpWidth, DEC);
  Serial.print(", ");
  Serial.println(bmpHeight, DEC);
  disableSPI();    // release SPI so we can use those pins to draw
 
  bmpdraw(bmpFile, 0, 0);
  // disable the SD card interface after we are done!
  disableSPI();
}


void loop()
{

}

/*********************************************/
// This procedure reads a bitmap and draws it to the screen
// its sped up by reading many pixels worth of data at a time
// instead of just one pixel at a time. increading the buffer takes
// more RAM but makes the drawing a little faster. 20 pixels' worth
// is probably a good place

#define BUFFPIXEL 20

void bmpdraw(File f, int x, int y) {

  enableSPI();     // enable the hardware SPI to talk to the SD card
  bmpFile.seek(bmpImageoffset);
  disableSPI();    // release it so we can use those pins
  
  uint32_t time = millis();
  uint16_t p;
  uint8_t g, b;
  int i, j;
  
  uint8_t sdbuffer[3 * BUFFPIXEL];  // 3 * pixels to buffer
  uint8_t buffidx = 3*BUFFPIXEL;
  
  Serial.print("rotation = "); Serial.println(tft.getRotation(), DEC);
  
  for (i=0; i< bmpHeight; i++) {
    // bitmaps are stored with the BOTTOM line first so we have to move 'up'

    if (tft.getRotation() == 3) {
      tft.writeRegister(TFTLCD_ENTRY_MOD, 0x1028);
      tft.goTo(x+i, y); 
    } else if  (tft.getRotation() == 2) {
      tft.writeRegister(TFTLCD_ENTRY_MOD, 0x1020);
      tft.goTo(x+bmpWidth, y+i); 
    } else if  (tft.getRotation() == 1) {
      tft.writeRegister(TFTLCD_ENTRY_MOD, 0x1018);
      tft.goTo(x+bmpHeight-1-i, y); 
    } else if  (tft.getRotation() == 0) {
      tft.writeRegister(TFTLCD_ENTRY_MOD, 0x1030);
      tft.goTo(x, y+bmpHeight-i); 
    }
    
    for (j=0; j<bmpWidth; j++) {
      // read more pixels
      if (buffidx >= 3*BUFFPIXEL) {
        enableSPI();     // enable the hardware SPI to talk to the SD card
        bmpFile.read(sdbuffer, 3*BUFFPIXEL);
        disableSPI();    // release it so we can use those pins
        buffidx = 0;
      }
      
      // convert pixel from 888 to 565
      b = sdbuffer[buffidx++];     // blue
      g = sdbuffer[buffidx++];     // green
      p = sdbuffer[buffidx++];     // red
      
      p >>= 3;
      p <<= 6;
      
      g >>= 2;
      p |= g;
      p <<= 5;
      
      b >>= 3;
      p |= b;
     
       // write out the 16 bits of color
      tft.writeData(p);
    }
  }
  tft.writeRegister(TFTLCD_ENTRY_MOD, 0x1030);
  Serial.print(millis() - time, DEC);
  Serial.println(" ms");
}

boolean bmpReadHeader(File f) {
   // read header
  uint32_t tmp;
  
  if (read16(f) != 0x4D42) {
    // magic bytes missing
    return false;
  }
 
  // read file size
  tmp = read32(f);  
  Serial.print("size 0x"); Serial.println(tmp, HEX);
  
  // read and ignore creator bytes
  read32(f);
  
  bmpImageoffset = read32(f);  
  Serial.print("offset "); Serial.println(bmpImageoffset, DEC);
  
  // read DIB header
  tmp = read32(f);
  Serial.print("header size "); Serial.println(tmp, DEC);
  bmpWidth = read32(f);
  bmpHeight = read32(f);

  
  if (read16(f) != 1)
    return false;
    
  bmpDepth = read16(f);
  Serial.print("bitdepth "); Serial.println(bmpDepth, DEC);

  if (read32(f) != 0) {
    // compression not supported!
    return false;
  }
  
  Serial.print("compression "); Serial.println(tmp, DEC);

  return true;
}

/*********************************************/

// These read data from the SD card file and convert them to big endian 
// (the data is stored in little endian format!)

// LITTLE ENDIAN!
uint16_t read16(File f) {
  uint16_t d;
  uint8_t b;
  b = f.read();
  d = f.read();
  d <<= 8;
  d |= b;
  return d;
}


// LITTLE ENDIAN!
uint32_t read32(File f) {
  uint32_t d;
  uint16_t b;
 
  b = read16(f);
  d = read16(f);
  d <<= 16;
  d |= b;
  return d;
}


