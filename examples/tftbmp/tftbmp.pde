// BMP-loading demo rigged specifically for the SAMD21 branch
// of TFTLCD and the ItsyBitsy M4 board.  TFTLCD lib MUST be
// configured for the breakout board option, plus there's
// some wiring shenanigans...

// LCD_WR MUST go to pin D4, because we're using a specific
// timer/counter for PWM output.  The pin # could be changed
// IF a corresponding timer change is made in the SAMD21 TFTLIB.

// One of two additional wiring changes MUST be made.  Either:
// LCD_WR MUST go through an inverter (e.g. 74HC04)
//  -or-
// The TFT 'CS' pin MUST be tied HIGH (ignoring LCD_CS setting)
// If you opt for this latter arrangement, you CANNOT read the
// device ID from the display (or anything else) -- see setup()
// where 'identifier' is hardcoded;

// Data pins are as follows:
//   D0 connects to digital pin 0  (Notice these are
//   D1 connects to digital pin 1   NOT in order!)
//   D2 connects to digital pin 7
//   D3 connects to digital pin 9
//   D4 connects to digital pin 10
//   D5 connects to digital pin 11
//   D6 connects to digital pin 13
//   D7 connects to digital pin 12

#include <SD.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library

#define LCD_CS A3 // Chip Select (see notes above)
#define LCD_CD A2 // Command/Data
#define LCD_RD A0 // LCD Read strobe
#define LCD_WR  4 // LCD Write strobe (see notes above)

#define LCD_RESET A4 // Alternately just connect to Arduino's reset pin

// DO NOT use the SD card slot on the TFT breakout -- it doesn't
// appear to work when using the parallel interface.  Instead, a
// separate SD breakout is needed.

#define SD_CS A5 // SD card delect

// A switch or jumper on A1 selects DMA vs non-DMA BMP loading.
// There's really not a huge performance difference in this case
// just because the bottleneck is in the SD card access and color
// conversion operations...BUT...it does demonstrate how the
// pushColorsDMA() function works, and how to use a callback to
// load the next block of data while the current block is sent.
#define DMA_SELECT A1 // Hi/lo chooses DMA vs non-DMA DMA loader

// In the SD card, place 24 bit color BMP files (be sure they are 24-bit!)
// There are examples in the sketch folder

// our TFT wiring
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

void setup()
{
  Serial.begin(9600);
  while(!Serial);

  pinMode(DMA_SELECT, INPUT_PULLUP);

  tft.reset();

  uint16_t identifier = tft.readID();

  // SEE NOTES ABOVE - this is necessary IF using the
  // hard-wired CS (and no inverter) option.
  identifier = 0x9341;

  if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    return;
  }

  tft.begin(identifier);
  tft.fillScreen(0);

  Serial.print(F("Initializing SD card..."));
  if (!SD.begin(SD_CS)) {
    Serial.println(F("failed!"));
    tft.fillScreen(0xF800);
    for(;;);
  }

  Serial.println(F("OK!"));
  tft.fillScreen(0x001F);

  if(digitalRead(DMA_SELECT))
    bmpDrawDMA("woof.bmp", 0, 0);
  else
    bmpDraw("woof.bmp", 0, 0);
//  delay(1000);
}

void loop()
{
  for(int i = 0; i<4; i++) {
    tft.setRotation(i);
    tft.fillScreen(0);
    for(int j=0; j <= 200; j += 50) {
      if(digitalRead(DMA_SELECT))
        bmpDrawDMA("miniwoof.bmp", j, j);
      else
        bmpDraw("miniwoof.bmp", j, j);
    }
//    delay(1000);
  }
}

// Common functions/vars for both BMP loaders ------------------------------

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

// "Vanilla" (non-DMA) BMP Loader ------------------------------------------

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 64

void bmpDraw(char *filename, int x, int y) {
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
  uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
  uint8_t  lcdidx = 0;
  boolean  first = true;
  uint16_t col16;

  if((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');
  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.println(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.println(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) { // For each scanline...
          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each column...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              // Push LCD buffer to the display first
              if(lcdidx > 0) {
                tft.pushColors(lcdbuffer, lcdidx, first);
                lcdidx = 0;
                first  = false;
              }
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            col16 = tft.color565(r,g,b);
            lcdbuffer[lcdidx++] = (col16 * 0x00010001) >> 8; // Flip hi/lo bytes
          } // end pixel
        } // end scanline
        // Write any remaining data to LCD
        if(lcdidx > 0) {
          tft.pushColors(lcdbuffer, lcdidx, first);
        } 
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }
  tft.setAddrWindow(0, 0, tft.width() - 1, tft.height() - 1);

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));
}

// DMA BMP Loader ----------------------------------------------------------

// DMA buffer: 320 pixels max width, DMALINES height, 2 bytes/pixel, 2 bufs
// SD buffer: 320 pixels max width, one scanline
#define DMALINES 16
uint8_t dmabuf[DMALINES * 320 * 2 * 2];
uint8_t sdbuf[320 * 3];

File     bmpFile;
uint32_t bmpImageoffset;        // Start of image data in file
int      lineNum, linesToGo;    // Current, remaining lines to load
boolean  flip;                  // BMP is stored bottom-to-top
int      bmpHeight;             // Uncropped height in pixels
int      croppedWidth;          // Cropped width in pixels
uint32_t rowSize;               // Not always bmpWidth; may have padding

void bmpCallback(uint8_t *dest, uint16_t len) {
  int      row, col, linesThisPass;
  uint8_t  r, g, b, *ptr;
  uint16_t col16;
  uint32_t pos;

  linesThisPass = (linesToGo > DMALINES) ? DMALINES : linesToGo;

  for(row=0; row<linesThisPass; row++, lineNum++) { // For each scanline...
    // Seek to start of scan line.  It might seem labor-
    // intensive to be doing this on every line, but this
    // method covers a lot of gritty details like cropping
    // and scanline padding.  Also, the seek only takes
    // place if the file position actually needs to change
    // (avoids a lot of cluster math in SD library).
    if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
      pos = bmpImageoffset + (bmpHeight - 1 - lineNum) * rowSize;
    else     // Bitmap is stored top-to-bottom
      pos = bmpImageoffset + lineNum * rowSize;
    if(bmpFile.position() != pos) { // Need seek?
      bmpFile.seek(pos);
    }

    bmpFile.read(sdbuf, croppedWidth * 3); // Read scanline
    ptr = sdbuf;
    for(col=0; col<croppedWidth; col++) { // For each column...
      // Convert pixel from BMP to TFT format
      b = *ptr++;
      g = *ptr++;
      r = *ptr++;
      col16 = tft.color565(r,g,b);
      *dest++ = col16 >> 8; // High byte
      *dest++ = col16;      // Low byte
    } // end pixel
  } // end scanline

  linesToGo -= linesThisPass;
}

void bmpDrawDMA(char *filename, int x, int y) {
  int      bmpWidth;              // Image width in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  boolean  goodBmp = false;       // Set to true on valid header parse
  int      w, h;
  uint32_t startTime = millis();

  if((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');
  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.println(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.println(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        } else {
          flip      = true;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);
        croppedWidth = w;
        lineNum      = 0;
        linesToGo    = h;
        tft.pushColorsDMA(w * h * 2, dmabuf, w * DMALINES * 2, bmpCallback);

        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }
  tft.setAddrWindow(0, 0, tft.width() - 1, tft.height() - 1);

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));
}

