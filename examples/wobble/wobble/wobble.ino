#include <SD.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library

#define LCD_CS A3 // Chip Select (see notes above)
#define LCD_CD A2 // Command/Data
#define LCD_RD A0 // LCD Read strobe
#define LCD_WR  4 // LCD Write strobe (see notes above)

#define LCD_RESET A4 // Alternately just connect to Arduino's reset pin

#define DMA_SELECT A1 // Hi/lo chooses DMA vs non-DMA effect

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#define DMALINES 16
// DMA buffer is 320 pixels * DMALINES * 2 bytes/pixel * 2 buffers
uint8_t dmabuf[320 * DMALINES * 2 * 2];
// Pixel buffer is slightly wider than screen, for X-scrolling
uint16_t pixels[320 + 64];
uint32_t startTime;

void setup()
{
  Serial.begin(9600);
  while(!Serial);

  pinMode(DMA_SELECT, INPUT_PULLUP);

  // Initialize pixel buffer with alternating red and white bands,
  // 32 pixels wide.  0x00F8 is 16-bit red (0xF800) endian-swapped
  // so bytes can be copied directly to screen.
  for(int i=0; i<320+64; i++)
    pixels[i] = (i & 32) ? 0x00F8 : 0xFFFF;

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
    return;
  }

  tft.begin(identifier);
  tft.setRotation(1);
  tft.fillScreen(0);
  startTime = millis();
}

int lineNum;
int frame = 0;

// pushColorsDMA() callback function -- fills DMALINES scanlines with
// data from pixels[] array.
void myCallback(uint8_t *dest, uint16_t len) {
  for(int i=0; i<DMALINES; i++) {
    // Wave up to 64 pixels horizontally (extra width in pixels[] array)
    int offset = (int)((sin((float)(lineNum + frame) / 40.0) + 1.0) * 31.5);
    // Change offset every 32 lines for checkerboard effect
    if((lineNum + frame/8) & 32) offset = (offset + 32) % 63;
    memcpy(dest, &pixels[offset], len / DMALINES);
    lineNum++;
    dest += 320 * 2; // Offset to next scanline (2 bytes/pixel)
  }
}

void loop() {
  tft.setAddrWindow(0, 0, tft.width() - 1, tft.height() - 1);
  lineNum = 0;
  if(digitalRead(DMA_SELECT)) {
    tft.pushColorsDMA(tft.width() * tft.height() * 2, dmabuf, tft.width() * DMALINES * 2, myCallback);
  } else {
    bool first = true;
    while(lineNum < tft.height()) {
      // Fill the DMA buffer, but don't issue it
      myCallback(dmabuf, tft.width() * DMALINES * 2);
      // Then send it using non-DMA function:
      tft.pushColors((uint16_t *)dmabuf, tft.width() * DMALINES, first);
      first = false;
    }
  }
  frame++;
  uint32_t elapsed = (millis() - startTime) / 1000;
  if(elapsed > 0) {
    Serial.print(frame / elapsed);
    Serial.println(" fps");
  }
}

