// Graphics library by ladyada/adafruit with init code from Rossum 
// MIT license

#ifndef _ADAFRUIT_TFTLCD_H_
#define _ADAFRUIT_TFTLCD_H_

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <Adafruit_GFX.h>

// comment or uncomment the next line for special pinout!
#define USE_ADAFRUIT_SHIELD_PINOUT


// register names from Peter Barrett's Microtouch code

#define ILI932X_START_OSC			0x00
#define ILI932X_DRIV_OUT_CTRL		0x01
#define ILI932X_DRIV_WAV_CTRL		0x02
#define ILI932X_ENTRY_MOD			0x03
#define ILI932X_RESIZE_CTRL			0x04
#define ILI932X_DISP_CTRL1			0x07
#define ILI932X_DISP_CTRL2			0x08
#define ILI932X_DISP_CTRL3			0x09
#define ILI932X_DISP_CTRL4			0x0A
#define ILI932X_RGB_DISP_IF_CTRL1	0x0C
#define ILI932X_FRM_MARKER_POS		0x0D
#define ILI932X_RGB_DISP_IF_CTRL2	0x0F
#define ILI932X_POW_CTRL1			0x10
#define ILI932X_POW_CTRL2			0x11
#define ILI932X_POW_CTRL3			0x12
#define ILI932X_POW_CTRL4			0x13
#define ILI932X_GRAM_HOR_AD			0x20
#define ILI932X_GRAM_VER_AD			0x21
#define ILI932X_RW_GRAM				0x22
#define ILI932X_POW_CTRL7			0x29
#define ILI932X_FRM_RATE_COL_CTRL	0x2B
#define ILI932X_GAMMA_CTRL1			0x30
#define ILI932X_GAMMA_CTRL2			0x31
#define ILI932X_GAMMA_CTRL3			0x32
#define ILI932X_GAMMA_CTRL4			0x35 
#define ILI932X_GAMMA_CTRL5			0x36
#define ILI932X_GAMMA_CTRL6			0x37
#define ILI932X_GAMMA_CTRL7			0x38
#define ILI932X_GAMMA_CTRL8			0x39
#define ILI932X_GAMMA_CTRL9			0x3C
#define ILI932X_GAMMA_CTRL10			0x3D
#define ILI932X_HOR_START_AD			0x50
#define ILI932X_HOR_END_AD			0x51
#define ILI932X_VER_START_AD			0x52
#define ILI932X_VER_END_AD			0x53
#define ILI932X_GATE_SCAN_CTRL1		0x60
#define ILI932X_GATE_SCAN_CTRL2		0x61
#define ILI932X_GATE_SCAN_CTRL3		0x6A
#define ILI932X_PART_IMG1_DISP_POS	0x80
#define ILI932X_PART_IMG1_START_AD	0x81
#define ILI932X_PART_IMG1_END_AD		0x82
#define ILI932X_PART_IMG2_DISP_POS	0x83
#define ILI932X_PART_IMG2_START_AD	0x84
#define ILI932X_PART_IMG2_END_AD		0x85
#define ILI932X_PANEL_IF_CTRL1		0x90
#define ILI932X_PANEL_IF_CTRL2		0x92
#define ILI932X_PANEL_IF_CTRL3		0x93
#define ILI932X_PANEL_IF_CTRL4		0x95
#define ILI932X_PANEL_IF_CTRL5		0x97
#define ILI932X_PANEL_IF_CTRL6		0x98

#define HX8347G_COLADDRSTART2 0x02
#define HX8347G_COLADDRSTART1 0x03
#define HX8347G_COLADDREND2 0x04
#define HX8347G_COLADDREND1 0x05

#define HX8347G_ROWADDRSTART2 0x06
#define HX8347G_ROWADDRSTART1 0x07
#define HX8347G_ROWADDREND2 0x08
#define HX8347G_ROWADDREND1 0x09

#define HX8347G_MEMACCESS 0x16


#define TFTLCD_DELAYCMD                 0xFF

#define swap(a, b) { int16_t t = a; a = b; b = t; }

class Adafruit_TFTLCD : public Adafruit_GFX {
 public:
  Adafruit_TFTLCD(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset);

  uint16_t Color565(uint8_t r, uint8_t g, uint8_t b);

  // drawing primitives!
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  uint16_t readPixel(int16_t x, int16_t y);
  void fillScreen(uint16_t color);
  void drawFastVLine(int16_t x0, int16_t y0, int16_t length, uint16_t color);
  void drawFastHLine(int16_t x0, int16_t y0, int16_t length, uint16_t color);

  void fillRect(int16_t x0, int16_t y0, int16_t w, int16_t h, uint16_t color);

  // commands
  void begin(uint16_t id = 0x9325);
  void goTo(int x, int y);
  void setWindow(int x1, int y1, int x2, int y2);
  void reset(void);

  /* low level */

  void writeData(uint16_t d);
  void writeCommand(uint16_t c);
  void writeCommand8(uint8_t c);
  uint16_t readData(void);
  uint16_t readRegister(uint16_t addr);
  void writeRegister8(uint8_t a, uint8_t d);
  void writeRegister16(uint16_t addr, uint16_t data);

  static const uint16_t TFTWIDTH = 240;
  static const uint16_t TFTHEIGHT = 320;

  void writeData_unsafe(uint16_t d);

  void setWriteDir(void);
  void setReadDir(void);
  void write8(uint8_t d);

 private:
  uint16_t driver;

  void drawFastLine(int16_t x0, int16_t y0, int16_t l, uint16_t color, uint8_t flag);
  uint8_t read8(void);

  uint8_t _cs, _cd, _reset, _wr, _rd;

  uint8_t csport, cdport, wrport, rdport;
  uint8_t cspin, cdpin, wrpin, rdpin;
};

#endif
