// Graphics library by ladyada/adafruit with init code from Rossum 
// MIT license

#include "TFTLCD.h"
#include <avr/pgmspace.h>

void TFTLCD::goHome(void) {
  writeRegister(0x0020, 0X0000);     // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(0x0021, 0X0000);     // GRAM Address Set (Vertical Address) (R21h)
  writeCommand(0x0022);            // Write Data to GRAM (R22h)
}

// draw a rectangle
void TFTLCD::drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, 
		      uint16_t color) {
  // smarter version
  drawHorizontalLine(x, y, w, color);
  drawHorizontalLine(x, y+h-1, w, color);
  drawVerticalLine(x, y, h, color);
  drawVerticalLine(x+w-1, y, h, color);
}

// fill a circle
void TFTLCD::fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawVerticalLine(x0, y0-r, 2*r+1, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    drawVerticalLine(x0+x, y0-y, 2*y+1, color);
    drawVerticalLine(x0-x, y0-y, 2*y+1, color);
    drawVerticalLine(x0+y, y0-x, 2*x+1, color);
    drawVerticalLine(x0-y, y0-x, 2*x+1, color);
  }
}

// draw a circle outline
void TFTLCD::drawCircle(uint16_t x0, uint16_t y0, uint16_t r, 
			uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawPixel(x0, y0+r, color);
  drawPixel(x0, y0-r, color);
  drawPixel(x0+r, y0, color);
  drawPixel(x0-r, y0, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
    
  }
}

// fill a rectangle
void TFTLCD::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, 
		      uint16_t fillcolor) {
  // smarter version
  while (h--)
    drawHorizontalLine(x, y++, w, fillcolor);
}


void TFTLCD::drawVerticalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
  if (x >= width) return;

  drawFastLine(x,y,length,color,1);
}

void TFTLCD::drawHorizontalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
  if (y >= height) return;
  drawFastLine(x,y,length,color,0);
}

void TFTLCD::drawFastLine(uint16_t x, uint16_t y, uint16_t length, 
			  uint16_t color, uint8_t rotflag)
{
  if (rotflag)
    setRotation(2);

  writeRegister(TFTLCD_GRAM_HOR_AD, x); // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(TFTLCD_GRAM_VER_AD, y); // GRAM Address Set (Vertical Address) (R21h)
  writeCommand(TFTLCD_RW_GRAM);  // Write Data to GRAM (R22h)

  digitalWrite(_cs, LOW);
  digitalWrite(_cd, HIGH);
  digitalWrite(_rd, HIGH);
  digitalWrite(_wr, HIGH);

  setWriteDir();
  while (length--) {
    writeData_unsafe(color); 
  }
  digitalWrite(_cs, HIGH);

  if (rotflag)
    setRotation(3);
}



// bresenham's algorithm - thx wikpedia
void TFTLCD::drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, 
		      uint16_t color) {
  uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  uint16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;}

  for (; x0<x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}


void TFTLCD::fillScreen(uint16_t color) {
  goHome();
  uint32_t i;
  
  i = 320;
  i *= 240;
  
  digitalWrite(_cs, LOW);
  digitalWrite(_cd, HIGH);
  digitalWrite(_rd, HIGH);
  
  digitalWrite(_wr, HIGH);
  setWriteDir();
  while (i--) {
    writeData_unsafe(color); 
  }
  digitalWrite(_cs, HIGH);

}

void TFTLCD::drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  if ((x >= width) || (y >= height)) return;
  writeRegister(TFTLCD_GRAM_HOR_AD, x); // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(TFTLCD_GRAM_VER_AD, y); // GRAM Address Set (Vertical Address) (R21h)
  writeCommand(TFTLCD_RW_GRAM);  // Write Data to GRAM (R22h)
  writeData(color);
}

static const uint16_t _regValues[] PROGMEM = {
  TFTLCD_START_OSC, 0x0001,     // start oscillator

  TFTLCD_DELAYCMD, 50,          // this will make a delay of 50 milliseconds

  TFTLCD_DRIV_OUT_CTRL, 0x0100, 
  TFTLCD_DRIV_WAV_CTRL, 0x0700,
  TFTLCD_ENTRY_MOD, 0x1030,
  TFTLCD_RESIZE_CTRL, 0x0000,
  TFTLCD_DISP_CTRL2, 0x0202,
  TFTLCD_DISP_CTRL3, 0x0000,
  TFTLCD_DISP_CTRL4, 0x0000,
  TFTLCD_RGB_DISP_IF_CTRL1, 0x0,
  TFTLCD_FRM_MARKER_POS, 0x0,
  TFTLCD_RGB_DISP_IF_CTRL2, 0x0,
  
  TFTLCD_POW_CTRL1, 0x0000,
  TFTLCD_POW_CTRL2, 0x0007,
  TFTLCD_POW_CTRL3, 0x0000,
  TFTLCD_POW_CTRL4, 0x0000,

  TFTLCD_DELAYCMD, 200,  
  
  TFTLCD_POW_CTRL1, 0x1690,
  TFTLCD_POW_CTRL2, 0x0227,

  TFTLCD_DELAYCMD, 50,  

  TFTLCD_POW_CTRL3, 0x001A,

  TFTLCD_DELAYCMD, 50,  

  TFTLCD_POW_CTRL4, 0x1800,
  TFTLCD_POW_CTRL7, 0x002A,

  TFTLCD_DELAYCMD,50,
  
  TFTLCD_GAMMA_CTRL1, 0x0000,    
  TFTLCD_GAMMA_CTRL2, 0x0000, 
  TFTLCD_GAMMA_CTRL3, 0x0000,
  TFTLCD_GAMMA_CTRL4, 0x0206,   
  TFTLCD_GAMMA_CTRL5, 0x0808,  
  TFTLCD_GAMMA_CTRL6, 0x0007,  
  TFTLCD_GAMMA_CTRL7, 0x0201,
  TFTLCD_GAMMA_CTRL8, 0x0000,  
  TFTLCD_GAMMA_CTRL9, 0x0000,  
  TFTLCD_GAMMA_CTRL10, 0x0000,  
 
  TFTLCD_GRAM_HOR_AD, 0x0000,  
  TFTLCD_GRAM_VER_AD, 0x0000,  
  TFTLCD_HOR_START_AD, 0x0000,
  TFTLCD_HOR_END_AD, 0x00EF,
  TFTLCD_VER_START_AD, 0X0000,
  TFTLCD_VER_END_AD, 0x013F,
   

  TFTLCD_GATE_SCAN_CTRL1, 0xA700,     // Driver Output Control (R60h)
  TFTLCD_GATE_SCAN_CTRL2, 0x0003,     // Driver Output Control (R61h)
  TFTLCD_GATE_SCAN_CTRL3, 0x0000,     // Driver Output Control (R62h)

  TFTLCD_PANEL_IF_CTRL1, 0X0010,     // Panel Interface Control 1 (R90h)
  TFTLCD_PANEL_IF_CTRL2, 0X0000,
  TFTLCD_PANEL_IF_CTRL3, 0X0003,
  TFTLCD_PANEL_IF_CTRL4, 0X1100,
  TFTLCD_PANEL_IF_CTRL5, 0X0000,
  TFTLCD_PANEL_IF_CTRL6, 0X0000,

  // Display On
  TFTLCD_DISP_CTRL1, 0x0133,     // Display Control (R07h)
};

void TFTLCD::initDisplay(void) {
  uint16_t a, d;

  reset();
  
  for (uint8_t i = 0; i < sizeof(_regValues) / 4; i++) {
    a = pgm_read_word(_regValues + i*2);
    d = pgm_read_word(_regValues + i*2 + 1);

    if (a == 0xFF) {
      delay(d);
    } else {
      writeRegister(a, d);
      //Serial.print("addr: "); Serial.print(a); 
      //Serial.print(" data: "); Serial.println(d, HEX);
    }
  }
}

void TFTLCD::setRotation(uint8_t x) {
  switch (x) {
  case 0:
    writeRegister(TFTLCD_ENTRY_MOD, 0x1000);
    _width = 240; 
    _height = 320;
    break;
  case 1:
    _width = 240; 
    _height = 320;
    writeRegister(TFTLCD_ENTRY_MOD, 0x1038);
    break;
  case 2:
    _width = 240; 
    _height = 320;
    writeRegister(TFTLCD_ENTRY_MOD, 0x1028);
    break;
  case 3:
    _width = 240; 
    _height = 320;
    writeRegister(TFTLCD_ENTRY_MOD, 0x1030);
    break;
 }
}

/********************************* low level pin initialization */

TFTLCD::TFTLCD(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset) {
  _cs = cs;
  _cd = cd;
  _wr = wr;
  _rd = rd;
  _reset = reset;
  
  _width = 240;
  _height = 320;

  // disable the LCD
  digitalWrite(_cs, HIGH);
  pinMode(_cs, OUTPUT);  
  
  digitalWrite(_cd, HIGH);
  pinMode(_cd, OUTPUT);  
  
  digitalWrite(_wr, HIGH);
  pinMode(_wr, OUTPUT);  
  
  digitalWrite(_rd, HIGH);
  pinMode(_rd, OUTPUT);  

  digitalWrite(_reset, HIGH); 
  pinMode(_reset, OUTPUT); 
}


/********************************** low level pin interface */

void TFTLCD::reset(void) {
  digitalWrite(_reset, LOW);
  delay(2);
  digitalWrite(_reset, HIGH);
}

inline void TFTLCD::setWriteDir(void) {
  DDRB |= 0x3;
  DDRD |= 0xFC;
}

inline void TFTLCD::setReadDir(void) {
  DDRB &= ~0x3;
  DDRD &= ~0xFC;
}

inline void TFTLCD::write8(uint8_t d) {
 PORTB = (PINB & 0xFC) | (d & 0x3);  // bottom 2 bits
 PORTD = (PORTD & 0x3) | (d & 0xFC); // top 6 bits
}

inline uint8_t TFTLCD::read8(void) {
 uint8_t d;
 
 d = PIND & 0xFC;  // top 6 bits
 d |= PINB & 0x3;  // bottom 2 bits

 return d;
}

/********************************** low level readwrite interface */

// the C/D pin is high during write
void TFTLCD::writeData(uint16_t data) {
  digitalWrite(_cs, LOW);
  digitalWrite(_cd, HIGH);
  digitalWrite(_rd, HIGH);
  
  digitalWrite(_wr, HIGH);
  setWriteDir();
  write8(data >> 8);
  digitalWrite(_wr, LOW);
  digitalWrite(_wr, HIGH);
  write8(data);
  digitalWrite(_wr, LOW);
  digitalWrite(_wr, HIGH);
  
  digitalWrite(_cs, HIGH);
}

// this is a 'sped up' version, with no direction setting, or pin initialization
// not for external usage, but it does speed up stuff like a screen fill
inline void TFTLCD::writeData_unsafe(uint16_t data) {
  write8(data >> 8);
  digitalWrite(_wr, LOW);
  digitalWrite(_wr, HIGH);
  write8(data);
  digitalWrite(_wr, LOW);
  digitalWrite(_wr, HIGH);
}

// the C/D pin is low during write
void TFTLCD::writeCommand(uint16_t cmd) {
  digitalWrite(_cs, LOW);
  digitalWrite(_cd, LOW);
  digitalWrite(_rd, HIGH);
  
  digitalWrite(_wr, HIGH);
  setWriteDir();
  write8(cmd >> 8);
  digitalWrite(_wr, LOW);
  digitalWrite(_wr, HIGH);
  write8(cmd);
  digitalWrite(_wr, LOW);
  digitalWrite(_wr, HIGH);
  
  digitalWrite(_cs, HIGH);
}

uint16_t TFTLCD::readData() {
 uint16_t d = 0;
 
 digitalWrite(_rd, HIGH);
 digitalWrite(_cd, HIGH);
 digitalWrite(_wr, HIGH);
 digitalWrite(_cs, LOW);
 
 setReadDir();
 digitalWrite(_rd, LOW);
 delay(1);
 d = read8();
 d <<= 8;
 digitalWrite(_rd, HIGH);
 digitalWrite(_rd, LOW);
 delay(1);
 d |= read8();
 digitalWrite(_rd, HIGH);
 
 digitalWrite(_cs, HIGH);
   
 return d;
}


/************************************* medium level data reading/writing */

uint16_t TFTLCD::readRegister(uint16_t addr) {
   writeCommand(addr);
   return readData();
}

void TFTLCD::writeRegister(uint16_t addr, uint16_t data) {
   writeCommand(addr);
   writeData(data);
}




