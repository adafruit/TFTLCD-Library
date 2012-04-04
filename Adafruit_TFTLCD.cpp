#include "Adafruit_TFTLCD.h"

// Graphics library by ladyada/adafruit with init code from Rossum 
// MIT license

static volatile uint8_t *wrportreg;


#ifdef USE_ADAFRUIT_SHIELD_PINOUT
// special defines for the dataport
 #define DATAPORT1 PORTD
 #define DATAPIN1 PIND
 #define DATADDR1 DDRD

 #define DATAPORT2 PORTB
 #define DATAPIN2 PINB
 #define DATADDR2 DDRB

 #define DATA1_MASK 0xD0
 #define DATA2_MASK 0x2F

 #define WRPORT PORTC
 #define RDPORT PORTC
 #define CSPORT PORTC
 #define CDPORT PORTC
 #define WRPIN  1
 #define CDPIN  2
 #define CSPIN  3
 #define RDPIN  4

// for mega & shield usage, we just hardcoded it (its messy)

#else
 // for the breakout board tutorial, two ports are used :/
 #define DATAPORT1 PORTD
 #define DATAPIN1  PIND
 #define DATADDR1  DDRD

 #define DATAPORT2 PORTB
 #define DATAPIN2  PINB
 #define DATADDR2  DDRB

 #define DATA1_MASK 0xFC  // top 6 bits
 #define DATA2_MASK 0x03  // bottom 2 bits

 // Megas have lots of pins, we'll use port A - all 8 bits in a row - pins 22 thru 29
 #define MEGA_DATAPORT PORTA
 #define MEGA_DATAPIN  PINA
 #define MEGA_DATADDR  DDRA
#endif


#include "glcdfont.c"
#include <avr/pgmspace.h>
#include "pins_arduino.h"
#include "wiring_private.h"


void Adafruit_TFTLCD::goTo(int x, int y) {
  writeRegister(0x0020, x);     // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(0x0021, y);     // GRAM Address Set (Vertical Address) (R21h)
  writeCommand(0x0022);            // Write Data to GRAM (R22h)
}


uint16_t Adafruit_TFTLCD::Color565(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c;
  c = r >> 3;
  c <<= 6;
  c |= g >> 2;
  c <<= 5;
  c |= b >> 3;

  return c;
}

// fill a rectangle
void Adafruit_TFTLCD::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, 
		      uint16_t fillcolor) {
  // smarter version
  while (h--)
    drawFastHLine(x, y++, w, fillcolor);
}


void Adafruit_TFTLCD::drawFastVLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
  if (x >= _width) return;

  drawFastLine(x,y,length,color,1);
}

void Adafruit_TFTLCD::drawFastHLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color)
{
  if (y >= _height) return;
  drawFastLine(x,y,length,color,0);
}

void Adafruit_TFTLCD::drawFastLine(uint16_t x, uint16_t y, uint16_t length, 
			  uint16_t color, uint8_t rotflag)
{
  uint16_t newentrymod;

  switch (rotation) {
  case 0:
    if (rotflag)
      newentrymod = 0x1028;   // we want a 'vertical line'
    else 
      newentrymod = 0x1030;   // we want a 'horizontal line'
    break;
  case 1:
    swap(x, y);
    // first up fix the X
    x = TFTWIDTH - x - 1;
    if (rotflag)
      newentrymod = 0x1000;   // we want a 'vertical line'
    else 
      newentrymod = 0x1028;   // we want a 'horizontal line'
    break;
  case 2:
    x = TFTWIDTH - x - 1;
    y = TFTHEIGHT - y - 1;
    if (rotflag)
      newentrymod = 0x1008;   // we want a 'vertical line'
    else 
      newentrymod = 0x1020;   // we want a 'horizontal line'
    break;
  case 3:
    swap(x,y);
    y = TFTHEIGHT - y - 1;
    if (rotflag)
      newentrymod = 0x1030;   // we want a 'vertical line'
    else 
      newentrymod = 0x1008;   // we want a 'horizontal line'
    break;
  }
  
  writeRegister(TFTLCD_ENTRY_MOD, newentrymod);

  writeRegister(TFTLCD_GRAM_HOR_AD, x); // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(TFTLCD_GRAM_VER_AD, y); // GRAM Address Set (Vertical Address) (R21h)
  writeCommand(TFTLCD_RW_GRAM);  // Write Data to GRAM (R22h)


  *portOutputRegister(csport) &= ~cspin;
  //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) |= cdpin;
  //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
  *portOutputRegister(wrport) |= wrpin;
  //digitalWrite(_wr, HIGH);

  setWriteDir();
  while (length--) {
    writeData_unsafe(color); 
  }

  // set back to default
  *portOutputRegister(csport) |= cspin;
  //digitalWrite(_cs, HIGH);
  writeRegister(TFTLCD_ENTRY_MOD, 0x1030);
}




void Adafruit_TFTLCD::fillScreen(uint16_t color) {
  goTo(0,0);
  uint32_t i;
  
  i = 320;
  i *= 240;
  
  *portOutputRegister(csport) &= ~cspin;
  //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) |= cdpin;
  //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
  *portOutputRegister(wrport) |= wrpin;
  //digitalWrite(_wr, HIGH);

  setWriteDir();
  while (i--) {
    writeData_unsafe(color); 
  }

  *portOutputRegister(csport) |= cspin;
  //digitalWrite(_cs, HIGH);
}

void Adafruit_TFTLCD::drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  // check rotation, move pixel around if necessary
  switch (rotation) {
  case 1:
    swap(x, y);
    x = TFTWIDTH - x - 1;
    break;
  case 2:
    x = TFTWIDTH - x - 1;
    y = TFTHEIGHT - y - 1;
    break;
  case 3:
    swap(x, y);
    y = TFTHEIGHT - y - 1;
    break;
  }
    
  if ((x >= TFTWIDTH) || (y >= TFTHEIGHT)) return;
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

void Adafruit_TFTLCD::begin(void) {
  constructor(240,320);

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

/********************************* low level pin initialization */

Adafruit_TFTLCD::Adafruit_TFTLCD(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset) {
  _cs = cs;
  _cd = cd;
  _wr = wr;
  _rd = rd;
  _reset = reset;
  
  rotation = 0;
  _width = TFTWIDTH;
  _height = TFTHEIGHT;

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

  csport = digitalPinToPort(_cs);
  cdport = digitalPinToPort(_cd);
  wrport = digitalPinToPort(_wr);
  rdport = digitalPinToPort(_rd);

  cspin = digitalPinToBitMask(_cs);
  cdpin = digitalPinToBitMask(_cd);
  wrpin = digitalPinToBitMask(_wr);
  rdpin = digitalPinToBitMask(_rd);

  wrportreg = portOutputRegister(wrport);

  cursor_y = cursor_x = 0;
  textsize = 1;
  textcolor = 0xFFFF;
}


/********************************** low level pin interface */

void Adafruit_TFTLCD::reset(void) {
  if (_reset)
    digitalWrite(_reset, LOW);
  delay(2); 
  if (_reset)
    digitalWrite(_reset, HIGH);

  // resync
  writeData(0);
  writeData(0);
  writeData(0);  
  writeData(0);
}

inline void Adafruit_TFTLCD::setWriteDir(void) {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__)
  DATADDR2 |= DATA2_MASK;
  DATADDR1 |= DATA1_MASK;
#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) 

  #ifdef USE_ADAFRUIT_SHIELD_PINOUT
  DDRH |= 0x78;
  DDRB |= 0xB0;
  DDRG |= _BV(5);
  #else
  MEGA_DATADDR = 0xFF;
  #endif
#else
  #error "No pins defined!"
#endif
}

inline void Adafruit_TFTLCD::setReadDir(void) {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328) || (__AVR_ATmega8__)
  DATADDR2 &= ~DATA2_MASK;
  DATADDR1 &= ~DATA1_MASK;
#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) 

  #ifdef USE_ADAFRUIT_SHIELD_PINOUT
  DDRH &= ~0x78;
  DDRB &= ~0xB0;
  DDRG &= ~_BV(5);
  #else
  MEGA_DATADDR = 0;
  #endif
#else
  #error "No pins defined!"
#endif
}

inline void Adafruit_TFTLCD::write8(uint8_t d) {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328) || (__AVR_ATmega8__)

  DATAPORT2 = (DATAPORT2 & DATA1_MASK) | 
    (d & DATA2_MASK);
  DATAPORT1 = (DATAPORT1 & DATA2_MASK) | 
    (d & DATA1_MASK); // top 6 bits
  
#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) 


#ifdef USE_ADAFRUIT_SHIELD_PINOUT

  // bit 6/7 (PH3 & 4)
  // first two bits 0 & 1 (PH5 & 6)
  PORTH &= ~(0x78);
  PORTH |= ((d&0xC0) >> 3) | ((d&0x3) << 5);

  // bits 2 & 3 (PB4 & PB5)
  // bit 5 (PB7)
  PORTB &= ~(0xB0); 
  PORTB |= ((d & 0x2C) << 2);

  // bit 4  (PG5)
  if (d & _BV(4))
    PORTG |= _BV(5);
  else
    PORTG &= ~_BV(5);

  #else
     MEGA_DATAPORT = d;  
  #endif

#else
  #error "No pins defined!"
#endif
}

inline uint8_t Adafruit_TFTLCD::read8(void) {
 uint8_t d;
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328) || (__AVR_ATmega8__)

 d = DATAPIN1 & DATA1_MASK; 
 d |= DATAPIN2 & DATA2_MASK; 

#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__)  || defined(__AVR_ATmega1280__) 

#ifdef USE_ADAFRUIT_SHIELD_PINOUT

  // bit 6/7 (PH3 & 4)
  // first two bits 0 & 1 (PH5 & 6)
 d = (PINH & 0x60) >> 5;
 d |= (PINH & 0x18) << 3;

  // bits 2 & 3 & 5 (PB4 & PB5, PB7)
 d |= (PINB & 0xB0) >> 2;

  // bit 4  (PG5)
  if (PING & _BV(5))
    d |= _BV(4);

#else
 d = MEGA_DATAPIN;  
#endif

#else

  #error "No pins defined!"

#endif

 return d;
}

/********************************** low level readwrite interface */

// the C/D pin is high during write
void Adafruit_TFTLCD::writeData(uint16_t data) {

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT |= _BV(WRPIN);
  CSPORT &= ~_BV(CSPIN);
  CDPORT |= _BV(CDPIN);
  RDPORT |= _BV(RDPIN);
#else
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);
  *portOutputRegister(csport) &= ~cspin;
  //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) |= cdpin;
  //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
#endif

  setWriteDir();
  write8(data >> 8);
  
#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
#else
  *wrportreg &= ~wrpin;
  //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);
#endif

  write8(data);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
#else
  *wrportreg &= ~wrpin;
  //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);
#endif

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  CSPORT |= _BV(CSPIN);
#else
  *portOutputRegister(csport) |= cspin;
  //digitalWrite(_cs, HIGH);
#endif
}

// this is a 'sped up' version, with no direction setting, or pin initialization
// not for external usage, but it does speed up stuff like a screen fill
inline void Adafruit_TFTLCD::writeData_unsafe(uint16_t data) {
  write8(data >> 8);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
#else
  *wrportreg &= ~wrpin;
  //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);
#endif

  write8(data);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
#else
  *wrportreg &= ~wrpin;
  //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);
#endif
}

// the C/D pin is low during write
void Adafruit_TFTLCD::writeCommand(uint16_t cmd) {
#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT |= _BV(WRPIN);
  CSPORT &= ~_BV(CSPIN);
  CDPORT &= ~_BV(CDPIN);
  RDPORT |= _BV(RDPIN);
#else
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);
  *portOutputRegister(csport) &= ~cspin;
  //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) &= ~cdpin;
  //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
#endif

  setWriteDir();
  write8(cmd >> 8);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
#else
  *wrportreg &= ~wrpin;
  //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);
#endif

  write8(cmd);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
#else
  *wrportreg &= ~wrpin;
  //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);
#endif

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  CSPORT |= _BV(CSPIN);
#else
  *portOutputRegister(csport) |= cspin;
  //digitalWrite(_cs, HIGH);
#endif
}

uint16_t Adafruit_TFTLCD::readData() {
 uint16_t d = 0;
 
  *portOutputRegister(csport) &= ~cspin;
  //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) |= cdpin;
  //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
  *portOutputRegister(wrport) |= wrpin;
  //digitalWrite(_wr, HIGH);
  
  setReadDir();

  *portOutputRegister(rdport) &= ~rdpin;
  //digitalWrite(_rd, LOW);

  delayMicroseconds(10);
  d = read8();
  d <<= 8;

  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
  *portOutputRegister(rdport) &= ~rdpin;
  //digitalWrite(_rd, LOW);

  delayMicroseconds(10);
  d |= read8();

  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
  
  *portOutputRegister(csport) |= cspin;
  //digitalWrite(_cs, HIGH);
   
  return d;
}


/************************************* medium level data reading/writing */

uint16_t Adafruit_TFTLCD::readRegister(uint16_t addr) {
   writeCommand(addr);
   return readData();
}

void Adafruit_TFTLCD::writeRegister(uint16_t addr, uint16_t data) {
   writeCommand(addr);
   writeData(data);
}




