#include "Adafruit_TFTLCD.h"

// Graphics library by ladyada/adafruit with init code from Rossum 
// MIT license

static volatile uint8_t *wrportreg;


#ifdef USE_ADAFRUIT_SHIELD_PINOUT
// special defines for the dataport

// for mega & shield usage, we just hardcoded it (its messy)
 #define DATAPORT1 PORTD
 #define DATAPIN1 PIND
 #define DATADDR1 DDRD

 #define DATAPORT2 PORTB
 #define DATAPIN2 PINB
 #define DATADDR2 DDRB

 #define DATA1_MASK 0xD0
 #define DATA2_MASK 0x2F

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__)
 #define WRPORT PORTC
 #define RDPORT PORTC
 #define CSPORT PORTC
 #define CDPORT PORTC
 #define WRPIN  1
 #define CDPIN  2
 #define CSPIN  3
 #define RDPIN  0
#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) 
 #define WRPORT PORTF
 #define RDPORT PORTF
 #define CSPORT PORTF
 #define CDPORT PORTF
 #define WRPIN  1
 #define CDPIN  2
 #define CSPIN  3
 #define RDPIN  0
#endif 


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


#include <avr/pgmspace.h>
#include "pins_arduino.h"
#include "wiring_private.h"

void Adafruit_TFTLCD::setAddrWindow(int x1, int y1, int x2, int y2) {
 
  if (driver == 0x9325 || driver == 0x9328) {
    writeRegister16(0x0050, x1); // Set address window
    writeRegister16(0x0051, x2);
    writeRegister16(0x0052, y1);
    writeRegister16(0x0053, y2);
    writeRegister16(0x0020, x1); // Set address counter to top-left
    writeRegister16(0x0021, y1);
  } else if(driver == 0x7575) {
    writeRegister8(HX8347G_COLADDRSTART2, x1>>8);
    writeRegister8(HX8347G_COLADDRSTART1, x1);
    writeRegister8(HX8347G_ROWADDRSTART2, y1>>8);
    writeRegister8(HX8347G_ROWADDRSTART1, y1);
    writeRegister8(HX8347G_COLADDREND2  , x2>>8);
    writeRegister8(HX8347G_COLADDREND1  , x2);
    writeRegister8(HX8347G_ROWADDREND2  , y2>>8);
    writeRegister8(HX8347G_ROWADDREND1  , y2);
  }
  writeCommand(0x0022); // Write Data to GRAM
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
void Adafruit_TFTLCD::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, 
		      uint16_t fillcolor) {
  // smarter version
  while (h--)
    drawFastHLine(x, y++, w, fillcolor);
}


void Adafruit_TFTLCD::drawFastVLine(int16_t x, int16_t y, int16_t length, uint16_t color)
{
  if (x >= _width) return;

  drawFastLine(x,y,length,color,1);
}

void Adafruit_TFTLCD::drawFastHLine(int16_t x, int16_t y, int16_t length, uint16_t color)
{
  if (y >= _height) return;
  drawFastLine(x,y,length,color,0);
}

void Adafruit_TFTLCD::drawFastLine(int16_t x, int16_t y, int16_t length, 
			  uint16_t color, uint8_t rotflag)
{
  if ((driver == 0x9325) || (driver == 0x9328)) {

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
    
    writeRegister16(ILI932X_ENTRY_MOD, newentrymod);
    
    writeRegister16(ILI932X_GRAM_HOR_AD, x); // GRAM Address Set (Horizontal Address) (R20h)
    writeRegister16(ILI932X_GRAM_VER_AD, y); // GRAM Address Set (Vertical Address) (R21h)
    writeCommand(ILI932X_RW_GRAM);  // Write Data to GRAM (R22h)
  }
  if (driver == 0x7575) {
    uint16_t endx, endy;
    if (rotation == 0) {
      endx = x, endy = y;

      if (rotflag) endy += length;
      else	endx += length;
    } else if (rotation == 1) {
      swap(x, y);
      // first up fix the X
      x = TFTWIDTH - x - 1;
      endx = x, endy = y;

      if (rotflag) x -= (length-1);
      else	endy += length;
    } else if (rotation == 2) {
      x = TFTWIDTH - x - 1;
      y = TFTHEIGHT - y - 1;
      endx = x, endy = y;

      if (rotflag) y -= (length-1);
      else         x -= (length-1);
    } else if (rotation == 3) {
      swap(x,y);
      y = TFTHEIGHT - y - 1;
      endx = x, endy = y;

      if (rotflag) endx += (length-1);
      else         y -= (length-1);
    }
    // Serial.print("("); Serial.print(x); Serial.print(", "); Serial.print(y); Serial.print(") -> ");

    writeRegister8(HX8347G_COLADDRSTART2, x>>8);
    writeRegister8(HX8347G_COLADDRSTART1, x);
    writeRegister8(HX8347G_ROWADDRSTART2, y>>8);
    writeRegister8(HX8347G_ROWADDRSTART1, y);

    // Serial.print("("); Serial.print(endx); Serial.print(", "); Serial.print(endy); Serial.println(")");
    writeRegister8(HX8347G_COLADDREND2, endx>>8);
    writeRegister8(HX8347G_COLADDREND1, endx);
    writeRegister8(HX8347G_ROWADDREND2, endy>>8);
    writeRegister8(HX8347G_ROWADDREND1, endy);
    writeCommand(0x0022);            // Write Data to GRAM (R22h)
  }

  *portOutputRegister(csport) &= ~cspin;  //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) |= cdpin;   //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;   //digitalWrite(_rd, HIGH);
  *portOutputRegister(wrport) |= wrpin;   //digitalWrite(_wr, HIGH);

  setWriteDir();
  while (length--) {
    writeData_unsafe(color); 
  }

  // set back to default
  *portOutputRegister(csport) |= cspin;    //digitalWrite(_cs, HIGH);

  if (driver == 0x7575) {
    setAddrWindow(0, 0, TFTWIDTH, TFTHEIGHT);
  } else {
    writeRegister16(ILI932X_ENTRY_MOD, 0x1030);
  }
}



void Adafruit_TFTLCD::fillScreen(uint16_t color) {
  uint32_t i = (uint32_t)TFTWIDTH * (uint32_t)TFTHEIGHT;
  
  setAddrWindow(0, 0, TFTWIDTH, TFTHEIGHT);

  *portOutputRegister(csport) &= ~cspin;    //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) |=  cdpin;    //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |=  rdpin;    //digitalWrite(_rd, HIGH);
  *portOutputRegister(wrport) |=  wrpin;    //digitalWrite(_wr, HIGH);

  setWriteDir();
  while (i--) {
    writeData_unsafe(color); 
  }

  *portOutputRegister(csport) |=  cspin;    //digitalWrite(_cs, HIGH);
}

void Adafruit_TFTLCD::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if ((x >= _width) || (y >= _height)) return;

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
    
  if (driver == 0x9328 || driver == 0x9325) {
    writeRegister16(ILI932X_GRAM_HOR_AD, x); // GRAM Address Set (Horizontal Address)
    writeRegister16(ILI932X_GRAM_VER_AD, y); // GRAM Address Set (Vertical Address)
    writeCommand(ILI932X_RW_GRAM);  // Write Data to GRAM
  } else if (driver == 0x7575) {
    writeRegister8(HX8347G_COLADDRSTART2, x >> 8);
    writeRegister8(HX8347G_COLADDRSTART1, x & 0xFF);
    writeRegister8(HX8347G_ROWADDRSTART2, y >> 8);
    writeRegister8(HX8347G_ROWADDRSTART1, y & 0xFF);
    writeCommand8(0x0022);            // Write Data to GRAM (R22h)
  }
  writeData(color);
}

uint16_t Adafruit_TFTLCD::readPixel(int16_t x, int16_t y)
{
  if ((x >= _width) || (y >= _height)) return 0;

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
    
  if (driver == 0x9328 || driver == 0x9325) {
    writeRegister16(ILI932X_GRAM_HOR_AD, x); // GRAM Address Set (Horizontal Address)
    writeRegister16(ILI932X_GRAM_VER_AD, y); // GRAM Address Set (Vertical Address)
    writeCommand(ILI932X_RW_GRAM);  // Read Data to GRAM
  } else if (driver == 0x7575) {
    writeRegister8(HX8347G_COLADDRSTART2, x >> 8);
    writeRegister8(HX8347G_COLADDRSTART1, x & 0xFF);
    writeRegister8(HX8347G_ROWADDRSTART2, y >> 8);
    writeRegister8(HX8347G_ROWADDRSTART1, y & 0xFF);
    writeCommand8(0x0022);            // Read Data from GRAM (R22h)
  }
  return readData();
}


static const uint16_t HX8347G_regValues[] PROGMEM = {
  0x2E, 0x89,
  0x29, 0x8F,
  0x2B, 0x02,
  0xE2, 0x00,
  0xE4, 0x01,
  0xE5, 0x10,
  0xE6, 0x01,
  0xE7, 0x10,
  0xE8, 0x70,
  0xF2, 0x00,
  0xEA, 0x00,
  0xEB, 0x20,
  0xEC, 0x3C,
  0xED, 0xC8,
  0xE9, 0x38,
  0xF1, 0x01,

  // skip gamma, do later

  0x1B, 0x1A,
  0x1A, 0x02,
  0x24, 0x61,
  0x25, 0x5C,
  
  0x18, 0x36,
  0x19, 0x01,
  0x1F, 0x88,
  TFTLCD_DELAYCMD, 5, // delay 5 ms
  0x1F, 0x80,
  TFTLCD_DELAYCMD, 5,
  0x1F, 0x90,
  TFTLCD_DELAYCMD, 5,
  0x1F, 0xD4,
  TFTLCD_DELAYCMD, 5,
  0x17, 0x05,

  0x36, 0x09,
  0x28, 0x38,
  TFTLCD_DELAYCMD, 40,
  0x28, 0x3C,

  0x02, 0x00,
  0x03, 0x00,
  0x04, 0x00,
  0x05, 0xEF,
  0x06, 0x00,
  0x07, 0x00,
  0x08, 0x01,
  0x09, 0x3F,
};

static const uint16_t ILI932x_regValues[] PROGMEM = {
  ILI932X_START_OSC, 0x0001,     // start oscillator

  TFTLCD_DELAYCMD, 50,          // this will make a delay of 50 milliseconds

  ILI932X_DRIV_OUT_CTRL, 0x0100, 
  ILI932X_DRIV_WAV_CTRL, 0x0700,
  ILI932X_ENTRY_MOD, 0x1030,
  ILI932X_RESIZE_CTRL, 0x0000,
  ILI932X_DISP_CTRL2, 0x0202,
  ILI932X_DISP_CTRL3, 0x0000,
  ILI932X_DISP_CTRL4, 0x0000,
  ILI932X_RGB_DISP_IF_CTRL1, 0x0,
  ILI932X_FRM_MARKER_POS, 0x0,
  ILI932X_RGB_DISP_IF_CTRL2, 0x0,
  
  ILI932X_POW_CTRL1, 0x0000,
  ILI932X_POW_CTRL2, 0x0007,
  ILI932X_POW_CTRL3, 0x0000,
  ILI932X_POW_CTRL4, 0x0000,

  TFTLCD_DELAYCMD, 200,  
  
  ILI932X_POW_CTRL1, 0x1690,
  ILI932X_POW_CTRL2, 0x0227,

  TFTLCD_DELAYCMD, 50,  

  ILI932X_POW_CTRL3, 0x001A,

  TFTLCD_DELAYCMD, 50,  

  ILI932X_POW_CTRL4, 0x1800,
  ILI932X_POW_CTRL7, 0x002A,

  TFTLCD_DELAYCMD,50,
  
  ILI932X_GAMMA_CTRL1, 0x0000,    
  ILI932X_GAMMA_CTRL2, 0x0000, 
  ILI932X_GAMMA_CTRL3, 0x0000,
  ILI932X_GAMMA_CTRL4, 0x0206,   
  ILI932X_GAMMA_CTRL5, 0x0808,  
  ILI932X_GAMMA_CTRL6, 0x0007,  
  ILI932X_GAMMA_CTRL7, 0x0201,
  ILI932X_GAMMA_CTRL8, 0x0000,  
  ILI932X_GAMMA_CTRL9, 0x0000,  
  ILI932X_GAMMA_CTRL10, 0x0000,  
 
  ILI932X_GRAM_HOR_AD, 0x0000,  
  ILI932X_GRAM_VER_AD, 0x0000,  
  ILI932X_HOR_START_AD, 0x0000,
  ILI932X_HOR_END_AD, 0x00EF,
  ILI932X_VER_START_AD, 0X0000,
  ILI932X_VER_END_AD, 0x013F,
   

  ILI932X_GATE_SCAN_CTRL1, 0xA700,     // Driver Output Control (R60h)
  ILI932X_GATE_SCAN_CTRL2, 0x0003,     // Driver Output Control (R61h)
  ILI932X_GATE_SCAN_CTRL3, 0x0000,     // Driver Output Control (R62h)

  ILI932X_PANEL_IF_CTRL1, 0X0010,     // Panel Interface Control 1 (R90h)
  ILI932X_PANEL_IF_CTRL2, 0X0000,
  ILI932X_PANEL_IF_CTRL3, 0X0003,
  ILI932X_PANEL_IF_CTRL4, 0X1100,
  ILI932X_PANEL_IF_CTRL5, 0X0000,
  ILI932X_PANEL_IF_CTRL6, 0X0000,

  // Display On
  ILI932X_DISP_CTRL1, 0x0133,     // Display Control (R07h)
};

void Adafruit_TFTLCD::begin(uint16_t id) {
  uint16_t a, d;

  constructor(TFTWIDTH, TFTHEIGHT);

  reset();
  driver = id;

  if (id == 0x9325 || id == 0x9328) {
    for (uint8_t i = 0; i < sizeof(ILI932x_regValues) / 4; i++) {
      a = pgm_read_word(ILI932x_regValues + i*2);
      d = pgm_read_word(ILI932x_regValues + i*2 + 1);
      
      if (a == 0xFF) {
	delay(d);
      } else {
	writeRegister16(a, d);
	//Serial.print("addr: "); Serial.print(a); 
	//Serial.print(" data: "); Serial.println(d, HEX);
      }
    }
  } else if (id == 0x7575) {
    for (uint8_t i = 0; i < sizeof(HX8347G_regValues) / 4; i++) {
      a = pgm_read_word(HX8347G_regValues + i*2);
      d = pgm_read_word(HX8347G_regValues + i*2 + 1);
      
      if (a == 0xFF) {
	delay(d);
      } else {
	writeRegister8(a, d);
	//Serial.print("addr: "); Serial.print(a); 
	//Serial.print(" data: "); Serial.println(d, HEX);
      }
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
  *wrportreg |=  wrpin;                   //digitalWrite(_wr, HIGH);
  *portOutputRegister(csport) &= ~cspin;  //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) |= cdpin;   //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;   //digitalWrite(_rd, HIGH);
#endif

  setWriteDir();
  write8(data >> 8);
  
#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
#else
  *wrportreg &= ~wrpin;    //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;    //digitalWrite(_wr, HIGH);
#endif

  write8(data);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
  CSPORT |= _BV(CSPIN);
#else
  *wrportreg &= ~wrpin;   //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;   //digitalWrite(_wr, HIGH);
  *portOutputRegister(csport) |= cspin;    //digitalWrite(_cs, HIGH);
#endif
}

inline void Adafruit_TFTLCD::writeRegister8(uint8_t addr, uint8_t data) {
#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT |= _BV(WRPIN);
  CSPORT &= ~_BV(CSPIN);
  CDPORT &= ~_BV(CDPIN);
  RDPORT |= _BV(RDPIN);
#else
  *wrportreg |=  wrpin;                    //digitalWrite(_wr, HIGH);
  *portOutputRegister(csport) &= ~cspin;   //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) &= ~cdpin;   //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;    //digitalWrite(_rd, HIGH);
#endif

  setWriteDir();
  write8(addr);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
#else
  *wrportreg &= ~wrpin;   //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;   //digitalWrite(_wr, HIGH);
#endif

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  CDPORT |= _BV(CDPIN);
#else
  *portOutputRegister(cdport) |= cdpin;     //digitalWrite(_cd, HIGH);
#endif

  write8(data);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
  CSPORT |= _BV(CSPIN);
#else
  *wrportreg &= ~wrpin;                    //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;                    //digitalWrite(_wr, HIGH);
  *portOutputRegister(csport) |= cspin;    //digitalWrite(_cs, HIGH);
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
  *wrportreg &= ~wrpin;   //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;   //digitalWrite(_wr, HIGH);
#endif

  write8(data);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
#else
  *wrportreg &= ~wrpin;   //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;   //digitalWrite(_wr, HIGH);
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
  *wrportreg |=  wrpin;                    //digitalWrite(_wr, HIGH);
  *portOutputRegister(csport) &= ~cspin;   //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) &= ~cdpin;   //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;    //digitalWrite(_rd, HIGH);
#endif

  setWriteDir();
  write8(cmd >> 8);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
#else
  *wrportreg &= ~wrpin;    //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;    //digitalWrite(_wr, HIGH);
#endif

  write8(cmd);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
  CSPORT |= _BV(CSPIN);
#else
  *wrportreg &= ~wrpin;    //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;    //digitalWrite(_wr, HIGH);
  *portOutputRegister(csport) |= cspin;   //digitalWrite(_cs, HIGH);
#endif
}

inline void Adafruit_TFTLCD::writeCommand8(uint8_t cmd) {
#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT |= _BV(WRPIN);
  CSPORT &= ~_BV(CSPIN);
  CDPORT &= ~_BV(CDPIN);
  RDPORT |= _BV(RDPIN);
#else
  *wrportreg |=  wrpin;                    //digitalWrite(_wr, HIGH);
  *portOutputRegister(csport) &= ~cspin;   //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) &= ~cdpin;   //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;    //digitalWrite(_rd, HIGH);
#endif

  setWriteDir();
  write8(cmd);

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  WRPORT &= ~_BV(WRPIN);
  WRPORT |= _BV(WRPIN);
  CSPORT |= _BV(CSPIN);
#else
  *wrportreg &= ~wrpin;   //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;   //digitalWrite(_wr, HIGH);
  *portOutputRegister(csport) |= cspin;  //digitalWrite(_cs, HIGH);
#endif
}

uint16_t Adafruit_TFTLCD::readData() {
 uint16_t d = 0;
 
  *portOutputRegister(csport) &= ~cspin;  //digitalWrite(_cs, LOW);
  *portOutputRegister(cdport) |= cdpin;   //digitalWrite(_cd, HIGH);
  *portOutputRegister(rdport) |= rdpin;   //digitalWrite(_rd, HIGH);
  *portOutputRegister(wrport) |= wrpin;   //digitalWrite(_wr, HIGH);
  
  setReadDir();

  *portOutputRegister(rdport) &= ~rdpin;  //digitalWrite(_rd, LOW);

  delayMicroseconds(10);
  d = read8();
  d <<= 8;

  *portOutputRegister(rdport) |= rdpin;   //digitalWrite(_rd, HIGH);
  *portOutputRegister(rdport) &= ~rdpin;  //digitalWrite(_rd, LOW);

  delayMicroseconds(10);
  d |= read8();

  *portOutputRegister(rdport) |= rdpin;   //digitalWrite(_rd, HIGH);
  *portOutputRegister(csport) |= cspin;   //digitalWrite(_cs, HIGH);
   
  return d;
}


/************************************* medium level data reading/writing */

uint16_t Adafruit_TFTLCD::readRegister(uint16_t addr) {
   writeCommand(addr);
   return readData();
}


void Adafruit_TFTLCD::writeRegister16(uint16_t addr, uint16_t data) {
  writeCommand(addr);
  writeData(data);
}




