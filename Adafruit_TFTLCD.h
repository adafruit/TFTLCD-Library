/*!
 * @brief Adafruit_TFTLCD.h
 */
// IMPORTANT: SEE COMMENTS @ LINE 15 REGARDING SHIELD VS BREAKOUT BOARD USAGE.

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

// **** IF USING THE LCD BREAKOUT BOARD, COMMENT OUT THIS NEXT LINE. ****
// **** IF USING THE LCD SHIELD, LEAVE THE LINE ENABLED:             ****

//#define USE_ADAFRUIT_SHIELD_PINOUT 1

class Adafruit_TFTLCD : public Adafruit_GFX {

public:
  /*!
   * @brief TFTLCD constructor
   * @param cs Chip select pin
   * @param cd Command/data pin
   * @param wr Write pin
   * @param rd Read pin
   * @param rst Reset pin
   */
  Adafruit_TFTLCD(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t rst);
  Adafruit_TFTLCD(void);

  /*!
   * @brief Begins connection
   * @param id LCD controller id
   */
  void begin(uint16_t id = 0x9325);
  /*!
   * @brief Sets a specified pixel the specified color
   * @param x X location of the pixel
   * @param y Y location of the pixel
   * @param color Color to set the pixel
   */
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  /*!
   * @brief Draw fast horizontal line
   * @param x0 Starting X position
   * @param y0 Starting Y position
   * @param w Length of the line
   * @param color Color of the line
   */
  void drawFastHLine(int16_t x0, int16_t y0, int16_t w, uint16_t color);
  /*!
   * @brief Draw fast vertical line
   * @param x0 Starting X position
   * @param y0 Starting Y position
   * @param h Height of the line
   * @param color Color of the line
   */
  void drawFastVLine(int16_t x0, int16_t y0, int16_t h, uint16_t color);
  /*!
   * @brief Makes a filled rectangle
   * @param x Initial x value of the rectangle
   * @param y Initial y value of the rectangle
   * @param w Width of the rectangle
   * @param h Height of the rectangle
   * @param c Color of the rectangle
   */
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
  /*!
   * @brief Fills screen with the specified color
   * @param color Color to set the screen
   */
  void fillScreen(uint16_t color);
  /*!
   * @brief Resets the display
   */
  void reset(void);
  /*!
   * @brief Appears to be deprecated. Sets a register
   * @param ptr Pointer to the register to set
   * @param n How many bytes to set
   */
  void setRegisters8(uint8_t *ptr, uint8_t n);
  /*!
   * @brief Appears to be deprecated. Sets a register
   * @param ptr Pointer to the register to set
   * @param n How many bytes to set
   */
  void setRegisters16(uint16_t *ptr, uint8_t n);
  /*!
   * @brief Rotates the display
   * @param x How much to rotate the display
   */
  void setRotation(uint8_t x);
  // These methods are public in order for BMP examples to work:
  /*!
   * @brief Sets the LCD address window (and address counter, on 932X).
   * Relevant to rect/screen fills and H/V lines. Input coordinates are
   * assumed pre-sorted (e.g. x2 >= x1)
   * @param x1 X1 of the lcd window
   * @param y1 Y1 of the lcd window
   * @param x2 X2 of the lcd window
   * @param y2 Y2 of the lcd window
   */
  void setAddrWindow(int x1, int y1, int x2, int y2);
  /*!
   * @brief Issues 'raw' an array of 16-bit color values to the LCD; used
   * externally by BMP examples.  Assumes that setWindowAddr() has
   * previously been set to define the bounds.  Max 255 pixels at
   * a time (BMP examples read in small chunks due to limited RAM).
   * @param data Pointer to array with data
   * @param len How much of the data to issue
   * @param first If first is true, CGRAM write command is only issued on the
   * first call
   */
  void pushColors(uint16_t *data, uint8_t len, boolean first);

  /*!
   * @brief Pass 8-bit (each) R,G,B, get back 16-bit packed color
   * @param r Red value
   * @param g Green value
   * @param b Blue value
   * @return Returns the 16-bit packed color
   */
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b),
      /*!
       * @brief Reads the state of a single pixel.
       * Because this function is used infrequently, it configures the ports for
       * the read operation, reads the data, then restores the ports to the
       * write configuration.  Write operations happen a LOT, so it's
       * advantageous to leave the ports in that state as a default.
       * @param x X value of the pixel to read
       * @param y Y value of the pixel to read
       * @return Returns the state of the pixel
       */
      readPixel(int16_t x, int16_t y),
      /*!
       *@brief Reads the ID of the device
       * Because this function is used infrequently, it configures the ports for
       * the read operation, reads the data, then restores the ports to the
       *write configuration.  Write operations happen a LOT, so it's
       *advantageous to leave the ports in that state as a default.
       * @return Returns the ID of the device
       */
      readID(void);
  /*!
   * @brief Reads the specified register
   * @param r Address of the register to read
   * @return Returns the data from the register
   */
  uint32_t readReg(uint8_t r);

private:
  void init(),
  // These items may have previously been defined as macros
  // in pin_magic.h.  If not, function versions are declared:
#ifndef write8
      write8(uint8_t value),
#endif
#ifndef setWriteDir
      setWriteDir(void),
#endif
#ifndef setReadDir
      setReadDir(void),
#endif
#ifndef writeRegister8
      writeRegister8(uint8_t a, uint8_t d),
#endif
#ifndef writeRegister16
      writeRegister16(uint16_t a, uint16_t d),
#endif
      writeRegister24(uint8_t a, uint32_t d),
      writeRegister32(uint8_t a, uint32_t d),
#ifndef writeRegisterPair
      writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d),
#endif
      setLR(void), flood(uint16_t color, uint32_t len);
  uint8_t driver;

#ifndef read8
  uint8_t read8fn(void);
#define read8isFunctionalized
#endif

#ifndef USE_ADAFRUIT_SHIELD_PINOUT

#ifdef __AVR__
  volatile uint8_t *csPort, *cdPort, *wrPort, *rdPort;
  uint8_t csPinSet, cdPinSet, wrPinSet, rdPinSet, csPinUnset, cdPinUnset,
      wrPinUnset, rdPinUnset, _reset;
#endif
#if defined(__SAM3X8E__)
  Pio *csPort, *cdPort, *wrPort, *rdPort;
  uint32_t csPinSet, cdPinSet, wrPinSet, rdPinSet, csPinUnset, cdPinUnset,
      wrPinUnset, rdPinUnset, _reset;
#endif

#endif
};

// For compatibility with sketches written for older versions of library.
// Color function name was changed to 'color565' for parity with 2.2" LCD
// library.
#define Color565 color565

#endif
