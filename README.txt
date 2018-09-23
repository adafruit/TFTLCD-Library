I added esp32 support for the 8-bit interface on the Adafruit TFT breakout board, NOT the Adafruit TFT shield.

The pins used are hardcoded due to the use of bitmasks. I hope someone will have use of it.
Depending on your esp32 breakout board the locations of the pins used will differ.

Pins:

D0 - Gpio 2
D1 - Gpio 3
D2 - Gpio 4
D3 - Gpio 5
D4 - Gpio 15
D5 - Gpio 16
D6 - Gpio 17
D7 - Gpio 18

The control pins (CS, CD, WR, RD, RST) are free to choose.