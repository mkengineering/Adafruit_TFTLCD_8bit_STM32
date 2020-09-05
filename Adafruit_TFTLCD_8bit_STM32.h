// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license
// ported to STM32duino ST Core by MKE

#ifndef _ADAFRUIT_TFTLCD_8BIT_STM32_H_
#define _ADAFRUIT_TFTLCD_8BIT_STM32_H_

#include "Arduino.h"
#include <Adafruit_GFX.h>

/*****************************************************************************/
// Data and Control Port / Pin Definitions
/*****************************************************************************/

// Example: PB5-> PORT = GPIOB, MASK = GPIO_PIN_5

// Data and Control Ports

#define TFT_DATA_PORT	GPIOA
#define TFT_CNTRL_PORT	GPIOB

// Masks for Control Pins

#define TFT_RD_MASK		GPIO_PIN_9
#define TFT_WR_MASK		GPIO_PIN_8
#define TFT_RS_MASK		GPIO_PIN_7
#define TFT_CS_MASK		GPIO_PIN_6
#define TFT_RST_MASK	GPIO_PIN_5

/*****************************************************************************/
// TFT Controller Chip
/*****************************************************************************/

#define ID_932X    		0
#define ID_7575    		1
#define ID_9341    		2
#define ID_HX8357D 		3
#define ID_9338    		4
#define ID_UNKNOWN 		0xFF

/*****************************************************************************/
// TFT Size
/*****************************************************************************/

#define TFTWIDTH   240
#define TFTHEIGHT  320

#define TFTLCD_DELAY 0xFF

#define Color565 color565

/*****************************************************************************/
// Color Definitions
/*****************************************************************************/

#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

/*****************************************************************************/
// Macro Definitions
/*****************************************************************************/

#define RD_ACTIVE               { TFT_CNTRL_PORT->BRR  = TFT_RD_MASK; }
#define RD_IDLE                 { TFT_CNTRL_PORT->BSRR = TFT_RD_MASK; }

#define WR_ACTIVE				{ TFT_CNTRL_PORT->BRR  = TFT_WR_MASK; }
#define WR_IDLE					{ TFT_CNTRL_PORT->BSRR = TFT_WR_MASK; }
#define WR_STROBE 				{ WR_ACTIVE; WR_IDLE; }

#define CD_COMMAND				{ TFT_CNTRL_PORT->BRR  = TFT_RS_MASK; }
#define CD_DATA					{ TFT_CNTRL_PORT->BSRR = TFT_RS_MASK; }

#define CS_ACTIVE				{ TFT_CNTRL_PORT->BRR  = TFT_CS_MASK; }
#define CS_IDLE					{ TFT_CNTRL_PORT->BSRR = TFT_CS_MASK; }
#define CS_ACTIVE_CD_COMMAND	{ TFT_CNTRL_PORT->BRR  = (TFT_CS_MASK|TFT_RS_MASK); }

#define RST_ACTIVE				{ TFT_CNTRL_PORT->BRR = TFT_RST_MASK; }
#define RST_IDLE				{ TFT_CNTRL_PORT->BSRR = TFT_RST_MASK; }
#define RST_TOGGLE				{ RST_IDLE; RST_ACTIVE; RST_IDLE; }

extern uint8_t read8_(void);

#define read8(x) ( x = read8_() )

// set control pins (PB5-PB9) to output mode and enable apb2 clock for gpiob
#define setCntrlDir() { RCC->APB2ENR |= 0b1000; TFT_CNTRL_PORT->CRH = 0x00000033; TFT_CNTRL_PORT->CRL = 0x33300000; }

// set the data pins (PA0-PA7) to input mode and enable apb2 clock for gpioa
#define setReadDir() { RCC->APB2ENR |= 0b0100; TFT_DATA_PORT->CRL = 0x88888888; }

// set the data pins (PA0-PA7) to output mode and enable apb2 clock for gpioa
#define setWriteDir() { RCC->APB2ENR |= 0b0100; TFT_DATA_PORT->CRL = 0x33333333; }

// set pins to output the 8 bit value
#define write8(c) { TFT_DATA_PORT->BSRR = (uint32_t)(0x00FF0000 + ((c)&0xFF)); WR_STROBE; }

/*****************************************************************************/

#define swap(a, b) { int16_t t = a; a = b; b = t; }

/*****************************************************************************/
class Adafruit_TFTLCD_8bit_STM32 : public Adafruit_GFX {

 public:

  Adafruit_TFTLCD_8bit_STM32(void);

  void     begin(uint16_t id = 0x9341);
  void     drawPixel(int16_t x, int16_t y, uint16_t color);
  void     drawFastHLine(int16_t x0, int16_t y0, int16_t w, uint16_t color);
  void     drawFastVLine(int16_t x0, int16_t y0, int16_t h, uint16_t color);
  void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
  void     fillScreen(uint16_t color);
  void     reset(void);
  void     setRegisters8(uint8_t *ptr, uint8_t n);
  void     setRegisters16(uint16_t *ptr, uint8_t n);
  void     setRotation(uint8_t x);
       // These methods are public in order for BMP examples to work:
  void     setAddrWindow(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
  void     invertDisplay(boolean i),
			pushColors(uint16_t *data, int16_t len, boolean first),
           drawBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t * bitmap);
  uint16_t readPixel(int16_t x, int16_t y), readID(void);
/*****************************************************************************/
// Pass 8-bit (each) R,G,B, get back 16-bit packed color
// color coding on bits:
// high byte sill be sent first
// bit nr: 		15	14	13	12	11	 10	09	08		07	06	05	 04	03	02	01	00
// color/bit:	R5	R4	R3	R2	R1 | G5	G4	G3		G2	G1	G0 | B5	B4	B3	B2	B1
// 								R0=R5											B0=B5
/*****************************************************************************/
  uint16_t inline color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }

 private:
  void     init(), flood(uint16_t color, uint32_t len);
  uint8_t  driver;
};

extern uint16_t readReg(uint8_t r);
extern uint32_t readReg32(uint8_t r);
extern void writeCommand(uint16_t c);
extern void writeRegister8(uint16_t a, uint8_t d);
extern void writeRegister16(uint16_t a, uint16_t d);
extern void writeRegister24(uint16_t a, uint32_t d);
extern void writeRegister32(uint16_t a, uint32_t d);
extern void writeRegisterPair(uint16_t aH, uint16_t aL, uint16_t d);

extern Adafruit_TFTLCD_8bit_STM32 tft;

#endif
