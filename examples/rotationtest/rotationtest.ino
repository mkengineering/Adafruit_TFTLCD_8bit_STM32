#include <Adafruit_TFTLCD_8bit_STM32.h> // Hardware-specific library

// check the pin connections in the header file.

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


Adafruit_TFTLCD_8bit_STM32 tft;


void setup(void) {
  Serial.begin(9600);
  delay(6000);
  Serial.println(F("TFT LCD test"));

  tft.reset();

  uint16_t identifier = tft.readID();

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

  tft.fillScreen(BLACK);

  Serial.println(F("This is a test of the rotation capabilities of the TFT library!"));
  Serial.println(F("Press <SEND> (or type a character) to advance"));
}

void loop(void) {
  rotatePixel();
  rotateLine();
  rotateFastline();
  rotateDrawrect();
  rotateFillrect();
  rotateDrawcircle();
  rotateFillcircle();
  rotateText();
}

void rotateText() {
  for (uint8_t i=0; i<4; i++) {
    tft.fillScreen(BLACK);
    Serial.println(tft.getRotation(), DEC);

    tft.setCursor(0, 30);
    tft.setTextColor(RED);
    tft.setTextSize(1);
    tft.println("Hello World!");
    tft.setTextColor(YELLOW);
    tft.setTextSize(2);
    tft.println("Hello World!");
    tft.setTextColor(GREEN);
    tft.setTextSize(3);
    tft.println("Hello World!");
    tft.setTextColor(BLUE);
    tft.setTextSize(4);
    tft.print(1234.567);

    while (!Serial.available());
    Serial.read();  Serial.read();  Serial.read();

    tft.setRotation(tft.getRotation()+1);
  }
}

void rotateFillcircle(void) {
  for (uint8_t i=0; i<4; i++) {
    tft.fillScreen(BLACK);
    Serial.println(tft.getRotation(), DEC);

    tft.fillCircle(10, 30, 10, YELLOW);

    while (!Serial.available());
    Serial.read();  Serial.read();  Serial.read();

    tft.setRotation(tft.getRotation()+1);
  }
}

void rotateDrawcircle(void) {
  for (uint8_t i=0; i<4; i++) {
    tft.fillScreen(BLACK);
    Serial.println(tft.getRotation(), DEC);

    tft.drawCircle(10, 30, 10, YELLOW);

    while (!Serial.available());
    Serial.read();  Serial.read();  Serial.read();

    tft.setRotation(tft.getRotation()+1);
  }
}

void rotateFillrect(void) {
  for (uint8_t i=0; i<4; i++) {
    tft.fillScreen(BLACK);
    Serial.println(tft.getRotation(), DEC);

    tft.fillRect(10, 20, 10, 20, GREEN);

    while (!Serial.available());
    Serial.read();  Serial.read();  Serial.read();

    tft.setRotation(tft.getRotation()+1);
  }
}

void rotateDrawrect(void) {
  for (uint8_t i=0; i<4; i++) {
    tft.fillScreen(BLACK);
    Serial.println(tft.getRotation(), DEC);

    tft.drawRect(10, 20, 10, 20, GREEN);

    while (!Serial.available());
    Serial.read();  Serial.read();  Serial.read();

    tft.setRotation(tft.getRotation()+1);
  }
}

void rotateFastline(void) {
  for (uint8_t i=0; i<4; i++) {
    tft.fillScreen(BLACK);
    Serial.println(tft.getRotation(), DEC);

    tft.drawFastHLine(0, 20, tft.width(), RED);
    tft.drawFastVLine(20, 0, tft.height(), BLUE);

    while (!Serial.available());
    Serial.read();  Serial.read();  Serial.read();

    tft.setRotation(tft.getRotation()+1);
  }
}

void rotateLine(void) {
  for (uint8_t i=0; i<4; i++) {
    tft.fillScreen(BLACK);
    Serial.println(tft.getRotation(), DEC);

    tft.drawLine(tft.width()/2, tft.height()/2, 0, 0, RED);
    while (!Serial.available());
    Serial.read();  Serial.read();  Serial.read();

    tft.setRotation(tft.getRotation()+1);
  }
}

void rotatePixel(void) {
  for (uint8_t i=0; i<4; i++) {
    tft.fillScreen(BLACK);
    Serial.println(tft.getRotation(), DEC);

    tft.drawPixel(10,20, RED);
    while (!Serial.available());
    Serial.read();  Serial.read();  Serial.read();

    tft.setRotation(tft.getRotation()+1);
  }
}

