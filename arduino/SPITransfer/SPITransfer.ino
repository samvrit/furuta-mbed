#include <SPI.h>

static const int spiClk = 1000000; // 1 MHz

//uninitalised pointers to SPI objects
SPIClass * hspi = NULL;
double angleInDegree;

void setup() {
  hspi = new SPIClass(HSPI);
  hspi->begin(); 
  pinMode(15, OUTPUT); //HSPI SS
  Serial.begin(115200);
}

// the loop function runs over and over again until power down or reset
void loop() {
  hspiCommand();
  delayMicroseconds(100);
}

void hspiCommand() {
  byte stuff = 0x0000;
  uint16_t angle;
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(15, LOW);
  angle = hspi->transfer16(stuff);
  digitalWrite(15, HIGH);
  hspi->endTransaction();
  angleInDegree = (angle*360.0)/65536.0;
  Serial.println(angleInDegree, 3);
}
