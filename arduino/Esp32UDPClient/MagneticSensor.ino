unsigned int getAngle() {
  uint16_t angle;
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(15, LOW);
  angle = hspi->transfer16(0x0000);
  digitalWrite(15, HIGH);
  hspi->endTransaction();
  return (unsigned int)((angle*628)/65536.0);
}
