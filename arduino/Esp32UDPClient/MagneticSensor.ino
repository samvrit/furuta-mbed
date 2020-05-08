float getAngle() {
  uint16_t angle;
  int16_t angle_signed;
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(15, LOW);
  angle = hspi->transfer16(0x0000);
  digitalWrite(15, HIGH);
  hspi->endTransaction();
  angle_signed = angle > 32768 ? (int16_t)(angle - 32768) : (int16_t)angle;
  return (float)(angle_signed * ANGLE_SCALING_FACTOR);
}
