float getAngle() {
  int32_t angle;
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(15, LOW);
  angle = hspi->transfer16(0x0000);
  digitalWrite(15, HIGH);
  hspi->endTransaction();
  angle = angle > 32768 ? (angle - 32768) : angle;
  return (float)(angle * ANGLE_SCALING_FACTOR);
}
