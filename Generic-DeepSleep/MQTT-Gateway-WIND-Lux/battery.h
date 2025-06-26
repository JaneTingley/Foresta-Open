// Given a battery reading of 0-4095, returns the battery voltage
float readingToBatteryVoltage(int batteryReading, float calibration = 2.0f) {
  // In theory, calibration should always be 2.0 as the schematic shows a
  // 2 x 1MOhm voltage divider.
  // Perhaps the resistors are a bit inaccurate, so we allow for calibration per board.
  // TODO: How to calibrate?
  return calibration * 3.3f * batteryReading / 4095;
}

// Given a battery voltage, return an estimate of remaining capacity in percent.
int batteryVoltageToPercent(float voltage) {
  static float voltageTable[21][2] = {
    {4.2,  100},
    {4.15, 95},
    {4.11, 90},
    {4.08, 85},
    {4.02, 80},
    {3.98, 75},
    {3.95, 70},
    {3.91, 65},
    {3.87, 60},
    {3.85, 55},
    {3.84, 50},
    {3.82, 45},
    {3.80, 40},
    {3.79, 35},
    {3.77, 30},
    {3.75, 25},
    {3.73, 20},
    {3.71, 15},
    {3.69, 10},
    {3.61, 5},
    {3.27, 0},
  };
  for (int i=1;i<21;i++) {
    if (voltage >= voltageTable[i][0]) {
      return voltageTable[i][1];
    }
  }
  return 0;    
}
