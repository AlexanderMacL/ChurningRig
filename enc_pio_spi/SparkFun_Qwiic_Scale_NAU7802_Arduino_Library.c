/*

  MODS MADE SO THIS IS NOW IN C
  
  This is an Arduino library written for the NAU7802 24-bit wheatstone
  bridge and load cell amplifier.
  By Nathan Seidle @ SparkFun Electronics, March 3nd, 2019

  The NAU7802 is an I2C device that converts analog signals to a 24-bit
  digital signal. This makes it possible to create your own digital scale
  either by hacking an off-the-shelf bathroom scale or by creating your
  own scale using a load cell.

  The NAU7802 is a better version of the popular HX711 load cell amplifier.
  It uses a true I2C interface so that it can share the bus with other
  I2C devices while still taking very accurate 24-bit load cell measurements
  up to 320Hz.

  https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a board from SparkFun!
  https://www.sparkfun.com/products/15242
*/

#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

i2c_inst_t *_i2cPort;                //This stores the user's requested i2c port
const uint8_t _deviceAddress = 0x2A; //Default unshifted 7-bit address of the NAU7802

//y = mx+b
int32_t _zeroOffset;      //This is b
float _calibrationFactor; //This is m. User provides this number so that we can output y when requested

uint32_t millis() {
  return to_ms_since_boot(get_absolute_time());
}

void NAU7802setDevice(i2c_inst_t * i2cport) {
  _i2cPort = i2cport;
}

//Sets up the NAU7802 for basic function
//If initialize is true (or not specified), default init and calibration is performed
//If initialize is false, then it's up to the caller to initalize and calibrate
//Returns true upon completion
bool NAU7802begin(bool calibrateChannels)
{
  if (_i2cPort == NULL) return false;

  //Check if the device ack's over I2C
  if (NAU7802isConnected() == false)
  {
    //There are rare times when the sensor is occupied and doesn't ack. A 2nd try resolves this.
    if (NAU7802isConnected() == false)
      return (false);
  }

  bool result = true; //Accumulate a result as we do the setup
    result &= NAU7802reset(); //Reset all registers
    result &= NAU7802powerUp(); //Power on analog and digital sections of the scale
    result &= NAU7802setLDO(NAU7802_LDO_3V3); //Set LDO to 3.3V
    result &= NAU7802setGain(NAU7802_GAIN_128); //Set gain to 128
    result &= NAU7802setSampleRate(NAU7802_SPS_320); //Set samples per second to 320
    result &= NAU7802setRegister(NAU7802_ADC, 0x30); //Turn off CLK_CHP. From 9.1 power on sequencing.
    result &= NAU7802setBit(5, NAU7802_I2C_CONTROL); // Turn on internal strong pullups on i2c bus
    // result &= NAU7802setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
    if (calibrateChannels) {
      result &= NAU7802setChannel(NAU7802_CHANNEL_1);
      result &= NAU7802calibrateAFE(); //Cal analog front end channel 1
      result &= NAU7802setChannel(NAU7802_CHANNEL_2);
      result &= NAU7802calibrateAFE(); //Cal analog front end channel 2
      result &= NAU7802setChannel(NAU7802_CHANNEL_1);
    }

  return (result);
}

//Returns true if device is present
//Tests for device ack to I2C address
bool NAU7802isConnected()
{
  uint8_t a;
  if ((a=i2c_read_timeout_us(_i2cPort, _deviceAddress, &a, 1, false, I2C_TIMEOUT)) != 1) {
    return (false); //Sensor did not ACK
  }
  return (true);    //All good
}

//Returns true if Cycle Ready bit is set (conversion is complete)
bool NAU7802available()
{
  return (NAU7802getBit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL));
}

//Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
//Takes approximately 344ms to calibrate; wait up to 1000ms.
//It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
bool NAU7802calibrateAFE()
{
  NAU7802beginCalibrateAFE();
  return NAU7802waitForCalibrateAFE(1000);
}

//Begin asynchronous calibration of the analog front end.
// Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
void NAU7802beginCalibrateAFE()
{
  NAU7802setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2);
}

//Check calibration status.
NAU7802_Cal_Status NAU7802calAFEStatus()
{
  if (NAU7802getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2))
  {
    return NAU7802_CAL_IN_PROGRESS;
  }

  if (NAU7802getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2))
  {
    return NAU7802_CAL_FAILURE;
  }

  // Calibration passed
  return NAU7802_CAL_SUCCESS;
}

//Wait for asynchronous AFE calibration to complete with optional timeout.
//If timeout is not specified (or set to 0), then wait indefinitely.
//Returns true if calibration completes succsfully, otherwise returns false.
bool NAU7802waitForCalibrateAFE(uint32_t timeout_ms)
{
  uint32_t begin = millis();
  NAU7802_Cal_Status cal_ready;

  while ((cal_ready = NAU7802calAFEStatus()) == NAU7802_CAL_IN_PROGRESS)
  {
    if ((timeout_ms > 0) && ((millis() - begin) > timeout_ms))
    {
      break;
    }
    sleep_ms(1);
  }

  if (cal_ready == NAU7802_CAL_SUCCESS)
  {
    return (true);
  }
  return (false);
}

//Set the readings per second
//10, 20, 40, 80, and 320 samples per second is available
bool NAU7802setSampleRate(uint8_t rate)
{
  if (rate > 0b111)
    rate = 0b111; //Error check

  uint8_t value = NAU7802getRegister(NAU7802_CTRL2);
  value &= 0b10001111; //Clear CRS bits
  value |= rate << 4;  //Mask in new CRS bits

  return (NAU7802setRegister(NAU7802_CTRL2, value));
}

//Select between 1 and 2
bool NAU7802setChannel(uint8_t channelNumber)
{
  if (channelNumber == NAU7802_CHANNEL_1)
    return (NAU7802clearBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); //Channel 1 (default)
  else
    return (NAU7802setBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); //Channel 2
}

int32_t NAU7802getDataFromChannel(uint8_t channel) { // this is quite slow
  if (!NAU7802setChannel(channel)) return -1;
  uint8_t counter = 0;
  // NAU7802beginCalibrateAFE();
  while (counter++ < 200) { // read next data to prevent existing adc value skewing average
    if (NAU7802available()) {
      NAU7802getReading();
      counter = 0;
      break;
    }
    sleep_ms(1);
  }
  while (counter++ < 200) {
    if (NAU7802available()) {
      return NAU7802getAverage(10);
    }
    sleep_ms(1);
  }
  return -1;
}

//Power up digital and analog sections of scale
bool NAU7802powerUp()
{
  NAU7802setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
  NAU7802setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

  //Wait for Power Up bit to be set - takes approximately 200us
  uint8_t counter = 0;
  while (1)
  {
    if (NAU7802getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL) == true)
      break; //Good to go
    sleep_ms(1);
    if (counter++ > 100)
      return (false); //Error
  }
  return (true);
}

//Puts scale into low-power mode
bool NAU7802powerDown()
{
  NAU7802clearBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
  return (NAU7802clearBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL));
}

//Resets all registers to Power Of Defaults
bool NAU7802reset()
{
  bool ret = NAU7802setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL); //Set RR
  sleep_ms(1);
  ret &= NAU7802clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);
  return (ret); //Clear RR to leave reset state
}

//Set the onboard Low-Drop-Out voltage regulator to a given value
//2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
bool NAU7802setLDO(uint8_t ldoValue)
{
  NAU7802setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);
  if (ldoValue > 0b111)
    ldoValue = 0b111; //Error check

  //Set the value of the LDO
  uint8_t value = NAU7802getRegister(NAU7802_CTRL1);
  value &= 0b11000111;    //Clear LDO bits
  value |= ldoValue << 3; //Mask in new LDO bits
  return NAU7802setRegister(NAU7802_CTRL1, value);

  // return (NAU7802setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL)); //Enable the internal LDO
}

//Set the gain
//x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
bool NAU7802setGain(uint8_t gainValue)
{
  if (gainValue > 0b111)
    gainValue = 0b111; //Error check

  uint8_t value = NAU7802getRegister(NAU7802_CTRL1);
  value &= 0b11111000; //Clear gain bits
  value |= gainValue;  //Mask in new bits

  return (NAU7802setRegister(NAU7802_CTRL1, value));
}

//Get the revision code of this IC
uint8_t NAU7802getRevisionCode()
{
  uint8_t revisionCode = NAU7802getRegister(NAU7802_DEVICE_REV);
  return (revisionCode & 0x0F);
}

//Returns 24-bit reading
//Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
int32_t NAU7802getReading()
{
  uint8_t reg = NAU7802_ADCO_B2;
  if (i2c_write_timeout_us(_i2cPort, _deviceAddress, &reg, 1, true, I2C_TIMEOUT) != 1)
    return (-1); //Sensor did not ACK
  
  uint8_t bytesread[3];
  if (i2c_read_timeout_us(_i2cPort, _deviceAddress, bytesread, 3, false, I2C_TIMEOUT) != 3)
    return (-1); //3 bytes could not be read

  uint32_t valueRaw = (uint32_t)bytesread[0] << 16; //MSB
  valueRaw |= (uint32_t)bytesread[1] << 8;          //MidSB
  valueRaw |= (uint32_t)bytesread[2];               //LSB

  // the raw value coming from the ADC is a 24-bit number, so the sign bit now
  // resides on bit 23 (0 is LSB) of the uint32_t container. By shifting the
  // value to the left, I move the sign bit to the MSB of the uint32_t container.
  // By casting to a signed int32_t container I now have properly recovered
  // the sign of the original value
  int32_t valueShifted = (int32_t)(valueRaw << 8);

  // shift the number back right to recover its intended magnitude
  int32_t value = (valueShifted >> 8);

  return (value);
}

//Return the average of a given number of readings
//Gives up after 1000ms so don't call this function to average 8 samples setup at 1Hz output (requires 8s)
// NB HAVE SHORTENED TIMEOUT
int32_t NAU7802getAverage(uint8_t averageAmount)
{
  long total = 0;
  uint8_t samplesAquired = 0;

  unsigned long startTime = millis();
  while (1)
  {
    if (NAU7802available() == true)
    {
      total += NAU7802getReading();
      if (++samplesAquired == averageAmount)
        break; //All done
    }
    if (millis() - startTime > 50)
      return (PICO_ERROR_TIMEOUT); //Timeout - Bail with error
    sleep_ms(1);
  }
  total /= averageAmount;

  return (total);
}

//Call when scale is setup, level, at running temperature, with nothing on it
void NAU7802calculateZeroOffset(uint8_t averageAmount)
{
  NAU7802setZeroOffset(NAU7802getAverage(averageAmount));
}

//Sets the internal variable. Useful for users who are loading values from NVM.
void NAU7802setZeroOffset(int32_t newZeroOffset)
{
  _zeroOffset = newZeroOffset;
}

int32_t NAU7802getZeroOffset()
{
  return (_zeroOffset);
}

//Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
void NAU7802calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount)
{
  int32_t onScale = NAU7802getAverage(averageAmount);
  float newCalFactor = (onScale - _zeroOffset) / (float)weightOnScale;
  NAU7802setCalibrationFactor(newCalFactor);
}

//Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
//If you don't know your cal factor, call setZeroOffset(), then calculateCalibrationFactor() with a known weight
void NAU7802setCalibrationFactor(float newCalFactor)
{
  _calibrationFactor = newCalFactor;
}

float NAU7802getCalibrationFactor()
{
  return (_calibrationFactor);
}

//Returns the y of y = mx + b using the current weight on scale, the cal factor, and the offset.
float NAU7802getWeight(bool allowNegativeWeights, uint8_t samplesToTake)
{
  int32_t onScale = NAU7802getAverage(samplesToTake);

  //Prevent the current reading from being less than zero offset
  //This happens when the scale is zero'd, unloaded, and the load cell reports a value slightly less than zero value
  //causing the weight to be negative or jump to millions of pounds
  if (allowNegativeWeights == false)
  {
    if (onScale < _zeroOffset)
      onScale = _zeroOffset; //Force reading to zero
  }

  float weight = (onScale - _zeroOffset) / _calibrationFactor;
  return (weight);
}

//Set Int pin to be high when data is ready (default)
bool NAU7802setIntPolarityHigh()
{
  return (NAU7802clearBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //0 = CRDY pin is high active (ready when 1)
}

//Set Int pin to be low when data is ready
bool NAU7802setIntPolarityLow()
{
  return (NAU7802setBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //1 = CRDY pin is low active (ready when 0)
}

//Mask & set a given bit within a register
bool NAU7802setBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = NAU7802getRegister(registerAddress);
  value |= (1 << bitNumber); //Set this bit
  return (NAU7802setRegister(registerAddress, value));
}

//Mask & clear a given bit within a register
bool NAU7802clearBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = NAU7802getRegister(registerAddress);
  value &= ~(1 << bitNumber); //Set this bit
  return (NAU7802setRegister(registerAddress, value));
}

//Return a given bit within a register
bool NAU7802getBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = NAU7802getRegister(registerAddress);
  value &= (1 << bitNumber); //Clear all but this bit
  return (value);
}

//Get contents of a register
uint8_t NAU7802getRegister(uint8_t registerAddress)
{
  uint8_t val, err;
  if (i2c_write_timeout_us(_i2cPort, _deviceAddress, &registerAddress, 1, false, I2C_TIMEOUT) != 1) {
    // printf("No ack sending address 0x%x\n",registerAddress);
    return (-1); //Sensor did not ACK
  }
  if ((err = i2c_read_timeout_us(_i2cPort, _deviceAddress, &val, 1, false, I2C_TIMEOUT)) != 1) {
    // printf("Error %d reading from 0x%x\n", err, registerAddress);
    return (-1); //Error reading
  }
  return val;
}

//Send a given value to be written to given address
//Return true if successful
bool NAU7802setRegister(uint8_t registerAddress, uint8_t value)
{
  uint8_t dat[2] = {registerAddress, value};
  if (i2c_write_timeout_us(_i2cPort, _deviceAddress, dat, 2, false, I2C_TIMEOUT) != 2) {
    return (false); //Sensor did not ACK
  }
  return (true);
}