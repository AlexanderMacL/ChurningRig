#ifndef _INVERTER_DRIVE_H
#define INVERTER_DRIVE_H

//#define SOFTWARE_SERIAL

#include <Arduino.h>

#ifdef SOFTWARE_SERIAL
#include <SoftwareSerial.h>
#endif

#define RX_PIN  0
#define TX_PIN  1
#define R_T_PIN 5

#define SER_START_CHAR  '<'
#define SER_END_CHAR    '>'
#define SER_TIMEOUT     50

#define SWITCH_ON       1
#define SWITCH_OFF      0
#define SWITCH_INVALID  -1

class InverterDrive {
#ifdef SOFTWARE_SERIAL
  SoftwareSerial serial;
  public:
    InverterDrive() : serial(RX_PIN, TX_PIN) {}
#else
  HardwareSerial &serial = Serial1;
  public:
    InverterDrive() {}
#endif
    void begin();

  public:
    enum responseError : int {
      E_SUCCESS = 0,
      E_ADDR = 0x1,
      E_CMD  = 0x2,
      E_DATA_C_H = 0x4,
      E_DATA_C_L = 0x8,
      E_DATA_W_H = 0x10,
      E_DATA_W_L = 0x20,
      E_CRC  = 0x40
    };

  private:
    int sendInverterWord(uint8_t addr, uint8_t cmdbyte, uint16_t cmd, uint16_t wrd, bool validate);
    int getResponse(uint8_t * rbuf, uint8_t bytesexpected);
    int validateResponse(uint8_t addr, uint8_t cmdbyte, uint16_t cmd, uint16_t wrd);
    unsigned int crc_cal_value(unsigned char *data_value, unsigned char data_length);

  public:
    int start();
    int start(int dir);
    int stop();
    int reset();
    int coast();
    int speed(float rpm);
    int direction(int dir);
    int writeInverterWord(uint8_t addr, uint16_t cmd, uint16_t wrd, bool validate);
    int readRegisters(uint16_t * regbuf, uint16_t startreg, uint16_t numregs);
    uint16_t readRegister(uint16_t reg);
};

#endif
