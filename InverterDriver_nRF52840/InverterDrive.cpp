#include <Arduino.h>
#include "InverterDrive.h"

void InverterDrive::begin() {
  serial.begin(57600, SERIAL_8N1);
}

int InverterDrive::start() {
  return writeInverterWord(0x01, 0x2000, 0x0001, true);
}

int InverterDrive::start(int dir) {
  return writeInverterWord(0x01, 0x2000, dir?0x1:0x2, true);
}

int InverterDrive::stop() {
  return writeInverterWord(0x01, 0x2000, 0x0005, true);
}

int InverterDrive::reset() {
  return writeInverterWord(0x01, 0x2000, 0x0007, true);
}

int InverterDrive::coast() {
  return writeInverterWord(0x01, 0x2000, 0x0006, true);
}

int InverterDrive::speed(float rpm) {
  return writeInverterWord(0x01, 0x2001, static_cast<uint16_t>(rpm*5/3), true); // 1/60 * 100
}

int InverterDrive::direction(int dir) {
  if (dir<0) {
    int cdir = readRegister(0x000D);
    dir = cdir?0:1;
  }
  return writeInverterWord(0x01,0x000D,dir,true);
}

int InverterDrive::sendInverterWord(uint8_t addr, uint8_t cmdbyte, uint16_t cmd, uint16_t wrd, bool validate) {
  uint8_t buf[8] = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
  buf[0] = addr;
  buf[1] = cmdbyte; // comms command
  buf[2] = (cmd & 0xFF00) >> 8; // data address high byte
  buf[3] = cmd & 0x00FF; // data address low byte
  buf[4] = (wrd & 0xFF00) >> 8; // data content high byte
  buf[5] = wrd & 0x00FF; // data content low byte
  unsigned int CRC = crc_cal_value(buf, 6);
  buf[6] = CRC & 0x00FF; // CRC low byte
  buf[7] = (CRC & 0xFF00) >> 8; // CRC high byte
//  Serial.println("Writing:");
//  for (int i=0;i<8;i++) {
//    Serial.println(buf[i],HEX);
//  }
  digitalWrite(R_T_PIN, HIGH);
  serial.write(buf,8);
  serial.flush();
  digitalWrite(R_T_PIN, LOW);
  int r = E_SUCCESS;
  if (validate) {
    delay(SER_TIMEOUT);
    r = validateResponse(addr, cmdbyte, cmd, wrd);
  }
  return r;
}

int InverterDrive::writeInverterWord(uint8_t addr, uint16_t cmd, uint16_t wrd, bool validate) {
  return sendInverterWord(addr, 0x06, cmd, wrd, validate);
}

unsigned int InverterDrive::crc_cal_value(unsigned char *data_value, unsigned char data_length) {
  int i;
  unsigned int crc_value=0xffff;
  while(data_length--) {
    crc_value^=*data_value++;
    for(i=0;i<8;i++) {
      if(crc_value&0x0001)crc_value=(crc_value>>1)^0xa001;
      else crc_value=crc_value>>1;
    }
  }
  return(crc_value);
}

int InverterDrive::getResponse(uint8_t * rbuf, uint8_t bytesexpected) {
  int i;
  //Serial.println("Reading:");
  for (i=0;i<bytesexpected;i++) {
    if (!(serial.available()>0)) break;
    rbuf[i] = serial.read();
    //Serial.println(rbuf[i],HEX);
  }
  return i;
}

int InverterDrive::validateResponse(uint8_t addr, uint8_t cmdbyte, uint16_t cmd, uint16_t wrd) {
  uint8_t rbuf[8] = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
//  Serial.println("Reading:");
  for (int i=0;i<8;i++) {
    if (!(serial.available()>0)) break;
    rbuf[i] = serial.read();
//    Serial.println(rbuf[i],HEX);
  }
  int r = E_SUCCESS;
  if (rbuf[0] != addr) r |= E_ADDR;
  if (rbuf[1] != cmdbyte) r |= E_CMD;
  if (rbuf[2] != ((cmd>>8)&0xFF)) r |= E_DATA_C_H;
  if (rbuf[3] != (cmd&0xFF)) r |= E_DATA_C_L;
  if (rbuf[4] != ((wrd>>8)&0xFF)) r |= E_DATA_W_H;
  if (rbuf[5] != (wrd&0xFF)) r |= E_DATA_W_L;
  unsigned int CRC = crc_cal_value(rbuf, 6);
  if (rbuf[6] != (CRC&0xFF)) r |= E_CRC;
  if (rbuf[7] != ((CRC>>8)&0xFF)) r |= E_CRC;
  return r;
}

int InverterDrive::readRegisters(uint16_t * regbuf, uint16_t startreg, uint16_t numregs) {
  sendInverterWord(0x01, 0x03, startreg, numregs, false);
  delay(SER_TIMEOUT);
  uint8_t rbuf[37]; // max required capacity is 2*16+5
  if (getResponse(rbuf, 2*numregs+5) == 2*numregs+5) {
//    Serial.println("CRC");
//    Serial.println(crc_cal_value(rbuf, 2*numregs+3), HEX);
//    Serial.println(rbuf[2*numregs+3]+(0xFF00&(rbuf[2*numregs+4]<<8)), HEX);
    if (crc_cal_value(rbuf, 2*numregs+3) == (rbuf[2*numregs+3]+(0xFF00&(rbuf[2*numregs+4]<<8)))) {
      for (int i=0; i<2*numregs; i++) {
        regbuf[i/2+i%2] = static_cast<uint16_t>(rbuf[i+4])+(static_cast<uint16_t>(rbuf[i+3])<<8);
      }
      return E_SUCCESS;
    } else {
      return E_CRC;
    }
  }
}

uint16_t InverterDrive::readRegister(uint16_t reg) {
  uint16_t r = 0xFFFF;
  readRegisters(&r, reg, 1);
  return r;
}
