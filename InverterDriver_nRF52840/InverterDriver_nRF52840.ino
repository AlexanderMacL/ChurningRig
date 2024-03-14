#include "InverterDrive.h"
#include "Seeed_MCP9600.h"
#include "picoSPI.h"
#include <SPI.h>
#include <Adafruit_DotStar.h>
//#include <nrfx_spis.h>

struct thermocouple {
  int pin;
  float temperature;
  int timeselected;
};

struct tempCtrlParams {
  float Kff; // 10ths / degree-20C
  float Kp; // 10ths / degree error
  float Ki; // 10ths / degree error / second
  float KiLIM; // 10ths
  int counterinterval; // ms
  int counter;
  int counts;
  float integral;
  int lastcountertime;
  int lastctrltime;
  float setpoint;
  int HEATER_PIN;
  thermocouple * THERMOCOUPLE;
  bool enabled;
};

int a;
int timestamp, prevtimestamp;
float datafreq = 5; // Hz
int readSerial(char* b);
int writeSerial(const char* d);
int parseSwitch(char * d);
void LEDcolour(int R, int G, int B);
byte writeSPI(byte val);
void readSpeed(uint32_t * ts, float * sd, uint8_t * EE);
void send8bitDataSPI(uint8_t len, char * d);
int requestDataSPI(uint8_t type, uint8_t len, uint8_t * datbuf);
int parse_spi_buffer(uint8_t * spiRxBuf, uint8_t * spiMsgBuf);
int inc_counter_overflow(int * counter, int ovflen);
char * hptr;
char * dptr;
int F = 10; // samplerate in 0.1Hz
float sampint = 1000.0; // interval in ms = 1000/F
float prevmillis = 0;
int tempctrlinterval = 2500; // ms
const int tempctrlmaxcounts = 10;
int thermocoupleconversiontime = 280;
int currentthermocouple = 0;

char buf[255]; // 256 - 2 + 1

bool motordata = false;
bool encoderdata = false;
bool guidata = false;
bool speedfollow = false;
bool tempdata = false;
bool loaddata = false;

InverterDrive inverter;

MCP9600 sensor;

#define NUMPIXELS 1 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    8
#define CLOCKPIN   6
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);

#define RELAY_5 A1
#define RELAY_6 A2
#define RELAY_7 A3
#define RELAY_8 A4

#define S3_PIN 12
#define S2_PIN 11
#define S1_PIN 10
#define S0_PIN 9
#define EN_PIN 7
#define CS_PIN 2

#define CLUTCH_PIN RELAY_6
#define HEATER_1_PIN RELAY_8
#define HEATER_2_PIN RELAY_5

#define NUM_THERMOCOUPLES 7

struct thermocouple thermocouples[NUM_THERMOCOUPLES];

struct tempCtrlParams statorTempCtrl = {
  .Kff = 0.05,
  .Kp = 3.0,
  .Ki = 0.5,
  .KiLIM = 5.0,
  .counterinterval = 500,
  .counter = 0,
  .counts = 0,
  .integral = 0,
  .lastcountertime = 0,
  .lastctrltime = 0,
  .setpoint = 0,
  .HEATER_PIN = HEATER_1_PIN,
  .THERMOCOUPLE = &thermocouples[6],
  .enabled = false
};

struct tempCtrlParams gearboxTempCtrl = {
  .Kff = 0.05,
  .Kp = 2.0,
  .Ki = 0.5,
  .KiLIM = 5.0,
  .counterinterval = 500,
  .counter = 0,
  .counts = 0,
  .integral = 0,
  .lastcountertime = 0,
  .lastctrltime = 0,
  .setpoint = 0,
  .HEATER_PIN = HEATER_2_PIN,
  .THERMOCOUPLE = &thermocouples[1],
  .enabled = false
};

struct tempCtrlParams* tempCtrlTarget = &statorTempCtrl;

/*
  volatile boolean SPIreceived;
  volatile byte SPIbytesreceived[10],SPIbytesend,SPIpos=0;
  volatile float SPIfloatreceived;
  volatile int SPIintreceived;
  // to make this work you need to find the spis example in the nordic nrf5 SDK and work out which register the SPIS peripheral reads to for the RX
  nrfx_spis_evt_t SPI_event;

  nrfx_spis_event_handler_t SPI_ISR (nrfx_spis_evt_t const *p_event, void *p_context)                        //Interrupt routine function
  {
  SPIbytesreceived[SPIpos++] = SPDR;                 //Value received from master if store in variable SPIbytereceived
  if ((SPIbytesreceived[SPIpos-1] == '>') && (SPIpos > 9)) {
    SPIpos = 0;
    if (SPIbytesreceived[0] == '<') {
      SPIfloatreceived = *(float *)(SPIbytesreceived+1);
      SPIintreceived = *(int *)(SPIbytesreceived+5);
      SPIreceived = true;                     //Sets received as True
    }
  }
  }
*/

void setup() {

  strip.begin();
  strip.setPixelColor(0, 0, 255, 255);
  strip.setBrightness(32);
  strip.show();


  pinMode(R_T_PIN, OUTPUT);
  pinMode(RELAY_5, OUTPUT);
  pinMode(CLUTCH_PIN, OUTPUT);
  pinMode(RELAY_7, OUTPUT);
  pinMode(RELAY_8, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S0_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(RELAY_5, HIGH);
  digitalWrite(CLUTCH_PIN, HIGH);
  digitalWrite(RELAY_7, HIGH);
  digitalWrite(RELAY_8, HIGH);
  digitalWrite(R_T_PIN, LOW);
  digitalWrite(S3_PIN, LOW);
  digitalWrite(S2_PIN, LOW);
  digitalWrite(S1_PIN, LOW);
  digitalWrite(S0_PIN, LOW);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(CS_PIN, HIGH);
  // (MISO,OUTPUT);                   //Sets MISO as OUTPUT (Have to Send data to Master IN)

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Native USB only
  }

  SPI.begin();

  Serial.println("InverterDrive");
  inverter.begin();
  // SPIreceived = false;

  initialise_thermocouples();
  if (sensor.init(THER_TYPE_K)) {
    Serial.println("Sensor init failed!!");
  }
  sensor_basic_config();
  setSamplingInterval(F);
}

void loop() {
  // put your main code here, to run repeatedly:
  timestamp = millis();
  measure_temperatures();
  if (float(timestamp - prevtimestamp + 1) > 1000.0 / datafreq) {
    char pbuf[500];
    float spd;
    uint32_t tst;
    uint8_t ee;
    int32_t ldc[4];
    if (loaddata || guidata) readSpeed(&tst, &spd, &ee, ldc);
    else readSpeed(&tst, &spd, &ee);
    if (ee != 0) {
      tst = 0;
      spd = 0;
      sprintf(pbuf, "_ENCODER_ERROR %d", ee);
      writeSerial(pbuf);
    }
    if (speedfollow && ee == 0) {
      inverter.speed(abs(spd) * 93 / 869); // GEAR RATIO APPLIED
      Serial.print("<_SPEED_CHANGE ");
      Serial.print(spd);
      Serial.println(">");
    }
    if (tempdata) {
      for (int j=0;j<NUM_THERMOCOUPLES;j++) {
        Serial.print(thermocouples[j].temperature);
        Serial.print(" ");
      }
      Serial.println();
    }
    if (loaddata) {
//      for (int j=0;j<4;j++) {
//        Serial.print(ldc[j]);
//        Serial.print(" ");
//      }
//      Serial.println();
      int32_t torquereading = ldc[0]+ldc[1]-ldc[2]-ldc[3];
      Serial.println(torquereading);
    }
    dotempctrl(&statorTempCtrl);
    dotempctrl(&gearboxTempCtrl);
    if (guidata) {
      uint16_t rbuf[7] = {0, 0, 0, 0, 0, 0, 0};
      int EE = inverter.readRegisters(rbuf, 0x1109, 2);
      int torque = rbuf[0];
      int opspeed = rbuf[1];
      EE &= inverter.readRegisters(rbuf, 0x3000, 7);
      if (EE != 0) {
        sprintf(pbuf, "_INVERTER_ERROR %d", EE);
        writeSerial(pbuf);
      }
      int voltage = rbuf[3];
      int current = rbuf[4];
      int power = rbuf[6];
      int frequency = rbuf[0];
      
      sprintf(pbuf, "__ %d,%f,%f,%d,%d,%d,%d,%d,%d", tst, spd, thermocouples[0].temperature, voltage, current, power, torque, frequency, opspeed);
      for (int j=0;j<NUM_THERMOCOUPLES;j++) {
        sprintf(pbuf,"%s,%f",pbuf,thermocouples[j].temperature);
      }
      for (int j=0;j<4;j++) {
        sprintf(pbuf,"%s,%ld",pbuf,ldc[j]);
      }
      writeSerial(pbuf);
    }
    prevtimestamp = timestamp;
    if (motordata) {
      char pbuf[100];
      uint16_t rbuf[8];
      if (inverter.readRegisters(rbuf, 0x3000, 7) == InverterDrive::E_SUCCESS) {
        sprintf(pbuf, " Bus V %04dV, Out V %04dV, Out I %06.1fA, Op Speed %05dRPM, Out P %06.1f%%, Out T %05d, Op f %05d, Set f %05d",
                rbuf[2], rbuf[3], static_cast<float>(rbuf[4]) / 10.0, rbuf[5], static_cast<float>(rbuf[6]) / 10.0, rbuf[7], rbuf[0], rbuf[1]);
        //        sprintf(pbuf,", %d  %d  %d  %d  %d  %d  %d", rbuf[0], rbuf[1], rbuf[2], rbuf[3], rbuf[4], rbuf[5], rbuf[6]);
        Serial.println(pbuf);
      } else writeSerial("Drive Comms Error");
    }
  }

  //  if (encoderdata && SPIreceived) {
  //    Serial.println("%06d, %f+06.2",SPIintreceived,SPIfloatreceived);
  //    SPIreceived = false;
  //  }

  a = readSerial(buf);
  if (a > 0) { // handle Serial RX
    hptr = strtok(buf, " "); // truncate message to extract header
    dptr = strtok(NULL, " "); // extract data if present

    if (strcmp("_ID", hptr) == 0) {
      Serial.println("<nRF52840>"); // <_ID>
    } else if (strcmp("_START", hptr) == 0) {
      Serial.print("<_START "); Serial.print(inverter.start()); Serial.println(">"); // <_START>
    } else if (strcmp("_STOP", hptr) == 0) {
      Serial.print("<_STOP "); Serial.print(inverter.stop()); Serial.println(">"); // <_STOP>
    } else if (strcmp("_COAST", hptr) == 0) {
      Serial.print("<_COAST "); Serial.print(inverter.coast()); Serial.println(">"); // <_COAST>
    } else if (strcmp("_SPEED", hptr) == 0) {
      Serial.print("<_SPEED "); Serial.print(inverter.speed(strtof(dptr, NULL))); Serial.println(">"); // <_SPEED ?.?>
    } else if (strcmp("_SET_RATE", hptr) == 0) {
      Serial.print("<_RATE "); Serial.print(datafreq = strtof(dptr, NULL)); Serial.println(">"); // <_SET_RATE ?.?>
    } else if (strcmp("_CLUTCH", hptr) == 0) {
      digitalWrite(CLUTCH_PIN, (parseSwitch(dptr) == SWITCH_ON) ? LOW : HIGH); Serial.print("<"); Serial.print(hptr); Serial.print(" "); Serial.print(dptr); Serial.println(">"); // <_CLUTCH 0/1>
    } else if (strcmp("_DATA", hptr) == 0) {
      sensor_basic_config();
      guidata = (parseSwitch(dptr) == SWITCH_ON); Serial.print("<"); Serial.print(hptr); Serial.print(" "); Serial.print(dptr); Serial.println(">"); // <_DATA 0/1>
    } else if (strcmp("_SPEED_FOLLOW", hptr) == 0) {
      speedfollow = (parseSwitch(dptr) == SWITCH_ON); Serial.print("<"); Serial.print(hptr); Serial.print(" "); Serial.print(dptr); Serial.println(">"); // <_SPEED_FOLLOW 0/1>
    } else if (strcmp("_TEMP_CTRL_TARGET", hptr) == 0) {
      if (strcmp("STATOR", dptr) == 0) {
        tempCtrlTarget = &statorTempCtrl;
      } else if (strcmp("GEARBOX", dptr) == 0) {
        tempCtrlTarget = &gearboxTempCtrl;
      }
      if (strcmp("STATOR", dptr) == 0 || strcmp("GEARBOX", dptr) == 0) {
        Serial.print("<"); Serial.print(hptr); Serial.print(" "); Serial.print(dptr); Serial.println(">"); // <_TEMP_CTRL_TARGET STATOR/GEARBOX>
      }
    } else if (strcmp("_TEMP_CTRL", hptr) == 0) {
      tempCtrlTarget->enabled = (parseSwitch(dptr) == SWITCH_ON); Serial.print("<"); Serial.print(hptr); Serial.print(" "); Serial.print(dptr); Serial.println(">"); // <_TEMP_CTRL 0/1>
    } else if (strcmp("_TEMP_CTRL_BOTH", hptr) == 0) {
      statorTempCtrl.enabled = (parseSwitch(dptr) == SWITCH_ON);
      gearboxTempCtrl.enabled = (parseSwitch(dptr) == SWITCH_ON);
      Serial.print("<"); Serial.print(hptr); Serial.print(" "); Serial.print(dptr); Serial.println(">"); // <_TEMP_CTRL_BOTH 0/1>
    } else if (strcmp("_TEMP_SETPOINT", hptr) == 0) {
      Serial.print("<_TEMP_SETPOINT "); Serial.print(tempCtrlTarget->setpoint = strtof(dptr, NULL)); Serial.println(">"); // <_TEMP_SETPOINT ?.?>
    } else if (strcmp("_Ki", hptr) == 0) {
      Serial.print("<_Ki "); Serial.print(tempCtrlTarget->Ki = strtof(dptr, NULL)); Serial.println(">"); // <_Ki ?.?>
    } else if (strcmp("_Kp", hptr) == 0) {
      Serial.print("<_Kp "); Serial.print(tempCtrlTarget->Kp = strtof(dptr, NULL)); Serial.println(">"); // <_Kp ?.?>
    } else if (strcmp("_Kff", hptr) == 0) {
      Serial.print("<_Kff "); Serial.print(tempCtrlTarget->Kff = strtof(dptr, NULL)); Serial.println(">"); // <_Kff ?.?>
    } else if (strcmp("_KiLIM", hptr) == 0) {
      Serial.print("<_KiLIM "); Serial.print(tempCtrlTarget->KiLIM = strtof(dptr, NULL)); Serial.println(">"); // <_KiLIM ?.?>
    } else if (strcmp("_DIR", hptr) == 0) {
      if (inverter.direction(parseSwitch(dptr)) == 0) {
        Serial.print("<"); Serial.print(hptr); Serial.print(" "); Serial.print(dptr); Serial.println(">"); // <_DIR 0/1>
      }
    } else if (strcmp("START", hptr) == 0) { // <START>
      Serial.print("Sending START to inverter, error code ");
      Serial.println(inverter.start(), BIN);
      LEDcolour(0, 225, 0);
    } else if (strcmp("STOP", hptr) == 0) { // <STOP>
      Serial.print("Sending STOP to inverter, error code ");
      Serial.println(inverter.stop(), BIN);
      LEDcolour(255, 0, 0);
    } else if (strcmp("RESET", hptr) == 0) { // <RESET>
      Serial.print("Sending RESET to inverter, error code ");
      Serial.println(inverter.reset(), BIN);
    } else if (strcmp("COAST", hptr) == 0) { // <COAST>
      Serial.print("Sending COAST to inverter, error code ");
      Serial.println(inverter.coast(), BIN);
    } else if (strcmp("SPEED", hptr) == 0) { // <SPEED ?.?> in rpm
      Serial.print("Sending SPEED to inverter, error code ");
      Serial.println(inverter.speed(strtof(dptr, NULL)), BIN);
    } else if (strcmp("DIRECTION", hptr) == 0) { // <DIRECTION ?>
      Serial.print("Sending DIRECTION to inverter, error code ");
      Serial.println(inverter.direction(parseSwitch(dptr)), BIN);
    } else if (strcmp("WRITE_INVERTER", hptr) == 0) { // <WRITE_INVERTER 0x????????>
      long u = strtol(dptr, NULL, 16);
      Serial.print("Sending 0x");
      Serial.print(u, HEX);
      Serial.print(" to inverter, error code ");
      Serial.println(inverter.writeInverterWord(0x01, 0xFFFF & (u >> 16), 0xFFFF & u, true), BIN);
    } else if (strcmp("READ_INVERTER", hptr) == 0) { // <READ_INVERTER 0x????>
      long u = strtol(dptr, NULL, 16);
      Serial.print("Reading register 0x");
      Serial.print(u, HEX);
      Serial.print(" on inverter, value: 0x");
      Serial.println(inverter.readRegister(u & 0xFFFF), HEX);
    } else if (strcmp("BRAKING_SPEED", hptr) == 0) { // <BRAKING_SPEED ?>
      long u = strtol(dptr, NULL, 10) * 100 / 60; // convert rpm to Hz and Hz to 0.01 Hz
      Serial.print("Writing braking register 0x0109: 0x");
      Serial.print(u, HEX);
      Serial.print(", error code ");
      Serial.println(inverter.writeInverterWord(0x01, 0x0109, 0xFFFF & u, true), BIN);
    } else if (strcmp("BRAKING_TIME", hptr) == 0) { // <BRAKING_TIME ?.?>
      long u = long(strtof(dptr, NULL) * 100); // convert s to 0.01 s
      Serial.print("Writing braking register 0x010C: 0x");
      Serial.print(u, HEX);
      Serial.print(", error code ");
      Serial.println(inverter.writeInverterWord(0x01, 0x010C, 0xFFFF & u, true), BIN);
    } else if (strcmp("BRAKING_WAITTIME", hptr) == 0) { // <BRAKING_WAITTIME ?.?>
      long u = long(strtof(dptr, NULL) * 100); // convert s to 0.01 s
      Serial.print("Writing braking register 0x010A: 0x");
      Serial.print(u, HEX);
      Serial.print(", error code ");
      Serial.println(inverter.writeInverterWord(0x01, 0x010A, 0xFFFF & u, true), BIN);
    } else if (strcmp("BRAKING_CURRENT", hptr) == 0) { // <BRAKING_CURRENT ?.?>
      long u = long(strtof(dptr, NULL) * 10); // convert % to 0.1 %
      Serial.print("Writing braking register 0x010B: 0x");
      Serial.print(u, HEX);
      Serial.print(", error code ");
      Serial.println(inverter.writeInverterWord(0x01, 0x010B, 0xFFFF & u, true), BIN);
    } else if (strcmp("WRITE_SPI", hptr) == 0) { // <WRITE_SPI ?> where ? is a string
      Serial.print("Sending ");
      Serial.print(dptr);
      Serial.println(" on SPI");
      send8bitDataSPI(strlen(dptr), dptr);
    } else if (strcmp("REQUEST_SPI", hptr) == 0) { // <REQUEST_SPI>
      Serial.println("Requesting current data on SPI (timestamp speed)");
      uint8_t SPIbuf[SPI_BUF_LEN];
      int wdsrxd = requestDataSPI(SPI_CMD | SPI_REQ | SPI_REQ_CUR | SPI_REQ_T_TIMESTAMP | SPI_REQ_T_SPEED, 1, SPIbuf);
      Serial.print("Received: 0x");
      for (int j = 0; j < wdsrxd; j += 8) {
        Serial.print(*(uint32_t *)&SPIbuf[j]);
        Serial.print(" ");
        Serial.print(*(float *)&SPIbuf[j + 4]);
      }
      Serial.println(" from SPI");
    } else if (strcmp("CLUTCH", hptr) == 0) { // <CLUTCH>
      if (parseSwitch(dptr) == SWITCH_ON) {
        digitalWrite(CLUTCH_PIN, LOW);
      } else digitalWrite(CLUTCH_PIN, HIGH);
      LEDcolour(0, 0, 255);
    } else if (strcmp("RELAY_ON", hptr) == 0) { // <RELAY_ON>
      int relay_pin;
      switch (*dptr - '0') {
        case 5:
          relay_pin = RELAY_5; break;
        case 6:
          relay_pin = RELAY_6; break;
        case 7:
          relay_pin = RELAY_7; break;
        case 8:
          relay_pin = RELAY_8; break;
        default:
          relay_pin = RELAY_5;
      }
      digitalWrite(relay_pin, LOW);
      LEDcolour(0, 255, 128);
    } else if (strcmp("RELAY_OFF", hptr) == 0) { // <RELAY_OFF>
      int relay_pin;
      switch (*dptr - '0') {
        case 5:
          relay_pin = RELAY_5; break;
        case 6:
          relay_pin = RELAY_6; break;
        case 7:
          relay_pin = RELAY_7; break;
        case 8:
          relay_pin = RELAY_8; break;
        default:
          relay_pin = RELAY_5;
      }
      digitalWrite(relay_pin, HIGH);
      LEDcolour(0, 128, 255);
    } else if (strcmp("DATA", hptr) == 0) { // <DATA>
      if (parseSwitch(dptr) == SWITCH_OFF) {
        motordata = false;
      } else {
        motordata = true;
      }
    } else if (strcmp("TEMP", hptr) == 0) {
      tempdata = (parseSwitch(dptr) == SWITCH_ON); Serial.print("<"); Serial.print(hptr); Serial.print(" "); Serial.print(dptr); Serial.println(">"); // <TEMP 0/1>
    } else if (strcmp("THERMOCOUPLE_CONVERSION_TIME", hptr) == 0) {
      thermocoupleconversiontime = strtod(dptr, NULL); Serial.print("<"); Serial.print(hptr); Serial.print(" "); Serial.print(dptr); Serial.println(">"); // <THERMOCOUPLE_CONVERSION_TIME ?> ms
    } else if (strcmp("THERMOCOUPLE_RESOLUTION", hptr) == 0) { // <THERMOCOUPLE_RESOLUTION ?> 12, 14, 16, 18
      int res = strtod(dptr, NULL);
      byte RES = -1;
      switch(res) {
        case 12:
          RES = ADC_12BIT_RESOLUTION;
          break;
        case 14:
          RES = ADC_14BIT_RESOLUTION;
          break;
        case 16:
          RES = ADC_16BIT_RESOLUTION;
          break;
        case 18:
          RES = ADC_18BIT_RESOLUTION;
          break;
      }
      if (RES!=-1) {
        MCP9600_err_t ret = sensor.set_ADC_meas_resolution(RES);
        if (ret==NO_ERROR) {Serial.print("<"); Serial.print(hptr); Serial.print(" "); Serial.print(dptr); Serial.println(">");}
      }
    } else if (strcmp("LOAD", hptr) == 0) {
      loaddata = (parseSwitch(dptr) == SWITCH_ON); Serial.print("<"); Serial.print(hptr); Serial.print(" "); Serial.print(dptr); Serial.println(">"); // <LOAD 0/1>
    }
  }
}

void readSpeed(uint32_t * ts, float * sd, uint8_t * EE) {
  uint8_t SPIbuf[SPI_BUF_LEN];
  if (requestDataSPI(SPI_CMD | SPI_REQ | SPI_REQ_CUR | SPI_REQ_T_TIMESTAMP | SPI_REQ_T_SPEED, 1, SPIbuf)) {
    *ts = *(uint32_t *)&SPIbuf[0];
    *sd = *(float *)&SPIbuf[4];
    if (EE != NULL) *EE = SPIbuf[8];
  } else {
    *ts = 0;
    *sd = 0;
    if (EE != NULL) *EE = SPI_NOT_RECEIVED;
  }
}

void readSpeed(uint32_t * ts, float * sd, uint8_t * EE, int32_t ld[]) {
  uint8_t SPIbuf[SPI_BUF_LEN];
  if (requestDataSPI(SPI_CMD | SPI_REQ | SPI_REQ_CUR | SPI_REQ_T_TIMESTAMP | SPI_REQ_T_SPEED | SPI_REQ_T_LOAD, 1, SPIbuf)) {
    *ts = *(uint32_t *)&SPIbuf[0];
    *sd = *(float *)&SPIbuf[4];
    if (EE != NULL) *EE = SPIbuf[8];
    for (int j=0;j<4;j++) ld[j] = *(int32_t *)&SPIbuf[9+4*j];
  } else {
    *ts = 0;
    *sd = 0;
    for (int j=0;j<4;j++) ld[j] = 0;
    if (EE != NULL) *EE = SPI_NOT_RECEIVED;
  }
}

byte writeSPI(byte val) {
  // take the chip select low to select the device:
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  byte rr = SPI.transfer(val); // Send data
  // take the chip select high to de-select:
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return rr;
}

void LEDcolour(int R, int G, int B) {
  strip.setPixelColor(0, R, G, B);
  strip.show();
}

MCP9600_err_t sensor_basic_config()
{
  MCP9600_err_t ret = NO_ERROR;
  CHECK_RESULT(ret, sensor.set_filt_coefficients(FILT_OFF));
  CHECK_RESULT(ret, sensor.set_cold_junc_resolution(COLD_JUNC_RESOLUTION_0_625));
  CHECK_RESULT(ret, sensor.set_ADC_meas_resolution(ADC_16BIT_RESOLUTION));
  //CHECK_RESULT(ret,sensor.set_burst_mode_samp(BURST_32_SAMPLE));
  CHECK_RESULT(ret, sensor.set_sensor_mode(NORMAL_OPERATION));
  return ret;
}

MCP9600_err_t get_temperature(float *value)
{
  MCP9600_err_t ret = NO_ERROR;
  float hot_junc = 0;
  ret = sensor.read_hot_junc(&hot_junc);
  if (ret!=NO_ERROR) {
    sensor.init(THER_TYPE_K);
    sensor_basic_config();
    ret = sensor.read_hot_junc(&hot_junc);
    if (ret!=NO_ERROR) {
      writeSerial("Temperature Sensor Error");
    }
  }
  *value = hot_junc;

  return ret;
}

//void sendTemperature() {
//  char dbuf[100];
//  char fbuf[10];
//  int n = 0;
//  float temp = 0;
//  get_temperature(&temp);
//  //dtostrf(temp,5,2,fbuf);
//  sprintf(dbuf, "%ld, %5.2f", millis(), temp);
//  Serial.print(dbuf);
//}

void setSamplingInterval(int f) {
  sampint = 10000.0 / f; // Convert Hz x10^-1 to ms
}

bool intervalElapsed() {
  if ((millis() - roundf(prevmillis)) > roundf(1.3 * sampint)) { // if really behind, clear backlog
    prevmillis = millis();
    return false; // stops bouncy data on switch
  } else if ((millis() - roundf(prevmillis)) > roundf(sampint)) {
    prevmillis += sampint;
    return true;
  } else return false;
}

//    This function reads characters between < and > on the serial RX safely
//    b is the initial address of a c-style null-terminated string in which the message is stored
//    the length of the string is returned as int
int readSerial(char* b) {
  int c = 0; // byte counter
  char z = 0;
  while (Serial.available() > 0 && z != SER_START_CHAR) { // read serial stream until start character reached or no new bytes available
    z = Serial.read();
  }
  unsigned long t = millis(); // start timer
  if (z == SER_START_CHAR) { // if start character reached
    while ((millis() - t) < SER_TIMEOUT) { // if not timed out
      while (Serial.available() > 0) { // if there are new bytes received
        b[c] = Serial.read(); // read byte to b
        if (b[c] == SER_END_CHAR || c > 255) break;
        c++;
      }
      if (b[c] == SER_END_CHAR || c > 255) break;
    }
    if (b[c] != SER_END_CHAR) { // if no end character was received before timeout, message incomplete - discard
      c = 0;
    }
  }
  b[c] = '\0'; // null-terminate (overwrites end character)
  return c;
}

int parseSwitch(char * d) {
  if (d == NULL) return SWITCH_INVALID;
  if (d[0] == '\0') return SWITCH_INVALID;
  if (strcmp("1", dptr) == 0 || strcmp("ON", dptr) == 0 || strcmp("on", dptr) == 0) {
    return SWITCH_ON;
  }
  if (strcmp("0", dptr) == 0 || strcmp("OFF", dptr) == 0 || strcmp("off", dptr) == 0) {
    return SWITCH_OFF;
  }
  return SWITCH_INVALID;
}

//    This function writes a command on the serial TX, between < and >
//    b is the initial address of a c-style null-terminated string in which the message is stored
//    the number of characters sent is returned as int
int writeSerial(const char* d) {
  if (d[0] == '\0') return 0;
  char a[259] = {SER_START_CHAR, '\0'};
  char b[4] = {SER_END_CHAR, '\r', '\n', '\0'};
  strcat(a, d);
  strcat(a, b);
  Serial.write(a);
  return strlen(a) - 2;
}

void CStoggle() {
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(10);
}

void send8bitDataSPI(uint8_t len, char * d) {
  SPI.beginTransaction(SPISettings(SPI_BAUD, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);

  SPI.transfer(SPI_START_BYTE);     // START WORD
  CStoggle();

  SPI.transfer(SPI_CMD);            // TYPE
  CStoggle();

  SPI.transfer(len);                // LENGTH
  CStoggle();

  for (int i = 0; i < len; i++) {
    SPI.transfer(d[i]);
    CStoggle();
  }
  SPI.transfer(SPI_END_BYTE);       // END WORD

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

// usage:
// requestDataSPI(SPI_CMD | SPI_REQ | SPI_REQ_CUR | SPI_REQ_T_TIMESTAMP | SPI_REQ_T_SPEED, 1, SPIbuf);
int requestDataSPI(uint8_t type, uint8_t len, uint8_t * datbuf) {
  SPI.beginTransaction(SPISettings(SPI_BAUD, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);

  SPI.transfer(SPI_START_BYTE);     // START BYTE
  CStoggle();
  SPI.transfer(type);               // TYPE
  CStoggle();
  SPI.transfer(0x1);               // LENGTH
  CStoggle();
  SPI.transfer(len);                // VALS REQUESTED
  CStoggle();
  SPI.transfer(SPI_END_BYTE);       // END BYTE
  CStoggle();

  uint8_t bb[SPI_BUF_LEN];
  int16_t r;
  for (int i = 0; i < SPI_BUF_LEN; i++) {
    bb[i] = SPI.transfer(0x00);
    //    Serial.print(bb[i],HEX);
    //    Serial.print(" ");
    CStoggle();
    r = parse_spi_buffer(bb, datbuf);
    if (r > 0) {
      break;
    }
  }
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return r;
}

// extracts the data from the first parseable packet in the Rx buffer and returns the number of data dwords extracted, or -1 if there were no parseable packets
int parse_spi_buffer(uint8_t * spiRxBuf, uint8_t * spiMsgBuf) {
  for (int i = 0; i < SPI_BUF_LEN; i++) {
    if (spiRxBuf[i] == SPI_START_BYTE) {
      int cc = i;
      inc_counter_overflow(&cc, SPI_BUF_LEN); // jump past start
      if (spiRxBuf[cc] & SPI_RSP) { // message type - response from slave - master expects responses
        packettype = spiRxBuf[cc];
        inc_counter_overflow(&cc, SPI_BUF_LEN); // jump past type

        spi_data_len = spiRxBuf[cc];
        inc_counter_overflow(&cc, SPI_BUF_LEN); // jump past length
        if (spiRxBuf[(cc + spi_data_len) % SPI_BUF_LEN] == SPI_END_BYTE) { // check end word
          // act on command and process any data
          int j;
          if (spi_data_len > 0) { // if data copy to contiguous msgBuf
            for (j = 0; j < spi_data_len; j++) {
              spiMsgBuf[j] = spiRxBuf[cc];
              inc_counter_overflow(&cc, SPI_BUF_LEN);
            }
          }
          spiRxBuf[i] = 0; // erase (half of) start character so packet not reparsed
          spiRxBuf[cc] = 0; // erase (half of) end character so packet not reparsed
          spiMsgBuf[j] = '\0'; // null-terminate so msgBuf can be printed as string
          return j;
        }
      }
    }
  }
  return -1;
}

// increments counter by 2 and returns to zero before reaching ovflen
int inc_counter_overflow(int * counter, int ovflen) {
  int a = *counter;
  if (++(*counter) > (ovflen - 1)) {
    *counter -= ovflen;
  }
  return a;
}

void dotempctrl(struct tempCtrlParams * tcp) {
  if (tcp->enabled) {
    int timenow = millis();
    if (tcp->lastcountertime + tcp->counterinterval < timenow) {
      tcp->lastcountertime = timenow;
      if (tcp->counter++ < tcp->counts) {
        digitalWrite(tcp->HEATER_PIN, LOW);
      } else {
        digitalWrite(tcp->HEATER_PIN, HIGH);
      }
      //      if ((tcp->maxcounts-tcp->counter) < tcp->counts) { // does a dovetaily thing where the heaters are off at different times
      //        digitalWrite(tcp->HEATER_2_PIN, LOW);
      //      } else {
      //        digitalWrite(tcp->HEATER_2_PIN, HIGH);
      //      }
      if (tcp->counter >= tempctrlmaxcounts) tcp->counter = 0;
    }
    if (tcp->lastctrltime + tempctrlinterval < timenow) {
      tcp->lastctrltime = timenow;
      float error = tcp->setpoint - tcp->THERMOCOUPLE->temperature;
      tcp->integral += error * tempctrlinterval / 1000.0;
      if (tcp->integral > tcp->KiLIM) tcp->integral = tcp->KiLIM;
      else if (tcp->integral < -1 * tcp->KiLIM) tcp->integral = -1 * tcp->KiLIM;
      tcp->counts = tcp->Kff * (tcp->setpoint - 20) + tcp->Kp * error + tcp->Ki * tcp->integral;
      if (tcp->counts > tempctrlmaxcounts) tcp->counts = tempctrlmaxcounts;
      else if (tcp->counts < 0) tcp->counts = 0;
//      Serial.print("HEATER_PIN: ");
//      Serial.print(tcp->HEATER_PIN);
//      Serial.print("  Counts: ");
//      Serial.print(tcp->counts);
//      Serial.print("  Counter: ");
//      Serial.print(tcp->counter);
//      Serial.print("  Temp: ");
//      Serial.print(tcp->THERMOCOUPLE->temperature);
//      Serial.print("  Error: ");
//      Serial.print(error);
//      Serial.print("  Integral: ");
//      Serial.println(tcp->integral);
    }
  }
  else {
    digitalWrite(tcp->HEATER_PIN, HIGH);
    //    digitalWrite(HEATER_2_PIN, HIGH);
  }
}

void select_thermocouple(struct thermocouple * tc) {
  digitalWrite(S3_PIN, tc->pin & 0x8 ? HIGH : LOW);
  digitalWrite(S2_PIN, tc->pin & 0x4 ? HIGH : LOW);
  digitalWrite(S1_PIN, tc->pin & 0x2 ? HIGH : LOW);
  digitalWrite(S0_PIN, tc->pin & 0x1 ? HIGH : LOW);
}

void measure_temperatures() {
  int timenow = millis();
  if (thermocouples[currentthermocouple].timeselected + thermocoupleconversiontime < timenow) {
    get_temperature(&thermocouples[currentthermocouple].temperature);
    if (++currentthermocouple>=NUM_THERMOCOUPLES) currentthermocouple = 0;
    select_thermocouple(&thermocouples[currentthermocouple]);
    thermocouples[currentthermocouple].timeselected = timenow;
  }
}

void initialise_thermocouples() {
  for (int i=0;i<NUM_THERMOCOUPLES;i++) {
    thermocouples[i].pin = i;
  }
}
