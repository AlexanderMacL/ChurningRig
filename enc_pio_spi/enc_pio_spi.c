/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "enc_pio_spi.pio.h"

#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"


#define PIO_ENC_PIN_1 16
#define PIO_ENC_PIN_2 17
#define PIO_ENC_PIN_IR 18

#define I2C0_SDA_PIN 4
#define I2C0_SCL_PIN 5

#define I2C1_SDA_PIN 10
#define I2C1_SCL_PIN 11

#define SPI1_TX 15
#define SPI1_RX 12
#define SPI1_SCK 14
#define SPI1_CS 13

#define SPI_P spi1

#define DATA_BUF_LEN 1000
#define SPI_BUF_LEN 64
#define SPI_START_BYTE 0xF0
#define SPI_END_BYTE 0x0E
#define SPI_CMD 0x10
#define SPI_REQ 0x20
#define SPI_RSP 0x40
#define SPI_REQ_CUR 0x80
#define SPI_REQ_T_MASK 0x0F
#define SPI_REQ_T_TIMESTAMP 0x1
#define SPI_REQ_T_SPEED 0x2
#define SPI_REQ_T_LOAD 0x4
#define SPI_BAUD 125000
#define I2C_BAUD 100000

#define DIRCHNG 0x1
#define C_ERR 0x2
#define L1_ERR 0x4
#define L2_ERR 0x8

void irq_handler(uint gpio, uint32_t events);
void my_spi_handler();
int inc_counter_overflow(volatile int * counter, int ovflen);
int popcount(int a);
void TxPacket();
void parse_spi_buffer(void);
void DwordToByteBuf(uint32_t * d, uint8_t * buf);

int c_old = 0;
int c_new = 0;
bool c_err = false;
bool dirchng = false;
bool new_speed = true;
bool spi_new_message = false;
float speed = 0;
uint32_t timestamp;
volatile int status;
volatile int spival = 5;
volatile int buffrontc = 0;
volatile int bufreadc = 0;
volatile int spibufc = 0;
volatile int spitxc = 0;
volatile uint8_t spi_data_len = 0;
uint32_t timestampBuf[DATA_BUF_LEN];
uint32_t speedBuf[DATA_BUF_LEN];
uint8_t spiRxBuf[SPI_BUF_LEN];
uint8_t spiTxBuf[SPI_BUF_LEN];
uint8_t spiMsgBuf[SPI_BUF_LEN];
volatile bool packet_in_progress = false;
volatile int packet_remaining = 0;
volatile uint32_t packettype = 0;
// int spi_set_baud;


PIO pio = pio0;
PIO pio_1 = pio1;
uint sm0 = 0;
uint sm1 = 1;
uint sm2 = 2;
uint sm3 = 3;

bool loadcell_0_error = true;
bool loadcell_1_error = true;

int32_t loadCellData[4];
const uint8_t numAverages = 6;

int main() {
    const uint samp_rate = 5; // in Hz

    stdio_init_all();
    printf("Starting enc_pio_spi\n");

    uint offset = pio_add_program(pio, &enc_pio_counter_program);
    enc_pio_counter_init(pio, sm0, offset, PIO_ENC_PIN_1, PIO_ENC_PIN_IR);

    offset = pio_add_program(pio, &enc_pio_metronome_program);
    enc_pio_metronome_init(pio, sm1, offset, PIO_ENC_PIN_IR, samp_rate);

    //irq_set_enabled(PIO0_IRQ_0, true);

    gpio_set_irq_enabled_with_callback(PIO_ENC_PIN_IR, GPIO_IRQ_EDGE_FALL, true, &irq_handler);

    // Configure SPI
    gpio_init(SPI1_TX);
    gpio_init(SPI1_RX);
    gpio_init(SPI1_SCK);
    gpio_init(SPI1_CS);
    gpio_set_function(SPI1_TX, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_RX, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_CS, GPIO_FUNC_SPI);
    spi_init(SPI_P, SPI_BAUD);
    // spi_set_baud = spi_set_baudrate(SPI_P, 1000000);
    spi_set_format(SPI_P, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    spi_set_slave(SPI_P, true);

    irq_set_enabled(SPI1_IRQ, true);
    irq_set_exclusive_handler(SPI1_IRQ, my_spi_handler);

    /* enable auto rx */
    spi_get_hw(SPI_P)->imsc = SPI_SSPIMSC_RXIM_BITS | SPI_SSPIMSC_RTIM_BITS;

    // Use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c0, I2C_BAUD);
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_PIN);
    gpio_pull_up(I2C0_SCL_PIN);

    i2c_init(i2c1, I2C_BAUD);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);

    NAU7802setDevice(i2c0);
    loadcell_0_error = !NAU7802begin(false);

    NAU7802setDevice(i2c1);
    loadcell_1_error = !NAU7802begin(false);

    while (true) {
        // if (c_err) {
        //     printf("Error detected\n");
        //     c_err = false;
        // }
        // if (dirchng) {
        //     printf("Direction change detected\n");
        //     dirchng = false;
        // }



        if (new_speed) {
            speed = (float)((int32_t)c_new)*samp_rate*60/144.0; // in rpm
            timestamp = to_ms_since_boot(get_absolute_time());
            timestampBuf[buffrontc] = timestamp;
            speedBuf[inc_counter_overflow(&buffrontc, DATA_BUF_LEN)] = * (uint32_t *) &speed;
            // printf("%06d, %+05.2f, %d, %d, %d, %d, %d\n", timestamp, speed, spival, spitxc, spi_get_hw(SPI_P)->cr0, spi_get_hw(SPI_P)->mis, spi_get_hw(SPI_P)->sr);
            c_old = c_new;
            new_speed = false;
        }
        if (spi_new_message) {
            // printf("%s\n",spiMsgBuf);
            spi_new_message = false;
        }


        // printf("HELLO!\n");
        // NAU7802setDevice(i2c0); // this is fast
        // // printf(NAU7802reset()?"RESET\n":"NOT RESET\n");
        // // printf(NAU7802powerUp()?"POWERED UP\n":"NOT POWERED UP\n");
        // // printf(NAU7802setLDO(NAU7802_LDO_3V3)?"SET LDO\n":"NOT SET LDO\n");
        // sleep_ms(100);
        // printf(NAU7802begin()?"BEGUN\n":"NOT BEGUN\n");
        // int i;
        // for (i = 0;i<20;i++) {
        //     printf("%x ",NAU7802getRegister(i));
        // }
        // printf("\n");


        // get data from both sensors alternately to leave time for the channel switch to happen
        NAU7802setDevice(i2c0); // this is fast
        if (loadcell_0_error) loadcell_0_error = !NAU7802begin(false);
        NAU7802getAverage(1);
        loadCellData[0] = NAU7802getAverage(numAverages);
        NAU7802setChannel(NAU7802_CHANNEL_2);
        NAU7802setDevice(i2c1);
        if (loadcell_1_error) loadcell_1_error = !NAU7802begin(false);
        NAU7802getAverage(1);
        loadCellData[2] = NAU7802getAverage(numAverages);
        NAU7802setChannel(NAU7802_CHANNEL_2);
        NAU7802setDevice(i2c0);
        NAU7802getAverage(1);
        loadCellData[3] = NAU7802getAverage(numAverages);
        NAU7802setChannel(NAU7802_CHANNEL_1);
        NAU7802setDevice(i2c1);
        NAU7802getAverage(1);
        loadCellData[1] = NAU7802getAverage(numAverages);
        NAU7802setChannel(NAU7802_CHANNEL_1);
        // printf("%d, %d, %d, %d\n", loadCellData[0], loadCellData[1], loadCellData[2], loadCellData[3]);
        // if loadCellData suggests NAU7802 isn't responding (e.g. because bus has frozen), clock out any residual data and then try to reset NAU7802
        if (loadCellData[0]==PICO_ERROR_TIMEOUT) {
            gpio_set_function(I2C0_SCL_PIN,GPIO_FUNC_SIO);
            gpio_set_dir(I2C0_SCL_PIN, GPIO_OUT);
            for (int i=0;i<10;i++) {
                gpio_put(I2C0_SCL_PIN,false);
                sleep_us(5);
                gpio_put(I2C0_SCL_PIN,true);
                sleep_us(5);
            } // toggle clock to release data line
            sleep_ms(1);
            gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
            gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
            gpio_pull_up(I2C0_SDA_PIN);
            gpio_pull_up(I2C0_SCL_PIN);
            i2c_init(i2c0, I2C_BAUD);
            // NAU7802setDevice(i2c0);
            // loadcell_0_error = !NAU7802begin(false);
        }
        if (loadCellData[2]==PICO_ERROR_TIMEOUT) {
            gpio_set_function(I2C1_SCL_PIN,GPIO_FUNC_SIO);
            gpio_set_dir(I2C1_SCL_PIN, GPIO_OUT);
            for (int i=0;i<10;i++) {
                gpio_put(I2C1_SCL_PIN,false);
                sleep_us(5);
                gpio_put(I2C1_SCL_PIN,true);
                sleep_us(5);
            } // toggle clock to release data line
            sleep_ms(1);
            gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
            gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
            gpio_pull_up(I2C1_SDA_PIN);
            gpio_pull_up(I2C1_SCL_PIN);
            i2c_init(i2c1, I2C_BAUD);
            // NAU7802setDevice(i2c1);
            // loadcell_1_error = !NAU7802begin(false);
        }
    }
}

void irq_handler(uint gpio, uint32_t events) {

// handle gpio interrupt
        int32_t x;
        if (pio_sm_get_rx_fifo_level(pio, sm0) < 1) {c_err = true;}
        else {
            x = pio_sm_get(pio, sm0); // read ticks word from RX FIFO
            if (pio->irq & 0x4) c_err = true;
            else if ((pio->irq & 0x1) && (pio->irq & 0x2)) dirchng = true;
            else if (pio->irq & 0x1) c_new = x;
            else if (pio->irq & 0x2) c_new = x * -1;
            else c_new = 0;
            new_speed = true;
        }
        while (pio_sm_get_rx_fifo_level(pio, sm0)>0) {pio_sm_get(pio, sm0);} // empty FIFO
        pio->irq = 0x07; // clear flags
        gpio_acknowledge_irq(gpio, events);
}

void my_spi_handler() {
    // handle spi cs interrupt
    status = spi_get_hw(SPI_P)->sr;

    while (status & SPI_SSPSR_RNE_BITS) { // rx
    // either if the SSPRXINTR is triggered (possibly when FIFO half full or more, datasheet says less?)
    // or if 32 bits' time have elapsed without the FIFO having been emptied (SSPRTINTR)
        // spival = spival+2;
        spiRxBuf[inc_counter_overflow(&spibufc, SPI_BUF_LEN)] = spi_get_hw(SPI_P)->dr;
        // printf("0x%x ",spiRxBuf[spibufc-1]);
        status = spi_get_hw(SPI_P)->sr;
    }
    if (packet_in_progress) TxPacket();
    else parse_spi_buffer();
    // printf("\n");

    irq_clear(SPI1_IRQ);
}

// spi message:
// S start (byte)
// C command (byte)
// M message length (byte)
// D data (byte)s
// E end (byte)
// 0xSS 0xCC 0xMM 0xDD ... 0xDD 0xEE
// st   cmd  len  data ... data end

void parse_spi_buffer() {
    for (int i=0;i<SPI_BUF_LEN;i++) {
        if (spiRxBuf[i]==SPI_START_BYTE) {
            int cc = i;
            inc_counter_overflow(&cc, SPI_BUF_LEN); // jump past start
            if (spiRxBuf[cc] & SPI_CMD) { // message type - command from master - slave only expects commands
                packettype = spiRxBuf[cc];
                inc_counter_overflow(&cc, SPI_BUF_LEN); // jump past type

                spi_data_len = spiRxBuf[cc];
                inc_counter_overflow(&cc, SPI_BUF_LEN); // jump past length
                for (int j=0;j<spi_data_len+4;j++) { // WHY IS THIS HERE
                }
                if (spiRxBuf[(cc + spi_data_len)%SPI_BUF_LEN] == SPI_END_BYTE && !packet_in_progress) { // check end char
                    // act on command and process any data
                    if (packettype & SPI_REQ) { // if data requested
                        spiTxBuf[0] = SPI_START_BYTE;
                        spiTxBuf[1] = (packettype & (SPI_REQ|SPI_REQ_CUR|SPI_REQ_T_MASK)) | SPI_RSP;
                        int kk = 0;
                        if (packettype & SPI_REQ_CUR) {
                            packet_remaining = 
                                ((packettype&SPI_REQ_T_TIMESTAMP)?4:0) 
                                + ((packettype&SPI_REQ_T_SPEED)?5:0) 
                                + ((packettype&SPI_REQ_T_LOAD)?16:0);
                            inc_counter_overflow(&cc, SPI_BUF_LEN); // ignore numdatarequested byte
                            spiTxBuf[2] = packet_remaining;
                            while(kk<packet_remaining) {
                                if (packettype & SPI_REQ_T_TIMESTAMP) {
                                    DwordToByteBuf(&timestamp, &spiTxBuf[kk+3]);
                                    kk+=4;
                                }
                                if (packettype & SPI_REQ_T_SPEED) { // 4 bytes speed float, 1 byte error status
                                    DwordToByteBuf((uint32_t *) &speed, &spiTxBuf[kk+3]);
                                    kk+=4;
                                    spiTxBuf[kk+3] = (c_err?C_ERR:0) | (dirchng?DIRCHNG:0) | (((packettype&SPI_REQ_T_LOAD)&&loadcell_0_error)?L1_ERR:0) | (((packettype&SPI_REQ_T_LOAD)&&loadcell_1_error)?L2_ERR:0);
                                    c_err = false;
                                    dirchng = false;
                                    kk+=1;
                                }
                                if (packettype & SPI_REQ_T_LOAD) { // 16 bytes load ints
                                    for (int j=0;j<4;j++) {
                                        DwordToByteBuf((uint32_t *)&loadCellData[j], &spiTxBuf[kk+3]);
                                        kk+=4;
                                    }
                                }
                            }
                        } else { // NB when SPI_REQ_CUR is not asserted, no speed error byte is sent and SPI_REQ_T_SPEED implies 4 bytes only
                            uint8_t numdatarequested = spiRxBuf[inc_counter_overflow(&cc, SPI_BUF_LEN)];  // read and jump past numdatarequested byte
                            packet_remaining = (
                                ((packettype&SPI_REQ_T_TIMESTAMP)?4:0) 
                                + ((packettype&SPI_REQ_T_SPEED)?4:0)
                                + ((packettype&SPI_REQ_T_LOAD)?16:0)
                                ) * numdatarequested;
                            spiTxBuf[2] = packet_remaining;
                            while(kk<packet_remaining) {
                                if (packettype & SPI_REQ_T_TIMESTAMP) {
                                    DwordToByteBuf(&timestampBuf[bufreadc], &spiTxBuf[kk+3]);
                                    kk+=4;
                                }
                                if (packettype & SPI_REQ_T_SPEED) {
                                    DwordToByteBuf(&speedBuf[bufreadc], &spiTxBuf[kk+3]);
                                    kk+=4;
                                }
                                if (packettype & SPI_REQ_T_LOAD) { // 16 bytes load ints
                                    for (int j = 0; j < 4; j++) {
                                        DwordToByteBuf((uint32_t *)&loadCellData[j], &spiTxBuf[kk+3]);
                                        kk+=4;
                                    }
                                }
                                inc_counter_overflow(&bufreadc, DATA_BUF_LEN);
                            }
                        }
                        spiTxBuf[kk+3] = SPI_END_BYTE;
                        packet_remaining += 4; // account for start, end, cmd and length bytes
                        packet_in_progress = true;
                        spi_get_hw(SPI_P)->imsc |= SPI_SSPIMSC_TXIM_BITS; // unmask tx fifo half full or less interrupt
                        TxPacket();
                    }

                    if (spi_data_len>0 && !(packettype&(SPI_REQ|SPI_REQ_CUR))) { // if data copy to contiguous msgBuf and set newmessage flag
                        int j;
                        for (j=0;j<spi_data_len;j++) {
                            spiMsgBuf[j] = spiRxBuf[inc_counter_overflow(&cc, SPI_BUF_LEN)];
                        }
                        spiMsgBuf[j] = '\0'; // null-terminate so msgBuf can be printed to serial
                        spi_new_message = true;
                    }
                    spiRxBuf[i] = 0; // erase start character so packet not reparsed
                    spiRxBuf[cc] = 0; // erase end character so packet not reparsed
                }
            }
        }
    }
}

void TxPacket() {
    while (packet_in_progress && packet_remaining>0 && spi_get_hw(SPI_P)->sr & SPI_SSPSR_TNF_BITS) { // only if fifo is not full
        spi_get_hw(SPI_P)->dr = spiTxBuf[inc_counter_overflow(&spitxc,SPI_BUF_LEN)];
        packet_remaining--;
    }
    if (packet_in_progress && (packet_remaining<=0)) {
        packet_in_progress = false;
        spitxc = 0;
        spi_get_hw(SPI_P)->imsc &= ~SPI_SSPIMSC_TXIM_BITS; // remask tx fifo half full or less interrupt
    }
}

// increments counter and zeroes if overflow
int inc_counter_overflow(volatile int * counter, int ovflen) {
    int a = *counter;
    if (++(*counter) > (ovflen-1)) {
        *counter = 0;
    }
    return a;
}

int popcount(int a) {
    int p = 0;
    for (int i=0;i<sizeof(int);i++) {// pretty sure this needs a *8 on the sizeof because sizeof is in bytes
        p += (a>>i) & 0x1;
    }
    return p;
}

void DwordToByteBuf(uint32_t * d, uint8_t * buf) {
    *(buf++) = (*d>>0) & 0xFF;
    *(buf++) = (*d>>8) & 0xFF;
    *(buf++) = (*d>>16) & 0xFF;
    *buf = (*d>>24) & 0xFF;
}