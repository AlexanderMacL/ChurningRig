//#define SPI_BUF_LEN 128
//#define SPI_START_WORD 0xF0F0F0F0
//#define SPI_END_WORD 0x0E0E0E0E
//#define SPI_CMD 0xC0
//#define SPI_CMD_MASK 0x000000FF
//#define SPI_REQ 0x1000
//#define SPI_RSP 0x2000
//#define SPI_REQ_MASK 0x00007000
//#define SPI_REQ_CUR 0x8000 
//#define SPI_REQ_T_MASK 0x00000F00
//#define SPI_REQ_T_TIMESTAMP 0x100
//#define SPI_REQ_T_SPEED 0x200

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

#define DIRCHNG 0x1
#define C_ERR 0x2
#define L1_ERR 0x4
#define L2_ERR 0x8
#define SPI_NOT_RECEIVED 0x10

uint8_t packettype;
uint8_t spi_data_len;
