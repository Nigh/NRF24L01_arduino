
#include "nrf24l01.h"

#define PIN_GND  46  //Output
#define PIN_VCC  47  //Output
#define PIN_CE   48  //Output
#define PIN_nCS  49  //Output
#define PIN_SCK  50  //Output
#define PIN_MOSI 51  //Output
#define PIN_MISO 52  //input
#define PIN_IRQ  53  //input

// #define PIN_GND  22  //Output
// #define PIN_VCC  23  //Output
// #define PIN_CE   24  //Output
// #define PIN_nCS  25  //Output
// #define PIN_SCK  26  //Output
// #define PIN_MOSI 27  //Output
// #define PIN_MISO 28  //input
// #define PIN_IRQ  29  //input

#define MAX_TX  	0x10  //达到最大发送次数中断
#define TX_OK   	0x20  //TX发送完成中断
#define RX_OK   	0x40  //接收到数据中断

// #define MY_MAC_0 0x88
// #define MY_MAC_1 0x07
// #define MY_MAC_2 0x4B
// #define MY_MAC_3 0x1F
// #define MY_MAC_4 0x28
// #define MY_MAC_5 0xB2

#define MY_MAC_0	0xEF
#define MY_MAC_1	0xFF
#define MY_MAC_2	0xC0
#define MY_MAC_3	0xAA
#define MY_MAC_4	0x18
#define MY_MAC_5	0x01

#define SPI_READ 0xA5

inline uint8_t spi_byte(uint8_t byte)
{
	if (byte != SPI_READ)
		shiftOut(PIN_MOSI, PIN_SCK, MSBFIRST, byte);
	else
		return shiftIn(PIN_MISO, PIN_SCK, MSBFIRST);
	return 0;
}

void nrf_cmd(uint8_t cmd, uint8_t data)
{
	digitalWrite(PIN_nCS, LOW);
	spi_byte(cmd);
	spi_byte(data);
	digitalWrite(PIN_nCS, HIGH);
}

void nrf_simplebyte(uint8_t cmd)
{
	digitalWrite(PIN_nCS, LOW);
	spi_byte(cmd);
	digitalWrite(PIN_nCS, HIGH);
}

unsigned char nrf_read(unsigned char reg)
{
	unsigned char ret;
	digitalWrite(PIN_nCS, LOW);
	spi_byte(reg);
	ret = spi_byte(SPI_READ);
	digitalWrite(PIN_nCS, HIGH);
	return ret;
}

void nrf_manybytes(uint8_t *data, uint8_t len)
{

	digitalWrite(PIN_nCS, LOW);
	do {
		spi_byte(*data++);
	} while (--len);
	digitalWrite(PIN_nCS, HIGH);
}

void nrf_readManyBytes(unsigned char reg, unsigned char *buf, unsigned char length)
{
	unsigned char ret;
	digitalWrite(PIN_nCS, LOW);
	spi_byte(reg);
	// *buf++ = shiftIn(PIN_MISO, PIN_SCK, MSBFIRST);
	while (length--) *buf++ = spi_byte(SPI_READ);
	digitalWrite(PIN_nCS, HIGH);
}

void nrf_writeTxData(unsigned char reg, unsigned char *ptr, unsigned char length)
{
	digitalWrite(PIN_nCS, LOW);
	spi_byte(reg);
	do {
		spi_byte(*ptr++);
	} while (--length);
	digitalWrite(PIN_nCS, HIGH);
}

void btLeCrc(const uint8_t *data, uint8_t len, uint8_t *dst)
{
	uint8_t v, t, d;
	while (len--) {
		d = *data++;
		for (v = 0; v < 8; v++, d >>= 1) {
			t = dst[0] >> 7;
			dst[0] <<= 1;
			if (dst[1] & 0x80) dst[0] |= 1;
			dst[1] <<= 1;
			if (dst[2] & 0x80) dst[1] |= 1;
			dst[2] <<= 1;
			if (t != (d & 1)) {
				dst[2] ^= 0x5B;
				dst[1] ^= 0x06;
			}
		}
	}
}

void btLeWhiten(uint8_t *data, uint8_t len, uint8_t whitenCoeff)
{
	uint8_t  m;
	while (len--) {
		for (m = 1; m; m <<= 1) {
			if (whitenCoeff & 0x80) {
				whitenCoeff ^= 0x11;
				(*data) ^= m;
			}
			whitenCoeff <<= 1;
		}
		data++;
	}
}

inline uint8_t btLeWhitenStart(uint8_t chan)
{
	//the value we actually use is what BT'd use left shifted one...makes our life easier
	return swapbits(chan) | 2;
}

void btLePacketEncode(uint8_t *packet, uint8_t len, uint8_t chan)
{
	//length is of packet, including crc. pre-populate crc in packet with initial crc value!
	uint8_t i, dataLen = len - 3;
	btLeCrc(packet, dataLen, packet + dataLen);
	for (i = 0; i < 3; i++, dataLen++) packet[dataLen] = swapbits(packet[dataLen]);
	btLeWhiten(packet, len, btLeWhitenStart(chan));
	for (i = 0; i < len; i++) packet[i] = swapbits(packet[i]);
}

uint8_t swapbits(uint8_t a)
{
	uint8_t v = 0;
	if (a & 0x80) v |= 0x01;
	if (a & 0x40) v |= 0x02;
	if (a & 0x20) v |= 0x04;
	if (a & 0x10) v |= 0x08;
	if (a & 0x08) v |= 0x10;
	if (a & 0x04) v |= 0x20;
	if (a & 0x02) v |= 0x40;
	if (a & 0x01) v |= 0x80;
	return v;
}

#define hex_print(hex) Serial.print("0x");Serial.print(hex,HEX);Serial.print(" ")
unsigned char txData[] = {'1', '2', '5', '7', '9'};
unsigned char TxAddr[] = {0x34, 0x43, 0x10, 0x10, 0x01}; //发送地址
void setup()
{
	unsigned char _;

	pinMode(PIN_nCS, OUTPUT);
	pinMode(PIN_CE, OUTPUT);
	pinMode(PIN_GND, OUTPUT);
	pinMode(PIN_VCC, OUTPUT);
	pinMode(PIN_SCK, OUTPUT);
	pinMode(PIN_MOSI, OUTPUT);
	pinMode(PIN_MISO, INPUT_PULLUP);
	pinMode(PIN_IRQ, INPUT_PULLUP);

	digitalWrite(PIN_GND, 0);
	digitalWrite(PIN_VCC, 1);
	digitalWrite(PIN_CE, 0);
	digitalWrite(PIN_nCS, 1);
	digitalWrite(PIN_SCK, 0);
	digitalWrite(PIN_MOSI, 0);

}

// byte incoming = shiftIn(dataPin, clockPin, bitOrder)
void loop()
{
	static const uint8_t chRf[] = {2, 26, 80};
	static const uint8_t chLe[] = {37, 38, 39};
	uint8_t i, L, ch = 0;
	uint8_t offset, offset2, _, buf[32];
	char num[3] = {'0', '0', '0'};

	Serial.begin(115200);
	Serial.println("init start");
	randomSeed(micros());
	delay(10);
	digitalWrite(PIN_CE, 0);
	nrf_cmd(0x20, 0x12);	//on, no crc, int on RX/TX done
	_ = nrf_read(0x00);
	Serial.print("NRF CONFIG:");
	Serial.println(_, HEX);
	nrf_cmd(0x21, 0x00);	//no auto-acknowledge
	nrf_cmd(0x22, 0x00);	//no RX
	nrf_cmd(0x23, 0x02);	//5-byte address
	nrf_cmd(0x24, 0x00);	//no auto-retransmit
	nrf_cmd(0x26, 0x06);	//1MBps at 0dBm
	nrf_cmd(0x27, 0x3E);	//clear various flags
	nrf_cmd(0x3C, 0x00);	//no dynamic payloads
	nrf_cmd(0x3D, 0x00);	//no features
	nrf_cmd(0x31, 32);		//always RX 32 bytes
	nrf_cmd(0x22, 0x01);	//RX on pipe 0

	buf[0] = 0x30;			//set addresses
	buf[1] = swapbits(0x8E);
	buf[2] = swapbits(0x89);
	buf[3] = swapbits(0xBE);
	buf[4] = swapbits(0xD6);
	nrf_manybytes(buf, 5);
	buf[0] = 0x2A;
	nrf_manybytes(buf, 5);
	delay(1);
	Serial.println("init finish");
	_ = 0;
	offset = 0;
	offset2 = 0;
	while (1) {
		_++;
		if (_ > 8) {
			offset++;
			offset2 += 7;
			_ = 0;
			num[0] += 1;
			if (num[0] > '9') {
				num[0] = '0'; num[1] += 1;
			}
			if (num[1] > '9') {
				num[1] = '0'; num[2] += 1;
			}
			if (num[2] > '9') {
				num[2] = '0';
			}
		}

		L = 0;

		buf[L++] = 0x42;	//PDU type, given address is random
		buf[L++] = 0x11 + 7;	//17 bytes of payload

		buf[L++] = MY_MAC_0 + offset;
		buf[L++] = MY_MAC_1 + offset2;
		buf[L++] = MY_MAC_2;
		buf[L++] = MY_MAC_3;
		buf[L++] = MY_MAC_4;
		buf[L++] = MY_MAC_5;

		buf[L++] = 2;		//flags (LE-only, limited discovery mode)
		buf[L++] = 0x01;
		buf[L++] = 0x05;

		buf[L++] = 7 + 7; // + 8;
		buf[L++] = 0x08;

		buf[L++] = random(30, 128);
		buf[L++] = 'B';
		buf[L++] = 'L';
		buf[L++] = 'E';
		buf[L++] = '_';
		buf[L++] = 'P';

		buf[L++] = 'L';
		buf[L++] = 'U';
		buf[L++] = 'S';

		buf[L++] = '_';
		buf[L++] = num[2];
		buf[L++] = num[1];
		buf[L++] = num[0];

		buf[L++] = 0x55;	//CRC start value: 0x555555
		buf[L++] = 0x55;
		buf[L++] = 0x55;

		if (++ch == sizeof(chRf)) ch = 0;

		nrf_cmd(0x25, chRf[ch]);
		nrf_cmd(0x27, 0x6E);	//clear flags

		btLePacketEncode(buf, L, chLe[ch]);

		nrf_simplebyte(0xE2); //Clear RX Fifo
		nrf_simplebyte(0xE1); //Clear TX Fifo

		digitalWrite(PIN_nCS, LOW); // cbi(PORTB, PIN_nCS);
		spi_byte(0xA0);
		for (i = 0 ; i < L ; i++) spi_byte(buf[i]);
		digitalWrite(PIN_nCS, HIGH); // sbi(PORTB, PIN_nCS);

		nrf_cmd(0x20, 0x12);	//tx on
		digitalWrite(PIN_CE, HIGH); // sbi(PORTB, PIN_CE);	 //do tx
		delay(10);
		digitalWrite(PIN_CE, LOW); // cbi(PORTB, PIN_CE);	(in preparation of switching to RX quickly)
		delay(1);
	}
}
