/**************************************************************************/
/*! 
    @file     legacy.c
    @author   Luke-Jr
    @date     4 July, 2012
    @version  0.1

    Legacy ModMiner protocol implementation
*/
/**************************************************************************/

#include "drivers/jtag/jtag.h"
#include "core/gpio/gpio.h"

#include "mux.h"

#define PRODID "ModMiner Quad v0.4-ljr-alpha"

#define msg    muxbuf
#define msglen muxbuflen
#define step   muxstep

static uint32_t elen;

uint8_t fpgamax;
uint8_t fpgaidx[5] = {0,0,0,0,0xff};
uint8_t bcs[5];

const uint8_t tscspin[] = {0x14,0x14,0x2b,0x2b};
const uint8_t tssclpin[] = {0x30,0x12,0x11,0x10};
const uint8_t tssdopin[] = {0x31,0x23,0x0b,0x1a};

uint8_t temps_initialized = 0;

void temps_init()
{
	int i;
	SCB_SYSAHBCLKCTRL |= SCB_SYSAHBCLKCTRL_IOCON | SCB_SYSAHBCLKCTRL_GPIO;
	IOCON_JTAG_TDI_PIO0_11 = 0xc1;
	IOCON_JTAG_TMS_PIO1_0 = 0xc1;
	IOCON_JTAG_TDO_PIO1_1 = 0xc1;
	IOCON_JTAG_nTRST_PIO1_2 = 0xc1;
	IOCON_PIO1_4 = 0xc0;
	IOCON_PIO1_10 = 0xc0;
	IOCON_PIO2_3 = 0xc0;
	IOCON_PIO2_11 = 0xc0;
	IOCON_PIO3_0 = 0xc0;
	IOCON_PIO3_1 = 0xc0;
	for (i=0; i<4; ++i)
	{
		gpioSetValue(tscspin[i] >> 4, tscspin[i] & 0xf, 1);
		gpioSetValue(tssclpin[i] >> 4, tssclpin[i] & 0xf, 0);
		gpioSetDir(tscspin[i] >> 4, tscspin[i] & 0xf, gpioDirection_Output);
		gpioSetDir(tssclpin[i] >> 4, tssclpin[i] & 0xf, gpioDirection_Output);
		gpioSetDir(tssdopin[i] >> 4, tssdopin[i] & 0xf, gpioDirection_Input);
	}
	temps_initialized = 1;
}

uint8_t fpgaMap()
{
	uint8_t jtagportCount, jtagport, jtagdevTotal;
	int8_t jtagdevCount;
	jtagdevTotal = 0;
	jtagportCount = jtagDetectPorts();
	for (jtagport = 0; jtagport < jtagportCount; ++jtagport)
	{
		jtagdevCount = jtagDetect(jtagport);
		if (jtagdevCount > 0)
			fpgaidx[jtagdevTotal++] = jtagport;
	}
	fpgamax = jtagdevTotal;
	return jtagdevTotal;
}

void int2bits(uint32_t n, uint8_t*b, uint8_t bits)
{
	uint8_t i;
	for (i=(bits+7)/8; i>0; )
		b[--i] = 0;
	for (i=0; i<bits; ++i)
	{
		if (n & 1)
			b[i/8] |= 0x80 >> (i % 8);
		n >>= 1;
	}
}

uint32_t bits2int(uint8_t*b, uint8_t bits)
{
	uint32_t n, i;
	n = 0;
	for (i=0; i<bits; ++i)
		if (b[i/8] & (0x80 >> (i % 8)))
			n |= 1<<i;
	return n;
}

void bitendianflip(void*n, size_t bits)
{
	int i;
	uint8_t*b = n;
	// FIXME: this should probably work with non-byte boundaries
	bits /= 8;
	for (i=0; i<bits; ++i)
		b[i] = ((b[i] &    1) ? 0x80 : 0)
		     | ((b[i] &    2) ? 0x40 : 0)
		     | ((b[i] &    4) ? 0x20 : 0)
		     | ((b[i] &    8) ? 0x10 : 0)
		     | ((b[i] & 0x10) ?    8 : 0)
		     | ((b[i] & 0x20) ?    4 : 0)
		     | ((b[i] & 0x40) ?    2 : 0)
		     | ((b[i] & 0x80) ?    1 : 0);
}

void checksum(uint8_t*b, uint8_t bits)
{
	uint8_t i;
	uint8_t checksum = 1;
	for(i=0; i<bits; ++i)
		checksum ^= (b[i/8] & (0x80 >> (i % 8))) ? 1 : 0;
	if (checksum)
		b[i/8] |= 0x80 >> (i % 8);
}

void fpgaGetRegisterAsBytes(uint8_t jtag, uint8_t addr, uint8_t*buf)
{
	jtagWrite(jtag, JTAG_REG_IR, (const uint8_t*)"\x40", 6);
	buf[0] = 0;
	int2bits(addr, &buf[0], 4);
	checksum(buf, 5);
	jtagWrite(jtag, JTAG_REG_DR, buf, 6);
	jtagRead (jtag, JTAG_REG_DR, buf, 32);
	jtagReset(jtag);
}

uint32_t fpgaGetRegister(uint8_t jtag, uint8_t addr)
{
	uint8_t buf[4];
	fpgaGetRegisterAsBytes(jtag, addr, buf);
	return bits2int(buf, 32);
}

void fpgaSetRegisterFromBytes(uint8_t jtag, uint8_t addr, uint8_t*nv)
{
	uint8_t buf[38];
	jtagWrite(jtag, JTAG_REG_IR, (const uint8_t*)"\x40", 6);
	buf[0] = nv[0];
	buf[1] = nv[1];
	buf[2] = nv[2];
	buf[3] = nv[3];
	int2bits(addr, &buf[4], 4);
	buf[4] |= 8;
	checksum(buf, 37);
	jtagWrite(jtag, JTAG_REG_DR, buf, 38);
	jtagRun(jtag);
}

void fpgaSetRegister(uint8_t jtag, uint8_t addr, uint32_t nv)
{
	uint8_t buf[38];
	jtagWrite(jtag, JTAG_REG_IR, (const uint8_t*)"\x40", 6);
	int2bits(nv, &buf[0], 32);
	int2bits(addr, &buf[4], 4);
	buf[4] |= 8;
	checksum(buf, 37);
	jtagWrite(jtag, JTAG_REG_DR, buf, 38);
	jtagRun(jtag);
}

bool lmmRx(uint8_t c)
{
	// Old ModMiner protocol
	uint8_t jtag, x;
	msg[msglen++] = c;
	jtag = fpgaidx[msg[1]];
	switch (msg[0]) {
	case 0:  // Ping Pong
		pf_write("\0", 1);
		return true;
	case 1:  // Version
		pf_write(PRODID, sizeof(PRODID));
		return true;
	case 2:  // Get FPGA Count
		x = fpgaMap();
		pf_write(&x, 1);
		return true;
	case 3:  // Read ID Code
	{
		uint8_t idcode[4];
		if (msglen < 2)
			break;
		jtagReset(jtag);
		jtagRead (jtag, JTAG_REG_DR, idcode, 32);
		jtagReset(jtag);
		bitendianflip(idcode, 32);
		pf_write(idcode, 4);
		return true;
	}
	case 4:  // Read USER Code
	{
		uint8_t usercode[4];
		if (msglen < 2)
			break;
		jtagWrite(jtag, JTAG_REG_IR, (const uint8_t*)"\x10", 6);
		jtagRead (jtag, JTAG_REG_DR, usercode, 32);
		jtagReset(jtag);
		bitendianflip(usercode, 32);
		pf_write(usercode, 4);
		return true;
	}
	case 5:  // Program Bitstream
		switch (step) {
		case 0:
		{
			uint8_t i;
			if (msglen < 6)
				break;
			step = 1;
			msglen = 2;
			elen = msg[2] | ((uint32_t)msg[3] << 8) | ((uint32_t)msg[4] << 16) | ((uint32_t)msg[5] << 24);
			jtagWrite(jtag, JTAG_REG_IR, (const uint8_t*)"\xd0", 6);  // JPROGRAM
			do {
				i = 0xff;  // BYPASS while reading status
				jtagRead(jtag, JTAG_REG_IR, &i, 6);
			} while (i & 8);
			jtagWrite(jtag, JTAG_REG_IR, (const uint8_t*)"\xa0", 6);  // CFG_IN
			pf_write("\1", 1);
			// NOTE: for whatever reason, the FPGAs don't like immediately filling DR after CFG_IN; therefore, don't try to optimize this
			break;
		}
		case 1:
		{
			if (msglen < 34)
				break;
			pf_write("\1", 1);
			jtagSWrite(jtag, JTAG_REG_DR, &msg[2], 256);
			step = 2;
			msglen = 2;
			elen -= 32;
			break;
		}
		case 2:
		{
			uint8_t i;
			uint8_t needlen = (elen < 32) ? elen : 32;
			if (msglen < needlen+2)
				break;
			pf_write("\1", 1);
			elen -= needlen;
			jtagSWriteMore(jtag, &msg[2], 8*needlen, !elen);
			if (elen)
			{
				msglen = 2;
				break;
			}
			// Last data block
			jtagWrite(jtag, JTAG_REG_IR, (const uint8_t*)"\x30", 6);  // JSTART
			for (i=0; i<16; ++i)
				jtagRun(jtag);
			i = 0xff;  // BYPASS
			jtagRead(jtag, JTAG_REG_IR, &i, 6);
			i = (i & 4) ? 1 : 0;
			pf_write(&i, 1);
			return true;
		}
		}
		break;
	case 6:  // Set Clock Speed
	{
		uint8_t rv;
		if (msglen < 6)
			break;
		bcs[jtag] = msg[2]>200;
		if (bcs[jtag]) msg[2] = 50;
		fpgaSetRegister(jtag, 0xD, msg[2]);
		rv = !bcs[jtag];
		pf_write(&rv, 1);
		return true;
	}
	case 7:  // Read Clock Speed
	{
		uint8_t buf[4];
		if (msglen < 2)
			break;
		fpgaGetRegisterAsBytes(jtag, 0xD, buf);
		bitendianflip(buf, 32);
		pf_write(buf, 4);
		return true;
	}
	case 8:  // Send Job
	{
		uint8_t i, j;
		if (msglen < 46)
			break;
		for (i=1, j=2; i<12; ++i, j+=4)
			fpgaSetRegister(jtag, i, msg[j] | (msg[j+1]<<8) | (msg[j+2]<<16) | (msg[j+3]<<24));
		pf_write("\1", 1);
		return true;
	}
	case 9:  // Read Nonce
	{
		uint8_t buf[4];
		if (msglen < 2)
			break;
		fpgaGetRegisterAsBytes(jtag, 0xE, buf);
		if (bcs[jtag])
			buf[0] = 0;
		bitendianflip(buf, 32);
		pf_write(buf, 4);
		return true;
	}
	case 0xa:  // Read Temperature
	case 0xd:  // Read Temperature High Resolution
	{
		if (msglen < 2)
			break;
		uint8_t i = msg[1], j, bits = msg[0]==0xd?16:9;
		uint16_t temp = 0;
		if (i < 4)
		{
			if (!temps_initialized) temps_init();
			gpioSetValue(tscspin[i] >> 4, tscspin[i] & 0xf, 0);
			for (j=0; j<bits; ++j)
			{
				gpioSetValue(tssclpin[i] >> 4, tssclpin[i] & 0xf, 1);
				temp = (temp << 1) | gpioGetValue(tssdopin[i] >> 4, tssdopin[i] & 0xf);
				gpioSetValue(tssclpin[i] >> 4, tssclpin[i] & 0xf, 0);
			}
			gpioSetValue(tscspin[i] >> 4, tscspin[i] & 0xf, 1);
		}
		pf_write(&temp, msg[0]==0xd?2:1);
		return true;
	}
	case 0xb:  // Set Register
	{
		if (msglen < 7)
			break;
		bitendianflip(&msg[3], 32);
		fpgaSetRegisterFromBytes(jtag, msg[2], &msg[3]);
		pf_write("\1", 1);
		return true;
	}
	case 0xc:  // Read Register
	{
		uint8_t buf[4];
		if (msglen < 3)
			break;
		fpgaGetRegisterAsBytes(jtag, msg[2], buf);
		bitendianflip(buf, 32);
		pf_write(buf, 4);
		return true;
	}
	}
	return false;
}
