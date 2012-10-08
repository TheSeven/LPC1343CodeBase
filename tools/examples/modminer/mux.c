#include <stdint.h>
#include <stdio.h>

#include "sysdefs.h"

#ifdef CFG_INTERFACE
  #include "core/cmd/cmd.h"
#endif

void muxRx(uint8_t);

void muxPoll()
{
  int c;
  while (EOF != (c = pf_getchar()))
    muxRx(c);
}

static enum {
  MUX_NONE,
  MUX_CMD,
  MUX_COMPAT,
  MUX_MHBP,
} muxMode;

uint8_t muxbuf[256];
uint8_t muxbuflen;
uint8_t muxstep;

extern bool lmmRx(uint8_t);

void muxRx(uint8_t c)
{
	if (muxMode == MUX_NONE)
	{
tryNewMux:
		muxbuflen = 0;
		muxstep = 0;
		if (c == 0xfe)
			muxMode = MUX_MHBP;
		else
		if (c < 0x10)
			muxMode = MUX_COMPAT;
#ifdef CFG_INTERFACE
		else
		if (c < 0x80)
			muxMode = MUX_CMD;
#endif
	}
	switch (muxMode) {
	case MUX_NONE:
		break;
#ifdef CFG_INTERFACE
	case MUX_CMD:
		if (c < 8 || c > 0x7f)
			goto tryNewMux;
		if (cmdRx(c))
			goto muxDone;
		break;
#endif
	case MUX_COMPAT:
		if (lmmRx(c))
			goto muxDone;
		break;
	case MUX_MHBP:
		// TODO: MHBP
		break;
	}
	return;

muxDone:
	muxMode = MUX_NONE;
}

