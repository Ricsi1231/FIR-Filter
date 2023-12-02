#include "FIR_Filter.h"

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f};

void FIRFilter_Init(FIRFilter *fir) 
{
	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) 
    {
		fir->buf[n] = 0.0f;
	}
	
	fir->bufIndex = 0;
	fir->out = 0.0f;
}

float FIRFilter_Update(FIRFilter *fir, float inp) 
{
	fir->buf[fir->bufIndex] = inp;
	fir->bufIndex++;

	if (fir->bufIndex == FIR_FILTER_LENGTH) 
    {
		fir->bufIndex = 0;
	}

	fir->out = 0.0f;

	uint8_t sumIndex = fir->bufIndex;

	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) 
    {
		if (sumIndex > 0) 
        {
			sumIndex--;
		} 
		else 
		{
			sumIndex = FIR_FILTER_LENGTH - 1;
		}

		fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];
	}

	return fir->out;
}
