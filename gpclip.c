
/*-----------------------------------------------------------------*
 * Gain pitch clipping routines                                    *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~                                    *
 * To avoid mismatch between encoder and decoder and  then unstable*
 * synthesis due to LTP error accumulation, the LTP gain is limited*
 * to 1 when pitch is highly predictive for a while (around 250ms).*
 *                                                                 *
 * The limitation (clipping) is done when pitch gain is higher than* 
 * a threshold for about 250 ms The threshold is fixed between     *
 * 0.94 and 1.                                                     *
 *    thres = 1.0 when minimum distance on ISF is 120 or higher.   *
 *    thres = 0.94 when minimum distance on ISF is 50 (ISF_GAP).   *
 * Typically, the threshold is decreased on high short term pred.  *
 * which reduces the impact of the clipping for less resonnant LP  *
 * filters                                                         * 
 *-----------------------------------------------------------------*/

#include "typedef.h"
#include "basic_op.h"
#include "count.h"
#include "bits.h"

#define DIST_ISF_MAX_IO 384                /* 150 Hz (6400Hz=16384) */
#define DIST_ISF_MAX    307                /* 120 Hz (6400Hz=16384) */
#define DIST_ISF_THRES  154                /* 60     (6400Hz=16384) */
#define GAIN_PIT_THRES  14746              /* 0.9 in Q14 */

#define GAIN_PIT_MIN    9830               /* 0.6 in Q14 */
#define M               16


void Init_gp_clip(
     Word16 mem[]                          /* (o) : memory of gain of pitch clipping algorithm */
)
{
    mem[0] = DIST_ISF_MAX;                 move16();
    mem[1] = GAIN_PIT_MIN;                 move16();
}


Word16 Gp_clip(
     Word16 ser_size,                      /* (i)   : size of the bitstream                      */
     Word16 mem[]                          /* (i/o) : memory of gain of pitch clipping algorithm */
)
{
    Word16 clip;
    Word16 thres;

    clip = 0;                              move16();  /* move16 */
    test(); test();
    if( (ser_size == NBBITS_7k) || (ser_size == NBBITS_9k) )
	{
		/* clipping is activated when filtered pitch gain > threshold (0.94 to 1 in Q14) */
		/* thres = 0.9f + (0.1f*mem[0]/DIST_ISF_MAX); */
		thres = add(14746, mult(1638, extract_l(L_mult(mem[0], (Word16)(16384/DIST_ISF_MAX_IO)))));

		test();
		if (sub(mem[1], thres) > 0)
			clip = 1;                          move16();
	}
	else if ((sub(mem[0], DIST_ISF_THRES) < 0) && (sub(mem[1], GAIN_PIT_THRES) > 0))
        clip = 1;                          move16();


    return (clip);
}


void Gp_clip_test_isf(
     Word16 ser_size,                      /* (i)   : size of the bitstream                      */
     Word16 isf[],                         /* (i)   : isf values (in frequency domain)           */
     Word16 mem[]                          /* (i/o) : memory of gain of pitch clipping algorithm */
)
{
    Word16 i, dist, dist_min;

    dist_min = sub(isf[1], isf[0]);

    for (i = 2; i < M - 1; i++)
    {
        dist = sub(isf[i], isf[i - 1]);
        test();
        if (sub(dist, dist_min) < 0)
        {
            dist_min = dist;               move16();
        }
    }

    dist = extract_h(L_mac(L_mult(26214, mem[0]), 6554, dist_min));

    test(); test();
    if( (ser_size == NBBITS_7k) || (ser_size == NBBITS_9k) )
	{
		test();
		if (sub(dist, DIST_ISF_MAX_IO) > 0)
		{
			dist = DIST_ISF_MAX_IO;               move16();
		}
	}
	else
	{
		test();
		if (sub(dist, DIST_ISF_MAX) > 0)
		{
			dist = DIST_ISF_MAX;               move16();
		}
	}
    mem[0] = dist;                         move16();

    return;
}


void Gp_clip_test_gain_pit(
     Word16 ser_size,                      /* (i)   : size of the bitstream                        */
     Word16 gain_pit,                      /* (i) Q14 : gain of quantized pitch                    */
     Word16 mem[]                          /* (i/o)   : memory of gain of pitch clipping algorithm */
)
{
    Word16 gain;
    Word32 L_tmp;

    test(); test();
    if( (ser_size == NBBITS_7k) || (ser_size == NBBITS_9k) )
	{
		/* long term LTP gain average (>250ms) */
		/* gain = 0.98*mem[1] + 0.02*gain_pit; */
		L_tmp = L_mult(32113, mem[1]);
		L_tmp = L_mac(L_tmp, 655, gain_pit);
	}
	else
	{
		L_tmp = L_mult(29491, mem[1]);
		L_tmp = L_mac(L_tmp, 3277, gain_pit);
	}
    gain = extract_h(L_tmp);

    test();
    if (sub(gain, GAIN_PIT_MIN) < 0)
    {
        gain = GAIN_PIT_MIN;               move16();
    }
    mem[1] = gain;                         move16();

    return;
}
