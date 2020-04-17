/*--------------------------------------------------------------------------*
 *                         COD_MAIN.H                                       *
 *--------------------------------------------------------------------------*
 *       Static memory in the encoder				                        *
 *--------------------------------------------------------------------------*/

#include "cnst.h"                          /* coder constant parameters */

#include "wb_vad.h"
#include "dtx.h"

typedef struct
{
    Word16 mem_decim[2 * L_FILT16k];       /* speech decimated filter memory */
    Word16 mem_sig_in[6];                  /* hp50 filter memory */
    Word16 mem_preemph;                    /* speech preemph filter memory */
    Word16 old_speech[L_TOTAL - L_FRAME];  /* old speech vector at 12.8kHz */
    Word16 old_wsp[PIT_MAX / OPL_DECIM];   /* old decimated weighted speech vector */
    Word16 old_exc[PIT_MAX + L_INTERPOL];  /* old excitation vector */
    Word16 mem_levinson[M + 2];            /* levinson routine memory */
    Word16 ispold[M];                      /* old isp (immittance spectral pairs) */
    Word16 ispold_q[M];                    /* quantized old isp */
    Word16 past_isfq[M];                   /* past isf quantizer */
    Word16 mem_wsp;                        /* wsp vector memory */
    Word16 mem_decim2[3];                  /* wsp decimation filter memory */
    Word16 mem_w0;                         /* target vector memory */
    Word16 mem_syn[M];                     /* synthesis memory */
    Word16 tilt_code;                      /* tilt of code */
    Word16 old_wsp_max;                    /* old wsp maximum value */
    Word16 old_wsp_shift;                  /* old wsp shift */
    Word16 Q_old;                          /* old scaling factor */
    Word16 Q_max[2];                       /* old maximum scaling factor */
    Word16 gp_clip[2];                     /* gain of pitch clipping memory */
    Word16 qua_gain[4];                    /* gain quantizer memory */

    Word16 old_T0_med;
    Word16 ol_gain;
    Word16 ada_w;
    Word16 ol_wght_flg;
    Word16 old_ol_lag[5];
    Word16 hp_wsp_mem[9];
    Word16 old_hp_wsp[L_FRAME / OPL_DECIM + (PIT_MAX / OPL_DECIM)];
    VadVars *vadSt;
    dtx_encState *dtx_encSt;
    Word16 first_frame;

    Word16 isfold[M];                      /* old isf (frequency domain) */
    Word32 L_gc_thres;                     /* threshold for noise enhancer */
    Word16 mem_syn_hi[M];                  /* modified synthesis memory (MSB) */
    Word16 mem_syn_lo[M];                  /* modified synthesis memory (LSB) */
    Word16 mem_deemph;                     /* speech deemph filter memory */
    Word16 mem_sig_out[6];                 /* hp50 filter memory for synthesis */
    Word16 mem_hp400[6];                   /* hp400 filter memory for synthesis */
    Word16 mem_oversamp[2 * L_FILT];       /* synthesis oversampled filter memory */
    Word16 mem_syn_hf[M];                  /* HF synthesis memory */
    Word16 mem_hf[2 * L_FILT16k];          /* HF band-pass filter memory */
    Word16 mem_hf2[2 * L_FILT16k];         /* HF band-pass filter memory */
    Word16 seed2;                          /* random memory for HF generation */
    Word16 vad_hist;

    Word16 gain_alpha;

} Coder_State;

/*
********************************************************************************
*
*      File             : typedef.c
*      Purpose          : Basic types.
*
********************************************************************************
*/


#ifndef typedef_h
#define typedef_h "$Id $"

#undef ORIGINAL_TYPEDEF_H /* define to get "original" ETSI version
                             of typedef.h                           */

#ifdef ORIGINAL_TYPEDEF_H
/*
 * this is the original code from the ETSI file typedef.h
 */
   
#if defined(__BORLANDC__) || defined(__WATCOMC__) || defined(_MSC_VER) || defined(__ZTC__)
typedef signed char Word8;
typedef short Word16;
typedef long Word32;
typedef int Flag;

#elif defined(__sun)
typedef signed char Word8;
typedef short Word16;
typedef long Word32;
typedef int Flag;

#elif defined(__unix__) || defined(__unix)
typedef signed char Word8;
typedef short Word16;
typedef int Word32;
typedef int Flag;

#endif
#else /* not original typedef.h */

/*
 * use (improved) type definition file typdefs.h and add a "Flag" type
 */
#include "typedefs.h"
typedef int Flag;

#endif

#endif

/********************************************************************************
*
*      File             : typedefs.h
*      Description      : Definition of platform independent data
*                         types and constants
*
*
*      The following platform independent data types and corresponding
*      preprocessor (#define) constants are defined:
*
*        defined type  meaning           corresponding constants
*        ----------------------------------------------------------
*        Char          character         (none)
*        Bool          boolean           true, false
*        Word8         8-bit signed      minWord8,   maxWord8
*        UWord8        8-bit unsigned    minUWord8,  maxUWord8
*        Word16        16-bit signed     minWord16,  maxWord16
*        UWord16       16-bit unsigned   minUWord16, maxUWord16
*        Word32        32-bit signed     minWord32,  maxWord32
*        UWord32       32-bit unsigned   minUWord32, maxUWord32
*        Float         floating point    minFloat,   maxFloat
*
*
*      The following compile switches are #defined:
*
*        PLATFORM      string indicating platform progam is compiled on
*                      possible values: "OSF", "PC", "SUN"
*
*        OSF           only defined if the current platform is an Alpha
*        PC            only defined if the current platform is a PC
*        SUN           only defined if the current platform is a Sun
*        
*        LSBFIRST      is defined if the byte order on this platform is
*                      "least significant byte first" -> defined on DEC Alpha
*                      and PC, undefined on Sun
*
********************************************************************************
*/
#ifndef typedefs_h
#define typedefs_h "$Id $"

/*
********************************************************************************
*                         INCLUDE FILES
********************************************************************************
*/
#include <float.h>
#include <limits.h>



/*
********************************************************************************
*                         DEFINITION OF CONSTANTS 
********************************************************************************
*/
/*
 ********* define char type
 */
typedef char Char;

/*
 ********* define 8 bit signed/unsigned types & constants
 */
#if SCHAR_MAX == 127
typedef signed char Word8;
#define minWord8  SCHAR_MIN
#define maxWord8  SCHAR_MAX

typedef unsigned char UWord8;
#define minUWord8 0
#define maxUWord8 UCHAR_MAX
#else
#error cannot find 8-bit type
#endif


/*
 ********* define 16 bit signed/unsigned types & constants
 */
#if INT_MAX == 32767
typedef int Word16;
#define minWord16     INT_MIN
#define maxWord16     INT_MAX
typedef unsigned int UWord16;
#define minUWord16    0
#define maxUWord16    UINT_MAX
#elif SHRT_MAX == 32767
typedef short Word16;
#define minWord16     SHRT_MIN
#define maxWord16     SHRT_MAX
typedef unsigned short UWord16;
#define minUWord16    0
#define maxUWord16    USHRT_MAX
#else
#error cannot find 16-bit type
#endif


/*
 ********* define 32 bit signed/unsigned types & constants
 */
#if INT_MAX == 2147483647
typedef int Word32;
#define minWord32     INT_MIN
#define maxWord32     INT_MAX
typedef unsigned int UWord32;
#define minUWord32    0
#define maxUWord32    UINT_MAX
#elif LONG_MAX == 2147483647
typedef long Word32;
#define minWord32     LONG_MIN
#define maxWord32     LONG_MAX
typedef unsigned long UWord32;
#define minUWord32    0
#define maxUWord32    ULONG_MAX
#else
#error cannot find 32-bit type
#endif

/*
 ********* define floating point type & constants
 */
/* use "#if 0" below if Float should be double;
   use "#if 1" below if Float should be float
 */
#if 0
typedef float Float;
#define maxFloat      FLT_MAX
#define minFloat      FLT_MIN
#else
typedef double Float;
#define maxFloat      DBL_MAX
#define minFloat      DBL_MIN
#endif

/*
 ********* define complex type
 */
typedef struct {
  Float r;  /* real      part */
  Float i;  /* imaginary part */
} CPX;

/*
 ********* define boolean type
 */
typedef int Bool;
#define false 0
#define true 1

/*
 ********* Check current platform
 */
#if defined(__MSDOS__)
#define PC
#define PLATFORM "PC"
#define LSBFIRST
#elif defined(__osf__)
#define OSF
#define PLATFORM "OSF"
#define LSBFIRST
#elif defined(__sun__) || defined(__sun)
#define SUN
#define PLATFORM "SUN"
#undef LSBFIRST
#elif defined(linux) && defined(i386)
#define PC
#define PLATFORM "PC"
#define LSBFIRST
#else
#error "can't determine architecture; adapt typedefs.h to your platform"
#endif

#endif

/*
********************************************************************************
*
*      File             : basicop2.c
*      Purpose          : Basic operators.
*
********************************************************************************
*/

/*___________________________________________________________________________
 |                                                                           |
 |   Constants and Globals                                                   |
 |                                                                           |
 |___________________________________________________________________________|
*/
extern Flag Overflow;
extern Flag Carry;

#define MAX_32 (Word32)0x7fffffffL
#define MIN_32 (Word32)0x80000000L

#define MAX_16 (Word16)+32767	/* 0x7fff */
#define MIN_16 (Word16)-32768	/* 0x8000 */

/*___________________________________________________________________________
 |                                                                           |
 |   Prototypes for basic arithmetic operators                               |
 |___________________________________________________________________________|
*/

Word16 add (Word16 var1, Word16 var2);    /* Short add,           1   */
Word16 sub (Word16 var1, Word16 var2);    /* Short sub,           1   */
Word16 abs_s (Word16 var1);               /* Short abs,           1   */
Word16 shl (Word16 var1, Word16 var2);    /* Short shift left,    1   */
Word16 shr (Word16 var1, Word16 var2);    /* Short shift right,   1   */
Word16 mult (Word16 var1, Word16 var2);   /* Short mult,          1   */
Word32 L_mult (Word16 var1, Word16 var2); /* Long mult,           1   */
Word16 negate (Word16 var1);              /* Short negate,        1   */
Word16 extract_h (Word32 L_var1);         /* Extract high,        1   */
Word16 extract_l (Word32 L_var1);         /* Extract low,         1   */
Word16 round (Word32 L_var1);             /* Round,               1   */
Word32 L_mac (Word32 L_var3, Word16 var1, Word16 var2);   /* Mac,  1  */
Word32 L_msu (Word32 L_var3, Word16 var1, Word16 var2);   /* Msu,  1  */
Word32 L_add (Word32 L_var1, Word32 L_var2);    /* Long add,        2 */
Word32 L_sub (Word32 L_var1, Word32 L_var2);    /* Long sub,        2 */
Word32 L_add_c (Word32 L_var1, Word32 L_var2);  /* Long add with c, 2 */
Word32 L_negate (Word32 L_var1);                /* Long negate,     2 */
Word16 mult_r (Word16 var1, Word16 var2);       /* Mult with round, 2 */
Word32 L_shl (Word32 L_var1, Word16 var2);      /* Long shift left, 2 */
Word32 L_shr (Word32 L_var1, Word16 var2);      /* Long shift right, 2*/
Word16 shr_r (Word16 var1, Word16 var2);        /* Shift right with
                                                   round, 2           */
Word32 L_deposit_h (Word16 var1);        /* 16 bit var1 -> MSB,     2 */
Word32 L_deposit_l (Word16 var1);        /* 16 bit var1 -> LSB,     2 */

Word32 L_shr_r (Word32 L_var1, Word16 var2); /* Long shift right with
                                                round,  3             */
Word32 L_abs (Word32 L_var1);            /* Long abs,              3  */
Word16 norm_s (Word16 var1);             /* Short norm,           15  */
Word16 div_s (Word16 var1, Word16 var2); /* Short division,       18  */
Word16 norm_l (Word32 L_var1);           /* Long norm,            30  */   

/*--------------------------------------------------------------------------*
 *                         CNST.H                                           *
 *--------------------------------------------------------------------------*
 *       Codec constant parameters (coder and decoder)                      *
 *--------------------------------------------------------------------------*/

#define CODEC_VERSION "7.0.0"

#define L_FRAME16k   320                   /* Frame size at 16kHz                        */
#define L_FRAME      256                   /* Frame size                                 */
#define L_SUBFR16k   80                    /* Subframe size at 16kHz                     */

#define L_SUBFR      64                    /* Subframe size                              */
#define NB_SUBFR     4                     /* Number of subframe per frame               */

#define L_NEXT       64                    /* Overhead in LP analysis                    */
#define L_WINDOW     384                   /* window size in LP analysis                 */
#define L_TOTAL      384                   /* Total size of speech buffer.               */
#define M            16                    /* Order of LP filter                         */
#define M16k             20

#define L_FILT16k    15                    /* Delay of down-sampling filter              */
#define L_FILT       12                    /* Delay of up-sampling filter                */

#define GP_CLIP      15565                 /* Pitch gain clipping = 0.95 Q14             */
#define PIT_SHARP    27853                 /* pitch sharpening factor = 0.85 Q15         */

#define PIT_MIN      34                    /* Minimum pitch lag with resolution 1/4      */
#define PIT_FR2      128                   /* Minimum pitch lag with resolution 1/2      */
#define PIT_FR1_9b   160                   /* Minimum pitch lag with resolution 1        */
#define PIT_FR1_8b   92                    /* Minimum pitch lag with resolution 1        */
#define PIT_MAX      231                   /* Maximum pitch lag                          */
#define L_INTERPOL   (16+1)                /* Length of filter for interpolation         */

#define OPL_DECIM    2                     /* Decimation in open-loop pitch analysis     */

#define PREEMPH_FAC  22282                 /* preemphasis factor (0.68 in Q15)           */
#define GAMMA1       30147                 /* Weighting factor (numerator) (0.92 in Q15) */
#define TILT_FAC     22282                 /* tilt factor (denominator) (0.68 in Q15)    */

#define Q_MAX        8                     /* scaling max for signal (see syn_filt_32)   */

#define RANDOM_INITSEED  21845             /* own random init value                      */

#define L_MEANBUF        3
#define ONE_PER_MEANBUF 10923

#define MODE_7k       0
#define MODE_9k       1
#define MODE_12k      2
#define MODE_14k      3
#define MODE_16k      4
#define MODE_18k      5
#define MODE_20k      6
#define MODE_23k      7
#define MODE_24k      8
#define MRDTX         9
#define NUM_OF_MODES  10                   /* see bits.h for bits definition             */

#define EHF_MASK (Word16)0x0008            /* homing frame pattern     

/*-------------------------------------------------------------------*
 *                         WB_VAD_C.H								 *
 *-------------------------------------------------------------------*
 * Constants for Voice Activity Detection.							 *
 *-------------------------------------------------------------------*/

#ifndef wb_vad_c_h
#define wb_vad_c_h

#define FRAME_LEN 256                      /* Length (samples) of the input frame          */
#define COMPLEN 12                         /* Number of sub-bands used by VAD              */

#define UNIRSHFT 7                         /* = log2(MAX_16/UNITY), UNITY = 256      */
#define SCALE 128                          /* (UNITY*UNITY)/512 */

#define TONE_THR (Word16)(0.65*MAX_16)     /* Threshold for tone detection   */

/* constants for speech level estimation */
#define SP_EST_COUNT 80
#define SP_ACTIVITY_COUNT 25
#define ALPHA_SP_UP (Word16)((1.0 - 0.85)*MAX_16)
#define ALPHA_SP_DOWN (Word16)((1.0 - 0.85)*MAX_16)

#define NOM_LEVEL 2050                     /* about -26 dBov Q15 */
#define SPEECH_LEVEL_INIT NOM_LEVEL        /* initial speech level */
#define MIN_SPEECH_LEVEL1  (Word16)(NOM_LEVEL * 0.063)  /* NOM_LEVEL -24 dB */
#define MIN_SPEECH_LEVEL2  (Word16)(NOM_LEVEL * 0.2)    /* NOM_LEVEL -14 dB */
#define MIN_SPEECH_SNR 4096                /* 0 dB, lowest SNR estimation, Q12 */

/* Time constants for background spectrum update */
#define ALPHA_UP1   (Word16)((1.0 - 0.95)*MAX_16)       /* Normal update, upwards:   */
#define ALPHA_DOWN1 (Word16)((1.0 - 0.936)*MAX_16)      /* Normal update, downwards  */
#define ALPHA_UP2   (Word16)((1.0 - 0.985)*MAX_16)      /* Forced update, upwards    */
#define ALPHA_DOWN2 (Word16)((1.0 - 0.943)*MAX_16)      /* Forced update, downwards  */
#define ALPHA3      (Word16)((1.0 - 0.95)*MAX_16)       /* Update downwards          */
#define ALPHA4      (Word16)((1.0 - 0.9)*MAX_16)        /* For stationary estimation */
#define ALPHA5      (Word16)((1.0 - 0.5)*MAX_16)        /* For stationary estimation */

/* Constants for VAD threshold */
#define THR_MIN  (Word16)(1.6*SCALE)       /* Minimum threshold               */
#define THR_HIGH (Word16)(6*SCALE)         /* Highest threshold               */
#define THR_LOW (Word16)(1.7*SCALE)        /* Lowest threshold               */
#define NO_P1 31744                        /* ilog2(1), Noise level for highest threshold */
#define NO_P2 19786                        /* ilog2(0.1*MAX_16), Noise level for lowest threshold */
#define NO_SLOPE (Word16)(MAX_16*(float)(THR_LOW-THR_HIGH)/(float)(NO_P2-NO_P1))

#define SP_CH_MIN (Word16)(-0.75*SCALE)
#define SP_CH_MAX (Word16)(0.75*SCALE)
#define SP_P1 22527                        /* ilog2(NOM_LEVEL/4) */
#define SP_P2 17832                        /* ilog2(NOM_LEVEL*4) */
#define SP_SLOPE (Word16)(MAX_16*(float)(SP_CH_MAX-SP_CH_MIN)/(float)(SP_P2-SP_P1))

/* Constants for hangover length */
#define HANG_HIGH  12                      /* longest hangover               */
#define HANG_LOW  2                        /* shortest hangover               */
#define HANG_P1 THR_LOW                    /* threshold for longest hangover */
#define HANG_P2 (Word16)(4*SCALE)          /* threshold for shortest hangover */
#define HANG_SLOPE (Word16)(MAX_16*(float)(HANG_LOW-HANG_HIGH)/(float)(HANG_P2-HANG_P1))

/* Constants for burst length */
#define BURST_HIGH 8                       /* longest burst length         */
#define BURST_LOW 3                        /* shortest burst length        */
#define BURST_P1 THR_HIGH                  /* threshold for longest burst */
#define BURST_P2 THR_LOW                   /* threshold for shortest burst */
#define BURST_SLOPE (Word16)(MAX_16*(float)(BURST_LOW-BURST_HIGH)/(float)(BURST_P2-BURST_P1))

/* Parameters for background spectrum recovery function */
#define STAT_COUNT 20                      /* threshold of stationary detection counter         */

#define STAT_THR_LEVEL 184                 /* Threshold level for stationarity detection        */
#define STAT_THR 1000                      /* Threshold for stationarity detection              */

/* Limits for background noise estimate */
#define NOISE_MIN 40                       /* minimum */
#define NOISE_MAX 20000                    /* maximum */
#define NOISE_INIT 150                     /* initial */

/* Thresholds for signal power (now calculated on 2 frames) */
#define VAD_POW_LOW (Word32)30000L         /* If input power is lower than this, VAD is set to 0 */
#define POW_TONE_THR (Word32)686080L       /* If input power is lower,tone detection flag is ignored */

/* Constants for the filter bank */
#define COEFF3   13363                     /* coefficient for the 3rd order filter     */
#define COEFF5_1 21955                     /* 1st coefficient the for 5th order filter */
#define COEFF5_2 6390                      /* 2nd coefficient the for 5th order filter */
#define F_5TH_CNT 5                        /* number of 5th order filters */
#define F_3TH_CNT 6                        /* number of 3th order filters */

#endif

/*-------------------------------------------------------------------*
 *                         WB_VAD.H                                  *
 *-------------------------------------------------------------------*
 * Functions and static memory for Voice Activity Detection.         *
 *-------------------------------------------------------------------*/

#ifndef wb_vad_h
#define wb_vad_h


/******************************************************************************
 *                         DEFINITION OF DATA TYPES
 ******************************************************************************/

typedef struct
{
    Word16 bckr_est[COMPLEN];              /* background noise estimate                */
    Word16 ave_level[COMPLEN];             /* averaged input components for stationary */
                                           /* estimation                               */
    Word16 old_level[COMPLEN];             /* input levels of the previous frame       */
    Word16 sub_level[COMPLEN];             /* input levels calculated at the end of a frame (lookahead)  */
    Word16 a_data5[F_5TH_CNT][2];          /* memory for the filter bank               */
    Word16 a_data3[F_3TH_CNT];             /* memory for the filter bank               */

    Word16 burst_count;                    /* counts length of a speech burst          */
    Word16 hang_count;                     /* hangover counter                         */
    Word16 stat_count;                     /* stationary counter                       */

    /* Note that each of the following two variables holds 15 flags. Each flag reserves 1 bit of the
     * variable. The newest flag is in the bit 15 (assuming that LSB is bit 1 and MSB is bit 16). */
    Word16 vadreg;                         /* flags for intermediate VAD decisions     */
    Word16 tone_flag;                      /* tone detection flags                     */

    Word16 sp_est_cnt;                     /* counter for speech level estimation      */
    Word16 sp_max;                         /* maximum level                            */
    Word16 sp_max_cnt;                     /* counts frames that contains speech       */
    Word16 speech_level;                   /* estimated speech level                   */
    Word32 prev_pow_sum;                   /* power of previous frame                  */

} VadVars;

/********************************************************************************
 *
 * DECLARATION OF PROTOTYPES
 ********************************************************************************/

Word16 wb_vad_init(VadVars ** st);
Word16 wb_vad_reset(VadVars * st);
void wb_vad_exit(VadVars ** st);
void wb_vad_tone_detection(VadVars * st, Word16 p_gain);
Word16 wb_vad(VadVars * st, Word16 in_buf[]);

#endif

/*--------------------------------------------------------------------------*
 *                         MATH_OP.H	                                    *
 *--------------------------------------------------------------------------*
 *       Mathematical operations					                        *
 *--------------------------------------------------------------------------*/

Word32 Isqrt(                              /* (o) Q31 : output value (range: 0<=val<1)         */
     Word32 L_x                            /* (i) Q0  : input value  (range: 0<=val<=7fffffff) */
);
void Isqrt_n(
     Word32 * frac,                        /* (i/o) Q31: normalized value (1.0 < frac <= 0.5) */
     Word16 * exp                          /* (i/o)    : exponent (value = frac x 2^exponent) */
);
Word32 Pow2(                               /* (o) Q0  : result       (range: 0<=val<=0x7fffffff) */
     Word16 exponant,                      /* (i) Q0  : Integer part.      (range: 0<=val<=30)   */
     Word16 fraction                       /* (i) Q15 : Fractionnal part.  (range: 0.0<=val<1.0) */
);
Word32 Dot_product12(                      /* (o) Q31: normalized result (1 < val <= -1) */
     Word16 x[],                           /* (i) 12bits: x vector                       */
     Word16 y[],                           /* (i) 12bits: y vector                       */
     Word16 lg,                            /* (i)    : vector length                     */
     Word16 * exp                          /* (o)    : exponent of result (0..+30)       */
);




