/*------------------------------------------------------------------------*
 *                         COD_MAIN.C                                     *
 *------------------------------------------------------------------------*
 * Performs the main encoder routine                                      *
 *------------------------------------------------------------------------*/

/*___________________________________________________________________________
 |                                                                           |
 | Fixed-point C simulation of AMR WB ACELP coding algorithm with 20 ms      |
 | speech frames for wideband speech signals.                                |
 |___________________________________________________________________________|
*/

#include <stdio.h>
#include <stdlib.h>

#include "typedef.h"
#include "basic_op.h"
#include "oper_32b.h"
#include "math_op.h"
#include "cnst.h"
#include "acelp.h"
#include "cod_main.h"
#include "bits.h"
#include "count.h"
#include "main.h"


/* LPC interpolation coef {0.45, 0.8, 0.96, 1.0}; in Q15 */
static Word16 interpol_frac[NB_SUBFR] = {14746, 26214, 31457, 32767};

/* isp tables for initialization */

static Word16 isp_init[M] =
{
   32138, 30274, 27246, 23170, 18205, 12540, 6393, 0,
   -6393, -12540, -18205, -23170, -27246, -30274, -32138, 1475
};

static Word16 isf_init[M] =
{
   1024, 2048, 3072, 4096, 5120, 6144, 7168, 8192,
   9216, 10240, 11264, 12288, 13312, 14336, 15360, 3840
};

/* High Band encoding */
static const Word16 HP_gain[16] =
{
   3624, 4673, 5597, 6479, 7425, 8378, 9324, 10264,
   11210, 12206, 13391, 14844, 16770, 19655, 24289, 32728
};


/*-----------------------------------------------------------------*
 *   Funtion  init_coder                                           *
 *            ~~~~~~~~~~                                           *
 *   ->Initialization of variables for the coder section.          *
 *-----------------------------------------------------------------*/

void Init_coder(void **spe_state)
{
    Coder_State *st;

    *spe_state = NULL;

    /*-------------------------------------------------------------------------*
     * Memory allocation for coder state.                                      *
     *-------------------------------------------------------------------------*/

    if ((st = (Coder_State *) malloc(sizeof(Coder_State))) == NULL)
    {
        printf("Can not malloc Coder_State structure!\n");
        return;
    }
    st->vadSt = NULL;                      
    st->dtx_encSt = NULL;                  

    wb_vad_init(&(st->vadSt));
    dtx_enc_init(&(st->dtx_encSt), isf_init);

    Reset_encoder((void *) st, 1);

    *spe_state = (void *) st;

    return;
}


void Reset_encoder(void *st, Word16 reset_all)
{
    Word16 i;

    Coder_State *cod_state;

    cod_state = (Coder_State *) st;

    Set_zero(cod_state->old_exc, PIT_MAX + L_INTERPOL);
    Set_zero(cod_state->mem_syn, M);
    Set_zero(cod_state->past_isfq, M);

    cod_state->mem_w0 = 0;                 
    cod_state->tilt_code = 0;              
    cod_state->first_frame = 1;            

    Init_gp_clip(cod_state->gp_clip);

    cod_state->L_gc_thres = 0;             

    if (reset_all != 0)
    {
        /* Static vectors to zero */

        Set_zero(cod_state->old_speech, L_TOTAL - L_FRAME);
        Set_zero(cod_state->old_wsp, (PIT_MAX / OPL_DECIM));
        Set_zero(cod_state->mem_decim2, 3);

        /* routines initialization */

        Init_Decim_12k8(cod_state->mem_decim);
        Init_HP50_12k8(cod_state->mem_sig_in);
        Init_Levinson(cod_state->mem_levinson);
        Init_Q_gain2(cod_state->qua_gain);
        Init_Hp_wsp(cod_state->hp_wsp_mem);

        /* isp initialization */

        Copy(isp_init, cod_state->ispold, M);
        Copy(isp_init, cod_state->ispold_q, M);

        /* variable initialization */

        cod_state->mem_preemph = 0;        
        cod_state->mem_wsp = 0;            
        cod_state->Q_old = 15;             
        cod_state->Q_max[0] = 15;          
        cod_state->Q_max[1] = 15;          
        cod_state->old_wsp_max = 0;        
        cod_state->old_wsp_shift = 0;      

        /* pitch ol initialization */

        cod_state->old_T0_med = 40;        
        cod_state->ol_gain = 0;            
        cod_state->ada_w = 0;              
        cod_state->ol_wght_flg = 0;        
        for (i = 0; i < 5; i++)
        {
            cod_state->old_ol_lag[i] = 40; 
        }
        Set_zero(cod_state->old_hp_wsp, (L_FRAME / 2) / OPL_DECIM + (PIT_MAX / OPL_DECIM));

        Set_zero(cod_state->mem_syn_hf, M);
        Set_zero(cod_state->mem_syn_hi, M);
        Set_zero(cod_state->mem_syn_lo, M);

        Init_HP50_12k8(cod_state->mem_sig_out);
        Init_Filt_6k_7k(cod_state->mem_hf);
        Init_HP400_12k8(cod_state->mem_hp400);

        Copy(isf_init, cod_state->isfold, M);

        cod_state->mem_deemph = 0;         

        cod_state->seed2 = 21845;          

        Init_Filt_6k_7k(cod_state->mem_hf2);
        cod_state->gain_alpha = 32767;     

        cod_state->vad_hist = 0;

        wb_vad_reset(cod_state->vadSt);
        dtx_enc_reset(cod_state->dtx_encSt, isf_init);
    }
    return;
}

void Close_coder(void *spe_state)
{
    wb_vad_exit(&(((Coder_State *) spe_state)->vadSt));
    dtx_enc_exit(&(((Coder_State *) spe_state)->dtx_encSt));
    free(spe_state);

    return;
}

/*-----------------------------------------------------------------*
 *   Funtion  coder                                                *
 *            ~~~~~                                                *
 *   ->Main coder routine.                                         *
 *                                                                 *
 *-----------------------------------------------------------------*/

void coder(
     Word16 speech16k[],                   /* input :  320 new speech samples (at 16 kHz)    */
     Word16 * ser_size,                    /* output:  bit rate of the used mode             */
     void *spe_state,                      /* i/o   :  State structure                       */
     Word16 *flag_VAD					   /* VAD_flag										*/
)
{

    /* Coder states */
    Coder_State *st;

    /* Speech vector */
    Word16 old_speech[L_TOTAL];
    Word16 *new_speech, *speech, *p_window;

    /* Weighted speech vector */
    Word16 old_wsp[L_FRAME + (PIT_MAX / OPL_DECIM)];
    Word16 *wsp;

    /* LPC coefficients */

    Word16 r_h[M + 1], r_l[M + 1];         /* Autocorrelations of windowed speech  */
    Word16 rc[M];                          /* Reflection coefficients.             */
    Word16 Ap[M + 1];                      /* A(z) with spectral expansion         */
    Word16 ispnew[M];                      /* immittance spectral pairs at 4nd sfr */
    Word16 isf[M];                         /* ISF (frequency domain) at 4nd sfr    */
    Word16 *p_A;                    /* ptr to A(z) for the 4 subframes      */
    Word16 A[NB_SUBFR * (M + 1)];          /* A(z) unquantized for the 4 subframes */

    /* Other vectors */

    Word16 code[L_SUBFR];                  /* Fixed codebook excitation          */
    Word16 error[M + L_SUBFR];             /* error of quantization              */
    Word16 buf[L_FRAME];                   /* VAD buffer                         */

    /* Scalars */

    Word16 i, i_subfr, vad_flag;
    Word16 T_op, T_op2;
    Word16 tmp, exp, Q_new, mu, shift, max;

    Word32 L_tmp, L_max;



    st = (Coder_State *) spe_state;

    *ser_size = nb_of_bits[0];         

    /*--------------------------------------------------------------------------*
     *          Initialize pointers to speech vector.                           *
     *                                                                          *
     *                                                                          *
     *                    |-------|-------|-------|-------|-------|-------|     *
     *                     past sp   sf1     sf2     sf3     sf4    L_NEXT      *
     *                    <-------  Total speech buffer (L_TOTAL)   ------>     *
     *              old_speech                                                  *
     *                    <-------  LPC analysis window (L_WINDOW)  ------>     *
     *                    |       <-- present frame (L_FRAME) ---->             *
     *                   p_window |       <----- new speech (L_FRAME) ---->     *
     *                            |       |                                     *
     *                          speech    |                                     *
     *                                 new_speech                               *
     *--------------------------------------------------------------------------*/

    new_speech = old_speech + L_TOTAL - L_FRAME - L_FILT;         /* New speech     */
    speech = old_speech + L_TOTAL - L_FRAME - L_NEXT;     /* Present frame  */
    p_window = old_speech + L_TOTAL - L_WINDOW; 

    wsp = old_wsp + (PIT_MAX / OPL_DECIM); 

    /* copy coder memory state into working space (internal memory for DSP) */

    Copy(st->old_speech, old_speech, L_TOTAL - L_FRAME);
    Copy(st->old_wsp, old_wsp, PIT_MAX / OPL_DECIM);

    /*---------------------------------------------------------------*
     * Down sampling signal from 16kHz to 12.8kHz                    *
     * -> The signal is extended by L_FILT samples (padded to zero)  *
     * to avoid additional delay (L_FILT samples) in the coder.      *
     * The last L_FILT samples are approximated after decimation and *
     * are used (and windowed) only in autocorrelations.             *
     *---------------------------------------------------------------*/

    Decim_12k8(speech16k, L_FRAME16k, new_speech, st->mem_decim);

    /* last L_FILT samples for autocorrelation window */
    Copy(st->mem_decim, code, 2 * L_FILT16k);
    Set_zero(error, L_FILT16k);            /* set next sample to zero */
    Decim_12k8(error, L_FILT16k, new_speech + L_FRAME, code);

    /*---------------------------------------------------------------*
     * Perform 50Hz HP filtering of input signal.                    *
     *---------------------------------------------------------------*/

    HP50_12k8(new_speech, L_FRAME, st->mem_sig_in);

    /* last L_FILT samples for autocorrelation window */
    Copy(st->mem_sig_in, code, 6);
    HP50_12k8(new_speech + L_FRAME, L_FILT, code);

    /*---------------------------------------------------------------*
     * Perform fixed preemphasis through 1 - g z^-1                  *
     * Scale signal to get maximum of precision in filtering         *
     *---------------------------------------------------------------*/

    mu = shr(PREEMPH_FAC, 1);              /* Q15 --> Q14 */

    /* get max of new preemphased samples (L_FRAME+L_FILT) */

    L_tmp = L_mult(new_speech[0], 16384);
    L_tmp = L_msu(L_tmp, st->mem_preemph, mu);
    L_max = L_abs(L_tmp);

    for (i = 1; i < L_FRAME + L_FILT; i++)
    {
        L_tmp = L_mult(new_speech[i], 16384);
        L_tmp = L_msu(L_tmp, new_speech[i - 1], mu);
        L_tmp = L_abs(L_tmp);
        
        if (L_sub(L_tmp, L_max) > (Word32) 0)
        {
            L_max = L_tmp;                 
        }
    }

    /* get scaling factor for new and previous samples */
    /* limit scaling to Q_MAX to keep dynamic for ringing in low signal */
    /* limit scaling to Q_MAX also to avoid a[0]<1 in syn_filt_32 */
    tmp = extract_h(L_max);
    
    if (tmp == 0)
    {
        shift = Q_MAX;                     
    } else
    {
        shift = sub(norm_s(tmp), 1);
        
        if (shift < 0)
        {
            shift = 0;                     
        }
        
        if (sub(shift, Q_MAX) > 0)
        {
            shift = Q_MAX;                 
        }
    }
    Q_new = shift;                         
    
    if (sub(Q_new, st->Q_max[0]) > 0)
    {
        Q_new = st->Q_max[0];              
    }
    
    if (sub(Q_new, st->Q_max[1]) > 0)
    {
        Q_new = st->Q_max[1];              
    }
    exp = sub(Q_new, st->Q_old);
    st->Q_old = Q_new;                     
    st->Q_max[1] = st->Q_max[0];           
    st->Q_max[0] = shift;                  

    /* preemphasis with scaling (L_FRAME+L_FILT) */

    tmp = new_speech[L_FRAME - 1];         

    for (i = L_FRAME + L_FILT - 1; i > 0; i--)
    {
        L_tmp = L_mult(new_speech[i], 16384);
        L_tmp = L_msu(L_tmp, new_speech[i - 1], mu);
        L_tmp = L_shl(L_tmp, Q_new);
        new_speech[i] = round(L_tmp);      
    }

    L_tmp = L_mult(new_speech[0], 16384);
    L_tmp = L_msu(L_tmp, st->mem_preemph, mu);
    L_tmp = L_shl(L_tmp, Q_new);
    new_speech[0] = round(L_tmp);          

    st->mem_preemph = tmp;                 

    /* scale previous samples and memory */

    Scale_sig(old_speech, L_TOTAL - L_FRAME - L_FILT, exp);
    Scale_sig(st->mem_syn, M, exp);
    Scale_sig(st->mem_decim2, 3, exp);
    Scale_sig(&(st->mem_wsp), 1, exp);
    Scale_sig(&(st->mem_w0), 1, exp);

    /*------------------------------------------------------------------------*
     *  Call VAD                                                              *
     *  Preemphesis scale down signal in low frequency and keep dynamic in HF.*
     *  Vad work slightly in futur (new_speech = speech + L_NEXT - L_FILT).   *
     *------------------------------------------------------------------------*/

    Copy(new_speech, buf, L_FRAME);

    Scale_sig(buf, L_FRAME, sub(1, Q_new));

    vad_flag = wb_vad(st->vadSt, buf);

	/* Export flag_VAD */
	if (vad_flag == 0)
		{
		*flag_VAD = 0; 		// 16 bit 0
		}
	else
		{
		*flag_VAD = -1;		// 16 bit 1   
		}
    if (vad_flag == 0)
    {
        st->vad_hist = add(st->vad_hist, 1);        
    } else
    {
        st->vad_hist = 0;              
    }

    /*------------------------------------------------------------------------*
     *  Perform LPC analysis                                                  *
     *  ~~~~~~~~~~~~~~~~~~~~                                                  *
     *   - autocorrelation + lag windowing                                    *
     *   - Levinson-durbin algorithm to find a[]                              *
     *   - convert a[] to isp[]                                               *
     *   - convert isp[] to isf[] for quantization                            *
     *   - quantize and code the isf[]                                        *
     *   - convert isf[] to isp[] for interpolation                           *
     *   - find the interpolated ISPs and convert to a[] for the 4 subframes  *
     *------------------------------------------------------------------------*/

    /* LP analysis centered at 4nd subframe */
    Autocorr(p_window, M, r_h, r_l);       /* Autocorrelations */
    Lag_window(r_h, r_l);                  /* Lag windowing    */
    Levinson(r_h, r_l, A, rc, st->mem_levinson);        /* Levinson Durbin  */
    Az_isp(A, ispnew, st->ispold);         /* From A(z) to ISP */

    /* Find the interpolated ISPs and convert to a[] for all subframes */
    Int_isp(st->ispold, ispnew, interpol_frac, A);

    /* update ispold[] for the next frame */
    Copy(ispnew, st->ispold, M);

    /* Convert ISPs to frequency domain 0..6400 */
    Isp_isf(ispnew, isf, M);


    /*----------------------------------------------------------------------*
     *  Perform PITCH_OL analysis                                           *
     *  ~~~~~~~~~~~~~~~~~~~~~~~~~                                           *
     * - Find the residual res[] for the whole speech frame                 *
     * - Find the weighted input speech wsp[] for the whole speech frame    *
     * - scale wsp[] to avoid overflow in pitch estimation                  *
     * - Find open loop pitch lag for whole speech frame                    *
     *----------------------------------------------------------------------*/

    p_A = A;                               
    for (i_subfr = 0; i_subfr < L_FRAME; i_subfr += L_SUBFR)
    {
        Weight_a(p_A, Ap, GAMMA1, M);
        Residu(Ap, M, &speech[i_subfr], &wsp[i_subfr], L_SUBFR);
        p_A += (M + 1);                    
    }
    Deemph2(wsp, TILT_FAC, L_FRAME, &(st->mem_wsp));

    /* find maximum value on wsp[] for 12 bits scaling */
    max = 0;                               
    for (i = 0; i < L_FRAME; i++)
    {
        tmp = abs_s(wsp[i]);
        
        if (sub(tmp, max) > 0)
        {
            max = tmp;                     
        }
    }
    tmp = st->old_wsp_max;                 
    
    if (sub(max, tmp) > 0)
    {
        tmp = max;                         /* tmp = max(wsp_max, old_wsp_max) */
        
    }
    st->old_wsp_max = max;                 

    shift = sub(norm_s(tmp), 3);
    
    if (shift > 0)
    {
        shift = 0;                         /* shift = 0..-3 */
        
    }
    /* decimation of wsp[] to search pitch in LF and to reduce complexity */
    LP_Decim2(wsp, L_FRAME, st->mem_decim2);

    /* scale wsp[] in 12 bits to avoid overflow */
    Scale_sig(wsp, L_FRAME / OPL_DECIM, shift);

    /* scale old_wsp (warning: exp must be Q_new-Q_old) */
    exp = add(exp, sub(shift, st->old_wsp_shift));
    st->old_wsp_shift = shift;
    Scale_sig(old_wsp, PIT_MAX / OPL_DECIM, exp);
    Scale_sig(st->old_hp_wsp, PIT_MAX / OPL_DECIM, exp);
    scale_mem_Hp_wsp(st->hp_wsp_mem, exp);

    /* Find open loop pitch lag for whole speech frame */

    
    if (sub(*ser_size, NBBITS_7k) == 0)
    {
        /* Find open loop pitch lag for whole speech frame */
        T_op = Pitch_med_ol(wsp, PIT_MIN / OPL_DECIM, PIT_MAX / OPL_DECIM,
            L_FRAME / OPL_DECIM, st->old_T0_med, &(st->ol_gain), st->hp_wsp_mem, st->old_hp_wsp, st->ol_wght_flg);
    } else
    {
        /* Find open loop pitch lag for first 1/2 frame */
        T_op = Pitch_med_ol(wsp, PIT_MIN / OPL_DECIM, PIT_MAX / OPL_DECIM,
            (L_FRAME / 2) / OPL_DECIM, st->old_T0_med, &(st->ol_gain), st->hp_wsp_mem, st->old_hp_wsp, st->ol_wght_flg);
    }

    
    if (sub(st->ol_gain, 19661) > 0)       /* 0.6 in Q15 */
    {
        st->old_T0_med = Med_olag(T_op, st->old_ol_lag);        
        st->ada_w = 32767;                 
    } else
    {
        st->ada_w = mult(st->ada_w, 29491);
    }

    
    if (sub(st->ada_w, 26214) < 0)
        st->ol_wght_flg = 0;
    else
        st->ol_wght_flg = 1;

    wb_vad_tone_detection(st->vadSt, st->ol_gain);

    T_op *= OPL_DECIM;                     

    
    if (sub(*ser_size, NBBITS_7k) != 0)
    {
        /* Find open loop pitch lag for second 1/2 frame */
        T_op2 = Pitch_med_ol(wsp + ((L_FRAME / 2) / OPL_DECIM), PIT_MIN / OPL_DECIM, PIT_MAX / OPL_DECIM,
            (L_FRAME / 2) / OPL_DECIM, st->old_T0_med, &(st->ol_gain), st->hp_wsp_mem, st->old_hp_wsp, st->ol_wght_flg);

        
        if (sub(st->ol_gain, 19661) > 0)   /* 0.6 in Q15 */
        {
            st->old_T0_med = Med_olag(T_op2, st->old_ol_lag);   
            st->ada_w = 32767;             
        } else
        {
            st->ada_w = mult(st->ada_w, 29491); 
        }

        
        if (sub(st->ada_w, 26214) < 0)
            st->ol_wght_flg = 0;
        else
            st->ol_wght_flg = 1;

        wb_vad_tone_detection(st->vadSt, st->ol_gain);

        T_op2 *= OPL_DECIM;                

    } else
    {
        T_op2 = T_op;                      
    }



    /*--------------------------------------------------*
     * Update signal for next frame.                    *
     * -> save past of speech[] and wsp[].              *
     *--------------------------------------------------*/

    Copy(&old_speech[L_FRAME], st->old_speech, L_TOTAL - L_FRAME);
    Copy(&old_wsp[L_FRAME / OPL_DECIM], st->old_wsp, PIT_MAX / OPL_DECIM);


    return;
}
/*___________________________________________________________________________
 |                                                                           |
 | Basic arithmetic operators.                                               |
 |                                                                           |
 |___________________________________________________________________________|
*/


/*___________________________________________________________________________
 |                                                                           |
 |   Local Functions                                                         |
 |___________________________________________________________________________|
*/
Word16 saturate (Word32 L_var1);

/*___________________________________________________________________________
 |                                                                           |
 |   Constants and Globals                                                   |
 |___________________________________________________________________________|
*/
Flag Overflow = 0;
Flag Carry = 0;

/*___________________________________________________________________________
 |                                                                           |
 |   Functions                                                               |
 |___________________________________________________________________________|
*/

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : saturate                                                |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |    Limit the 32 bit input to the range of a 16 bit word.                  |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1                                                                 |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var1 <= 0x7fff ffff.                 |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 
saturate (Word32 L_var1)
{
    Word16 var_out;

    if (L_var1 > 0X00007fffL)
    {
        Overflow = 1;
        var_out = MAX_16;
    }
    else if (L_var1 < (Word32) 0xffff8000L)
    {
        Overflow = 1;
        var_out = MIN_16;
    }
    else
    {
        var_out = extract_l (L_var1);


    }

    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : add                                                     |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |    Performs the addition (var1+var2) with overflow control and saturation;|
 |    the 16 bit result is set at +32767 when overflow occurs or at -32768   |
 |    when underflow occurs.                                                 |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 add (Word16 var1, Word16 var2)
{
    Word16 var_out;
    Word32 L_sum;

    L_sum = (Word32) var1 + var2;
    var_out = saturate (L_sum);


    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : sub                                                     |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |    Performs the subtraction (var1+var2) with overflow control and satu-   |
 |    ration; the 16 bit result is set at +32767 when overflow occurs or at  |
 |    -32768 when underflow occurs.                                          |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 sub (Word16 var1, Word16 var2)
{
    Word16 var_out;
    Word32 L_diff;

    L_diff = (Word32) var1 - var2;
    var_out = saturate (L_diff);


    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : abs_s                                                   |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |    Absolute value of var1; abs_s(-32768) = 32767.                         |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0x0000 0000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 abs_s (Word16 var1)
{
    Word16 var_out;

    if (var1 == MIN_16)
    {
        var_out = MAX_16;
    }
    else
    {
        if (var1 < 0)
        {
            var_out = (Word16)-var1;
        }
        else
        {
            var_out = var1;
        }
    }


    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : shl                                                     |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Arithmetically shift the 16 bit input var1 left var2 positions.Zero fill|
 |   the var2 LSB of the result. If var2 is negative, arithmetically shift   |
 |   var1 right by -var2 with sign extension. Saturate the result in case of |
 |   underflows or overflows.                                                |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 shl (Word16 var1, Word16 var2)
{
    Word16 var_out;
    Word32 result;

    if (var2 < 0)
    {
        if (var2 < -16)
            var2 = -16;
        var_out = shr (var1, (Word16)-var2);


    }
    else
    {
        result = (Word32) var1 *((Word32) 1 << var2);

        if ((var2 > 15 && var1 != 0) || (result != (Word32) ((Word16) result)))
        {
            Overflow = 1;
            var_out = (Word16)((var1 > 0) ? MAX_16 : MIN_16);
        }
        else
        {
            var_out = extract_l (result);


        }
    }


    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : shr                                                     |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Arithmetically shift the 16 bit input var1 right var2 positions with    |
 |   sign extension. If var2 is negative, arithmetically shift var1 left by  |
 |   -var2 with sign extension. Saturate the result in case of underflows or |
 |   overflows.                                                              |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 shr (Word16 var1, Word16 var2)
{
    Word16 var_out;

    if (var2 < 0)
    {
        if (var2 < -16)
            var2 = -16;
        var_out = shl (var1, (Word16)-var2);


    }
    else
    {
        if (var2 >= 15)
        {
            var_out = (Word16)((var1 < 0) ? -1 : 0);
        }
        else
        {
            if (var1 < 0)
            {
                var_out = (Word16)(~((~var1) >> var2));
            }
            else
            {
                var_out = (Word16)(var1 >> var2);
            }
        }
    }



    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : mult                                                    |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |    Performs the multiplication of var1 by var2 and gives a 16 bit result  |
 |    which is scaled i.e.:                                                  |
 |             mult(var1,var2) = extract_l(L_shr((var1 times var2),15)) and  |
 |             mult(-32768,-32768) = 32767.                                  |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 mult (Word16 var1, Word16 var2)
{
    Word16 var_out;
    Word32 L_product;

    L_product = (Word32) var1 *(Word32) var2;

    L_product = (L_product & (Word32) 0xffff8000L) >> 15;

    if (L_product & (Word32) 0x00010000L)
        L_product = L_product | (Word32) 0xffff0000L;

    var_out = saturate (L_product);


    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_mult                                                  |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   L_mult is the 32 bit result of the multiplication of var1 times var2    |
 |   with one shift left i.e.:                                               |
 |        L_mult(var1,var2) = L_shl((var1 times var2),1) and                   |
 |        L_mult(-32768,-32768) = 2147483647.                                |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var_out <= 0x7fff ffff.              |
 |___________________________________________________________________________|
*/

Word32 L_mult (Word16 var1, Word16 var2)
{
    Word32 L_var_out;

    L_var_out = (Word32) var1 *(Word32) var2;

    if (L_var_out != (Word32) 0x40000000L)
    {
        L_var_out *= 2;
    }
    else
    {
        Overflow = 1;
        L_var_out = MAX_32;
    }



    return (L_var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : negate                                                  |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Negate var1 with saturation, saturate in the case where input is -32768:|
 |                negate(var1) = sub(0,var1).                                |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 negate (Word16 var1)
{
    Word16 var_out;

    var_out = (Word16)((var1 == MIN_16) ? MAX_16 : -var1);


    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : extract_h                                               |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Return the 16 MSB of L_var1.                                            |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1                                                                 |
 |             32 bit long signed integer (Word32 ) whose value falls in the |
 |             range : 0x8000 0000 <= L_var1 <= 0x7fff ffff.                 |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 extract_h (Word32 L_var1)
{
    Word16 var_out;

    var_out = (Word16) (L_var1 >> 16);


    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : extract_l                                               |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Return the 16 LSB of L_var1.                                            |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1                                                                 |
 |             32 bit long signed integer (Word32 ) whose value falls in the |
 |             range : 0x8000 0000 <= L_var1 <= 0x7fff ffff.                 |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 extract_l (Word32 L_var1)
{
    Word16 var_out;

    var_out = (Word16) L_var1;


    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : round                                                   |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Round the lower 16 bits of the 32 bit input number into the MS 16 bits  |
 |   with saturation. Shift the resulting bits right by 16 and return the 16 |
 |   bit number:                                                             |
 |               round(L_var1) = extract_h(L_add(L_var1,32768))              |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1                                                                 |
 |             32 bit long signed integer (Word32 ) whose value falls in the |
 |             range : 0x8000 0000 <= L_var1 <= 0x7fff ffff.                 |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 round (Word32 L_var1)
{
    Word16 var_out;
    Word32 L_rounded;

    L_rounded = L_add (L_var1, (Word32) 0x00008000L);


    var_out = extract_h (L_rounded);


    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_mac                                                   |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Multiply var1 by var2 and shift the result left by 1. Add the 32 bit    |
 |   result to L_var3 with saturation, return a 32 bit result:               |
 |        L_mac(L_var3,var1,var2) = L_add(L_var3,L_mult(var1,var2)).         |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var3   32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var3 <= 0x7fff ffff.                 |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var_out <= 0x7fff ffff.              |
 |___________________________________________________________________________|
*/

Word32 L_mac (Word32 L_var3, Word16 var1, Word16 var2)
{
    Word32 L_var_out;
    Word32 L_product;

    L_product = L_mult (var1, var2);


    L_var_out = L_add (L_var3, L_product);


    return (L_var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_msu                                                   |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Multiply var1 by var2 and shift the result left by 1. Subtract the 32   |
 |   bit result to L_var3 with saturation, return a 32 bit result:           |
 |        L_msu(L_var3,var1,var2) = L_sub(L_var3,L_mult(var1,var2)).         |
 |                                                                           |
 |   Complexity weight : 1                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var3   32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var3 <= 0x7fff ffff.                 |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var_out <= 0x7fff ffff.              |
 |___________________________________________________________________________|
*/

Word32 L_msu (Word32 L_var3, Word16 var1, Word16 var2)
{
    Word32 L_var_out;
    Word32 L_product;

    L_product = L_mult (var1, var2);


    L_var_out = L_sub (L_var3, L_product);


    return (L_var_out);
}





/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_add                                                   |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   32 bits addition of the two 32 bits variables (L_var1+L_var2) with      |
 |   overflow control and saturation; the result is set at +2147483647 when  |
 |   overflow occurs or at -2147483648 when underflow occurs.                |
 |                                                                           |
 |   Complexity weight : 2                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1   32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var3 <= 0x7fff ffff.                 |
 |                                                                           |
 |    L_var2   32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var3 <= 0x7fff ffff.                 |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var_out <= 0x7fff ffff.              |
 |___________________________________________________________________________|
*/

Word32 L_add (Word32 L_var1, Word32 L_var2)
{
    Word32 L_var_out;

    L_var_out = L_var1 + L_var2;

    if (((L_var1 ^ L_var2) & MIN_32) == 0)
    {
        if ((L_var_out ^ L_var1) & MIN_32)
        {
            L_var_out = (L_var1 < 0) ? MIN_32 : MAX_32;
            Overflow = 1;
        }
    }


    return (L_var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_sub                                                   |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   32 bits subtraction of the two 32 bits variables (L_var1-L_var2) with   |
 |   overflow control and saturation; the result is set at +2147483647 when  |
 |   overflow occurs or at -2147483648 when underflow occurs.                |
 |                                                                           |
 |   Complexity weight : 2                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1   32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var3 <= 0x7fff ffff.                 |
 |                                                                           |
 |    L_var2   32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var3 <= 0x7fff ffff.                 |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var_out <= 0x7fff ffff.              |
 |___________________________________________________________________________|
*/

Word32 L_sub (Word32 L_var1, Word32 L_var2)
{
    Word32 L_var_out;

    L_var_out = L_var1 - L_var2;

    if (((L_var1 ^ L_var2) & MIN_32) != 0)
    {
        if ((L_var_out ^ L_var1) & MIN_32)
        {
            L_var_out = (L_var1 < 0L) ? MIN_32 : MAX_32;
            Overflow = 1;
        }
    }


    return (L_var_out);
}




/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_negate                                                |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Negate the 32 bit variable L_var1 with saturation; saturate in the case |
 |   where input is -2147483648 (0x8000 0000).                               |
 |                                                                           |
 |   Complexity weight : 2                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1   32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var3 <= 0x7fff ffff.                 |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var_out <= 0x7fff ffff.              |
 |___________________________________________________________________________|
*/

Word32 L_negate (Word32 L_var1)
{
    Word32 L_var_out;

    L_var_out = (L_var1 == MIN_32) ? MAX_32 : -L_var1;


    return (L_var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : mult_r                                                  |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Same as mult with rounding, i.e.:                                       |
 |     mult_r(var1,var2) = extract_l(L_shr(((var1 * var2) + 16384),15)) and  |
 |     mult_r(-32768,-32768) = 32767.                                        |
 |                                                                           |
 |   Complexity weight : 2                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 mult_r (Word16 var1, Word16 var2)
{
    Word16 var_out;
    Word32 L_product_arr;

    L_product_arr = (Word32) var1 *(Word32) var2;       /* product */
    L_product_arr += (Word32) 0x00004000L;      /* round */
    L_product_arr &= (Word32) 0xffff8000L;
    L_product_arr >>= 15;       /* shift */

    if (L_product_arr & (Word32) 0x00010000L)   /* sign extend when necessary */
    {
        L_product_arr |= (Word32) 0xffff0000L;
    }
    var_out = saturate (L_product_arr);


    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_shl                                                   |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Arithmetically shift the 32 bit input L_var1 left var2 positions. Zero  |
 |   fill the var2 LSB of the result. If var2 is negative, arithmetically    |
 |   shift L_var1 right by -var2 with sign extension. Saturate the result in |
 |   case of underflows or overflows.                                        |
 |                                                                           |
 |   Complexity weight : 2                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1   32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var3 <= 0x7fff ffff.                 |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var_out <= 0x7fff ffff.              |
 |___________________________________________________________________________|
*/

Word32 L_shl (Word32 L_var1, Word16 var2)
{
    Word32 L_var_out = 0L;

    if (var2 <= 0)
    {
        if (var2 < -32)
            var2 = -32;
        L_var_out = L_shr (L_var1, (Word16)-var2);


    }
    else
    {
        for (; var2 > 0; var2--)
        {
            if (L_var1 > (Word32) 0X3fffffffL)
            {
                Overflow = 1;
                L_var_out = MAX_32;
                break;
            }
            else
            {
                if (L_var1 < (Word32) 0xc0000000L)
                {
                    Overflow = 1;
                    L_var_out = MIN_32;
                    break;
                }
            }
            L_var1 *= 2;
            L_var_out = L_var1;
        }
    }


    return (L_var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_shr                                                   |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Arithmetically shift the 32 bit input L_var1 right var2 positions with  |
 |   sign extension. If var2 is negative, arithmetically shift L_var1 left   |
 |   by -var2 and zero fill the -var2 LSB of the result. Saturate the result |
 |   in case of underflows or overflows.                                     |
 |                                                                           |
 |   Complexity weight : 2                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1   32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var3 <= 0x7fff ffff.                 |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= L_var_out <= 0x7fff ffff.              |
 |___________________________________________________________________________|
*/

Word32 L_shr (Word32 L_var1, Word16 var2)
{
    Word32 L_var_out;

    if (var2 < 0)
    {
        if (var2 < -32)
            var2 = -32;
        L_var_out = L_shl (L_var1, (Word16)-var2);


    }
    else
    {
        if (var2 >= 31)
        {
            L_var_out = (L_var1 < 0L) ? -1 : 0;
        }
        else
        {
            if (L_var1 < 0)
            {
                L_var_out = ~((~L_var1) >> var2);
            }
            else
            {
                L_var_out = L_var1 >> var2;
            }
        }
    }


    return (L_var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : shr_r                                                   |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Same as shr(var1,var2) but with rounding. Saturate the result in case of|
 |   underflows or overflows :                                               |
 |    - If var2 is greater than zero :                                       |
 |          if (sub(shl(shr(var1,var2),1),shr(var1,sub(var2,1))))            |
 |          is equal to zero                                                 |
 |                     then                                                  |
 |                     shr_r(var1,var2) = shr(var1,var2)                     |
 |                     else                                                  |
 |                     shr_r(var1,var2) = add(shr(var1,var2),1)              |
 |    - If var2 is less than or equal to zero :                              |
 |                     shr_r(var1,var2) = shr(var1,var2).                    |
 |                                                                           |
 |   Complexity weight : 2                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word16 shr_r (Word16 var1, Word16 var2)
{
    Word16 var_out;

    if (var2 > 15)
    {
        var_out = 0;
    }
    else
    {
        var_out = shr (var1, var2);



        if (var2 > 0)
        {
            if ((var1 & ((Word16) 1 << (var2 - 1))) != 0)
            {
                var_out++;
            }
        }
    }


    return (var_out);
}



/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_deposit_h                                             |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Deposit the 16 bit var1 into the 16 MS bits of the 32 bit output. The   |
 |   16 LS bits of the output are zeroed.                                    |
 |                                                                           |
 |   Complexity weight : 2                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= var_out <= 0x7fff 0000.                |
 |___________________________________________________________________________|
*/

Word32 L_deposit_h (Word16 var1)
{
    Word32 L_var_out;

    L_var_out = (Word32) var1 << 16;


    return (L_var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_deposit_l                                             |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Deposit the 16 bit var1 into the 16 LS bits of the 32 bit output. The   |
 |   16 MS bits of the output are sign extended.                             |
 |                                                                           |
 |   Complexity weight : 2                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0xFFFF 8000 <= var_out <= 0x0000 7fff.                |
 |___________________________________________________________________________|
*/

Word32 L_deposit_l (Word16 var1)
{
    Word32 L_var_out;

    L_var_out = (Word32) var1;


    return (L_var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_shr_r                                                 |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Same as L_shr(L_var1,var2) but with rounding. Saturate the result in    |
 |   case of underflows or overflows :                                       |
 |    - If var2 is greater than zero :                                       |
 |          if (L_sub(L_shl(L_shr(L_var1,var2),1),L_shr(L_var1,sub(var2,1))))|
 |          is equal to zero                                                 |
 |                     then                                                  |
 |                     L_shr_r(L_var1,var2) = L_shr(L_var1,var2)             |
 |                     else                                                  |
 |                     L_shr_r(L_var1,var2) = L_add(L_shr(L_var1,var2),1)    |
 |    - If var2 is less than or equal to zero :                              |
 |                     L_shr_r(L_var1,var2) = L_shr(L_var1,var2).            |
 |                                                                           |
 |   Complexity weight : 3                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1                                                                 |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= var1 <= 0x7fff ffff.                   |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= var_out <= 0x7fff ffff.                |
 |___________________________________________________________________________|
*/

Word32 L_shr_r (Word32 L_var1, Word16 var2)
{
    Word32 L_var_out;

    if (var2 > 31)
    {
        L_var_out = 0;
    }
    else
    {
        L_var_out = L_shr (L_var1, var2);


        if (var2 > 0)
        {
            if ((L_var1 & ((Word32) 1 << (var2 - 1))) != 0)
            {
                L_var_out++;
            }
        }
    }


    return (L_var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : L_abs                                                   |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |    Absolute value of L_var1; Saturate in case where the input is          |
 |                                                               -214783648  |
 |                                                                           |
 |   Complexity weight : 3                                                   |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1                                                                 |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= var1 <= 0x7fff ffff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    L_var_out                                                              |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x0000 0000 <= var_out <= 0x7fff ffff.                |
 |___________________________________________________________________________|
*/

Word32 L_abs (Word32 L_var1)
{
    Word32 L_var_out;

    if (L_var1 == MIN_32)
    {
        L_var_out = MAX_32;
    }
    else
    {
        if (L_var1 < 0)
        {
            L_var_out = -L_var1;
        }
        else
        {
            L_var_out = L_var1;
        }
    }



    return (L_var_out);
}



/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : norm_s                                                  |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Produces the number of left shift needed to normalize the 16 bit varia- |
 |   ble var1 for positive values on the interval with minimum of 16384 and  |
 |   maximum of 32767, and for negative values on the interval with minimum  |
 |   of -32768 and maximum of -16384; in order to normalize the result, the  |
 |   following operation must be done :                                      |
 |                    norm_var1 = shl(var1,norm_s(var1)).                    |
 |                                                                           |
 |   Complexity weight : 15                                                  |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0xffff 8000 <= var1 <= 0x0000 7fff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0x0000 0000 <= var_out <= 0x0000 000f.                |
 |___________________________________________________________________________|
*/

Word16 norm_s (Word16 var1)
{
    Word16 var_out;

    if (var1 == 0)
    {
        var_out = 0;
    }
    else
    {
        if (var1 == -1)
        {
            var_out = 15;
        }
        else
        {
            if (var1 < 0)
            {
                var1 = (Word16)~var1;
            }
            for (var_out = 0; var1 < 0x4000; var_out++)
            {
                var1 <<= 1;
            }
        }
    }



    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : div_s                                                   |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Produces a result which is the fractional integer division of var1  by  |
 |   var2; var1 and var2 must be positive and var2 must be greater or equal  |
 |   to var1; the result is positive (leading bit equal to 0) and truncated  |
 |   to 16 bits.                                                             |
 |   If var1 = var2 then div(var1,var2) = 32767.                             |
 |                                                                           |
 |   Complexity weight : 18                                                  |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    var1                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0x0000 0000 <= var1 <= var2 and var2 != 0.            |
 |                                                                           |
 |    var2                                                                   |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : var1 <= var2 <= 0x0000 7fff and var2 != 0.            |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0x0000 0000 <= var_out <= 0x0000 7fff.                |
 |             It's a Q15 value (point between b15 and b14).                 |
 |___________________________________________________________________________|
*/

Word16 div_s (Word16 var1, Word16 var2)
{
    Word16 var_out = 0;
    Word16 iteration;
    Word32 L_num;
    Word32 L_denom;

    if ((var1 > var2) || (var1 < 0) || (var2 < 0))
    {
        printf ("Division Error var1=%d  var2=%d\n", var1, var2);
        abort(); /* exit (0); */
    }
    if (var2 == 0)
    {
        printf ("Division by 0, Fatal error \n");
        abort(); /* exit (0); */
    }
    if (var1 == 0)
    {
        var_out = 0;
    }
    else
    {
        if (var1 == var2)
        {
            var_out = MAX_16;
        }
        else
        {
            L_num = L_deposit_l (var1);


            L_denom = L_deposit_l (var2);



            for (iteration = 0; iteration < 15; iteration++)
            {
                var_out <<= 1;
                L_num <<= 1;

                if (L_num >= L_denom)
                {
                    L_num = L_sub (L_num, L_denom);


                    var_out = add (var_out, 1);


                }
            }
        }
    }



    return (var_out);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : norm_l                                                  |
 |                                                                           |
 |   Purpose :                                                               |
 |                                                                           |
 |   Produces the number of left shifts needed to normalize the 32 bit varia-|
 |   ble L_var1 for positive values on the interval with minimum of          |
 |   1073741824 and maximum of 2147483647, and for negative values on the in-|
 |   terval with minimum of -2147483648 and maximum of -1073741824; in order |
 |   to normalize the result, the following operation must be done :         |
 |                   norm_L_var1 = L_shl(L_var1,norm_l(L_var1)).             |
 |                                                                           |
 |   Complexity weight : 30                                                  |
 |                                                                           |
 |   Inputs :                                                                |
 |                                                                           |
 |    L_var1                                                                 |
 |             32 bit long signed integer (Word32) whose value falls in the  |
 |             range : 0x8000 0000 <= var1 <= 0x7fff ffff.                   |
 |                                                                           |
 |   Outputs :                                                               |
 |                                                                           |
 |    none                                                                   |
 |                                                                           |
 |   Return Value :                                                          |
 |                                                                           |
 |    var_out                                                                |
 |             16 bit short signed integer (Word16) whose value falls in the |
 |             range : 0x0000 0000 <= var_out <= 0x0000 001f.                |
 |___________________________________________________________________________|
*/

Word16 norm_l (Word32 L_var1)
{
    Word16 var_out;

    if (L_var1 == 0)
    {
        var_out = 0;
    }
    else
    {
        if (L_var1 == (Word32) 0xffffffffL)
        {
            var_out = 31;
        }
        else
        {
            if (L_var1 < 0)
            {
                L_var1 = ~L_var1;
            }
            for (var_out = 0; L_var1 < (Word32) 0x40000000L; var_out++)
            {
                L_var1 <<= 1;
            }
        }
    }


    return (var_out);
}


/*___________________________________________________________________________
 |                                                                           |
 |  This file contains mathematic operations in fixed point.                 |
 |                                                                           |
 |  Isqrt()              : inverse square root (16 bits precision).          |
 |  Pow2()               : 2^x  (16 bits precision).                         |
 |  Log2()               : log2 (16 bits precision).                         |
 |  Dot_product()        : scalar product of <x[],y[]>                       |
 |                                                                           |
 |  These operations are not standard double precision operations.           |
 |  They are used where low complexity is important and the full 32 bits     |
 |  precision is not necessary. For example, the function Div_32() has a     |
 |  24 bits precision which is enough for our purposes.                      |
 |                                                                           |
 |  In this file, the values use theses representations:                     |
 |                                                                           |
 |  Word32 L_32     : standard signed 32 bits format                         |
 |  Word16 hi, lo   : L_32 = hi<<16 + lo<<1  (DPF - Double Precision Format) |
 |  Word32 frac, Word16 exp : L_32 = frac << exp-31  (normalised format)     |
 |  Word16 int, frac        : L_32 = int.frac        (fractional format)     |
 |___________________________________________________________________________|
*/


/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : Isqrt                                                   |
 |                                                                           |
 |       Compute 1/sqrt(L_x).                                                |
 |       if L_x is negative or zero, result is 1 (7fffffff).                 |
 |---------------------------------------------------------------------------|
 |  Algorithm:                                                               |
 |                                                                           |
 |   1- Normalization of L_x.                                                |
 |   2- call Isqrt_n(L_x, exponant)                                          |
 |   3- L_y = L_x << exponant                                                |
 |___________________________________________________________________________|
*/
Word32 Isqrt(                              /* (o) Q31 : output value (range: 0<=val<1)         */
     Word32 L_x                            /* (i) Q0  : input value  (range: 0<=val<=7fffffff) */
)
{
    Word16 exp;
    Word32 L_y;

    exp = norm_l(L_x);
    L_x = L_shl(L_x, exp);                 /* L_x is normalized */
    exp = sub(31, exp);

    Isqrt_n(&L_x, &exp);

    L_y = L_shl(L_x, exp);                 /* denormalization   */

    return (L_y);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : Isqrt_n                                                 |
 |                                                                           |
 |       Compute 1/sqrt(value).                                              |
 |       if value is negative or zero, result is 1 (frac=7fffffff, exp=0).   |
 |---------------------------------------------------------------------------|
 |  Algorithm:                                                               |
 |                                                                           |
 |   The function 1/sqrt(value) is approximated by a table and linear        |
 |   interpolation.                                                          |
 |                                                                           |
 |   1- If exponant is odd then shift fraction right once.                   |
 |   2- exponant = -((exponant-1)>>1)                                        |
 |   3- i = bit25-b30 of fraction, 16 <= i <= 63 ->because of normalization. |
 |   4- a = bit10-b24                                                        |
 |   5- i -=16                                                               |
 |   6- fraction = table[i]<<16 - (table[i] - table[i+1]) * a * 2            |
 |___________________________________________________________________________|
*/
static Word16 table_isqrt[49] =
{
    32767, 31790, 30894, 30070, 29309, 28602, 27945, 27330, 26755, 26214,
    25705, 25225, 24770, 24339, 23930, 23541, 23170, 22817, 22479, 22155,
    21845, 21548, 21263, 20988, 20724, 20470, 20225, 19988, 19760, 19539,
    19326, 19119, 18919, 18725, 18536, 18354, 18176, 18004, 17837, 17674,
    17515, 17361, 17211, 17064, 16921, 16782, 16646, 16514, 16384
};

void Isqrt_n(
     Word32 * frac,                        /* (i/o) Q31: normalized value (1.0 < frac <= 0.5) */
     Word16 * exp                          /* (i/o)    : exponent (value = frac x 2^exponent) */
)
{
    Word16 i, a, tmp;

    
    if (*frac <= (Word32) 0)
    {
        *exp = 0;                          
        *frac = 0x7fffffffL;               
        return;
    }
    
    if (sub((Word16) (*exp & 1), 1) == 0)  /* If exponant odd -> shift right */
        *frac = L_shr(*frac, 1);

    *exp = negate(shr(sub(*exp, 1), 1));   

    *frac = L_shr(*frac, 9);               
    i = extract_h(*frac);                  /* Extract b25-b31 */
    *frac = L_shr(*frac, 1);               
    a = extract_l(*frac);                  /* Extract b10-b24 */
    a = (Word16) (a & (Word16) 0x7fff);    

    i = sub(i, 16);
    
    *frac = L_deposit_h(table_isqrt[i]);   /* table[i] << 16         */
    tmp = sub(table_isqrt[i], table_isqrt[i + 1]);      /* table[i] - table[i+1]) */
    
    *frac = L_msu(*frac, tmp, a);          /* frac -=  tmp*a*2       */

    return;
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : Pow2()                                                  |
 |                                                                           |
 |     L_x = pow(2.0, exponant.fraction)         (exponant = interger part)  |
 |         = pow(2.0, 0.fraction) << exponant                                |
 |---------------------------------------------------------------------------|
 |  Algorithm:                                                               |
 |                                                                           |
 |   The function Pow2(L_x) is approximated by a table and linear            |
 |   interpolation.                                                          |
 |                                                                           |
 |   1- i = bit10-b15 of fraction,   0 <= i <= 31                            |
 |   2- a = bit0-b9   of fraction                                            |
 |   3- L_x = table[i]<<16 - (table[i] - table[i+1]) * a * 2                 |
 |   4- L_x = L_x >> (30-exponant)     (with rounding)                       |
 |___________________________________________________________________________|
*/
static Word16 table_pow2[33] =
{
    16384, 16743, 17109, 17484, 17867, 18258, 18658, 19066, 19484, 19911,
    20347, 20792, 21247, 21713, 22188, 22674, 23170, 23678, 24196, 24726,
    25268, 25821, 26386, 26964, 27554, 28158, 28774, 29405, 30048, 30706,
    31379, 32066, 32767
};

Word32 Pow2(                               /* (o) Q0  : result       (range: 0<=val<=0x7fffffff) */
     Word16 exponant,                      /* (i) Q0  : Integer part.      (range: 0<=val<=30)   */
     Word16 fraction                       /* (i) Q15 : Fractionnal part.  (range: 0.0<=val<1.0) */
)
{
    Word16 exp, i, a, tmp;
    Word32 L_x;

    L_x = L_mult(fraction, 32);            /* L_x = fraction<<6           */
    i = extract_h(L_x);                    /* Extract b10-b16 of fraction */
    L_x = L_shr(L_x, 1);
    a = extract_l(L_x);                    /* Extract b0-b9   of fraction */
    a = (Word16) (a & (Word16) 0x7fff);    

    L_x = L_deposit_h(table_pow2[i]);      /* table[i] << 16        */
    tmp = sub(table_pow2[i], table_pow2[i + 1]);        /* table[i] - table[i+1] */
    L_x = L_msu(L_x, tmp, a);              /* L_x -= tmp*a*2        */

    exp = sub(30, exponant);
    L_x = L_shr_r(L_x, exp);

    return (L_x);
}

/*___________________________________________________________________________
 |                                                                           |
 |   Function Name : Dot_product12()                                         |
 |                                                                           |
 |       Compute scalar product of <x[],y[]> using accumulator.              |
 |                                                                           |
 |       The result is normalized (in Q31) with exponent (0..30).            |
 |---------------------------------------------------------------------------|
 |  Algorithm:                                                               |
 |                                                                           |
 |       dot_product = sum(x[i]*y[i])     i=0..N-1                           |
 |___________________________________________________________________________|
*/

Word32 Dot_product12(                      /* (o) Q31: normalized result (1 < val <= -1) */
     Word16 x[],                           /* (i) 12bits: x vector                       */
     Word16 y[],                           /* (i) 12bits: y vector                       */
     Word16 lg,                            /* (i)    : vector length                     */
     Word16 * exp                          /* (o)    : exponent of result (0..+30)       */
)
{
    Word16 i, sft;
    Word32 L_sum;

    L_sum = 1L;                            
    for (i = 0; i < lg; i++)
        L_sum = L_mac(L_sum, x[i], y[i]);

    /* Normalize acc in Q31 */

    sft = norm_l(L_sum);
    L_sum = L_shl(L_sum, sft);

    *exp = sub(30, sft);                     /* exponent = 0..30 */

    return (L_sum);
}

/*-------------------------------------------------------------------*
 *                         WB_VAD.C									 *
 *-------------------------------------------------------------------*
 * Voice Activity Detection.										 *
 *-------------------------------------------------------------------*/


/******************************************************************************
*                         PRIVATE PROGRAM CODE
******************************************************************************/

/******************************************************************************
* log2
*
*  Calculate Log2 and scale the signal:
*
*    ilog2(Word32 in) = -1024*log10(in * 2^-31)/log10(2), where in = [1, 2^31-1]
*
*  input   output
*  32768   16384
*  1       31744
*
* When input is in the range of [1,2^16], max error is 0.0380%.
*
*
*/

Word16 ilog2(                              /* return: output value of the log2 */
     Word16 mant                           /* i: value to be converted */
)
{
    Word16 i, ex, ex2, res;
    Word32 l_temp;

    
    if (mant <= 0)
    {
        mant = 1;                          
    }
    ex = norm_s(mant);
    mant = shl(mant, ex);

    for (i = 0; i < 3; i++)
        mant = mult(mant, mant);
    l_temp = L_mult(mant, mant);

    ex2 = norm_l(l_temp);
    mant = extract_h(L_shl(l_temp, ex2));

    res = shl(add(ex, 16), 10);
    res = add(res, shl(ex2, 6));
    res = sub(add(res, 127), shr(mant, 8));
    return (res);
}

/******************************************************************************
*
*     Function     : filter5
*     Purpose      : Fifth-order half-band lowpass/highpass filter pair with
*                    decimation.
*
*/
static void filter5(
     Word16 * in0,                         /* i/o : input values; output low-pass part  */
     Word16 * in1,                         /* i/o : input values; output high-pass part */
     Word16 data[]                         /* i/o : filter memory                       */
)
{
    Word16 temp0, temp1, temp2;

    temp0 = sub(*in0, mult(COEFF5_1, data[0]));
    temp1 = add(data[0], mult(COEFF5_1, temp0));
    data[0] = temp0;                       

    temp0 = sub(*in1, mult(COEFF5_2, data[1]));
    temp2 = add(data[1], mult(COEFF5_2, temp0));
    data[1] = temp0;                       

    *in0 = extract_h(L_shl(L_add(temp1, temp2), 15));   
    *in1 = extract_h(L_shl(L_sub(temp1, temp2), 15));   
}

/******************************************************************************
*
*     Function     : filter3
*     Purpose      : Third-order half-band lowpass/highpass filter pair with
*                    decimation.
*
*/
static void filter3(
     Word16 * in0,                         /* i/o : input values; output low-pass part  */
     Word16 * in1,                         /* i/o : input values; output high-pass part */
     Word16 * data                         /* i/o : filter memory                       */
)
{
    Word16 temp1, temp2;

    temp1 = sub(*in1, mult(COEFF3, *data));
    temp2 = add(*data, mult(COEFF3, temp1));
    *data = temp1;                         

    *in1 = extract_h(L_shl(L_sub(*in0, temp2), 15));    
    *in0 = extract_h(L_shl(L_add(*in0, temp2), 15));    
}

/******************************************************************************
*
*     Function   : level_calculation
*     Purpose    : Calculate signal level in a sub-band. Level is calculated
*                  by summing absolute values of the input data.
*
*                  Signal level calculated from of the end of the frame
*                  (data[count1 - count2]) is stored to (*sub_level)
*                  and added to the level of the next frame.
*
*/
static Word16 level_calculation(           /* return: signal level */
     Word16 data[],                        /* i   : signal buffer                                    */
     Word16 * sub_level,                   /* i   : level calculated at the end of the previous frame*/
										   /* o   : level of signal calculated from the last         */
										   /*       (count2 - count1) samples                        */
     Word16 count1,                        /* i   : number of samples to be counted                  */
     Word16 count2,                        /* i   : number of samples to be counted                  */
     Word16 ind_m,                         /* i   : step size for the index of the data buffer       */
     Word16 ind_a,                         /* i   : starting index of the data buffer                */
     Word16 scale                          /* i   : scaling for the level calculation                */
)
{
    Word32 l_temp1, l_temp2;
    Word16 level, i;

    l_temp1 = 0L;                          
    for (i = count1; i < count2; i++)
    {
        l_temp1 = L_mac(l_temp1, 1, abs_s(data[ind_m * i + ind_a]));
    }

    l_temp2 = L_add(l_temp1, L_shl(*sub_level, sub(16, scale)));
    *sub_level = extract_h(L_shl(l_temp1, scale));      

    for (i = 0; i < count1; i++)
    {
        l_temp2 = L_mac(l_temp2, 1, abs_s(data[ind_m * i + ind_a]));
    }
    level = extract_h(L_shl(l_temp2, scale));

    return level;
}

/******************************************************************************
*
*     Function     : filter_bank
*     Purpose      : Divide input signal into bands and calculate level of
*                    the signal in each band
*
*/
static void filter_bank(
     VadVars * st,                         /* i/o : State struct               */
     Word16 in[],                          /* i   : input frame                */
     Word16 level[]                        /* 0   : signal levels at each band */
)
{
    Word16 i;
    Word16 tmp_buf[FRAME_LEN];

    /* shift input 1 bit down for safe scaling */
    for (i = 0; i < FRAME_LEN; i++)
    {
        tmp_buf[i] = shr(in[i], 1);        
    }

    /* run the filter bank */
    for (i = 0; i < FRAME_LEN / 2; i++)
    {
        filter5(&tmp_buf[2 * i], &tmp_buf[2 * i + 1], st->a_data5[0]);
    }
    for (i = 0; i < FRAME_LEN / 4; i++)
    {
        filter5(&tmp_buf[4 * i], &tmp_buf[4 * i + 2], st->a_data5[1]);
        filter5(&tmp_buf[4 * i + 1], &tmp_buf[4 * i + 3], st->a_data5[2]);
    }
    for (i = 0; i < FRAME_LEN / 8; i++)
    {
        filter5(&tmp_buf[8 * i], &tmp_buf[8 * i + 4], st->a_data5[3]);
        filter5(&tmp_buf[8 * i + 2], &tmp_buf[8 * i + 6], st->a_data5[4]);
        filter3(&tmp_buf[8 * i + 3], &tmp_buf[8 * i + 7], &st->a_data3[0]);
    }
    for (i = 0; i < FRAME_LEN / 16; i++)
    {
        filter3(&tmp_buf[16 * i + 0], &tmp_buf[16 * i + 8], &st->a_data3[1]);
        filter3(&tmp_buf[16 * i + 4], &tmp_buf[16 * i + 12], &st->a_data3[2]);
        filter3(&tmp_buf[16 * i + 6], &tmp_buf[16 * i + 14], &st->a_data3[3]);
    }

    for (i = 0; i < FRAME_LEN / 32; i++)
    {
        filter3(&tmp_buf[32 * i + 0], &tmp_buf[32 * i + 16], &st->a_data3[4]);
        filter3(&tmp_buf[32 * i + 8], &tmp_buf[32 * i + 24], &st->a_data3[5]);
    }

    /* calculate levels in each frequency band */

    /* 4800 - 6400 Hz */
    level[11] = level_calculation(tmp_buf, &st->sub_level[11],
        FRAME_LEN / 4 - 48, FRAME_LEN / 4, 4, 1, 14);   
    /* 4000 - 4800 Hz */
    level[10] = level_calculation(tmp_buf, &st->sub_level[10],
        FRAME_LEN / 8 - 24, FRAME_LEN / 8, 8, 7, 15);   
    /* 3200 - 4000 Hz */
    level[9] = level_calculation(tmp_buf, &st->sub_level[9],
        FRAME_LEN / 8 - 24, FRAME_LEN / 8, 8, 3, 15);   
    /* 2400 - 3200 Hz */
    level[8] = level_calculation(tmp_buf, &st->sub_level[8],
        FRAME_LEN / 8 - 24, FRAME_LEN / 8, 8, 2, 15);   
    /* 2000 - 2400 Hz */
    level[7] = level_calculation(tmp_buf, &st->sub_level[7],
        FRAME_LEN / 16 - 12, FRAME_LEN / 16, 16, 14, 16);       
    /* 1600 - 2000 Hz */
    level[6] = level_calculation(tmp_buf, &st->sub_level[6],
        FRAME_LEN / 16 - 12, FRAME_LEN / 16, 16, 6, 16);        
    /* 1200 - 1600 Hz */
    level[5] = level_calculation(tmp_buf, &st->sub_level[5],
        FRAME_LEN / 16 - 12, FRAME_LEN / 16, 16, 4, 16);        
    /* 800 - 1200 Hz */
    level[4] = level_calculation(tmp_buf, &st->sub_level[4],
        FRAME_LEN / 16 - 12, FRAME_LEN / 16, 16, 12, 16);       
    /* 600 - 800 Hz */
    level[3] = level_calculation(tmp_buf, &st->sub_level[3],
        FRAME_LEN / 32 - 6, FRAME_LEN / 32, 32, 8, 17); 
    /* 400 - 600 Hz */
    level[2] = level_calculation(tmp_buf, &st->sub_level[2],
        FRAME_LEN / 32 - 6, FRAME_LEN / 32, 32, 24, 17);        
    /* 200 - 400 Hz */
    level[1] = level_calculation(tmp_buf, &st->sub_level[1],
        FRAME_LEN / 32 - 6, FRAME_LEN / 32, 32, 16, 17);        
    /* 0 - 200 Hz */
    level[0] = level_calculation(tmp_buf, &st->sub_level[0],
        FRAME_LEN / 32 - 6, FRAME_LEN / 32, 32, 0, 17); 
}

/******************************************************************************
*
*     Function   : update_cntrl
*     Purpose    : Control update of the background noise estimate.
*
*/
static void update_cntrl(
     VadVars * st,                         /* i/o : State structure                    */
     Word16 level[]                        /* i   : sub-band levels of the input frame */
)
{
    Word16 i, temp, stat_rat, exp;
    Word16 num, denom;
    Word16 alpha;

    /* if a tone has been detected for a while, initialize stat_count */
    
    if (sub((Word16) (st->tone_flag & 0x7c00), 0x7c00) == 0)
    {
        st->stat_count = STAT_COUNT;       
    } else
    {
        /* if 8 last vad-decisions have been "0", reinitialize stat_count */
        
        if ((st->vadreg & 0x7f80) == 0)
        {
            st->stat_count = STAT_COUNT;   
        } else
        {
            stat_rat = 0;                  
            for (i = 0; i < COMPLEN; i++)
            {
                
                if (sub(level[i], st->ave_level[i]) > 0)
                {
                    num = level[i];        
                    denom = st->ave_level[i];   
                } else
                {
                    num = st->ave_level[i];
                    denom = level[i];      
                }
                /* Limit nimimum value of num and denom to STAT_THR_LEVEL */
                
                if (sub(num, STAT_THR_LEVEL) < 0)
                {
                    num = STAT_THR_LEVEL;  
                }
                
                if (sub(denom, STAT_THR_LEVEL) < 0)
                {
                    denom = STAT_THR_LEVEL;
                }
                exp = norm_s(denom);
                denom = shl(denom, exp);

                /* stat_rat = num/denom * 64 */
                temp = div_s(shr(num, 1), denom);
                stat_rat = add(stat_rat, shr(temp, sub(8, exp)));
            }

            /* compare stat_rat with a threshold and update stat_count */
            
            if (sub(stat_rat, STAT_THR) > 0)
            {
                st->stat_count = STAT_COUNT;    
            } else
            {
                
                if ((st->vadreg & 0x4000) != 0)
                {
                    
                    if (st->stat_count != 0)
                    {
                        st->stat_count = sub(st->stat_count, 1);        
                    }
                }
            }
        }
    }

    /* Update average amplitude estimate for stationarity estimation */
    alpha = ALPHA4;                        
    
    if (sub(st->stat_count, STAT_COUNT) == 0)
    {
        alpha = 32767;                     
    } else if ((st->vadreg & 0x4000) == 0)
    {
        
        alpha = ALPHA5;                    
    }
    for (i = 0; i < COMPLEN; i++)
    {
        st->ave_level[i] = add(st->ave_level[i],
            mult_r(alpha, sub(level[i], st->ave_level[i])));    
    }
}

/******************************************************************************
*
*     Function     : hangover_addition
*     Purpose      : Add hangover after speech bursts
*
*/

static Word16 hangover_addition(           /* return: VAD_flag indicating final VAD decision */
     VadVars * st,                         /* i/o : State structure                     */
     Word16 low_power,                     /* i   : flag power of the input frame    */
     Word16 hang_len,                      /* i   : hangover length */
     Word16 burst_len                      /* i   : minimum burst length for hangover addition */
)
{
    /* if the input power (pow_sum) is lower than a threshold, clear counters and set VAD_flag to "0"         */
    
    if (low_power != 0)
    {
        st->burst_count = 0;               
        st->hang_count = 0;                
        return 0;
    }
    /* update the counters (hang_count, burst_count) */
    
    if ((st->vadreg & 0x4000) != 0)
    {
        st->burst_count = add(st->burst_count, 1);      
        
        if (sub(st->burst_count, burst_len) >= 0)
        {
            st->hang_count = hang_len;     
        }
        return 1;
    } else
    {
        st->burst_count = 0;               
        
        if (st->hang_count > 0)
        {
            st->hang_count = sub(st->hang_count, 1);    
            return 1;
        }
    }
    return 0;
}

/******************************************************************************
*
*     Function   : noise_estimate_update
*     Purpose    : Update of background noise estimate
*
*/

static void noise_estimate_update(
     VadVars * st,                         /* i/o : State structure                       */
     Word16 level[]                        /* i   : sub-band levels of the input frame */
)
{
    Word16 i, alpha_up, alpha_down, bckr_add;

    /* Control update of bckr_est[] */
    update_cntrl(st, level);

    /* Reason for using bckr_add is to avoid problems caused by fixed-point dynamics when noise level and
     * required change is very small. */
    bckr_add = 2;                          

    /* Choose update speed */
    
    if ((0x7800 & st->vadreg) == 0)
    {
        alpha_up = ALPHA_UP1;              
        alpha_down = ALPHA_DOWN1;          
    } else
    {
        
        if ((st->stat_count == 0))
        {
            alpha_up = ALPHA_UP2;          
            alpha_down = ALPHA_DOWN2;      
        } else
        {
            alpha_up = 0;                  
            alpha_down = ALPHA3;           
            bckr_add = 0;                  
        }
    }

    /* Update noise estimate (bckr_est) */
    for (i = 0; i < COMPLEN; i++)
    {
        Word16 temp;

        temp = sub(st->old_level[i], st->bckr_est[i]);

        
        if (temp < 0)
        {                                  /* update downwards */
            st->bckr_est[i] = add(-2, add(st->bckr_est[i],
                    mult_r(alpha_down, temp))); 

            /* limit minimum value of the noise estimate to NOISE_MIN */
            
            if (sub(st->bckr_est[i], NOISE_MIN) < 0)
            {
                st->bckr_est[i] = NOISE_MIN;    
            }
        } else
        {                                  /* update upwards */
            st->bckr_est[i] = add(bckr_add, add(st->bckr_est[i],
                    mult_r(alpha_up, temp)));   

            /* limit maximum value of the noise estimate to NOISE_MAX */
            
            if (sub(st->bckr_est[i], NOISE_MAX) > 0)
            {
                st->bckr_est[i] = NOISE_MAX;    
            }
        }
    }

    /* Update signal levels of the previous frame (old_level) */
    for (i = 0; i < COMPLEN; i++)
    {
        st->old_level[i] = level[i];       
    }
}

/******************************************************************************
*
*     Function     : vad_decision
*     Purpose      : Calculates VAD_flag
*
*/

static Word16 vad_decision(                /* return value : VAD_flag */
     VadVars * st,                         /* i/o : State structure                       */
     Word16 level[COMPLEN],                /* i   : sub-band levels of the input frame */
     Word32 pow_sum                        /* i   : power of the input frame           */
)
{
    Word16 i;
    Word32 L_snr_sum;
    Word32 L_temp;
    Word16 vad_thr, temp, noise_level;
    Word16 low_power_flag;
    Word16 hang_len, burst_len;
    Word16 ilog2_speech_level, ilog2_noise_level;
    Word16 temp2;

    /* Calculate squared sum of the input levels (level) divided by the background noise components
     * (bckr_est). */
    L_snr_sum = 0;                         
    for (i = 0; i < COMPLEN; i++)
    {
        Word16 exp;

        exp = norm_s(st->bckr_est[i]);
        temp = shl(st->bckr_est[i], exp);
        temp = div_s(shr(level[i], 1), temp);
        temp = shl(temp, sub(exp, UNIRSHFT - 1));
        L_snr_sum = L_mac(L_snr_sum, temp, temp);
    }

    /* Calculate average level of estimated background noise */
    L_temp = 0;                            
    for (i = 1; i < COMPLEN; i++)          /* ignore lowest band */
    {
        L_temp = L_add(L_temp, st->bckr_est[i]);
    }

    noise_level = extract_h(L_shl(L_temp, 12));
    /* if SNR is lower than a threshold (MIN_SPEECH_SNR), and increase speech_level */
    temp = shl(mult(noise_level, MIN_SPEECH_SNR), 3);

    
    if (sub(st->speech_level, temp) < 0)
    {
        st->speech_level = temp;           
    }
    ilog2_noise_level = ilog2(noise_level);

    /* If SNR is very poor, speech_level is probably corrupted by noise level. This is correctred by
     * subtracting MIN_SPEECH_SNR*noise_level from speech level */
    ilog2_speech_level = ilog2(sub(st->speech_level, temp));

    temp = add(mult(NO_SLOPE, sub(ilog2_noise_level, NO_P1)), THR_HIGH);

    temp2 = add(SP_CH_MIN, mult(SP_SLOPE, sub(ilog2_speech_level, SP_P1)));
    
    if (sub(temp2, SP_CH_MIN) < 0)
    {
        temp2 = SP_CH_MIN;                 
    }
    
    if (sub(temp2, SP_CH_MAX) > 0)
    {
        temp2 = SP_CH_MAX;                 
    }
    vad_thr = add(temp, temp2);

    
    if (sub(vad_thr, THR_MIN) < 0)
    {
        vad_thr = THR_MIN;                 
    }
    /* Shift VAD decision register */
    st->vadreg = shr(st->vadreg, 1);       

    /* Make intermediate VAD decision */
    
    if (L_sub(L_snr_sum, L_mult(vad_thr, 512 * COMPLEN)) > 0)
    {
        st->vadreg = (Word16) (st->vadreg | 0x4000);    
    }
    /* check if the input power (pow_sum) is lower than a threshold" */
    
    if (L_sub(pow_sum, VAD_POW_LOW) < 0)
    {
        low_power_flag = 1;                
    } else
    {
        low_power_flag = 0;                
    }
    /* Update background noise estimates */
    noise_estimate_update(st, level);

    /* Calculate values for hang_len and burst_len based on vad_thr */
    hang_len = add(mult(HANG_SLOPE, sub(vad_thr, HANG_P1)), HANG_HIGH);
    
    if (sub(hang_len, HANG_LOW) < 0)
    {
        hang_len = HANG_LOW;               
    };

    burst_len = add(mult(BURST_SLOPE, sub(vad_thr, BURST_P1)), BURST_HIGH);

    return (hangover_addition(st, low_power_flag, hang_len, burst_len));
}

/******************************************************************************
*
*     Estimate_Speech()
*     Purpose      : Estimate speech level
*
* Maximum signal level is searched and stored to the variable sp_max.
* The speech frames must locate within SP_EST_COUNT number of frames.
* Thus, noisy frames having occasional VAD = "1" decisions will not
* affect to the estimated speech_level.
*
*/
static void Estimate_Speech(
     VadVars * st,                         /* i/o : State structure    */
     Word16 in_level                       /* level of the input frame */
)
{
    Word16 alpha;

    /* if the required activity count cannot be achieved, reset counters */
    
    /* if (SP_ACTIVITY_COUNT  > SP_EST_COUNT - st->sp_est_cnt + st->sp_max_cnt) */
    if (sub(sub(st->sp_est_cnt, st->sp_max_cnt), SP_EST_COUNT - SP_ACTIVITY_COUNT) > 0)
    {
        st->sp_est_cnt = 0;                
        st->sp_max = 0;                    
        st->sp_max_cnt = 0;                
    }
    st->sp_est_cnt = add(st->sp_est_cnt, 1);    

    
    if (((st->vadreg & 0x4000) || (sub(in_level, st->speech_level) > 0))
        && (sub(in_level, MIN_SPEECH_LEVEL1) > 0))
    {
        /* update sp_max */
        
        if (sub(in_level, st->sp_max) > 0)
        {
            st->sp_max = in_level;         
        }
        st->sp_max_cnt = add(st->sp_max_cnt, 1);        
        
        if (sub(st->sp_max_cnt, SP_ACTIVITY_COUNT) >= 0)
        {
            Word16 tmp;

            /* update speech estimate */
            tmp = shr(st->sp_max, 1);      /* scale to get "average" speech level */

            /* select update speed */
            
            if (sub(tmp, st->speech_level) > 0)
            {
                alpha = ALPHA_SP_UP;       
            } else
            {
                alpha = ALPHA_SP_DOWN;     
            }
            
            if (sub(tmp, MIN_SPEECH_LEVEL2) > 0)
            {
                st->speech_level = add(st->speech_level,
                    mult_r(alpha, sub(tmp, st->speech_level))); 
            }
            /* clear all counters used for speech estimation */
            st->sp_max = 0;                
            st->sp_max_cnt = 0;            
            st->sp_est_cnt = 0;            
        }
    }
}

/******************************************************************************
*                         PUBLIC PROGRAM CODE
******************************************************************************/

/******************************************************************************
*
*  Function:   wb_vad_init
*  Purpose:    Allocates state memory and initializes state memory
*
*/

Word16 wb_vad_init(                        /* return: non-zero with error, zero for ok. */
     VadVars ** state                      /* i/o : State structure    */
)
{
    VadVars *s;

    if (state == (VadVars **) NULL)
    {
        fprintf(stderr, "vad_init: invalid parameter\n");
        return -1;
    }
    *state = NULL;

    /* allocate memory */
    if ((s = (VadVars *) malloc(sizeof(VadVars))) == NULL)
    {
        fprintf(stderr, "vad_init: can not malloc state structure\n");
        return -1;
    }
    wb_vad_reset(s);

    *state = s;

    return 0;
}

/******************************************************************************
*
*  Function:   wb_vad_reset
*  Purpose:    Initializes state memory
*
*/
Word16 wb_vad_reset(                       /* return: non-zero with error, zero for ok. */
     VadVars * state                       /* i/o : State structure    */
)
{
    Word16 i, j;

    if (state == (VadVars *) NULL)
    {
        fprintf(stderr, "vad_reset: invalid parameter\n");
        return -1;
    }
    state->tone_flag = 0;
    state->vadreg = 0;
    state->hang_count = 0;
    state->burst_count = 0;
    state->hang_count = 0;

    /* initialize memory used by the filter bank */
    for (i = 0; i < F_5TH_CNT; i++)
    {
        for (j = 0; j < 2; j++)
        {
            state->a_data5[i][j] = 0;
        }
    }

    for (i = 0; i < F_3TH_CNT; i++)
    {
        state->a_data3[i] = 0;
    }

    /* initialize the rest of the memory */
    for (i = 0; i < COMPLEN; i++)
    {
        state->bckr_est[i] = NOISE_INIT;
        state->old_level[i] = NOISE_INIT;
        state->ave_level[i] = NOISE_INIT;
        state->sub_level[i] = 0;
    }

    state->sp_est_cnt = 0;
    state->sp_max = 0;
    state->sp_max_cnt = 0;
    state->speech_level = SPEECH_LEVEL_INIT;
    state->prev_pow_sum = 0;
    return 0;
}

/******************************************************************************
*
*  Function:   wb_vad_exit
*  Purpose:    The memory used for state memory is freed
*
*/
void wb_vad_exit(
     VadVars ** state                      /* i/o : State structure    */
)
{
    if (state == NULL || *state == NULL)
        return;

    /* deallocate memory */
    free(*state);
    *state = NULL;
    return;
}

/******************************************************************************
*
*     Function     : wb_vad_tone_detection
*     Purpose      : Search maximum pitch gain from a frame. Set tone flag if
*                    pitch gain is high. This is used to detect
*                    signaling tones and other signals with high pitch gain.
*
*/
void wb_vad_tone_detection(
     VadVars * st,                         /* i/o : State struct            */
     Word16 p_gain                         /* pitch gain      */
)
{
    /* update tone flag */
    st->tone_flag = shr(st->tone_flag, 1); 

    /* if (pitch_gain > TONE_THR) set tone flag */
    
    if (sub(p_gain, TONE_THR) > 0)
    {
        st->tone_flag = (Word16) (st->tone_flag | 0x4000);      
    }
}

/******************************************************************************
*
*     Function     : wb_vad
*     Purpose      : Main program for Voice Activity Detection (VAD) for AMR
*
*/
Word16 wb_vad(                             /* Return value : VAD Decision, 1 = speech, 0 = noise */
     VadVars * st,                         /* i/o : State structure                 */
     Word16 in_buf[]                       /* i   : samples of the input frame   */
)
{
    Word16 level[COMPLEN];
    Word16 i;
    Word16 VAD_flag, temp;
    Word32 L_temp, pow_sum;

    /* Calculate power of the input frame. */
    L_temp = 0L;                           
    for (i = 0; i < FRAME_LEN; i++)
    {
        L_temp = L_mac(L_temp, in_buf[i], in_buf[i]);
    }

    /* pow_sum = power of current frame and previous frame */
    pow_sum = L_add(L_temp, st->prev_pow_sum);  

    /* save power of current frame for next call */
    st->prev_pow_sum = L_temp;             

    /* If input power is very low, clear tone flag */
    
    if (L_sub(pow_sum, POW_TONE_THR) < 0)
    {
        st->tone_flag = (Word16) (st->tone_flag & 0x1fff);      
    }
    /* Run the filter bank and calculate signal levels at each band */
    filter_bank(st, in_buf, level);

    /* compute VAD decision */
    VAD_flag = vad_decision(st, level, pow_sum);

    /* Calculate input level */
    L_temp = 0;                            
    for (i = 1; i < COMPLEN; i++)          /* ignore lowest band */
    {
        L_temp = L_add(L_temp, level[i]);
    }

    temp = extract_h(L_shl(L_temp, 12));

    Estimate_Speech(st, temp);             /* Estimate speech level */
    return (VAD_flag);
}

/*-------------------------------------------------------------------*
 *                         UTIL.C									 *
 *-------------------------------------------------------------------*
 * Vector routines												     *
 *-------------------------------------------------------------------*/

/*-------------------------------------------------------------------*
 * Function  Set zero()                                              *
 *           ~~~~~~~~~~                                              *
 * Set vector x[] to zero                                            *
 *-------------------------------------------------------------------*/

void Set_zero(
     Word16 x[],                           /* (o)    : vector to clear     */
     Word16 L                              /* (i)    : length of vector    */
)
{
    Word16 i;

    for (i = 0; i < L; i++)
    {
        x[i] = 0;                          
    }

    return;
}


/*-------------------------------------------------------------------*
 * Function  Copy:                                                   *
 *           ~~~~~                                                   *
 * Copy vector x[] to y[]                                            *
 *-------------------------------------------------------------------*/

void Copy(
     Word16 x[],                           /* (i)   : input vector   */
     Word16 y[],                           /* (o)   : output vector  */
     Word16 L                              /* (i)   : vector length  */
)
{
    Word16 i;

    for (i = 0; i < L; i++)
    {
        y[i] = x[i];                       
    }

    return;
}

/*-------------------------------------------------------------------*
 *                        WEIGHT_A.C								 *
 *-------------------------------------------------------------------*
 * Weighting of LPC coefficients.									 *
 *   ap[i]  =  a[i] * (gamma ** i)									 *
 *-------------------------------------------------------------------*/

void Weight_a(
     Word16 a[],                           /* (i) Q12 : a[m+1]  LPC coefficients             */
     Word16 ap[],                          /* (o) Q12 : Spectral expanded LPC coefficients   */
     Word16 gamma,                         /* (i) Q15 : Spectral expansion factor.           */
     Word16 m                              /* (i)     : LPC order.                           */
)
{
    Word16 i, fac;

    ap[0] = a[0];                          
    fac = gamma;                           
    for (i = 1; i < m; i++)
    {
        ap[i] = round(L_mult(a[i], fac));  
        fac = round(L_mult(fac, gamma));
    }
    ap[m] = round(L_mult(a[m], fac));      

    return;
}




