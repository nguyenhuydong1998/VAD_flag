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
            L_max = L_tmp;                 move32();
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
