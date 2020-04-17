/*___________________________________________________________________________
 |                                                                           |
 | Fixed-point C simulation of AMR WB ACELP coding algorithm with 20 ms      |
 | speech frames for wideband speech signals.                                |
 |___________________________________________________________________________|
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "typedef.h"
#include "basic_op.h"
#include "acelp.h"
#include "cnst.h"
#include "main.h"
#include "bits.h"
#include "count.h"
#include "cod_main.h"

/*-----------------------------------------------------------------*
 * CODER.C                                                         *
 * ~~~~~~~~~~                                                      *
 * Main program of the AMR WB ACELP wideband coder.                *
 *                                                                 *
 *    Usage : coder (-dtx) (-itu | -mime) mode speech_file  bitstream_file *
 *                                                                 *
 *    Format for speech_file:                                      *
 *      Speech is read from a binary file of 16 bits data.         *
 *                                                                 *
 *    Format for bitstream_file (default):                         *
 *																   *
 *        1 word (2-byte) for the type of frame type               *
 *          (TX_FRAME_TYPE or RX_FRAME_TYPE)                       *
 *        1 word (2-byte) for the frame type                       *
 *          (see dtx.h for possible values)                        *
 *        1 word (2-byte) for the mode indication                  *
 *          (see bits.h for possible values)                       *
 *        N words (2-byte) containning N bits.                     *
 *          Bit 0 = 0xff81 and Bit 1 = 0x007f                      *
 *																   *
 *    if option -itu defined:                         		       *
 *        1 word (2-byte) for sync word (0x6b21)                   *
 *        1 word (2-byte) frame length N                           *
 *        N words (2-byte) containing N 'soft' bits                *
 *              (bit 0 = 0x007f, bit 1 = 0x008f)                   *
 *                                                                 *
 *    if option -mime defined:                         		       *
 *        AMR-WB MIME/storage format, see RFC 3267 (sections 5.1 and 5.3) for details *
 *                                                                 *
 *    mode = 0..8 (bit rate = 6.60 to 23.85 k)                     *
 *                                                                 *
 *    -dtx if DTX is ON                                            *
 *-----------------------------------------------------------------*/


int main(int argc, char *argv[])
{
    FILE *f_speech;                        /* File of speech data                   */
    FILE *f_serial;                        /* File of serial bits for transmission  */
    FILE *f_mode = NULL;                   /* File of modes for each frame          */

    Word16 signal[L_FRAME16k];             /* Buffer for speech @ 16kHz             */
    Word16 prms[NB_BITS_MAX];

	Word16 flag_VAD = 0;					   /* VAD_lag 								*/

	Word16 coding_mode = 0, nb_bits, allow_dtx, mode_file, mode = 0, i;
    Word16 bitstreamformat;
    Word16 reset_flag;
    long frame;

    void *st;
    TX_State *tx_state;

    fprintf(stderr, "\n");
	fprintf(stderr, " ==================================================================================================\n");
	fprintf(stderr, " AMR Wideband Codec 3GPP TS26.190 / ITU-T G.722.2, Aug 25, 2003. Version %s.\n", CODEC_VERSION);
	fprintf(stderr, " ==================================================================================================\n");
    fprintf(stderr, "\n");

    /*-------------------------------------------------------------------------*
     * Open speech file and result file (output serial bit stream)             *
     *-------------------------------------------------------------------------*/

    if ((argc < 3) || (argc > 6))
    {
        fprintf(stderr, "Usage : coder  (-dtx) (-itu | -mime) mode speech_file  bitstream_file\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "Format for speech_file:\n");
        fprintf(stderr, "  Speech is read form a binary file of 16 bits data.\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "Format for bitstream_file (default):\n");
        fprintf(stderr, "  One word (2-byte) to indicate type of frame type.\n");
        fprintf(stderr, "  One word (2-byte) to indicate frame type.\n");
        fprintf(stderr, "  One word (2-byte) to indicate mode.\n");
        fprintf(stderr, "  N words (2-byte) containing N bits (bit 0 = 0xff81, bit 1 = 0x007f).\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "  if option -itu defined:\n");
		fprintf(stderr, "  One word (2-byte) for sync word (0x6b21)\n");
		fprintf(stderr, "  One word (2-byte) for frame length N.\n");
		fprintf(stderr, "  N words (2-byte) containing N bits (bit 0 = 0x007f, bit 1 = 0x0081).\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "  if option -mime defined:\n");
		fprintf(stderr, "  AMR-WB MIME/storage format, see RFC 3267 (sections 5.1 and 5.3) for details.\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "mode: 0 to 8 (9 bits rates) or\n");
        fprintf(stderr, "      -modefile filename\n");
        fprintf(stderr, " ===================================================================\n");
        fprintf(stderr, " mode   :  (0)  (1)   (2)   (3)   (4)   (5)   (6)   (7)   (8)     \n");
        fprintf(stderr, " bitrate: 6.60 8.85 12.65 14.25 15.85 18.25 19.85 23.05 23.85 kbit/s\n");
        fprintf(stderr, " ===================================================================\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "-dtx if DTX is ON, default is OFF\n");
        fprintf(stderr, "\n");
        exit(0);
    }

    allow_dtx = 0;
    if (strcmp(argv[1], "-dtx") == 0)
    {
        allow_dtx = 1;
        argv++;
    }

	bitstreamformat = 0;
    if (strcmp(argv[1], "-itu") == 0)
	{
	    bitstreamformat = 1;
	    argv++;
		fprintf(stderr, "Input bitstream format: ITU\n");
    } else
	{
		if (strcmp(argv[1], "-mime") == 0)
		{
			bitstreamformat = 2;
			argv++;
			fprintf(stderr, "Input bitstream format: MIME\n");
		} else
		{
			fprintf(stderr, "Input bitstream format: Default\n");
		}
    }
/*
    mode_file = 0;
    if (strcmp(argv[1], "-modefile") == 0)
    {
        mode_file = 1;
        argv++;
        if ((f_mode = fopen(argv[1], "r")) == NULL)
        {
            fprintf(stderr, "Error opening input file  %s !!\n", argv[1]);
            exit(0);
        }
        fprintf(stderr, "Mode file:  %s\n", argv[1]);
    } else
    {
        mode = (Word16)atoi(argv[1]);
        if ((mode < 0) || (mode > 8))
        {
            fprintf(stderr, " error in bit rate mode %d: use 0 to 8\n", mode);
            exit(0);
        } else
            nb_bits = nb_of_bits[mode];
    }
*/
	nb_bits = nb_of_bits[mode];
    if ((f_speech = fopen(argv[1], "rb")) == NULL)
    {
        fprintf(stderr, "Error opening input file  %s !!\n", argv[2]);
        exit(0);
    }
    fprintf(stderr, "Input speech file:  %s\n", argv[2]);

    if ((f_serial = fopen(argv[2], "wb")) == NULL)
    {
        fprintf(stderr, "Error opening output bitstream file %s !!\n", argv[3]);
        exit(0);
    }
    fprintf(stderr, "Output bitstream file:  %s\n", argv[3]);

    /*-------------------------------------------------------------------------*
     * Initialisation                                                          *
     *-------------------------------------------------------------------------*/

    Init_coder(&st);                       /* Initialize the coder */
    Init_write_serial(&tx_state);
    Init_WMOPS_counter();                  /* for complexity calculation */

    /*---------------------------------------------------------------------------*
     * Loop for every analysis/transmission frame.                               *
     *   -New L_FRAME data are read. (L_FRAME = number of speech data per frame) *
     *   -Conversion of the speech data from 16 bit integer to real              *
     *   -Call coder to encode the speech.                                       *
     *   -The compressed serial output stream is written to a file.              *
     *--------------------------------------------------------------------------*/

    fprintf(stderr, "\n --- Running ---\n");

    /* If MIME/storage format selected, write the magic number at the beginning of the bitstream file */
	if (bitstreamformat == 2)
	{
		fwrite("#!AMR-WB\n", sizeof(char), 9, f_serial);
	}

    frame = 0;

    while (fread(signal, sizeof(Word16), L_FRAME16k, f_speech) == L_FRAME16k)
    {
        Reset_WMOPS_counter();
/*
        if (mode_file)
        {
           if (fscanf(f_mode, "%hd", &mode) == EOF)
           {
              mode = coding_mode;
              fprintf(stderr, "\n end of mode control file reached\n");
              fprintf(stderr, " From now on using mode: %hd\n", mode);
              mode_file = 0;
           }

           if ((mode < 0) || (mode > 8))
           {
              fprintf(stderr, " error in bit rate mode %hd: use 0 to 8\n", mode);
              exit(0);
           }
        }
        coding_mode = mode;
*/
        frame++;
        fprintf(stderr, " Frames processed: %hd\r", frame);

        /* check for homing frame */
        reset_flag = encoder_homing_frame_test(signal);

        for (i = 0; i < L_FRAME16k; i++)   /* Delete the 2 LSBs (14-bit input) */
        {
            signal[i] = (Word16) (signal[i] & 0xfffC);       
        }

        coder(signal, &nb_bits, st, &flag_VAD);

        //Write_serial(f_serial, prms, coding_mode, mode, tx_state, bitstreamformat);

		fwrite(&flag_VAD, sizeof(Word16), 1, f_serial);	
		
        WMOPS_output((Word16) (coding_mode == MRDTX));

        /* perform homing if homing frame was detected at encoder input */
        if (reset_flag != 0)
        {
            Reset_encoder(st, 1);
        }
    }

    /* free allocated memory */
    Close_coder(st);
    Close_write_serial(tx_state);
    fclose(f_speech);
    fclose(f_serial);
    if (f_mode != NULL)
    {
       fclose(f_mode);
    }

    exit(0);
}
