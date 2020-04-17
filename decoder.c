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
#include "dtx.h"
#include "count.h"

/*-------------------------------------------------------------------*
 * DECODER.C                                                         *
 * ~~~~~~~~~                                                         *
 * Main program of the AMR WB ACELP wideband decoder.                *
 *                                                                   *
 *    Usage : decoder (-itu | -mime) bitstream_file synth_file bit_rate      *
 *                                                                   *
 *    Format for bitstream_file (default):                           *
 *																     *
 *        1 word (2-byte) for the type of frame type                 *
 *          (TX_FRAME_TYPE or RX_FRAME_TYPE)                         *
 *        1 word (2-byte) for the frame type                         *
 *          (see dtx.h for possible values)                          *
 *        1 word (2-byte) for the mode indication                    *
 *          (see bits.h for possible values)                         *
 *        N words (2-byte) containning N bits.                       *
 *          Bit 0 = 0xff81 and Bit 1 = 0x007f                        *
 *																     *
 *    if option -itu defined:                         		         *
 *        1 word (2-byte) for sync word (0x6b21)                     *
 *        1 word (2-byte) frame length N                             *
 *        N words (2-byte) containing N 'soft' bits                  *
 *              (bit 0 = 0x007f, bit 1 = 0x0081)                     *
 *                                                                   *
 *    if option -mime defined:                         		         *
 *        AMR-WB MIME/storage format, see RFC 3267 (sections 5.1 and 5.3) for details *
 *                                                                 *
 *    Format for synth_file:                                         *
 *      Synthesis is written to a binary file of 16 bits data.       *
 *                                                                   *
 *-------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    FILE *f_serial;                        /* File of serial bits for transmission  */
    FILE *f_synth;                         /* File of speech data                   */

    Word16 synth[L_FRAME16k];              /* Buffer for speech @ 16kHz             */
    Word16 prms[NB_BITS_MAX];

    Word16 nb_bits, mode, frame_type, frame_length;
    Word16 reset_flag = 0;
    Word16 reset_flag_old = 1;
    Word16 mode_old = 0;
    Word16 i;
    long frame;

	Word16 bitstreamformat;
	RX_State *rx_state;

	char magic[10];
    void *st;

      fprintf(stderr, "\n");
      fprintf(stderr, " ==================================================================================================\n");
      fprintf(stderr, " AMR Wideband Codec 3GPP TS26.190 / ITU-T G.722.2, Aug 25, 2003. Version %s.\n", CODEC_VERSION);
      fprintf(stderr, " ==================================================================================================\n");
      fprintf(stderr, "\n");

    /*-----------------------------------------------------------------*
     *           Read passed arguments and open in/out files           *
     *-----------------------------------------------------------------*/

    if (argc != 3 && argc != 4)
    {
        fprintf(stderr, "Usage : decoder  (-itu | -mime) bitstream_file  synth_file\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "Format for bitstream_file: (default)\n");
        fprintf(stderr, "  One word (2-byte) to indicate type of frame type.\n");
        fprintf(stderr, "  One word (2-byte) to indicate frame type.\n");
        fprintf(stderr, "  One word (2-byte) to indicate mode.\n");
        fprintf(stderr, "  N words (2-byte) containing N bits (bit 0 = 0xff81, bit 1 = 0x007f).\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "  if option -itu defined:\n");
		fprintf(stderr, "  One word (2-byte) for sync word (good frames: 0x6b21, bad frames: 0x6b20)\n");
		fprintf(stderr, "  One word (2-byte) for frame length N.\n");
		fprintf(stderr, "  N words (2-byte) containing N bits (bit 0 = 0x007f, bit 1 = 0x0081).\n");
		fprintf(stderr, "\n");
        fprintf(stderr, "  if option -mime defined:\n");
		fprintf(stderr, "  AMR-WB MIME/storage format, see RFC 3267 (sections 5.1 and 5.3) for details.\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "Format for synth_file:\n");
        fprintf(stderr, "  Synthesis is written to a binary file of 16 bits data.\n");
        fprintf(stderr, "\n");
        exit(0);
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

    /* Open file for synthesis and packed serial stream */

    if ((f_serial = fopen(argv[1], "rb")) == NULL)
    {
        fprintf(stderr, "Input file '%s' does not exist !!\n", argv[1]);
        exit(0);
    } else
        fprintf(stderr, "Input bitstream file:   %s\n", argv[1]);

    if ((f_synth = fopen(argv[2], "wb")) == NULL)
    {
        fprintf(stderr, "Cannot open file '%s' !!\n", argv[2]);
        exit(0);
    } else
        fprintf(stderr, "Synthesis speech file:  %s\n", argv[2]);
    /*-----------------------------------------------------------------*
     *           Initialization of decoder                             *
     *-----------------------------------------------------------------*/

    Init_decoder(&st);
	Init_read_serial(&rx_state);
    Init_WMOPS_counter();

    /*-----------------------------------------------------------------*
     *            Loop for each "L_FRAME" speech data                  *
     *-----------------------------------------------------------------*/

    fprintf(stderr, "\n --- Running ---\n");

	/* read and verify magic number if MIME/storage format specified */
	if (bitstreamformat == 2)
	{
		fread(magic, sizeof(char), 9, f_serial);

		if (strncmp(magic, "#!AMR-WB\n", 9))
		{
			fprintf(stderr, "%s%s\n", "Invalid magic number: ", magic);
			fclose(f_serial);
			fclose(f_synth);
			exit(0);
		}
	}

    frame = 0;
    while ((nb_bits = Read_serial(f_serial, prms, &frame_type, &mode, rx_state, bitstreamformat)) != 0)
    {
        Reset_WMOPS_counter();

        frame++;

        fprintf(stderr, " Frames processed: %ld\r", frame);

        if ((frame_type == RX_NO_DATA) | (frame_type == RX_SPEECH_LOST))
        {
           mode = mode_old;
           reset_flag = 0;
        }
        else
        {
           mode_old = mode;

        /* if homed: check if this frame is another homing frame */
        if (reset_flag_old == 1)
        {
            /* only check until end of first subframe */
            reset_flag = decoder_homing_frame_test_first(prms, mode);
        }
        }

        /* produce encoder homing frame if homed & input=decoder homing frame */
        if ((reset_flag != 0) && (reset_flag_old != 0))
        {
            for (i = 0; i < L_FRAME16k; i++)
            {
                synth[i] = EHF_MASK;
            }
        } else
        {
            decoder(mode, prms, synth, &frame_length, st, frame_type);
        }

        for (i = 0; i < L_FRAME16k; i++)   /* Delete the 2 LSBs (14-bit output) */
        {
            synth[i] = (Word16) (synth[i] & 0xfffC);      logic16(); move16();
        }

        fwrite(synth, sizeof(Word16), L_FRAME16k, f_synth);

        WMOPS_output((Word16) (mode == MRDTX));

        /* if not homed: check whether current frame is a homing frame */
        if (reset_flag_old == 0)
        {
            /* check whole frame */
            reset_flag = decoder_homing_frame_test(prms, mode);
        }
        /* reset decoder if current frame is a homing frame */
        if (reset_flag != 0)
        {
            Reset_decoder(st, 1);
        }
        reset_flag_old = reset_flag;

    }

    Close_decoder(st);
    Close_read_serial(rx_state);
    fclose(f_serial);
    fclose(f_synth);
    exit(0);
}
