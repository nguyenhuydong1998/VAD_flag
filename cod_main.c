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

#include "cod_main.h"
#include "bits.h"
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

/*****************************************************************************
 *  $Id$
 *
 *  This file contains operations in double precision.                       *
 *  These operations are not standard double precision operations.           *
 *  They are used where single precision is not enough but the full 32 bits  *
 *  precision is not necessary. For example, the function Div_32() has a     *
 *  24 bits precision which is enough for our purposes.                      *
 *                                                                           *
 *  The double precision numbers use a special representation:               *
 *                                                                           *
 *     L_32 = hi<<16 + lo<<1                                                 *
 *                                                                           *
 *  L_32 is a 32 bit integer.                                                *
 *  hi and lo are 16 bit signed integers.                                    *
 *  As the low part also contains the sign, this allows fast multiplication. *
 *                                                                           *
 *      0x8000 0000 <= L_32 <= 0x7fff fffe.                                  *
 *                                                                           *
 *  We will use DPF (Double Precision Format )in this file to specify        *
 *  this special format.                                                     *
 *****************************************************************************
*/


/*****************************************************************************
 *                                                                           *
 *  Function L_Extract()                                                     *
 *                                                                           *
 *  Extract from a 32 bit integer two 16 bit DPF.                            *
 *                                                                           *
 *  Arguments:                                                               *
 *                                                                           *
 *   L_32      : 32 bit integer.                                             *
 *               0x8000 0000 <= L_32 <= 0x7fff ffff.                         *
 *   hi        : b16 to b31 of L_32                                          *
 *   lo        : (L_32 - hi<<16)>>1                                          *
 *****************************************************************************
*/

void L_Extract (Word32 L_32, Word16 *hi, Word16 *lo)
{
    *hi = extract_h (L_32);
    *lo = extract_l (L_msu (L_shr (L_32, 1), *hi, 16384));
    return;
}

/*****************************************************************************
 *                                                                           *
 *  Function L_Comp()                                                        *
 *                                                                           *
 *  Compose from two 16 bit DPF a 32 bit integer.                            *
 *                                                                           *
 *     L_32 = hi<<16 + lo<<1                                                 *
 *                                                                           *
 *  Arguments:                                                               *
 *                                                                           *
 *   hi        msb                                                           *
 *   lo        lsf (with sign)                                               *
 *                                                                           *
 *   Return Value :                                                          *
 *                                                                           *
 *             32 bit long signed integer (Word32) whose value falls in the  *
 *             range : 0x8000 0000 <= L_32 <= 0x7fff fff0.                   *
 *                                                                           *
 *****************************************************************************
*/

Word32 L_Comp (Word16 hi, Word16 lo)
{
    Word32 L_32;

    L_32 = L_deposit_h (hi);
    return (L_mac (L_32, lo, 1));       /* = hi<<16 + lo<<1 */
}

/*****************************************************************************
 * Function Mpy_32()                                                         *
 *                                                                           *
 *   Multiply two 32 bit integers (DPF). The result is divided by 2**31      *
 *                                                                           *
 *   L_32 = (hi1*hi2)<<1 + ( (hi1*lo2)>>15 + (lo1*hi2)>>15 )<<1              *
 *                                                                           *
 *   This operation can also be viewed as the multiplication of two Q31      *
 *   number and the result is also in Q31.                                   *
 *                                                                           *
 * Arguments:                                                                *
 *                                                                           *
 *  hi1         hi part of first number                                      *
 *  lo1         lo part of first number                                      *
 *  hi2         hi part of second number                                     *
 *  lo2         lo part of second number                                     *
 *                                                                           *
 *****************************************************************************
*/

Word32 Mpy_32 (Word16 hi1, Word16 lo1, Word16 hi2, Word16 lo2)
{
    Word32 L_32;

    L_32 = L_mult (hi1, hi2);
    L_32 = L_mac (L_32, mult (hi1, lo2), 1);
    L_32 = L_mac (L_32, mult (lo1, hi2), 1);

    return (L_32);
}

/*****************************************************************************
 * Function Mpy_32_16()                                                      *
 *                                                                           *
 *   Multiply a 16 bit integer by a 32 bit (DPF). The result is divided      *
 *   by 2**15                                                                *
 *                                                                           *
 *                                                                           *
 *   L_32 = (hi1*lo2)<<1 + ((lo1*lo2)>>15)<<1                                *
 *                                                                           *
 * Arguments:                                                                *
 *                                                                           *
 *  hi          hi part of 32 bit number.                                    *
 *  lo          lo part of 32 bit number.                                    *
 *  n           16 bit number.                                               *
 *                                                                           *
 *****************************************************************************
*/

Word32 Mpy_32_16 (Word16 hi, Word16 lo, Word16 n)
{
    Word32 L_32;

    L_32 = L_mult (hi, n);
    L_32 = L_mac (L_32, mult (lo, n), 1);

    return (L_32);
}

/*****************************************************************************
 *                                                                           *
 *   Function Name : Div_32                                                  *
 *                                                                           *
 *   Purpose :                                                               *
 *             Fractional integer division of two 32 bit numbers.            *
 *             L_num / L_denom.                                              *
 *             L_num and L_denom must be positive and L_num < L_denom.       *
 *             L_denom = denom_hi<<16 + denom_lo<<1                          *
 *             denom_hi is a normalize number.                               *
 *                                                                           *
 *   Inputs :                                                                *
 *                                                                           *
 *    L_num                                                                  *
 *             32 bit long signed integer (Word32) whose value falls in the  *
 *             range : 0x0000 0000 < L_num < L_denom                         *
 *                                                                           *
 *    L_denom = denom_hi<<16 + denom_lo<<1      (DPF)                        *
 *                                                                           *
 *       denom_hi                                                            *
 *             16 bit positive normalized integer whose value falls in the   *
 *             range : 0x4000 < hi < 0x7fff                                  *
 *       denom_lo                                                            *
 *             16 bit positive integer whose value falls in the              *
 *             range : 0 < lo < 0x7fff                                       *
 *                                                                           *
 *   Return Value :                                                          *
 *                                                                           *
 *    L_div                                                                  *
 *             32 bit long signed integer (Word32) whose value falls in the  *
 *             range : 0x0000 0000 <= L_div <= 0x7fff ffff.                  *
 *                                                                           *
 *  Algorithm:                                                               *
 *                                                                           *
 *  - find = 1/L_denom.                                                      *
 *      First approximation: approx = 1 / denom_hi                           *
 *      1/L_denom = approx * (2.0 - L_denom * approx )                       *
 *                                                                           *
 *  -  result = L_num * (1/L_denom)                                          *
 *****************************************************************************
*/

Word32 Div_32 (Word32 L_num, Word16 denom_hi, Word16 denom_lo)
{
    Word16 approx, hi, lo, n_hi, n_lo;
    Word32 L_32;

    /* First approximation: 1 / L_denom = 1/denom_hi */

    approx = div_s ((Word16) 0x3fff, denom_hi);

    /* 1/L_denom = approx * (2.0 - L_denom * approx) */

    L_32 = Mpy_32_16 (denom_hi, denom_lo, approx);

    L_32 = L_sub ((Word32) 0x7fffffffL, L_32);

    L_Extract (L_32, &hi, &lo);

    L_32 = Mpy_32_16 (hi, lo, approx);

    /* L_num * (1/L_denom) */

    L_Extract (L_32, &hi, &lo);
    L_Extract (L_num, &n_hi, &n_lo);
    L_32 = Mpy_32 (n_hi, n_lo, hi, lo);
    L_32 = L_shl (L_32, 2);

    return (L_32);
}

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

/*-------------------------------------------------------------------*
 *                         DECIM54.C								 *
 *-------------------------------------------------------------------*
 * Decim_12k8   : decimation of 16kHz signal to 12.8kHz.             *
 * Oversamp_16k : oversampling from 12.8kHz to 16kHz.                *
 *-------------------------------------------------------------------*/


#define FAC4   4
#define FAC5   5
#define INV_FAC5   6554                    /* 1/5 in Q15 */
#define DOWN_FAC  26215                    /* 4/5 in Q15 */
#define UP_FAC    20480                    /* 5/4 in Q14 */

#define NB_COEF_DOWN  15
#define NB_COEF_UP    12

/* Local functions */
static void Down_samp(
     Word16 * sig,                         /* input:  signal to downsampling  */
     Word16 * sig_d,                       /* output: downsampled signal      */
     Word16 L_frame_d                      /* input:  length of output        */
);
static void Up_samp(
     Word16 * sig_d,                       /* input:  signal to oversampling  */
     Word16 * sig_u,                       /* output: oversampled signal      */
     Word16 L_frame                        /* input:  length of output        */
);
static Word16 Interpol(                    /* return result of interpolation */
     Word16 * x,                           /* input vector                   */
     Word16 * fir,                         /* filter coefficient             */
     Word16 frac,                          /* fraction (0..resol)            */
     Word16 resol,                         /* resolution                     */
     Word16 nb_coef                        /* number of coefficients         */
);


/* 1/5 resolution interpolation filter  (in Q14)  */
/* -1.5dB @ 6kHz, -6dB @ 6.4kHz, -10dB @ 6.6kHz, -20dB @ 6.9kHz, -25dB @ 7kHz, -55dB @ 8kHz */

static Word16 fir_up[120] =
{
    -1, -4, -7, -6, 0,
    12, 24, 30, 23, 0,
    -33, -62, -73, -52, 0,
    68, 124, 139, 96, 0,
    -119, -213, -235, -160, 0,
    191, 338, 368, 247, 0,
    -291, -510, -552, -369, 0,
    430, 752, 812, 542, 0,
    -634, -1111, -1204, -809, 0,
    963, 1708, 1881, 1288, 0,
    -1616, -2974, -3432, -2496, 0,
    3792, 8219, 12368, 15317, 16384,
    15317, 12368, 8219, 3792, 0,
    -2496, -3432, -2974, -1616, 0,
    1288, 1881, 1708, 963, 0,
    -809, -1204, -1111, -634, 0,
    542, 812, 752, 430, 0,
    -369, -552, -510, -291, 0,
    247, 368, 338, 191, 0,
    -160, -235, -213, -119, 0,
    96, 139, 124, 68, 0,
    -52, -73, -62, -33, 0,
    23, 30, 24, 12, 0,
    -6, -7, -4, -1, 0
};

static Word16 fir_down[120] =
{            /* table x4/5 */
    -1, -3, -6, -5,
    0, 9, 19, 24,
    18, 0, -26, -50,
    -58, -41, 0, 54,
    99, 111, 77, 0,
    -95, -170, -188, -128,
    0, 153, 270, 294,
    198, 0, -233, -408,
    -441, -295, 0, 344,
    601, 649, 434, 0,
    -507, -888, -964, -647,
    0, 770, 1366, 1505,
    1030, 0, -1293, -2379,
    -2746, -1997, 0, 3034,
    6575, 9894, 12254, 13107,
    12254, 9894, 6575, 3034,
    0, -1997, -2746, -2379,
    -1293, 0, 1030, 1505,
    1366, 770, 0, -647,
    -964, -888, -507, 0,
    434, 649, 601, 344,
    0, -295, -441, -408,
    -233, 0, 198, 294,
    270, 153, 0, -128,
    -188, -170, -95, 0,
    77, 111, 99, 54,
    0, -41, -58, -50,
    -26, 0, 18, 24,
    19, 9, 0, -5,
    -6, -3, -1, 0
};



void Init_Decim_12k8(
     Word16 mem[]                          /* output: memory (2*NB_COEF_DOWN) set to zeros */
)
{
    Set_zero(mem, 2 * NB_COEF_DOWN);
    return;
}

void Decim_12k8(
     Word16 sig16k[],                      /* input:  signal to downsampling  */
     Word16 lg,                            /* input:  length of input         */
     Word16 sig12k8[],                     /* output: decimated signal        */
     Word16 mem[]                          /* in/out: memory (2*NB_COEF_DOWN) */
)
{
    Word16 lg_down;
    Word16 signal[L_FRAME16k + (2 * NB_COEF_DOWN)];

    Copy(mem, signal, 2 * NB_COEF_DOWN);

    Copy(sig16k, signal + (2 * NB_COEF_DOWN), lg);

    lg_down = mult(lg, DOWN_FAC);

    Down_samp(signal + NB_COEF_DOWN, sig12k8, lg_down);

    Copy(signal + lg, mem, 2 * NB_COEF_DOWN);

    return;
}


void Init_Oversamp_16k(
     Word16 mem[]                          /* output: memory (2*NB_COEF_UP) set to zeros  */
)
{
    Set_zero(mem, 2 * NB_COEF_UP);
    return;
}



static void Down_samp(
     Word16 * sig,                         /* input:  signal to downsampling  */
     Word16 * sig_d,                       /* output: downsampled signal      */
     Word16 L_frame_d                      /* input:  length of output        */
)
{
    Word16 i, j, frac, pos;

    pos = 0;                                 /* position is in Q2 -> 1/4 resolution  */
    for (j = 0; j < L_frame_d; j++)
    {
        i = shr(pos, 2);                   /* integer part     */
        frac = (Word16) (pos & 3);           /* fractional part */

        sig_d[j] = Interpol(&sig[i], fir_down, frac, FAC4, NB_COEF_DOWN);       

        pos = add(pos, FAC5);              /* pos + 5/4 */
    }

    return;
}


static void Up_samp(
     Word16 * sig_d,                       /* input:  signal to oversampling  */
     Word16 * sig_u,                       /* output: oversampled signal      */
     Word16 L_frame                        /* input:  length of output        */
)
{
    Word16 i, j, pos, frac;

    pos = 0;                                 /* position with 1/5 resolution */

    for (j = 0; j < L_frame; j++)
    {
        i = mult(pos, INV_FAC5);           /* integer part = pos * 1/5 */
        frac = sub(pos, add(shl(i, 2), i));/* frac = pos - (pos/5)*5   */

        sig_u[j] = Interpol(&sig_d[i], fir_up, frac, FAC5, NB_COEF_UP); 

        pos = add(pos, FAC4);              /* position + 4/5 */
    }

    return;
}

/* Fractional interpolation of signal at position (frac/resol) */

static Word16 Interpol(                    /* return result of interpolation */
     Word16 * x,                           /* input vector                   */
     Word16 * fir,                         /* filter coefficient             */
     Word16 frac,                          /* fraction (0..resol)            */
     Word16 resol,                         /* resolution                     */
     Word16 nb_coef                        /* number of coefficients         */
)
{
    Word16 i, k;
    Word32 L_sum;

    x = x - nb_coef + 1;                   

    L_sum = 0L;                            
    for (i = 0, k = sub(sub(resol, 1), frac); i < 2 * nb_coef; i++, k = (Word16) (k + resol))
    {
        L_sum = L_mac(L_sum, x[i], fir[k]);
    }
    L_sum = L_shl(L_sum, 1);               /* saturation can occur here */

    return (round(L_sum));
}

/*-----------------------------------------------------------------------*
 *                         HP50.C										 *
 *-----------------------------------------------------------------------*
 * 2nd order high pass filter with cut off frequency at 31 Hz.           *
 * Designed with cheby2 function in MATLAB.                              *
 * Optimized for fixed-point to get the following frequency response:    *
 *                                                                       *
 *  frequency:     0Hz    14Hz  24Hz   31Hz   37Hz   41Hz   47Hz         *
 *  dB loss:     -infdB  -15dB  -6dB   -3dB  -1.5dB  -1dB  -0.5dB        *
 *                                                                       *
 * Algorithm:                                                            *
 *                                                                       *
 *  y[i] = b[0]*x[i] + b[1]*x[i-1] + b[2]*x[i-2]                         *
 *                   + a[1]*y[i-1] + a[2]*y[i-2];                        *
 *                                                                       *
 *  Word16 b[3] = {4053, -8106, 4053};       in Q12                     *
 *  Word16 a[3] = {8192, 16211, -8021};       in Q12                     *
 *                                                                       *
 *  float -->   b[3] = {0.989501953, -1.979003906,  0.989501953};        *
 *              a[3] = {1.000000000,  1.978881836, -0.979125977};        *
 *-----------------------------------------------------------------------*/




/* Initialization of static values */

void Init_HP50_12k8(Word16 mem[])
{
    Set_zero(mem, 6);
}


void HP50_12k8(
     Word16 signal[],                      /* input/output signal */
     Word16 lg,                            /* lenght of signal    */
     Word16 mem[]                          /* filter memory [6]   */
)
{
    Word16 i, x2;
    Word16 y2_hi, y2_lo, y1_hi, y1_lo, x0, x1;
    Word32 L_tmp;
	/* filter coefficients	*/
	static Word16 b[3] = {4053, -8106, 4053};  /* Q12 */
	static Word16 a[3] = {8192, 16211, -8021}; /* Q12 (x2) */

    y2_hi = mem[0];                        
    y2_lo = mem[1];                        
    y1_hi = mem[2];                        
    y1_lo = mem[3];                        
    x0 = mem[4];                           
    x1 = mem[5];                           

    for (i = 0; i < lg; i++)
    {
        x2 = x1;                           
        x1 = x0;                           
        x0 = signal[i];                    

        /* y[i] = b[0]*x[i] + b[1]*x[i-1] + b140[2]*x[i-2]  */
        /* + a[1]*y[i-1] + a[2] * y[i-2];  */

        
        L_tmp = 16384L;                    /* rounding to maximise precision */
        L_tmp = L_mac(L_tmp, y1_lo, a[1]);
        L_tmp = L_mac(L_tmp, y2_lo, a[2]);
        L_tmp = L_shr(L_tmp, 15);
        L_tmp = L_mac(L_tmp, y1_hi, a[1]);
        L_tmp = L_mac(L_tmp, y2_hi, a[2]);
        L_tmp = L_mac(L_tmp, x0, b[0]);
        L_tmp = L_mac(L_tmp, x1, b[1]);
        L_tmp = L_mac(L_tmp, x2, b[2]);

        L_tmp = L_shl(L_tmp, 2);           /* coeff Q12 --> Q14 */

        y2_hi = y1_hi;                     
        y2_lo = y1_lo;                     
        L_Extract(L_tmp, &y1_hi, &y1_lo);

        L_tmp = L_shl(L_tmp, 1);           /* coeff Q14 --> Q15 with saturation */
        signal[i] = round(L_tmp);          
    }

    mem[0] = y2_hi;                        
    mem[1] = y2_lo;                        
    mem[2] = y1_hi;                        
    mem[3] = y1_lo;                        
    mem[4] = x0;                           
    mem[5] = x1;                           

    return;
}

/*-------------------------------------------------------------------*
 *                         SCALE.C									 *
 *-------------------------------------------------------------------*
 * Scale signal to get maximum of dynamic.							 *
 *-------------------------------------------------------------------*/

void Scale_sig(
     Word16 x[],                           /* (i/o) : signal to scale               */
     Word16 lg,                            /* (i)   : size of x[]                   */
     Word16 exp                            /* (i)   : exponent: x = round(x << exp) */
)
{
    Word16 i;
    Word32 L_tmp;

    for (i = 0; i < lg; i++)
    {
        L_tmp = L_deposit_h(x[i]);
        L_tmp = L_shl(L_tmp, exp);         /* saturation can occur here */
        x[i] = round(L_tmp);               
    }

    return;
}

/*------------------------------------------------------------------------*
 *                         AUTOCORR.C                                     *
 *------------------------------------------------------------------------*
 *   Compute autocorrelations of signal with windowing                    *
 *                                                                        *
 *------------------------------------------------------------------------*/


#include "ham_wind.tab"


void Autocorr(
     Word16 x[],                           /* (i)    : Input signal                      */
     Word16 m,                             /* (i)    : LPC order                         */
     Word16 r_h[],                         /* (o) Q15: Autocorrelations  (msb)           */
     Word16 r_l[]                          /* (o)    : Autocorrelations  (lsb)           */
)
{
    Word16 i, j, norm, shift, y[L_WINDOW];
    Word32 L_sum, L_tmp;

    /* Windowing of signal */

    for (i = 0; i < L_WINDOW; i++)
    {
        y[i] = mult_r(x[i], window[i]);    
    }

    /* calculate energy of signal */

    L_sum = L_deposit_h(16);               /* sqrt(256), avoid overflow after rounding */
    for (i = 0; i < L_WINDOW; i++)
    {
        L_tmp = L_mult(y[i], y[i]);
        L_tmp = L_shr(L_tmp, 8);
        L_sum = L_add(L_sum, L_tmp);
    }

    /* scale signal to avoid overflow in autocorrelation */

    norm = norm_l(L_sum);
    shift = sub(4, shr(norm, 1));
    
    if (shift < 0)
    {
        shift = 0;                         
    }
    for (i = 0; i < L_WINDOW; i++)
    {
        y[i] = shr_r(y[i], shift);         
    }

    /* Compute and normalize r[0] */

    L_sum = 1;                             
    for (i = 0; i < L_WINDOW; i++)
        L_sum = L_mac(L_sum, y[i], y[i]);

    norm = norm_l(L_sum);
    L_sum = L_shl(L_sum, norm);
    L_Extract(L_sum, &r_h[0], &r_l[0]);    /* Put in DPF format (see oper_32b) */

    /* Compute r[1] to r[m] */

    for (i = 1; i <= m; i++)
    {
        L_sum = 0;                         
        for (j = 0; j < L_WINDOW - i; j++)
            L_sum = L_mac(L_sum, y[j], y[j + i]);

        L_sum = L_shl(L_sum, norm);
        L_Extract(L_sum, &r_h[i], &r_l[i]);
    }

    return;
}

/*---------------------------------------------------------*
 *                         LAG_WIND.C					   *
 *---------------------------------------------------------*
 * Lag_window on autocorrelations.                         *
 *                                                         *
 * r[i] *= lag_wind[i]                                     *
 *                                                         *
 *  r[i] and lag_wind[i] are in special double precision.  *
 *  See "oper_32b.c" for the format                        *
 *---------------------------------------------------------*/


#include "lag_wind.tab"


void Lag_window(
     Word16 r_h[],                         /* (i/o)   : Autocorrelations  (msb)          */
     Word16 r_l[]                          /* (i/o)   : Autocorrelations  (lsb)          */
)
{
    Word16 i;
    Word32 x;

    for (i = 1; i <= M; i++)
    {
        x = Mpy_32(r_h[i], r_l[i], lag_h[i - 1], lag_l[i - 1]);
        L_Extract(x, &r_h[i], &r_l[i]);
    }
    return;
}

/*---------------------------------------------------------------------------*
 *                         LEVINSON.C										 *
 *---------------------------------------------------------------------------*
 *                                                                           *
 *      LEVINSON-DURBIN algorithm in double precision                        *
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                        *
 *                                                                           *
 * Algorithm                                                                 *
 *                                                                           *
 *       R[i]    autocorrelations.                                           *
 *       A[i]    filter coefficients.                                        *
 *       K       reflection coefficients.                                    *
 *       Alpha   prediction gain.                                            *
 *                                                                           *
 *       Initialization:                                                     *
 *               A[0] = 1                                                    *
 *               K    = -R[1]/R[0]                                           *
 *               A[1] = K                                                    *
 *               Alpha = R[0] * (1-K**2]                                     *
 *                                                                           *
 *       Do for  i = 2 to M                                                  *
 *                                                                           *
 *            S =  SUM ( R[j]*A[i-j] ,j=1,i-1 ) +  R[i]                      *
 *                                                                           *
 *            K = -S / Alpha                                                 *
 *                                                                           *
 *            An[j] = A[j] + K*A[i-j]   for j=1 to i-1                       *
 *                                      where   An[i] = new A[i]             *
 *            An[i]=K                                                        *
 *                                                                           *
 *            Alpha=Alpha * (1-K**2)                                         *
 *                                                                           *
 *       END                                                                 *
 *                                                                           *
 * Remarks on the dynamics of the calculations.                              *
 *                                                                           *
 *       The numbers used are in double precision in the following format :  *
 *       A = AH <<16 + AL<<1.  AH and AL are 16 bit signed integers.         *
 *       Since the LSB's also contain a sign bit, this format does not       *
 *       correspond to standard 32 bit integers.  We use this format since   *
 *       it allows fast execution of multiplications and divisions.          *
 *                                                                           *
 *       "DPF" will refer to this special format in the following text.      *
 *       See oper_32b.c                                                      *
 *                                                                           *
 *       The R[i] were normalized in routine AUTO (hence, R[i] < 1.0).       *
 *       The K[i] and Alpha are theoretically < 1.0.                         *
 *       The A[i], for a sampling frequency of 8 kHz, are in practice        *
 *       always inferior to 16.0.                                            *
 *                                                                           *
 *       These characteristics allow straigthforward fixed-point             *
 *       implementation.  We choose to represent the parameters as           *
 *       follows :                                                           *
 *                                                                           *
 *               R[i]    Q31   +- .99..                                      *
 *               K[i]    Q31   +- .99..                                      *
 *               Alpha   Normalized -> mantissa in Q31 plus exponent         *
 *               A[i]    Q27   +- 15.999..                                   *
 *                                                                           *
 *       The additions are performed in 32 bit.  For the summation used      *
 *       to calculate the K[i], we multiply numbers in Q31 by numbers        *
 *       in Q27, with the result of the multiplications in Q27,              *
 *       resulting in a dynamic of +- 16.  This is sufficient to avoid       *
 *       overflow, since the final result of the summation is                *
 *       necessarily < 1.0 as both the K[i] and Alpha are                    *
 *       theoretically < 1.0.                                                *
 *___________________________________________________________________________*/


#define M   16
#define NC  (M/2)

void Init_Levinson(
     Word16 * mem                          /* output  :static memory (18 words) */
)
{
    Set_zero(mem, 18);                     /* old_A[0..M-1] = 0, old_rc[0..1] = 0 */
    return;
}


void Levinson(
     Word16 Rh[],                          /* (i)     : Rh[M+1] Vector of autocorrelations (msb) */
     Word16 Rl[],                          /* (i)     : Rl[M+1] Vector of autocorrelations (lsb) */
     Word16 A[],                           /* (o) Q12 : A[M]    LPC coefficients  (m = 16)       */
     Word16 rc[],                          /* (o) Q15 : rc[M]   Reflection coefficients.         */
     Word16 * mem                          /* (i/o)   :static memory (18 words)                  */
)
{
    Word16 i, j;
    Word16 hi, lo;
    Word16 Kh, Kl;                         /* reflection coefficient; hi and lo           */
    Word16 alp_h, alp_l, alp_exp;          /* Prediction gain; hi lo and exponent         */
    Word16 Ah[M + 1], Al[M + 1];           /* LPC coef. in double prec.                   */
    Word16 Anh[M + 1], Anl[M + 1];         /* LPC coef.for next iteration in double prec. */
    Word32 t0, t1, t2;                     /* temporary variable                          */
    Word16 *old_A, *old_rc;

    /* Last A(z) for case of unstable filter */

    old_A = mem;                           
    old_rc = mem + M;                      

    /* K = A[1] = -R[1] / R[0] */

    t1 = L_Comp(Rh[1], Rl[1]);             /* R[1] in Q31      */
    t2 = L_abs(t1);                        /* abs R[1]         */
    t0 = Div_32(t2, Rh[0], Rl[0]);         /* R[1]/R[0] in Q31 */
    
    if (t1 > 0)
        t0 = L_negate(t0);                 /* -R[1]/R[0]       */
    L_Extract(t0, &Kh, &Kl);               /* K in DPF         */
    rc[0] = Kh;                            
    t0 = L_shr(t0, 4);                     /* A[1] in Q27      */
    L_Extract(t0, &Ah[1], &Al[1]);         /* A[1] in DPF      */

    /* Alpha = R[0] * (1-K**2) */

    t0 = Mpy_32(Kh, Kl, Kh, Kl);           /* K*K      in Q31 */
    t0 = L_abs(t0);                        /* Some case <0 !! */
    t0 = L_sub((Word32) 0x7fffffffL, t0);  /* 1 - K*K  in Q31 */
    L_Extract(t0, &hi, &lo);               /* DPF format      */
    t0 = Mpy_32(Rh[0], Rl[0], hi, lo);     /* Alpha in Q31    */

    /* Normalize Alpha */

    alp_exp = norm_l(t0);
    t0 = L_shl(t0, alp_exp);
    L_Extract(t0, &alp_h, &alp_l);
    /* DPF format    */

    /*--------------------------------------*
     * ITERATIONS  I=2 to M                 *
     *--------------------------------------*/

    for (i = 2; i <= M; i++)
    {

        /* t0 = SUM ( R[j]*A[i-j] ,j=1,i-1 ) +  R[i] */

        t0 = 0;                            
        for (j = 1; j < i; j++)
            t0 = L_add(t0, Mpy_32(Rh[j], Rl[j], Ah[i - j], Al[i - j]));

        t0 = L_shl(t0, 4);                 /* result in Q27 -> convert to Q31 */
        /* No overflow possible            */
        t1 = L_Comp(Rh[i], Rl[i]);
        t0 = L_add(t0, t1);                /* add R[i] in Q31                 */

        /* K = -t0 / Alpha */

        t1 = L_abs(t0);
        t2 = Div_32(t1, alp_h, alp_l);     /* abs(t0)/Alpha                   */
        
        if (t0 > 0)
            t2 = L_negate(t2);             /* K =-t0/Alpha                    */
        t2 = L_shl(t2, alp_exp);           /* denormalize; compare to Alpha   */
        L_Extract(t2, &Kh, &Kl);           /* K in DPF                        */
        rc[i - 1] = Kh;                    

        /* Test for unstable filter. If unstable keep old A(z) */

        
        if (sub(abs_s(Kh), 32750) > 0)
        {
            A[0] = 4096;                     /* Ai[0] not stored (always 1.0) */
            for (j = 0; j < M; j++)
            {
                A[j + 1] = old_A[j];       
            }
            rc[0] = old_rc[0];             /* only two rc coefficients are needed */
            rc[1] = old_rc[1];
            
            return;
        }
        /*------------------------------------------*
         *  Compute new LPC coeff. -> An[i]         *
         *  An[j]= A[j] + K*A[i-j]     , j=1 to i-1 *
         *  An[i]= K                                *
         *------------------------------------------*/

        for (j = 1; j < i; j++)
        {
            t0 = Mpy_32(Kh, Kl, Ah[i - j], Al[i - j]);
            t0 = L_add(t0, L_Comp(Ah[j], Al[j]));
            L_Extract(t0, &Anh[j], &Anl[j]);
        }
        t2 = L_shr(t2, 4);                 /* t2 = K in Q31 ->convert to Q27  */
        L_Extract(t2, &Anh[i], &Anl[i]);   /* An[i] in Q27                    */

        /* Alpha = Alpha * (1-K**2) */

        t0 = Mpy_32(Kh, Kl, Kh, Kl);       /* K*K      in Q31 */
        t0 = L_abs(t0);                    /* Some case <0 !! */
        t0 = L_sub((Word32) 0x7fffffffL, t0);   /* 1 - K*K  in Q31 */
        L_Extract(t0, &hi, &lo);           /* DPF format      */
        t0 = Mpy_32(alp_h, alp_l, hi, lo); /* Alpha in Q31    */

        /* Normalize Alpha */

        j = norm_l(t0);
        t0 = L_shl(t0, j);
        L_Extract(t0, &alp_h, &alp_l);     /* DPF format    */
        alp_exp = add(alp_exp, j);         /* Add normalization to alp_exp */

        /* A[j] = An[j] */

        for (j = 1; j <= i; j++)
        {
            Ah[j] = Anh[j];                
            Al[j] = Anl[j];                
        }
    }

    /* Truncate A[i] in Q27 to Q12 with rounding */

    A[0] = 4096;                           
    for (i = 1; i <= M; i++)
    {
        t0 = L_Comp(Ah[i], Al[i]);
        old_A[i - 1] = A[i] = round(L_shl(t0, 1));      
    }
    old_rc[0] = rc[0];                     
    old_rc[1] = rc[1];                     

    return;
}

/*-----------------------------------------------------------------------*
 *                         Az_isp.C                                      *
 *-----------------------------------------------------------------------*
 * Compute the ISPs from  the LPC coefficients  (order=M)                *
 *-----------------------------------------------------------------------*
 *                                                                       *
 * The ISPs are the roots of the two polynomials F1(z) and F2(z)         *
 * defined as                                                            *
 *               F1(z) = A(z) + z^-m A(z^-1)                             *
 *  and          F2(z) = A(z) - z^-m A(z^-1)                             *
 *                                                                       *
 * For a even order m=2n, F1(z) has M/2 conjugate roots on the unit      *
 * circle and F2(z) has M/2-1 conjugate roots on the unit circle in      *
 * addition to two roots at 0 and pi.                                    *
 *                                                                       *
 * For a 16th order LP analysis, F1(z) and F2(z) can be written as       *
 *                                                                       *
 *   F1(z) = (1 + a[M])   PRODUCT  (1 - 2 cos(w_i) z^-1 + z^-2 )         *
 *                        i=0,2,4,6,8,10,12,14                           *
 *                                                                       *
 *   F2(z) = (1 - a[M]) (1 - z^-2) PRODUCT (1 - 2 cos(w_i) z^-1 + z^-2 ) *
 *                                 i=1,3,5,7,9,11,13                     *
 *                                                                       *
 * The ISPs are the M-1 frequencies w_i, i=0...M-2 plus the last         *
 * predictor coefficient a[M].                                           *
 *-----------------------------------------------------------------------*/


#include "grid100.tab"

#define M   16
#define NC  (M/2)

/* local function */
static Word16 Chebps2(Word16 x, Word16 f[], Word16 n);

void Az_isp(
     Word16 a[],                           /* (i) Q12 : predictor coefficients                 */
     Word16 isp[],                         /* (o) Q15 : Immittance spectral pairs              */
     Word16 old_isp[]                      /* (i)     : old isp[] (in case not found M roots)  */
)
{
    Word16 i, j, nf, ip, order;
    Word16 xlow, ylow, xhigh, yhigh, xmid, ymid, xint;
    Word16 x, y, sign, exp;
    Word16 *coef;
    Word16 f1[NC + 1], f2[NC];
    Word32 t0;

    /*-------------------------------------------------------------*
     * find the sum and diff polynomials F1(z) and F2(z)           *
     *      F1(z) = [A(z) + z^M A(z^-1)]                           *
     *      F2(z) = [A(z) - z^M A(z^-1)]/(1-z^-2)                  *
     *                                                             *
     * for (i=0; i<NC; i++)                                        *
     * {                                                           *
     *   f1[i] = a[i] + a[M-i];                                    *
     *   f2[i] = a[i] - a[M-i];                                    *
     * }                                                           *
     * f1[NC] = 2.0*a[NC];                                         *
     *                                                             *
     * for (i=2; i<NC; i++)            Divide by (1-z^-2)          *
     *   f2[i] += f2[i-2];                                         *
     *-------------------------------------------------------------*/

    for (i = 0; i < NC; i++)
    {
        t0 = L_mult(a[i], 16384);
        f1[i] = round(L_mac(t0, a[M - i], 16384));        /* =(a[i]+a[M-i])/2 */
        f2[i] = round(L_msu(t0, a[M - i], 16384));        /* =(a[i]-a[M-i])/2 */
    }
    f1[NC] = a[NC];                        

    for (i = 2; i < NC; i++)               /* Divide by (1-z^-2) */
        f2[i] = add(f2[i], f2[i - 2]);     

    /*---------------------------------------------------------------------*
     * Find the ISPs (roots of F1(z) and F2(z) ) using the                 *
     * Chebyshev polynomial evaluation.                                    *
     * The roots of F1(z) and F2(z) are alternatively searched.            *
     * We start by finding the first root of F1(z) then we switch          *
     * to F2(z) then back to F1(z) and so on until all roots are found.    *
     *                                                                     *
     *  - Evaluate Chebyshev pol. at grid points and check for sign change.*
     *  - If sign change track the root by subdividing the interval        *
     *    2 times and ckecking sign change.                                *
     *---------------------------------------------------------------------*/

    nf = 0;                                  /* number of found frequencies */
    ip = 0;                                  /* indicator for f1 or f2      */

    coef = f1;                             
    order = NC;                            

    xlow = grid[0];                        
    ylow = Chebps2(xlow, coef, order);

    j = 0;
    
    while ((nf < M - 1) && (j < GRID_POINTS))
    {
        j = add(j, 1);
        xhigh = xlow;                      
        yhigh = ylow;                      
        xlow = grid[j];                    
        ylow = Chebps2(xlow, coef, order);

        
        if (L_mult(ylow, yhigh) <= (Word32) 0)
        {
            /* divide 2 times the interval */

            for (i = 0; i < 2; i++)
            {
                xmid = add(shr(xlow, 1), shr(xhigh, 1));        /* xmid = (xlow + xhigh)/2 */

                ymid = Chebps2(xmid, coef, order);

                
                if (L_mult(ylow, ymid) <= (Word32) 0)
                {
                    yhigh = ymid;          
                    xhigh = xmid;          
                } else
                {
                    ylow = ymid;           
                    xlow = xmid;           
                }
            }

            /*-------------------------------------------------------------*
             * Linear interpolation                                        *
             *    xint = xlow - ylow*(xhigh-xlow)/(yhigh-ylow);            *
             *-------------------------------------------------------------*/

            x = sub(xhigh, xlow);
            y = sub(yhigh, ylow);

            
            if (y == 0)
            {
                xint = xlow;               
            } else
            {
                sign = y;                  
                y = abs_s(y);
                exp = norm_s(y);
                y = shl(y, exp);
                y = div_s((Word16) 16383, y);
                t0 = L_mult(x, y);
                t0 = L_shr(t0, sub(20, exp));
                y = extract_l(t0);         /* y= (xhigh-xlow)/(yhigh-ylow) in Q11 */

                
                if (sign < 0)
                    y = negate(y);

                t0 = L_mult(ylow, y);      /* result in Q26 */
                t0 = L_shr(t0, 11);        /* result in Q15 */
                xint = sub(xlow, extract_l(t0));        /* xint = xlow - ylow*y */
            }

            isp[nf] = xint;                
            xlow = xint;                   
            nf++;                          

            
            if (ip == 0)
            {
                ip = 1;                    
                coef = f2;                 
                order = NC - 1;            
            } else
            {
                ip = 0;                    
                coef = f1;                 
                order = NC;                
            }
            ylow = Chebps2(xlow, coef, order);
        }
        
    }

    /* Check if M-1 roots found */

    
    if (sub(nf, M - 1) < 0)
    {
        for (i = 0; i < M; i++)
        {
            isp[i] = old_isp[i];           
        }
    } else
    {
        isp[M - 1] = shl(a[M], 3);           /* From Q12 to Q15 with saturation */
    }

    return;
}


/*--------------------------------------------------------------*
 * function  Chebps2:                                           *
 *           ~~~~~~~                                            *
 *    Evaluates the Chebishev polynomial series                 *
 *--------------------------------------------------------------*
 *                                                              *
 *  The polynomial order is                                     *
 *     n = M/2   (M is the prediction order)                    *
 *  The polynomial is given by                                  *
 *    C(x) = f(0)T_n(x) + f(1)T_n-1(x) + ... +f(n-1)T_1(x) + f(n)/2 *
 * Arguments:                                                   *
 *  x:     input value of evaluation; x = cos(frequency) in Q15 *
 *  f[]:   coefficients of the pol.                      in Q11 *
 *  n:     order of the pol.                                    *
 *                                                              *
 * The value of C(x) is returned. (Satured to +-1.99 in Q14)    *
 *                                                              *
 *--------------------------------------------------------------*/

static Word16 Chebps2(Word16 x, Word16 f[], Word16 n)
{
    Word16 i, cheb;
    Word16 b0_h, b0_l, b1_h, b1_l, b2_h, b2_l;
    Word32 t0;

    /* Note: All computation are done in Q24. */

    t0 = L_mult(f[0], 4096);
    L_Extract(t0, &b2_h, &b2_l);           /* b2 = f[0] in Q24 DPF */

    t0 = Mpy_32_16(b2_h, b2_l, x);         /* t0 = 2.0*x*b2        */
    t0 = L_shl(t0, 1);
    t0 = L_mac(t0, f[1], 4096);            /* + f[1] in Q24        */
    L_Extract(t0, &b1_h, &b1_l);           /* b1 = 2*x*b2 + f[1]   */

    for (i = 2; i < n; i++)
    {
        t0 = Mpy_32_16(b1_h, b1_l, x);     /* t0 = 2.0*x*b1              */

        t0 = L_mac(t0, b2_h, -16384);
        t0 = L_mac(t0, f[i], 2048);
        t0 = L_shl(t0, 1);
        t0 = L_msu(t0, b2_l, 1);           /* t0 = 2.0*x*b1 - b2 + f[i]; */

        L_Extract(t0, &b0_h, &b0_l);       /* b0 = 2.0*x*b1 - b2 + f[i]; */

        b2_l = b1_l;                         /* b2 = b1; */
        b2_h = b1_h;                       
        b1_l = b0_l;                         /* b1 = b0; */
        b1_h = b0_h;                       
    }

    t0 = Mpy_32_16(b1_h, b1_l, x);         /* t0 = x*b1;              */
    t0 = L_mac(t0, b2_h, (Word16) - 32768);/* t0 = x*b1 - b2          */
    t0 = L_msu(t0, b2_l, 1);
    t0 = L_mac(t0, f[n], 2048);            /* t0 = x*b1 - b2 + f[i]/2 */

    t0 = L_shl(t0, 6);                     /* Q24 to Q30 with saturation */

    cheb = extract_h(t0);                  /* Result in Q14              */

    
    if (sub(cheb, -32768) == 0)
    {
        cheb = -32767;                     /* to avoid saturation in Az_isp */
        
    }
    return (cheb);
}

/*-----------------------------------------------------------------------*
 *                         ISP_AZ.C										 *
 *-----------------------------------------------------------------------*
 * Compute the LPC coefficients from isp (order=M)						 *
 *-----------------------------------------------------------------------*/



#define NC (M/2)
#define NC16k (M16k/2)

/* local function */

static void Get_isp_pol(Word16 * isp, Word32 * f, Word16 n);
static void Get_isp_pol_16kHz(Word16 * isp, Word32 * f, Word16 n);

void Isp_Az(
     Word16 isp[],                         /* (i) Q15 : Immittance spectral pairs            */
     Word16 a[],                           /* (o) Q12 : predictor coefficients (order = M)   */
     Word16 m,
     Word16 adaptive_scaling               /* (i) 0   : adaptive scaling disabled */
                                           /*     1   : adaptive scaling enabled  */
)
{
    Word16 i, j, hi, lo;
    Word32 f1[NC16k + 1], f2[NC16k];
    Word16 nc;
    Word32 t0;
    Word16 q, q_sug;
    Word32 tmax;

    nc = shr(m, 1);
    
    if (sub(nc, 8) > 0)
    {
        Get_isp_pol_16kHz(&isp[0], f1, nc);
        for (i = 0; i <= nc; i++)
        {
            f1[i] = L_shl(f1[i], 2);       
        }
    } else
        Get_isp_pol(&isp[0], f1, nc);

    
    if (sub(nc, 8) > 0)
    {
        Get_isp_pol_16kHz(&isp[1], f2, sub(nc, 1));
        for (i = 0; i <= nc - 1; i++)
        {
            f2[i] = L_shl(f2[i], 2);       
        }
    } else
        Get_isp_pol(&isp[1], f2, sub(nc, 1));

    /*-----------------------------------------------------*
     *  Multiply F2(z) by (1 - z^-2)                       *
     *-----------------------------------------------------*/

    for (i = sub(nc, 1); i > 1; i--)
    {
        f2[i] = L_sub(f2[i], f2[i - 2]);     /* f2[i] -= f2[i-2]; */
    }

    /*----------------------------------------------------------*
     *  Scale F1(z) by (1+isp[m-1])  and  F2(z) by (1-isp[m-1]) *
     *----------------------------------------------------------*/

    for (i = 0; i < nc; i++)
    {
        /* f1[i] *= (1.0 + isp[M-1]); */

        L_Extract(f1[i], &hi, &lo);
        t0 = Mpy_32_16(hi, lo, isp[m - 1]);
        f1[i] = L_add(f1[i], t0);          

        /* f2[i] *= (1.0 - isp[M-1]); */

        L_Extract(f2[i], &hi, &lo);
        t0 = Mpy_32_16(hi, lo, isp[m - 1]);
        f2[i] = L_sub(f2[i], t0);          
    }

    /*-----------------------------------------------------*
     *  A(z) = (F1(z)+F2(z))/2                             *
     *  F1(z) is symmetric and F2(z) is antisymmetric      *
     *-----------------------------------------------------*/

    /* a[0] = 1.0; */
    a[0] = 4096;                           
    tmax = 1;                              
    for (i = 1, j = sub(m, 1); i < nc; i++, j--)
    {
        /* a[i] = 0.5*(f1[i] + f2[i]); */

        t0 = L_add(f1[i], f2[i]);          /* f1[i] + f2[i]             */
        tmax |= L_abs(t0);                 logic32();
        a[i] = extract_l(L_shr_r(t0, 12)); /* from Q23 to Q12 and * 0.5 */
        

        /* a[j] = 0.5*(f1[i] - f2[i]); */

        t0 = L_sub(f1[i], f2[i]);          /* f1[i] - f2[i]             */
        tmax |= L_abs(t0);                 logic32();
        a[j] = extract_l(L_shr_r(t0, 12)); /* from Q23 to Q12 and * 0.5 */
        
    }

    /* rescale data if overflow has occured and reprocess the loop */

    
    if ( sub(adaptive_scaling, 1) == 0 )
       q = sub(4, norm_l(tmax));        /* adaptive scaling enabled */
    else
       q = 0;                  /* adaptive scaling disabled */

    
    if (q > 0)
    {
      q_sug = add(12, q);
      for (i = 1, j = sub(m, 1); i < nc; i++, j--)
        {
          /* a[i] = 0.5*(f1[i] + f2[i]); */

          t0 = L_add(f1[i], f2[i]);          /* f1[i] + f2[i]             */
          a[i] = extract_l(L_shr_r(t0, q_sug)); /* from Q23 to Q12 and * 0.5 */
          

          /* a[j] = 0.5*(f1[i] - f2[i]); */

          t0 = L_sub(f1[i], f2[i]);          /* f1[i] - f2[i]             */
          a[j] = extract_l(L_shr_r(t0, q_sug)); /* from Q23 to Q12 and * 0.5 */
          
        }
      a[0] = shr(a[0], q);                 
    }
    else
    {
      q_sug = 12;                          
      q     = 0;                           
    }


    /* a[NC] = 0.5*f1[NC]*(1.0 + isp[M-1]); */

    L_Extract(f1[nc], &hi, &lo);
    t0 = Mpy_32_16(hi, lo, isp[m - 1]);
    t0 = L_add(f1[nc], t0);
    a[nc] = extract_l(L_shr_r(t0, q_sug));    /* from Q23 to Q12 and * 0.5 */
    
    /* a[m] = isp[m-1]; */

    a[m] = shr_r(isp[m - 1], add(3,q));           /* from Q15 to Q12          */
    

    return;
}

/*-----------------------------------------------------------*
 * procedure Get_isp_pol:                                    *
 *           ~~~~~~~~~~~                                     *
 *   Find the polynomial F1(z) or F2(z) from the ISPs.       *
 * This is performed by expanding the product polynomials:   *
 *                                                           *
 * F1(z) =   product   ( 1 - 2 isp_i z^-1 + z^-2 )           *
 *         i=0,2,4,6,8                                       *
 * F2(z) =   product   ( 1 - 2 isp_i z^-1 + z^-2 )           *
 *         i=1,3,5,7                                         *
 *                                                           *
 * where isp_i are the ISPs in the cosine domain.            *
 *-----------------------------------------------------------*
 *                                                           *
 * Parameters:                                               *
 *  isp[]   : isp vector (cosine domaine)         in Q15     *
 *  f[]     : the coefficients of F1 or F2        in Q23     *
 *  n       : == NC for F1(z); == NC-1 for F2(z)             *
 *-----------------------------------------------------------*/

static void Get_isp_pol(Word16 * isp, Word32 * f, Word16 n)
{
    Word16 i, j, hi, lo;
    Word32 t0;


    /* All computation in Q23 */

    f[0] = L_mult(4096, 1024);               /* f[0] = 1.0;        in Q23  */
    f[1] = L_mult(isp[0], -256);             /* f[1] = -2.0*isp[0] in Q23  */

    f += 2;                                  /* Advance f pointer          */
    isp += 2;                                /* Advance isp pointer        */

    for (i = 2; i <= n; i++)
    {

        *f = f[-2];                        

        for (j = 1; j < i; j++, f--)
        {
            L_Extract(f[-1], &hi, &lo);
            t0 = Mpy_32_16(hi, lo, *isp);  /* t0 = f[-1] * isp    */
            t0 = L_shl(t0, 1);
            *f = L_sub(*f, t0);              /* *f -= t0            */
            *f = L_add(*f, f[-2]);           /* *f += f[-2]         */
        }
        *f = L_msu(*f, *isp, 256);           /* *f -= isp<<8        */
        f += i;                            /* Advance f pointer   */
        isp += 2;                          /* Advance isp pointer */
    }
    return;
}

static void Get_isp_pol_16kHz(Word16 * isp, Word32 * f, Word16 n)
{
    Word16 i, j, hi, lo;
    Word32 t0;

    /* All computation in Q23 */

    f[0] = L_mult(4096, 256);                /* f[0] = 1.0;        in Q23  */
    f[1] = L_mult(isp[0], -64);              /* f[1] = -2.0*isp[0] in Q23  */

    f += 2;                                  /* Advance f pointer          */
    isp += 2;                                /* Advance isp pointer        */

    for (i = 2; i <= n; i++)
    {
        *f = f[-2];                        

        for (j = 1; j < i; j++, f--)
        {
            L_Extract(f[-1], &hi, &lo);
            t0 = Mpy_32_16(hi, lo, *isp);  /* t0 = f[-1] * isp    */
            t0 = L_shl(t0, 1);
            *f = L_sub(*f, t0);              /* *f -= t0            */
            *f = L_add(*f, f[-2]);           /* *f += f[-2]         */
        }
        *f = L_msu(*f, *isp, 64);            /* *f -= isp<<8        */
        f += i;                            /* Advance f pointer   */
        isp += 2;                          /* Advance isp pointer */
    }
    return;
}

/*-------------------------------------------------------------------*
 *                         ISP_ISF.C								 *
 *-------------------------------------------------------------------*
 *   Isp_isf   Transformation isp to isf                             *
 *   Isf_isp   Transformation isf to isp                             *
 *                                                                   *
 * The transformation from isp[i] to isf[i] and isf[i] to isp[i] are *
 * approximated by a look-up table and interpolation.                *
 *-------------------------------------------------------------------*/


#include "isp_isf.tab"                     /* Look-up table for transformations */

void Isp_isf(
     Word16 isp[],                         /* (i) Q15 : isp[m] (range: -1<=val<1)                */
     Word16 isf[],                         /* (o) Q15 : isf[m] normalized (range: 0.0<=val<=0.5) */
     Word16 m                              /* (i)     : LPC order                                */
)
{
    Word16 i, ind;
    Word32 L_tmp;

    ind = 127;                               /* beging at end of table -1 */

    for (i = (Word16) (m - 1); i >= 0; i--)
    {
        
        if (sub(i, sub(m, 2)) >= 0)
        {                                  /* m-2 is a constant */
            ind = 127;                       /* beging at end of table -1 */
        }
        /* find value in table that is just greater than isp[i] */
        
        while (sub(table[ind], isp[i]) < 0)
            ind--;

        /* acos(isp[i])= ind*128 + ( ( isp[i]-table[ind] ) * slope[ind] )/2048 */

        L_tmp = L_mult(sub(isp[i], table[ind]), slope[ind]);
        isf[i] = round(L_shl(L_tmp, 4));   /* (isp[i]-table[ind])*slope[ind])>>11 */
        
        isf[i] = add(isf[i], shl(ind, 7)); 
    }

    isf[m - 1] = shr(isf[m - 1], 1);       

    return;
}

/*-------------------------------------------------------------------*
 *                         DEEMPH.C									 *
 *-------------------------------------------------------------------*
 * Deemphasis: filtering through 1/(1-mu z^-1)				         *
 *																	 *
 * Deemph2   --> signal is divided by 2.							 *
 * Deemph_32 --> for 32 bits signal.								 *
 *-------------------------------------------------------------------*/

void Deemph2(
     Word16 x[],                           /* (i/o)   : input signal overwritten by the output */
     Word16 mu,                            /* (i) Q15 : deemphasis factor                      */
     Word16 L,                             /* (i)     : vector size                            */
     Word16 * mem                          /* (i/o)   : memory (y[-1])                         */
)
{
    Word16 i;
    Word32 L_tmp;

    /* saturation can occur in L_mac() */

    L_tmp = L_mult(x[0], 16384);
    L_tmp = L_mac(L_tmp, *mem, mu);
    x[0] = round(L_tmp);                   

    for (i = 1; i < L; i++)
    {
        L_tmp = L_mult(x[i], 16384);
        L_tmp = L_mac(L_tmp, x[i - 1], mu);
        x[i] = round(L_tmp);               
    }

    *mem = x[L - 1];                       

    return;
}

/*-----------------------------------------------------------------------*
 *                         HP_WSP.C										 *
 *-----------------------------------------------------------------------*
 *                                                                       *
 * 3nd order high pass filter with cut off frequency at 180 Hz           *
 *                                                                       *
 * Algorithm:                                                            *
 *                                                                       *
 *  y[i] = b[0]*x[i] + b[1]*x[i-1] + b[2]*x[i-2] + b[3]*x[i-3]           *
 *                   + a[1]*y[i-1] + a[2]*y[i-2] + a[3]*y[i-3];          *
 *                                                                       *
 * float a_coef[HP_ORDER]= {                                             *
 *    -2.64436711600664f,                                                *
 *    2.35087386625360f,                                                 *
 *   -0.70001156927424f};                                                *
 *                                                                       *
 * float b_coef[HP_ORDER+1]= {                                           *
 *     -0.83787057505665f,                                               *
 *    2.50975570071058f,                                                 *
 *   -2.50975570071058f,                                                 *
 *    0.83787057505665f};                                                *
 *                                                                       *
 *-----------------------------------------------------------------------*/


/* Initialization of static values */

void Init_Hp_wsp(Word16 mem[])
{
    Set_zero(mem, 9);

    return;
}

void scale_mem_Hp_wsp(Word16 mem[], Word16 exp)
{
    Word16 i;
    Word32 L_tmp;

    for (i = 0; i < 6; i += 2)
    {
        L_tmp = L_Comp(mem[i], mem[i + 1]);/* y_hi, y_lo */
        L_tmp = L_shl(L_tmp, exp);
        L_Extract(L_tmp, &mem[i], &mem[i + 1]);
    }

    for (i = 6; i < 9; i++)
    {
        L_tmp = L_deposit_h(mem[i]);       /* x[i] */
        L_tmp = L_shl(L_tmp, exp);
        mem[i] = round(L_tmp);             
    }

    return;
}


void Hp_wsp(
     Word16 wsp[],                         /* i   : wsp[]  signal       */
     Word16 hp_wsp[],                      /* o   : hypass wsp[]        */
     Word16 lg,                            /* i   : lenght of signal    */
     Word16 mem[]                          /* i/o : filter memory [9]   */
)
{
    Word16 i;
    Word16 x0, x1, x2, x3;
    Word16 y3_hi, y3_lo, y2_hi, y2_lo, y1_hi, y1_lo;
    Word32 L_tmp;
	/* filter coefficients in Q12 */
	static Word16 a[4] = {8192, 21663, -19258, 5734};
	static Word16 b[4] = {-3432, +10280, -10280, +3432};

    y3_hi = mem[0];                        
    y3_lo = mem[1];                        
    y2_hi = mem[2];                        
    y2_lo = mem[3];                        
    y1_hi = mem[4];                        
    y1_lo = mem[5];                        
    x0 = mem[6];                           
    x1 = mem[7];                           
    x2 = mem[8];                           

    for (i = 0; i < lg; i++)
    {
        x3 = x2;                           
        x2 = x1;                           
        x1 = x0;                           
        x0 = wsp[i];                       

        /* y[i] = b[0]*x[i] + b[1]*x[i-1] + b140[2]*x[i-2] + b[3]*x[i-3]  */
        /* + a[1]*y[i-1] + a[2] * y[i-2]  + a[3]*y[i-3]  */

        
        L_tmp = 16384L;                    /* rounding to maximise precision */
        L_tmp = L_mac(L_tmp, y1_lo, a[1]);
        L_tmp = L_mac(L_tmp, y2_lo, a[2]);
        L_tmp = L_mac(L_tmp, y3_lo, a[3]);
        L_tmp = L_shr(L_tmp, 15);
        L_tmp = L_mac(L_tmp, y1_hi, a[1]);
        L_tmp = L_mac(L_tmp, y2_hi, a[2]);
        L_tmp = L_mac(L_tmp, y3_hi, a[3]);
        L_tmp = L_mac(L_tmp, x0, b[0]);
        L_tmp = L_mac(L_tmp, x1, b[1]);
        L_tmp = L_mac(L_tmp, x2, b[2]);
        L_tmp = L_mac(L_tmp, x3, b[3]);

        L_tmp = L_shl(L_tmp, 2);           /* coeff Q12 --> Q15 */

        y3_hi = y2_hi;                     
        y3_lo = y2_lo;                     
        y2_hi = y1_hi;                     
        y2_lo = y1_lo;                     
        L_Extract(L_tmp, &y1_hi, &y1_lo);

        L_tmp = L_shl(L_tmp, 1);           /* coeff Q14 --> Q15 */
        hp_wsp[i] = round(L_tmp);          
    }

    mem[0] = y3_hi;                        
    mem[1] = y3_lo;                        
    mem[2] = y2_hi;                        
    mem[3] = y2_lo;                        
    mem[4] = y1_hi;                        
    mem[5] = y1_lo;                        
    mem[6] = x0;                           
    mem[7] = x1;                           
    mem[8] = x2;                           

    return;
}
/*-----------------------------------------------------------------------*
 *                         HP400.C										 *
 *-----------------------------------------------------------------------*
 * Interpolation of the LP parameters in 4 subframes.					 *
 *-----------------------------------------------------------------------*/


#define MP1 (M+1)


void Int_isp(
     Word16 isp_old[],                     /* input : isps from past frame              */
     Word16 isp_new[],                     /* input : isps from present frame           */
     Word16 frac[],                        /* input : fraction for 3 first subfr (Q15)  */
     Word16 Az[]                           /* output: LP coefficients in 4 subframes    */
)
{
    Word16 i, k, fac_old, fac_new;
    Word16 isp[M];
    Word32 L_tmp;

    for (k = 0; k < 3; k++)
    {
        fac_new = frac[k];                 
        fac_old = add(sub(32767, fac_new), 1);  /* 1.0 - fac_new */

        for (i = 0; i < M; i++)
        {
            L_tmp = L_mult(isp_old[i], fac_old);
            L_tmp = L_mac(L_tmp, isp_new[i], fac_new);
            isp[i] = round(L_tmp);         
        }
        Isp_Az(isp, Az, M, 0);
        Az += MP1;
    }

    /* 4th subframe: isp_new (frac=1.0) */

    Isp_Az(isp_new, Az, M, 0);

    return;
}

/*-------------------------------------------------------------------*
 *                         LP_DEC2.C								 *
 *-------------------------------------------------------------------*
 * Decimate a vector by 2 with 2nd order fir filter.                 *
 *-------------------------------------------------------------------*/


#define L_FIR  5
#define L_MEM  (L_FIR-2)

/* static float h_fir[L_FIR] = {0.13, 0.23, 0.28, 0.23, 0.13}; */
/* fixed-point: sum of coef = 32767 to avoid overflow on DC */
static Word16 h_fir[L_FIR] = {4260, 7536, 9175, 7536, 4260};


void LP_Decim2(
     Word16 x[],                           /* in/out: signal to process         */
     Word16 l,                             /* input : size of filtering         */
     Word16 mem[]                          /* in/out: memory (size=3)           */
)
{
    Word16 *p_x, x_buf[L_FRAME + L_MEM];
    Word16 i, j, k;
    Word32 L_tmp;

    /* copy initial filter states into buffer */

    p_x = x_buf;                           
    for (i = 0; i < L_MEM; i++)
    {
        *p_x++ = mem[i];                   
    }
    for (i = 0; i < l; i++)
    {
        *p_x++ = x[i];                     
    }
    for (i = 0; i < L_MEM; i++)
    {
        mem[i] = x[l - L_MEM + i];         
    }

    for (i = 0, j = 0; i < l; i += 2, j++)
    {
        p_x = &x_buf[i];                   

        L_tmp = 0L;                        
        for (k = 0; k < L_FIR; k++)
            L_tmp = L_mac(L_tmp, *p_x++, h_fir[k]);

        x[j] = round(L_tmp);               
    }

    return;
/*-----------------------------------------------------------------------*
 *                         RESIDU.C										 *
 *-----------------------------------------------------------------------*
 * Compute the LPC residual by filtering the input speech through A(z)   *
 *-----------------------------------------------------------------------*/


void Residu(
     Word16 a[],                           /* (i) Q12 : prediction coefficients                     */
     Word16 m,                             /* (i)     : order of LP filter                          */
     Word16 x[],                           /* (i)     : speech (values x[-m..-1] are needed         */
     Word16 y[],                           /* (o) x2  : residual signal                             */
     Word16 lg                             /* (i)     : size of filtering                           */
)
{
    Word16 i, j;
    Word32 s;

    for (i = 0; i < lg; i++)
    {
        s = L_mult(x[i], a[0]);

        for (j = 1; j <= m; j++)
            s = L_mac(s, a[j], x[i - j]);

        s = L_shl(s, 3 + 1);               /* saturation can occur here */
        y[i] = round(s);                   
    }

    return;
}
}

/*------------------------------------------------------------------------*
 *                         P_MED_OL.C									  *
 *------------------------------------------------------------------------*
 * Compute the open loop pitch lag.										  *
 *------------------------------------------------------------------------*/

#include "p_med_ol.tab"


Word16 Pitch_med_ol(                       /* output: open loop pitch lag                             */
     Word16 wsp[],                         /* input : signal used to compute the open loop pitch      */
                                           /*         wsp[-pit_max] to wsp[-1] should be known        */
     Word16 L_min,                         /* input : minimum pitch lag                               */
     Word16 L_max,                         /* input : maximum pitch lag                               */
     Word16 L_frame,                       /* input : length of frame to compute pitch                */
     Word16 L_0,                           /* input : old_ open-loop pitch                            */
     Word16 * gain,                        /* output: normalize correlation of hp_wsp for the Lag     */
     Word16 * hp_wsp_mem,                  /* i:o   : memory of the hypass filter for hp_wsp[] (lg=9) */
     Word16 * old_hp_wsp,                  /* i:o   : hypass wsp[]                                    */
     Word16 wght_flg                       /* input : is weighting function used                      */
)
{
    Word16 i, j, Tm;
    Word16 hi, lo;
    Word16 *ww, *we, *hp_wsp;
    Word16 exp_R0, exp_R1, exp_R2;
    Word32 max, R0, R1, R2;

    ww = &corrweight[198];
    
    we = &corrweight[98 + L_max - L_0];
    

    max = MIN_32;                          
    Tm = 0;                                
    for (i = L_max; i > L_min; i--)
    {
        /* Compute the correlation */

        R0 = 0;                            
        for (j = 0; j < L_frame; j++)
            R0 = L_mac(R0, wsp[j], wsp[j - i]);

        /* Weighting of the correlation function.   */

        L_Extract(R0, &hi, &lo);
        R0 = Mpy_32_16(hi, lo, *ww);
        ww--;

        
        
        if ((L_0 > 0) && (wght_flg > 0))
        {
            /* Weight the neighbourhood of the old lag. */
            L_Extract(R0, &hi, &lo);
            R0 = Mpy_32_16(hi, lo, *we);
            we--;
            
        }
        
        if (L_sub(R0, max) >= 0)
        {
            max = R0;
            
            Tm = i;
            
        }
    }

    /* Hypass the wsp[] vector */

    hp_wsp = old_hp_wsp + L_max;           
    Hp_wsp(wsp, hp_wsp, L_frame, hp_wsp_mem);

    /* Compute normalize correlation at delay Tm */

    R0 = 0;                                
    R1 = 1L;                               
    R2 = 1L;                               
    for (j = 0; j < L_frame; j++)
    {
        R0 = L_mac(R0, hp_wsp[j], hp_wsp[j - Tm]);
        R1 = L_mac(R1, hp_wsp[j - Tm], hp_wsp[j - Tm]);
        R2 = L_mac(R2, hp_wsp[j], hp_wsp[j]);
    }

    /* gain = R0/ sqrt(R1*R2) */

    exp_R0 = norm_l(R0);
    R0 = L_shl(R0, exp_R0);

    exp_R1 = norm_l(R1);
    R1 = L_shl(R1, exp_R1);

    exp_R2 = norm_l(R2);
    R2 = L_shl(R2, exp_R2);


    R1 = L_mult(round(R1), round(R2));

    i = norm_l(R1);
    R1 = L_shl(R1, i);

    exp_R1 = add(exp_R1, exp_R2);
    exp_R1 = add(exp_R1, i);
    exp_R1 = sub(62, exp_R1);

    Isqrt_n(&R1, &exp_R1);

    R0 = L_mult(round(R0), round(R1));
    exp_R0 = sub(31, exp_R0);
    exp_R0 = add(exp_R0, exp_R1);

    *gain = round(L_shl(R0, exp_R0));
    

    /* Shitf hp_wsp[] for next frame */

    for (i = 0; i < L_max; i++)
    {
        old_hp_wsp[i] = old_hp_wsp[i + L_frame];
        
    }

    return (Tm);
}

/*____________________________________________________________________
 |
 |
 |  FUNCTION NAME median5
 |
 |      Returns the median of the set {X[-2], X[-1],..., X[2]},
 |      whose elements are 16-bit integers.
 |
 |  INPUT
 |      X[-2:2]   16-bit integers.
 |
 |  RETURN VALUE
 |      The median of {X[-2], X[-1],..., X[2]}.
 |_____________________________________________________________________
 */

Word16 median5(Word16 x[])
{
    Word16 x1, x2, x3, x4, x5;
    Word16 tmp;

    x1 = x[-2];                            
    x2 = x[-1];                            
    x3 = x[0];                             
    x4 = x[1];                             
    x5 = x[2];                             

    

    if (sub(x2, x1) < 0)
    {
        tmp = x1;
        x1 = x2;
        x2 = tmp;                          
    }
    if (sub(x3, x1) < 0)
    {
        tmp = x1;
        x1 = x3;
        x3 = tmp;                          
    }
    if (sub(x4, x1) < 0)
    {
        tmp = x1;
        x1 = x4;
        x4 = tmp;                          
    }
    if (sub(x5, x1) < 0)
    {
        x5 = x1;                           
    }
    if (sub(x3, x2) < 0)
    {
        tmp = x2;
        x2 = x3;
        x3 = tmp;                          
    }
    if (sub(x4, x2) < 0)
    {
        tmp = x2;
        x2 = x4;
        x4 = tmp;                          
    }
    if (sub(x5, x2) < 0)
    {
        x5 = x2;                           
    }
    if (sub(x4, x3) < 0)
    {
        x3 = x4;                           
    }
    if (sub(x5, x3) < 0)
    {
        x3 = x5;                           
    }
    return (x3);
}

/*____________________________________________________________________
 |
 |
 |  FUNCTION NAME med_olag
 |
 |
 |_____________________________________________________________________
 */


Word16 Med_olag(                           /* output : median of  5 previous open-loop lags       */
     Word16 prev_ol_lag,                   /* input  : previous open-loop lag                     */
     Word16 old_ol_lag[5]
)
{
    Word16 i;

    /* Use median of 5 previous open-loop lags as old lag */

    for (i = 4; i > 0; i--)
    {
        old_ol_lag[i] = old_ol_lag[i - 1]; 
    }

    old_ol_lag[0] = prev_ol_lag;           

    i = median5(&old_ol_lag[2]);

    return i;

}

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

