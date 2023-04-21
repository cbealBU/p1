
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#define OUTPUT_0_WIDTH 24
#define MESSAGE_WIDTH 128

#include <stddef.h>

static unsigned char msgBuff[MESSAGE_WIDTH];     // Global Variables to buffer serial port data
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 128
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
// function for converting all of the data to doubles
double parseCharToDouble(unsigned char *startByte)
{
    return (double)*startByte;
}

// function for converting all of the data to doubles
double parseShortToDouble(unsigned char *startByte)
{
    unsigned short *output = (unsigned short *)startByte;
    return (double)*output;
}

// function for converting all of the data to doubles
double parseFloatToDouble(unsigned char *startByte)
{
    float *output = (float*)startByte;
    return (double)*output;
}

// function for converting all of the data to doubles
double parseDoubleToDouble(unsigned char *startByte)
{
    double *output = (double*)startByte;
    return (double)*output;
}

// checksum function: the algorithm takes the data and subtracts the bytes from the checksum
// provided at the end of the array so that the final value is zero if the data is correct
unsigned short checksum (unsigned char *ptr, size_t sz)
{
    unsigned short chk = *(unsigned short*)(ptr+sz); // initialize the sum with the reference checksum
    while (sz-- != 0) // keep decreasing the value of sz, the offset to the end of the array
        chk -= (unsigned short)*(ptr+sz); // subtract the value of each byte (promoting to ushort since the checksum is 2 bytes)
    return chk;
}
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void gps_bin3_receive_Outputs_wrapper(const uint8_T *u0,
			const uint8_T *u1,
			real_T *y0,
			real_T *y1)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
int status = u1[0];
    
    if( 0 == status )
    {
        // Once the first byte is the $ character, copy the appropriate number of bytes for the bin message into a processing buffer
        for (int input_index = 0; input_index < MESSAGE_WIDTH; input_index++)
        {
            msgBuff[input_index] = (unsigned char) u0[input_index];
        }
        if ((unsigned char)13 != msgBuff[MESSAGE_WIDTH-2] && (unsigned char)11 != msgBuff[MESSAGE_WIDTH-1])
            status = -1; // Message not complete, set the flag to avoid parsing
    }
    
    // If the status from the preceding data copy and end check is good, calculate the checksum with the 52 bytes of
    // data following the header byte in a bin1 message. Set the status flag if the data is bad.
    if( 0 == status )
    {
        if (0 != checksum(msgBuff+8,116))
            status = -2;
    }

    // If status is still good at this point, there is a good message to parse
    if (0 == status)
    {
        y0[0] = parseDoubleToDouble(msgBuff+8);    // GPS time of week
        y0[1] = parseShortToDouble(msgBuff+16);    // GPS week number
        y0[2] = parseShortToDouble(msgBuff+20);    // SATS used
        y0[3] = parseCharToDouble(msgBuff+22);    // NAV mode
        y0[4] = parseDoubleToDouble(msgBuff+24);    // Latitude
        y0[5] = parseDoubleToDouble(msgBuff+32);    // Longitude
        y0[6] = parseFloatToDouble(msgBuff+40);    // Altitude
        y0[7] = parseFloatToDouble(msgBuff+44);    // Horizontal speed
        y0[8] = parseFloatToDouble(msgBuff+48);    // Vertical velocity
        y0[9] = parseFloatToDouble(msgBuff+52);    // Course over ground
        y0[10] = parseFloatToDouble(msgBuff+56);    // Heading
        y0[11] = parseFloatToDouble(msgBuff+60);    // Pitch (roll for P1)
        y0[12] = parseShortToDouble(msgBuff+68);    // Age of differential
        y0[13] = parseShortToDouble(msgBuff+70);    // Attitude status
        y0[14] = parseFloatToDouble(msgBuff+72);    // Yaw stddev
        y0[15] = parseFloatToDouble(msgBuff+76);    // Pitch (roll) stddev
        y0[16] = parseFloatToDouble(msgBuff+80);    // Horizontal RMS
        y0[17] = parseFloatToDouble(msgBuff+84);    // Vertical RMS
        y0[18] = parseFloatToDouble(msgBuff+100);    // Covariance North-North
        y0[19] = parseFloatToDouble(msgBuff+104);    // Covariance North-East
        y0[20] = parseFloatToDouble(msgBuff+108);    // Covariance North-Up
        y0[21] = parseFloatToDouble(msgBuff+112);    // Covariance East-East
        y0[22] = parseFloatToDouble(msgBuff+116);    // Covariance East-Up
        y0[23] = parseFloatToDouble(msgBuff+120);    // Covariance Up-Up
    }
    else
    {
        for (int output_index = 0; output_index < OUTPUT_0_WIDTH; output_index++)
        {
            y0[output_index] = (real_T)0;     // Zero the output if there is no valid data
        }
    }
    y1[0] = (real_T)status;     // Send out status variable with zero for good data, positive values for a pass through of a serial
    // reception error, and negative values for issues parsing the message
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


