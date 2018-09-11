
/**********************************************************************
 *  Includes
 *********************************************************************/
#include "rfid_thread.h"
#include "net_thread.h"  // g_sensor_event_flags
#include "app.h"

/**********************************************************************
 *  Constants
 *********************************************************************/

#define  RFID_DATA_LEN      16  // start char (1) + ID (10) + Checksum (2) + CR char (1) + LF char (1) + end char (1)
#define  RFID_ID_LEN        10  // only ID ascii character
#define RECEIVE_STRING_LENGTH RFID_DATA_LEN

/**********************************************************************
 *  Globals
 *********************************************************************/

char g_buffer[64];
char rfid_string[RFID_ID_LEN+1];  // including null at the end.
char temparray[RFID_DATA_LEN];    // this temporary data is for

/**********************************************************************
 *  Prototypes for subroutines
 *********************************************************************/
char CONVERT_ASCII_HEX(char value);

/**********************************************************************
 *  Thread's Main Code
 *********************************************************************/

void rfid_thread_entry(void)
{

    ssp_err_t ReturnVal;
    char      databyte;    // each data byte in RFID (for checksum calculation)
    char      hexvalue;    // the hex value of two ascii bytes.
    char      checksum;
    bool      errorflag = false;   // checksum error



    while (1)
    {

        ReturnVal = g_sf_comms0.p_api->read(g_sf_comms0.p_ctrl, (unsigned char *)g_buffer, RECEIVE_STRING_LENGTH, TX_WAIT_FOREVER);
        if (SSP_SUCCESS != ReturnVal)
        {
            set_leds(1, 0, 0);  // turn on the green led
            while(1);
        }

        // Processing ID (10 bytes) and checksum (2 bytes)
        errorflag = false;
        checksum = 0;
        for (int i=0; i<(RFID_ID_LEN+2); i++){
            if (i < RFID_ID_LEN) {
                rfid_string[i] = g_buffer[i+1];
            }
            databyte = g_buffer[i+1];
            databyte = CONVERT_ASCII_HEX(databyte);
            if (!(i%2)){
                hexvalue = databyte << 4;
            } else {
                hexvalue = hexvalue + databyte;
                if (i < 10)  // checksum only for the first 10 ascii data.
                  checksum = checksum ^ hexvalue;
            }
        }
        if (checksum != hexvalue) {
            errorflag = true;
        } else {
            tx_event_flags_set(&g_sensor_event_flags, RFID_TRANSFER_REQUEST, TX_OR);
        }


        tx_thread_sleep (1);
    }
}
/**********************************************************************
 *  Body for subroutines
 *********************************************************************/

/*
 *  Convert ascii character (like 'A') to hex value (like 0x0a)
 *
 *  Input: one byte data
 *  Output: one byte data
 *
 */

char CONVERT_ASCII_HEX(char value) {
  if (value >= '0' && value <='9')
     return(value - '0');
  else
     return(value - 'A' + 0x0a);
}
/**********************************************************************
 *  End of uart_thread_entry.c
 *********************************************************************/
