//---------------------------------------------------------------------------
// Filename    : CRC.H
// Project     : UAXT RS-485 Pa Control Interface board
// Author      : Chang Deng
// Date        : May 21, 2014
// Description : Functions for calculating CRC of data.
// Revision History:
//   1.0 - (5/21/14) Initial release
//---------------------------------------------------------------------------
#ifndef CRC_H_
#define CRC_H_
//---------------------------------------------------------------------------
#ifdef msp430x14x
 #include "msp430x14x.h"
#endif
//---------------------------------------------------------------------------
#undef EXTERN
#ifdef PORTS_C_
#define EXTERN
#else
#define EXTERN extern
#endif
//---------------------------------------------------------------------------

// Global functions
unsigned int CalculateCRC16(unsigned char *buffer, unsigned char length);

//---------------------------------------------------------------------------
#endif /*CRC_H_*/
