/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file ********************************************************************

   \brief
        Filter header file.

   \details
        This file contains defines, typedefs and prototypes for the filter
        implementation.

   \author
        Nexperia: http://www.nexperia.com

   \par Support Page
        For additional support, visit: https://www.nexperia.com/support

   $Author: Aanas Sayed $
   $Date: 2024/03/08 $  \n

 ******************************************************************************/

#ifndef FILTER_H
#define FILTER_H

// Include standard integer type definitions
#include "stdint.h"

//! Maximum value of integers
#define MAX_INT 32767

// Prototypes
int16_t calculateEMA(uint16_t currentSample, uint16_t previousEMA, uint8_t alphaExponent);

#endif
