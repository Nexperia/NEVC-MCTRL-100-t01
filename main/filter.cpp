/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file ********************************************************************

   \brief
        Filter source file.

   \details
        This file contains the exponential moving average (EMA) filter
        implementation.

   \author
        Nexperia: http://www.nexperia.com

   \par Support Page
        For additional support, visit: https://www.nexperia.com/support

   $Author: Aanas Sayed $
   $Date: 2024/03/08 $  \n

 ******************************************************************************/

#include "filter.h"

/*! \brief Exponential Moving Average (EMA) calculation algorithm.

    Calculates EMA from current sample, previous EMA and alpha.

    \param currentSample  Current measured sampled.
    \param previousEMA  Previously calculated EMA.
    \param alphaExponent  Used to drive alpha where alpha = 1 / (2 ^ alphaExponent).

    \return Returns the calculated Exponential Moving Average (EMA) as a 16-bit signed integer.
 */
int16_t calculateEMA(uint16_t currentSample, uint16_t previousEMA, uint8_t alphaExponent)
{
     if (currentSample > MAX_INT)
     {
          currentSample = MAX_INT;
     }
     if (previousEMA > MAX_INT)
     {
          previousEMA = MAX_INT;
     }

     int16_t newEMA = (int16_t)previousEMA;
     newEMA += ((int16_t)currentSample - (int16_t)previousEMA) >> alphaExponent;

     if (newEMA < 0)
     {
          newEMA = 0;
     }

     return newEMA;
}
