/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <limits>
#include <stdlib.h>

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

#pragma once

/*----------------------------------------------------------*\
| Helper functions
\*----------------------------------------------------------*/

inline bool isZero(double value)
{
  return abs(value) < std::numeric_limits<double>::epsilon();
}

inline double clampZero(double value, double min, double max)
{
  if (value < min && !isZero(value)) return min;
  if (value > max) return max;

  return value;
}

inline double mapZero(double value, double min, double max, double targetMin, double targetMax)
{
  if (!isZero(value))
  {
    return (value - min) / (max - min) * (targetMax - targetMin) + targetMin;
  }
  else
  {
    return 0.0;
  }
}
