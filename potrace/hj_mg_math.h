#pragma once

#include <vector>
#include <iostream>
#include <cmath>
#include <math.h>
#include <float.h>
#include <algorithm>

namespace hj_mgs_implement_edge
{
#define N_DBL_MAX 1.79769e+308
#define N_DBL_MIN 2.22507e-308
#define N_FLT_MAX 3.40282e+38
#define N_FLT_MIN 1.17549e-38
#define N_INT_MAX 2147483647
#define N_INT_MIN -2147483648
#define N_MIN_VAL -9999999

#define N_PI 3.14159265358979323846
#define N_PI_2 1.57079632679489661923
#define N_PI_4 0.78539816339744830962
#define N_2PI 6.283185307179586476925
#define N_4PI 12.56637061435917295385

    inline float NRadiansToDegrees(float radians)
    {
        return float(double(radians) * 57.2957795130823208768);
    }

    inline float NDegreesToRadians(float degrees)
    {
        return float(double(degrees) * 0.0174532925199432957692);
    }

    inline bool almostEqual(float x, float y, float tol = 1.0e-6)
    {
        return fabs(x - y) < tol;
    }

}// namespace hj_mgs_implement