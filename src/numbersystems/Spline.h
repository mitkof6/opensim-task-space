/******************************************
 * C++ Spherical Splines
 * Version: 1.0.9
 * Author:  Douglas Wilhelm Harder
 * Date:    2008/03/03
 *
 * Copyright (c) 2007-8 by Douglas Wilhelm Harder.
 * All rights reserved.
 *
 * References:
 *    Ken Shoemake, 'Animating Rotation with Quatenrion Curves', SIGGRAPH 95.
 *    Erik B. Dam, Martin Koch, Martin Lillholm,
 *      'Quaternions, Interpolation and Animation', Technical Report DIKU-TR-98/5.
 ******************************************/

#ifndef CA_UWATERLOO_ALUMNI_DWHARDER_SPLINE
#define CA_UWATERLOO_ALUMNI_DWHARDER_SPLINE

#include "internal/NumberSystemsDLL.h"

#include "Slerp.h"

template<typename T, typename S = double>
class NumberSystems_API Spline
{
private:
    unsigned int n;
    Slerp<T, S> * slerpq;
    Slerp<T, S> * slerpa;
    T q0, qn;

public:
    Spline(T *, unsigned int);
    ~Spline();
    T value(S) const;
};

#endif
