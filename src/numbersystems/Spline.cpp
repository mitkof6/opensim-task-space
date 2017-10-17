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

#include "Spline.h"
#include "Complex.h"
#include "Quaternion.h"
#include "Octonion.h"
#include "Sedenion.h"
#include "Trigintaduonion.h"

template<typename T, typename S>
Spline<T, S>::Spline(T * q, unsigned int N) :n(N), slerpq(new Slerp<T, S>[n - 1]), slerpa(new Slerp<T, S>[n - 1])
{
    T * a = new T[n];

    q0 = q[0];
    qn = q[n - 1];

    a[0] = q[0];

    for (unsigned int i = 1; i < n - 1; ++i)
    {
        T invqi = inverse(q[i]);

        a[i] = q[i] * exp(-static_cast<S>(0.25)*(
            log(invqi*q[i + 1]) + log(invqi*q[i - 1])
            ));
    }

    a[n - 1] = q[n - 1];

    for (unsigned int i = 0; i < n - 1; ++i)
    {
        slerpq[i] = Slerp<T, S>(q[i], q[i + 1]);
        slerpa[i] = Slerp<T, S>(a[i], a[i + 1]);
    }

    delete[] a;
}

template<typename T, typename S>
Spline<T, S>::~Spline()
{
    delete[] slerpq;
    delete[] slerpa;
}

template<typename T, typename S>
T Spline<T, S>::value(S t) const
{
    if (t <= 0)
    {
        return q0;
    }

    if (t >= n - 1)
    {
        return qn;
    }

    int m = static_cast<int>(std::floor(t));
    S ft = t - static_cast<S>(m);

    return Slerp<T, S>(slerpq[m].value(ft), slerpa[m].value(ft)).value(2 * ft*(1 - ft));
}

template class Spline< Quaternion<long double>, long double >;
template class Spline< Quaternion<double>, double >;
template class Spline< Quaternion<float>, float >;
template class Spline< Octonion<long double>, long double >;
template class Spline< Octonion<double>, double >;
template class Spline< Octonion<float>, float >;
template class Spline< Sedenion<long double>, long double >;
template class Spline< Sedenion<double>, double >;
template class Spline< Sedenion<float>, float >;
template class Spline< Trigintaduonion<long double>, long double >;
template class Spline< Trigintaduonion<double>, double >;
template class Spline< Trigintaduonion<float>, float >;