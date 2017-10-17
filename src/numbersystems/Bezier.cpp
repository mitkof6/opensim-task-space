/******************************************
 * C++ Spherical Bezier Curves
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

#include "Bezier.h"
#include "Complex.h"
#include "Quaternion.h"
#include "Octonion.h"
#include "Sedenion.h"
#include "Trigintaduonion.h"

template <> const Quaternion<double>    Quaternion<double>::I = Quaternion<double>(0, 1, 0, 0);
template <> const Quaternion<float>    Quaternion<float>::I = Quaternion<float>(0, 1, 0, 0);
template <> const Quaternion<long double>    Quaternion<long double>::I = Quaternion<long double>(0, 1, 0, 0);

template <> const Octonion<double>    Octonion<double>::I = Octonion<double>(0, 1, 0, 0, 0, 0, 0, 0);
template <> const Octonion<float>    Octonion<float>::I = Octonion<float>(0, 1, 0, 0, 0, 0, 0, 0);
template <> const Octonion<long double>    Octonion<long double>::I = Octonion<long double>(0, 1, 0, 0, 0, 0, 0, 0);

template <> const Sedenion<double>    Sedenion<double>::I = Sedenion<double>(0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<float>    Sedenion<float>::I = Sedenion<float>(0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>    Sedenion<long double>::I = Sedenion<long double>(0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

template<typename T, typename S>
Bezier<T, S>::Bezier(T * q, unsigned int N) :n(N - 1), slerps(new Slerp<T, S>[n])
{
    for (unsigned int i = 0; i < n; ++i)
    {
        slerps[i] = Slerp<T, S>(q[i], q[i + 1]);
    }
}

template<typename T, typename S>
Bezier<T, S>::~Bezier()
{
    delete[] slerps;
}

template<typename T, typename S>
T Bezier<T, S>::value(S t) const
{
    T * pts = new T[n];

    for (unsigned int i = 0; i < n; ++i)
    {
        pts[i] = slerps[i].value(t);
    }

    for (unsigned int j = n - 1; j >= 0; --j)
    {
        for (unsigned int i = 0; i < j; ++i)
        {
            pts[i] = Slerp<T, S>(pts[i], pts[i + 1]).value(t);
        }
    }

    T tmp = pts[0];

    delete[] pts;

    return tmp;
}

template class Bezier< Quaternion<long double>, long double >;
template class Bezier< Quaternion<double>, double >;
template class Bezier< Quaternion<float>, float >;
template class Bezier< Octonion<long double>, long double >;
template class Bezier< Octonion<double>, double >;
template class Bezier< Octonion<float>, float >;
template class Bezier< Sedenion<long double>, long double >;
template class Bezier< Sedenion<double>, double >;
template class Bezier< Sedenion<float>, float >;
template class Bezier< Trigintaduonion<long double>, long double >;
template class Bezier< Trigintaduonion<double>, double >;
template class Bezier< Trigintaduonion<float>, float >;
