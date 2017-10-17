/******************************************
 * C++ Spherical Linear intERPolations (SLERP)
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

#include "Slerp.h"
#include "Complex.h"
#include "Quaternion.h"
#include "Octonion.h"
#include "Sedenion.h"
#include "Trigintaduonion.h"
#include "Support.h"

#include <cmath>
#include <limits>

template< typename T, typename S>
Slerp<T, S>::Slerp(const T & q1, const T & q2)
{
    qA = q1;
    qB = q2;

    /*left = q1 / q1.abs();
      T right = q2 / q2.abs();

      equal = (q1 == q2);

      if (equal)
      {
      return;
      }

      right = left.conj() * right;

      S abs_imag = right.abs_imag();
      imag = right.imag() / abs_imag;

      ln = 0.5*std::log(
      right.real() * right.real() +
      abs_imag * abs_imag
      );

      atan = std::atan2(abs_imag, right.real());*/
}

template< typename T, typename S>
T Slerp<T, S>::value(S t) const
{
    if (t == 0)
    {
        return qA;
    }
    else if (t == 1)
    {
        return qB;
    }
    else
    {
        double C = qA[0] * qB[0] + qA[1] * qB[1] + qA[2] * qB[2] + qA[3] * qB[3];
        double theta = acos(C);

        if ((1 - C) <= std::numeric_limits<S>::epsilon())
        {
            return qA*(1 - t) + qB*t;
        }
        else if ((1 + C) <= std::numeric_limits<S>::epsilon())
        {
            T q2;
            q2[0] = qA[3];
            q2[1] = -qA[2];
            q2[2] = qA[1];
            q2[3] = -qA[0];
            return qA*(sin((1 - t)*(Support<double>::PI / 2))) + q2*sin(t*(Support<double>::PI / 2));
        }
        else
        {
            return qA*(sin((1 - t)*theta)) / sin(theta) + qB*sin(t*theta) / sin(theta);
        }
    }

    /*if (equal)
      {
      return left;
      }

      S exptln = std::exp(t*ln);

      return (left * (
      exptln*std::cos(t*atan) +
      imag*(exptln*std::sin(t*atan))
      )).imag();*/
}

template class Slerp< Quaternion<long double>, long double >;
template class Slerp< Quaternion<double>, double >;
template class Slerp< Quaternion<float>, float >;

template class Slerp< Octonion<long double>, long double >;
template class Slerp< Octonion<double>, double >;
template class Slerp< Octonion<float>, float >;
template class Slerp< Sedenion<long double>, long double >;
template class Slerp< Sedenion<double>, double >;
template class Slerp< Sedenion<float>, float >;
template class Slerp< Trigintaduonion<long double>, long double >;
template class Slerp< Trigintaduonion<double>, double >;
template class Slerp< Trigintaduonion<float>, float >;
