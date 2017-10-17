/******************************************
 * C++ Support for the Number Systems Package
 * Version: 1.0.9
 * Author:  Douglas Wilhelm Harder
 * Date:    2008/03/03
 *
 * Copyright (c) 2006-2008 by Douglas Wilhelm Harder.
 * All rights reserved.
 ******************************************/

#include <iostream>
#include <cmath>
#include <string>
#include <limits>
#include "Support.h"

template <typename T>
T Support<T>::Gamma(int n)
{
    return (n <= 0 || n >= 172) ? POS_INF : GAMMA_TAB[n - 1];
}

template <typename T>
T Support<T>::sec(T x)
{
    return 1.0 / std::cos(x);
}

template <typename T>
T Support<T>::csc(T x)
{
    return 1.0 / std::sin(x);
}

template <typename T>
T Support<T>::cot(T x)
{
    return 1.0 / std::tan(x);
}

template <typename T>
T Support<T>::sech(T x)
{
    return 1.0 / std::cosh(x);
}

template <typename T>
T Support<T>::csch(T x)
{
    return 1.0 / std::sinh(x);
}

template <typename T>
T Support<T>::coth(T x)
{
    return 1.0 / std::tanh(x);
}

template <typename T>
T Support<T>::sign(T x)
{
    unsigned char *chr = reinterpret_cast<unsigned char *>(&x);

    return bigendian ?
        ((chr[sizeof(T) - 1] & 0x80) ? -1.0 : 1.0) :
        ((chr[0] & 0x80) ? -1.0 : 1.0);
}

template <typename T>
bool Support<T>::is_pos_inf(T x)
{
    return x == POS_INF;
}

template <typename T>
bool Support<T>::is_neg_inf(T x)
{
    return x == NEG_INF;
}

template <typename T>
bool Support<T>::is_pos_zero(T x)
{
    return (x == 0) && sign(x) == 1.0;
}

template <typename T>
bool Support<T>::is_neg_zero(T x)
{
    return (x == 0) && sign(x) == -1.0;
}

template <typename T>
bool Support<T>::is_inf(T x)
{
    return (x == POS_INF) || (x == NEG_INF);
}

template <typename T>
bool Support<T>::is_nan(T x)
{
    return x != x;
}

/******************************************
* IO Stream Functions
******************************************/

template <typename T> void Support<T>::print_real(T r, std::ostream & out)
{
    if (is_pos_zero(r))
    {
        out << "0";
    }
    else if (is_neg_zero(r))
    {
        out << "-0";
    }
    else if (is_nan(r))
    {
        out << "NaN";
    }
    else
    {
        out << r;
    }
}

template <typename T> void Support<T>::print_imaginary(T i, const std::string & s, std::ostream & out)
{
    if (is_pos_zero(i))
    {
        out << " + 0" << s;
    }
    else if (is_neg_zero(i))
    {
        out << " - 0" << s;
    }
    else if (is_nan(i))
    {
        out << " + NaN" << s;
    }
    else if (i == 1.0)
    {
        out << " + " << s;
    }
    else if (i == -1.0)
    {
        out << " - " << s;
    }
    else if (i > 0)
    {
        out << " + " << i << s;
    }
    else
    {
        out << " - " << (-i) << s;
    }
}

template <typename T> void Support<T>::print_imaginary(T i, char c, std::ostream & out)
{
    if (is_pos_zero(i))
    {
        out << " + 0" << c;
    }
    else if (is_neg_zero(i))
    {
        out << " - 0" << c;
    }
    else if (is_nan(i))
    {
        out << " + NaN" << c;
    }
    else if (i == 1.0)
    {
        out << " + " << c;
    }
    else if (i == -1.0)
    {
        out << " - " << c;
    }
    else if (i > 0)
    {
        out << " + " << i << c;
    }
    else
    {
        out << " - " << (-i) << c;
    }
}

static bool is_bigendian()
{
    double d = 1.0;

    unsigned char *chr = reinterpret_cast<unsigned char *>(&d);

    return (chr[0] == 0);
}

template <typename T>
short Support<T>::bigendian = is_bigendian();

template class Support<double>;
template class Support<float>;
template class Support<long double>;

