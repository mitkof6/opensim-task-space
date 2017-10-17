/******************************************
 * C++ Sedenions
 * Version: 1.0.9
 * Author:  Douglas Wilhelm Harder
 * Date:    2008/03/03
 *
 * Copyright (c) 2006-2008 by Douglas Wilhelm Harder.
 * All rights reserved.
 ******************************************/

#include "Complex.h"
#include "Sedenion.h"
#include "Support.h"
#include <iostream>
#include <cmath>
#include <string>

// template <> const Sedenion<double>    Sedenion<double>::I = Sedenion<double>(0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
// template <> const Sedenion<float>    Sedenion<float>::I = Sedenion<float>(0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
// template <> const Sedenion<long double>    Sedenion<long double>::I = Sedenion<long double>(0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);


 /******************************************
  * Constructors
  ******************************************/

template <typename T> Sedenion<T>::Sedenion(T real, T imagi, T imagj, T imagk, T image1, T imagi1, T imagj1, T imagk1, T image2, T imagi2, T imagj2, T imagk2, T image3, T imagi3, T imagj3, T imagk3) :
    r(real), i(imagi), j(imagj), k(imagk),
    u1(image1), i1(imagi1), j1(imagj1), k1(imagk1),
    u2(image2), i2(imagi2), j2(imagj2), k2(imagk2),
    u3(image3), i3(imagi3), j3(imagj3), k3(imagk3)
{
    // empty constructor
}

template <typename T> Sedenion<T>::Sedenion(T real) :
    r(real), i(0.0), j(0.0), k(0.0),
    u1(0.0), i1(0.0), j1(0.0), k1(0.0),
    u2(0.0), i2(0.0), j2(0.0), k2(0.0),
    u3(0.0), i3(0.0), j3(0.0), k3(0.0)
{
    // empty constructor
}

/******************************************
 * Assignment Operator
 ******************************************/

template <typename T> const Sedenion<T> & Sedenion<T>::operator = (const T & real)
{
    r = real;
    i = 0.0;
    j = 0.0;
    k = 0.0;
    u1 = 0.0;
    i1 = 0.0;
    j1 = 0.0;
    k1 = 0.0;
    u2 = 0.0;
    i2 = 0.0;
    j2 = 0.0;
    k2 = 0.0;
    u3 = 0.0;
    i3 = 0.0;
    j3 = 0.0;
    k3 = 0.0;

    return *this;
}

/******************************************
 * Mutating Arithmetic Operators
 ******************************************/

template <typename T> Sedenion<T> & Sedenion<T>::operator += (const Sedenion<T> & q)
{
    r += q.r;
    i += q.i;
    j += q.j;
    k += q.k;
    u1 += q.u1;
    i1 += q.i1;
    j1 += q.j1;
    k1 += q.k1;
    u2 += q.u2;
    i2 += q.i2;
    j2 += q.j2;
    k2 += q.k2;
    u3 += q.u3;
    i3 += q.i3;
    j3 += q.j3;
    k3 += q.k3;

    return *this;
}

template <typename T> Sedenion<T> & Sedenion<T>::operator -= (const Sedenion<T> & q)
{
    r -= q.r;
    i -= q.i;
    j -= q.j;
    k -= q.k;
    u1 -= q.u1;
    i1 -= q.i1;
    j1 -= q.j1;
    k1 -= q.k1;
    u2 -= q.u2;
    i2 -= q.i2;
    j2 -= q.j2;
    k2 -= q.k2;
    u3 -= q.u3;
    i3 -= q.i3;
    j3 -= q.j3;
    k3 -= q.k3;

    return *this;
}

template <typename T> Sedenion<T> & Sedenion<T>::operator *= (const Sedenion<T> & q)
{
    T RE = r, I = i, J = j, K = k, U1 = u1, I1 = i1, J1 = j1, K1 = k1, U2 = u2, I2 = i2, J2 = j2, K2 = k2, U3 = u3, I3 = i3, J3 = j3;

    r = RE*q.r - I*q.i - J*q.j - K*q.k - U1*q.u1 - I1*q.i1 - J1*q.j1 - K1*q.k1 - U2*q.u2 - I2*q.i2 - J2*q.j2 - K2*q.k2 - U3*q.u3 - I3*q.i3 - J3*q.j3 - k3*q.k3;
    i = RE*q.i + I*q.r + J*q.k - K*q.j + U1*q.i1 - I1*q.u1 - J1*q.k1 + K1*q.j1 + U2*q.i2 - I2*q.u2 - J2*q.k2 + K2*q.j2 - U3*q.i3 + I3*q.u3 + J3*q.k3 - k3*q.j3;
    j = RE*q.j - I*q.k + J*q.r + K*q.i + U1*q.j1 + I1*q.k1 - J1*q.u1 - K1*q.i1 + U2*q.j2 + I2*q.k2 - J2*q.u2 - K2*q.i2 - U3*q.j3 - I3*q.k3 + J3*q.u3 + k3*q.i3;
    k = RE*q.k + I*q.j - J*q.i + K*q.r + U1*q.k1 - I1*q.j1 + J1*q.i1 - K1*q.u1 + U2*q.k2 - I2*q.j2 + J2*q.i2 - K2*q.u2 - U3*q.k3 + I3*q.j3 - J3*q.i3 + k3*q.u3;
    u1 = RE*q.u1 - I*q.i1 - J*q.j1 - K*q.k1 + U1*q.r + I1*q.i + J1*q.j + K1*q.k + U2*q.u3 + I2*q.i3 + J2*q.j3 + K2*q.k3 - U3*q.u2 - I3*q.i2 - J3*q.j2 - k3*q.k2;
    i1 = RE*q.i1 + I*q.u1 - J*q.k1 + K*q.j1 - U1*q.i + I1*q.r - J1*q.k + K1*q.j + U2*q.i3 - I2*q.u3 + J2*q.k3 - K2*q.j3 + U3*q.i2 - I3*q.u2 + J3*q.k2 - k3*q.j2;
    j1 = RE*q.j1 + I*q.k1 + J*q.u1 - K*q.i1 - U1*q.j + I1*q.k + J1*q.r - K1*q.i + U2*q.j3 - I2*q.k3 - J2*q.u3 + K2*q.i3 + U3*q.j2 - I3*q.k2 - J3*q.u2 + k3*q.i2;
    k1 = RE*q.k1 - I*q.j1 + J*q.i1 + K*q.u1 - U1*q.k - I1*q.j + J1*q.i + K1*q.r + U2*q.k3 + I2*q.j3 - J2*q.i3 - K2*q.u3 + U3*q.k2 + I3*q.j2 - J3*q.i2 - k3*q.u2;
    u2 = RE*q.u2 - I*q.i2 - J*q.j2 - K*q.k2 - U1*q.u3 - I1*q.i3 - J1*q.j3 - K1*q.k3 + U2*q.r + I2*q.i + J2*q.j + K2*q.k + U3*q.u1 + I3*q.i1 + J3*q.j1 + k3*q.k1;
    i2 = RE*q.i2 + I*q.u2 - J*q.k2 + K*q.j2 - U1*q.i3 + I1*q.u3 + J1*q.k3 - K1*q.j3 - U2*q.i + I2*q.r - J2*q.k + K2*q.j - U3*q.i1 + I3*q.u1 + J3*q.k1 - k3*q.j1;
    j2 = RE*q.j2 + I*q.k2 + J*q.u2 - K*q.i2 - U1*q.j3 - I1*q.k3 + J1*q.u3 + K1*q.i3 - U2*q.j + I2*q.k + J2*q.r - K2*q.i - U3*q.j1 - I3*q.k1 + J3*q.u1 + k3*q.i1;
    k2 = RE*q.k2 - I*q.j2 + J*q.i2 + K*q.u2 - U1*q.k3 + I1*q.j3 - J1*q.i3 + K1*q.u3 - U2*q.k - I2*q.j + J2*q.i + K2*q.r - U3*q.k1 + I3*q.j1 - J3*q.i1 + k3*q.u1;
    u3 = RE*q.u3 + I*q.i3 + J*q.j3 + K*q.k3 + U1*q.u2 - I1*q.i2 - J1*q.j2 - K1*q.k2 - U2*q.u1 + I2*q.i1 + J2*q.j1 + K2*q.k1 + U3*q.r - I3*q.i - J3*q.j - k3*q.k;
    i3 = RE*q.i3 - I*q.u3 + J*q.k3 - K*q.j3 + U1*q.i2 + I1*q.u2 + J1*q.k2 - K1*q.j2 - U2*q.i1 - I2*q.u1 + J2*q.k1 - K2*q.j1 + U3*q.i + I3*q.r + J3*q.k - k3*q.j;
    j3 = RE*q.j3 - I*q.k3 - J*q.u3 + K*q.i3 + U1*q.j2 - I1*q.k2 + J1*q.u2 + K1*q.i2 - U2*q.j1 - I2*q.k1 - J2*q.u1 + K2*q.i1 + U3*q.j - I3*q.k + J3*q.r + k3*q.i;
    k3 = RE*q.k3 + I*q.j3 - J*q.i3 - K*q.u3 + U1*q.k2 + I1*q.j2 - J1*q.i2 + K1*q.u2 - U2*q.k1 + I2*q.j1 - J2*q.i1 - K2*q.u1 + U3*q.k + I3*q.j - J3*q.i + k3*q.r;

    return *this;
}

template <typename T> Sedenion<T> & Sedenion<T>::operator /= (const Sedenion<T> & q)
{
    T denom = q.norm();
    T RE = r, I = i, J = j, K = k, U1 = u1, I1 = i1, J1 = j1, K1 = k1, U2 = u2, I2 = i2, J2 = j2, K2 = k2, U3 = u3, I3 = i3, J3 = j3;

    r = (RE*q.r + I*q.i + J*q.j + K*q.k + U1*q.u1 + I1*q.i1 + J1*q.j1 + K1*q.k1 + U2*q.u2 + I2*q.i2 + J2*q.j2 + K2*q.k2 + U3*q.u3 + I3*q.i3 + J3*q.j3 + k3*q.k3) / denom;
    i = (-RE*q.i + I*q.r - J*q.k + K*q.j - U1*q.i1 + I1*q.u1 + J1*q.k1 - K1*q.j1 - U2*q.i2 + I2*q.u2 + J2*q.k2 - K2*q.j2 + U3*q.i3 - I3*q.u3 - J3*q.k3 + k3*q.j3) / denom;
    j = (-RE*q.j + I*q.k + J*q.r - K*q.i - U1*q.j1 - I1*q.k1 + J1*q.u1 + K1*q.i1 - U2*q.j2 - I2*q.k2 + J2*q.u2 + K2*q.i2 + U3*q.j3 + I3*q.k3 - J3*q.u3 - k3*q.i3) / denom;
    k = (-RE*q.k - I*q.j + J*q.i + K*q.r - U1*q.k1 + I1*q.j1 - J1*q.i1 + K1*q.u1 - U2*q.k2 + I2*q.j2 - J2*q.i2 + K2*q.u2 + U3*q.k3 - I3*q.j3 + J3*q.i3 - k3*q.u3) / denom;
    u1 = (-RE*q.u1 + I*q.i1 + J*q.j1 + K*q.k1 + U1*q.r - I1*q.i - J1*q.j - K1*q.k - U2*q.u3 - I2*q.i3 - J2*q.j3 - K2*q.k3 + U3*q.u2 + I3*q.i2 + J3*q.j2 + k3*q.k2) / denom;
    i1 = (-RE*q.i1 - I*q.u1 + J*q.k1 - K*q.j1 + U1*q.i + I1*q.r + J1*q.k - K1*q.j - U2*q.i3 + I2*q.u3 - J2*q.k3 + K2*q.j3 - U3*q.i2 + I3*q.u2 - J3*q.k2 + k3*q.j2) / denom;
    j1 = (-RE*q.j1 - I*q.k1 - J*q.u1 + K*q.i1 + U1*q.j - I1*q.k + J1*q.r + K1*q.i - U2*q.j3 + I2*q.k3 + J2*q.u3 - K2*q.i3 - U3*q.j2 + I3*q.k2 + J3*q.u2 - k3*q.i2) / denom;
    k1 = (-RE*q.k1 + I*q.j1 - J*q.i1 - K*q.u1 + U1*q.k + I1*q.j - J1*q.i + K1*q.r - U2*q.k3 - I2*q.j3 + J2*q.i3 + K2*q.u3 - U3*q.k2 - I3*q.j2 + J3*q.i2 + k3*q.u2) / denom;
    u2 = (-RE*q.u2 + I*q.i2 + J*q.j2 + K*q.k2 + U1*q.u3 + I1*q.i3 + J1*q.j3 + K1*q.k3 + U2*q.r - I2*q.i - J2*q.j - K2*q.k - U3*q.u1 - I3*q.i1 - J3*q.j1 - k3*q.k1) / denom;
    i2 = (-RE*q.i2 - I*q.u2 + J*q.k2 - K*q.j2 + U1*q.i3 - I1*q.u3 - J1*q.k3 + K1*q.j3 + U2*q.i + I2*q.r + J2*q.k - K2*q.j + U3*q.i1 - I3*q.u1 - J3*q.k1 + k3*q.j1) / denom;
    j2 = (-RE*q.j2 - I*q.k2 - J*q.u2 + K*q.i2 + U1*q.j3 + I1*q.k3 - J1*q.u3 - K1*q.i3 + U2*q.j - I2*q.k + J2*q.r + K2*q.i + U3*q.j1 + I3*q.k1 - J3*q.u1 - k3*q.i1) / denom;
    k2 = (-RE*q.k2 + I*q.j2 - J*q.i2 - K*q.u2 + U1*q.k3 - I1*q.j3 + J1*q.i3 - K1*q.u3 + U2*q.k + I2*q.j - J2*q.i + K2*q.r + U3*q.k1 - I3*q.j1 + J3*q.i1 - k3*q.u1) / denom;
    u3 = (-RE*q.u3 - I*q.i3 - J*q.j3 - K*q.k3 - U1*q.u2 + I1*q.i2 + J1*q.j2 + K1*q.k2 + U2*q.u1 - I2*q.i1 - J2*q.j1 - K2*q.k1 + U3*q.r + I3*q.i + J3*q.j + k3*q.k) / denom;
    i3 = (-RE*q.i3 + I*q.u3 - J*q.k3 + K*q.j3 - U1*q.i2 - I1*q.u2 - J1*q.k2 + K1*q.j2 + U2*q.i1 + I2*q.u1 - J2*q.k1 + K2*q.j1 - U3*q.i + I3*q.r - J3*q.k + k3*q.j) / denom;
    j3 = (-RE*q.j3 + I*q.k3 + J*q.u3 - K*q.i3 - U1*q.j2 + I1*q.k2 - J1*q.u2 - K1*q.i2 + U2*q.j1 + I2*q.k1 + J2*q.u1 - K2*q.i1 - U3*q.j + I3*q.k + J3*q.r - k3*q.i) / denom;
    k3 = (-RE*q.k3 - I*q.j3 + J*q.i3 + K*q.u3 - U1*q.k2 - I1*q.j2 + J1*q.i2 - K1*q.u2 + U2*q.k1 - I2*q.j1 + J2*q.i1 + K2*q.u1 - U3*q.k - I3*q.j + J3*q.i + k3*q.r) / denom;

    return *this;
}

template <typename T> Sedenion<T> & Sedenion<T>::operator += (T x)
{
    r += x;

    return *this;
}

template <typename T> Sedenion<T> & Sedenion<T>::operator -= (T x)
{
    r -= x;

    return *this;
}

template <typename T> Sedenion<T> & Sedenion<T>::operator *= (T x)
{
    if (Support<T>::is_inf(x) && norm() > 0)
    {
        r *= (r == 0) ? Support<T>::sign(x) : x;
        i *= (i == 0) ? Support<T>::sign(x) : x;
        j *= (j == 0) ? Support<T>::sign(x) : x;
        k *= (k == 0) ? Support<T>::sign(x) : x;
        u1 *= (u1 == 0) ? Support<T>::sign(x) : x;
        i1 *= (i1 == 0) ? Support<T>::sign(x) : x;
        j1 *= (j1 == 0) ? Support<T>::sign(x) : x;
        k1 *= (k1 == 0) ? Support<T>::sign(x) : x;
        u2 *= (u2 == 0) ? Support<T>::sign(x) : x;
        i2 *= (i2 == 0) ? Support<T>::sign(x) : x;
        j2 *= (j2 == 0) ? Support<T>::sign(x) : x;
        k2 *= (k2 == 0) ? Support<T>::sign(x) : x;
        u3 *= (u3 == 0) ? Support<T>::sign(x) : x;
        i3 *= (i3 == 0) ? Support<T>::sign(x) : x;
        j3 *= (j3 == 0) ? Support<T>::sign(x) : x;
        k3 *= (k3 == 0) ? Support<T>::sign(x) : x;
    }
    else
    {
        r *= x;
        i *= x;
        j *= x;
        k *= x;
        u1 *= x;
        i1 *= x;
        j1 *= x;
        k1 *= x;
        u2 *= x;
        i2 *= x;
        j2 *= x;
        k2 *= x;
        u3 *= x;
        i3 *= x;
        j3 *= x;
        k3 *= x;
    }

    return *this;
}

template <typename T> Sedenion<T> & Sedenion<T>::operator /= (T x)
{
    if (x == 0.0 && !is_zero())
    {
        r /= (r == 0) ? Support<T>::sign(x) : x;
        i /= (i == 0) ? Support<T>::sign(x) : x;
        j /= (j == 0) ? Support<T>::sign(x) : x;
        k /= (k == 0) ? Support<T>::sign(x) : x;
        u1 /= (u1 == 0) ? Support<T>::sign(x) : x;
        i1 /= (i1 == 0) ? Support<T>::sign(x) : x;
        j1 /= (j1 == 0) ? Support<T>::sign(x) : x;
        k1 /= (k1 == 0) ? Support<T>::sign(x) : x;
        u2 /= (u2 == 0) ? Support<T>::sign(x) : x;
        i2 /= (i2 == 0) ? Support<T>::sign(x) : x;
        j2 /= (j2 == 0) ? Support<T>::sign(x) : x;
        k2 /= (k2 == 0) ? Support<T>::sign(x) : x;
        u3 /= (u3 == 0) ? Support<T>::sign(x) : x;
        i3 /= (i3 == 0) ? Support<T>::sign(x) : x;
        j3 /= (j3 == 0) ? Support<T>::sign(x) : x;
        k3 /= (k3 == 0) ? Support<T>::sign(x) : x;
    }
    else
    {
        r /= x;
        i /= x;
        j /= x;
        k /= x;
        u1 /= x;
        i1 /= x;
        j1 /= x;
        k1 /= x;
        u2 /= x;
        i2 /= x;
        j2 /= x;
        k2 /= x;
        u3 /= x;
        i3 /= x;
        j3 /= x;
        k3 /= x;
    }

    return *this;
}

template <typename T> Sedenion<T> & Sedenion<T>::operator ++()
{
    ++r;

    return *this;
}

template <typename T> Sedenion<T> Sedenion<T>::operator ++(int)
{
    Sedenion copy = *this;
    ++r;

    return copy;
}

template <typename T> Sedenion<T> & Sedenion<T>::operator --()
{
    --r;

    return *this;
}

template <typename T> Sedenion<T> Sedenion<T>::operator --(int)
{
    Sedenion copy = *this;
    --r;

    return copy;
}

/******************************************
 * Real-valued Functions
 ******************************************/

template <typename T> T Sedenion<T>::real() const
{
    return r;
}

template <typename T> T Sedenion<T>::operator [](int n) const
{
    return reinterpret_cast<const T *>(this)[n];
}

template <typename T> T& Sedenion<T>::operator [](int n)
{
    return reinterpret_cast<T *>(this)[n];
}

template <typename T> T Sedenion<T>::imag_i() const
{
    return i;
}

template <typename T> T Sedenion<T>::imag_j() const
{
    return j;
}

template <typename T> T Sedenion<T>::imag_k() const
{
    return k;
}

template <typename T> T Sedenion<T>::imag_u1() const
{
    return u1;
}

template <typename T> T Sedenion<T>::imag_i1() const
{
    return i1;
}

template <typename T> T Sedenion<T>::imag_j1() const
{
    return j1;
}

template <typename T> T Sedenion<T>::imag_k1() const
{
    return k1;
}

template <typename T> T Sedenion<T>::imag_u2() const
{
    return u2;
}

template <typename T> T Sedenion<T>::imag_i2() const
{
    return i2;
}

template <typename T> T Sedenion<T>::imag_j2() const
{
    return j2;
}

template <typename T> T Sedenion<T>::imag_k2() const
{
    return k2;
}

template <typename T> T Sedenion<T>::imag_u3() const
{
    return u3;
}

template <typename T> T Sedenion<T>::imag_i3() const
{
    return i3;
}

template <typename T> T Sedenion<T>::imag_j3() const
{
    return j3;
}

template <typename T> T Sedenion<T>::imag_k3() const
{
    return k3;
}

template <typename T> T Sedenion<T>::csgn() const
{
    return is_zero() ? 0.0 : Support<T>::sign(r);
}

template <typename T> T Sedenion<T>::abs() const
{
    return is_inf() ? Support<T>::POS_INF : std::sqrt(
        r*r + i*i + j*j + k*k + u1*u1 + i1*i1 + j1*j1 + k1*k1 +
        u2*u2 + i2*i2 + j2*j2 + k2*k2 + u3*u3 + i3*i3 + j3*j3 + k3*k3
    );
}

template <typename T> T Sedenion<T>::norm() const
{
    return is_inf() ? Support<T>::POS_INF :
        r*r + i*i + j*j + k*k + u1*u1 + i1*i1 + j1*j1 + k1*k1 +
        u2*u2 + i2*i2 + j2*j2 + k2*k2 + u3*u3 + i3*i3 + j3*j3 + k3*k3;
}

template <typename T> T Sedenion<T>::abs_imag() const
{
    return is_inf() ? Support<T>::POS_INF : std::sqrt(
        i*i + j*j + k*k + u1*u1 + i1*i1 + j1*j1 + k1*k1 +
        u2*u2 + i2*i2 + j2*j2 + k2*k2 + u3*u3 + i3*i3 + j3*j3 + k3*k3
    );
}

template <typename T> T Sedenion<T>::norm_imag() const
{
    return is_inf() ? Support<T>::POS_INF :
        i*i + j*j + k*k + u1*u1 + i1*i1 + j1*j1 + k1*k1 +
        u2*u2 + i2*i2 + j2*j2 + k2*k2 + u3*u3 + i3*i3 + j3*j3 + k3*k3;
}

template <typename T> T Sedenion<T>::arg() const
{
    return std::atan2(abs_imag(), r);
}

/******************************************
 * Sedenion<T>-valued Functions
 ******************************************/

template <typename T> Sedenion<T> Sedenion<T>::imag() const
{
    return Sedenion<T>(0.0, i, j, k, u1, i1, j1, k1, u2, i2, j2, k2, u3, i3, j3, k3);
}

template <typename T> Sedenion<T> Sedenion<T>::conj() const
{
    return Sedenion<T>(r, -i, -j, -k, -u1, -i1, -j1, -k1, -u2, -i2, -j2, -k2, -u3, -i3, -j3, -k3);
}

template <typename T> Sedenion<T> Sedenion<T>::operator * () const
{
    return Sedenion<T>(r, -i, -j, -k, -u1, -i1, -j1, -k1, -u2, -i2, -j2, -k2, -u3, -i3, -j3, -k3);
}

template <typename T> Sedenion<T> Sedenion<T>::signum() const
{
    T absq = abs();

    if (absq == 0.0 || Support<T>::is_nan(absq) || Support<T>::is_inf(absq))
    {
        return *this;
    }
    else
    {
        return Sedenion<T>(
            r / absq, i / absq, j / absq, k / absq,
            u1 / absq, i1 / absq, j1 / absq, k1 / absq,
            u2 / absq, i2 / absq, j2 / absq, k2 / absq,
            u3 / absq, i3 / absq, j3 / absq, k3 / absq
            );
    }
}

template <typename T> Sedenion<T> Sedenion<T>::sqr() const
{
    return Sedenion<T>(
        r*r - i*i - j*j - k*k - u1*u1 - i1*i1 - j1*j1 - k1*k1 - u2*u2 - i2*i2 - j2*j2 - k2*k2 - u3*u3 - i3*i3 - j3*j3 - k3*k3,
        2 * r*i, 2 * r*j, 2 * r*k,
        2 * r*u1, 2 * r*i1, 2 * r*j1, 2 * r*k1,
        2 * r*u2, 2 * r*i2, 2 * r*j2, 2 * r*k2,
        2 * r*u3, 2 * r*i3, 2 * r*j3, 2 * r*k3
        );
}

template <typename T> Sedenion<T> Sedenion<T>::sqrt() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).sqrt();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

template <typename T> Sedenion<T> Sedenion<T>::rotate(const Sedenion<T> & q) const
{
    // the assumption is that |q| = 1
    // in this case, q.inverse() == q.conj()

    T rr = q.r*q.r;
    T ii = q.i*q.i;
    T jj = q.j*q.j;
    T kk = q.k*q.k;
    T u1u1 = q.u1*q.u1;
    T i1i1 = q.i1*q.i1;
    T j1j1 = q.j1*q.j1;
    T k1k1 = q.k1*q.k1;
    T u2u2 = q.u2*q.u2;
    T i2i2 = q.i2*q.i2;
    T j2j2 = q.j2*q.j2;
    T k2k2 = q.k2*q.k2;
    T u3u3 = q.u3*q.u3;
    T i3i3 = q.i3*q.i3;
    T j3j3 = q.j3*q.j3;
    T k3k3 = q.k3*q.k3;

    T  iqi = i * q.i;
    T  iqi1 = i * q.i1;
    T  iqj = i * q.j;
    T  iqj1 = i * q.j1;
    T  iqk = i * q.k;
    T  iqk1 = i * q.k1;
    T  iqu1 = i * q.u1;
    T  jqi = j * q.i;
    T  jqi1 = j * q.i1;
    T  jqj = j * q.j;
    T  jqj1 = j * q.j1;
    T  jqk = j * q.k;
    T  jqk1 = j * q.k1;
    T  jqu1 = j * q.u1;
    T  kqi = k * q.i;
    T  kqi1 = k * q.i1;
    T  kqj = k * q.j;
    T  kqj1 = k * q.j1;
    T  kqk = k * q.k;
    T  kqk1 = k * q.k1;
    T  kqu1 = k * q.u1;
    T  u1qi = u1 * q.i;
    T  u1qi1 = u1 * q.i1;
    T  u1qj = u1 * q.j;
    T  u1qj1 = u1 * q.j1;
    T  u1qk = u1 * q.k;
    T  u1qk1 = u1 * q.k1;
    T  u1qu1 = u1 * q.u1;
    T  i1qi = i1 * q.i;
    T  i1qi1 = i1 * q.i1;
    T  i1qj = i1 * q.j;
    T  i1qj1 = i1 * q.j1;
    T  i1qk = i1 * q.k;
    T  i1qk1 = i1 * q.k1;
    T  i1qu1 = i1 * q.u1;
    T  j1qi = j1 * q.i;
    T  j1qi1 = j1 * q.i1;
    T  j1qj = j1 * q.j;
    T  j1qj1 = j1 * q.j1;
    T  j1qk = j1 * q.k;
    T  j1qk1 = j1 * q.k1;
    T  j1qu1 = j1 * q.u1;
    T  k1qi = k1 * q.i;
    T  k1qi1 = k1 * q.i1;
    T  k1qj = k1 * q.j;
    T  k1qj1 = k1 * q.j1;
    T  k1qk = k1 * q.k;
    T  k1qk1 = k1 * q.k1;
    T  k1qu1 = k1 * q.u1;
    T  u2qu2 = u2 * q.u2;
    T  i2qi1 = i2 * q.i1;
    T  i2qi2 = i2 * q.i2;
    T  i2qj = i2 * q.j;
    T  i2qj1 = i2 * q.j1;
    T  i2qk = i2 * q.k;
    T  i2qk1 = i2 * q.k1;
    T  i2qu1 = i2 * q.u1;
    T  j2qi = j2 * q.i;
    T  j2qi1 = j2 * q.i1;
    T  j2qj1 = j2 * q.j1;
    T  j2qj2 = j2 * q.j2;
    T  j2qk = j2 * q.k;
    T  j2qk1 = j2 * q.k1;
    T  j2qu1 = j2 * q.u1;
    T  k2qi = k2 * q.i;
    T  k2qi1 = k2 * q.i1;
    T  k2qj = k2 * q.j;
    T  k2qj1 = k2 * q.j1;
    T  k2qk1 = k2 * q.k1;
    T  k2qk2 = k2 * q.k2;
    T  k2qu1 = k2 * q.u1;
    T  u3qi = u3 * q.i;
    T  u3qi1 = u3 * q.i1;
    T  u3qj = u3 * q.j;
    T  u3qj1 = u3 * q.j1;
    T  u3qk = u3 * q.k;
    T  u3qk1 = u3 * q.k1;
    T  u3qu3 = u3 * q.u3;
    T  i3qi = i3 * q.i;
    T  i3qi3 = i3 * q.i3;
    T  i3qj = i3 * q.j;
    T  i3qj1 = i3 * q.j1;
    T  i3qk = i3 * q.k;
    T  i3qk1 = i3 * q.k1;
    T  i3qu1 = i3 * q.u1;
    T  j3qi = j3 * q.i;
    T  j3qi1 = j3 * q.i1;
    T  j3qj = j3 * q.j;
    T  j3qj3 = j3 * q.j3;
    T  j3qk = j3 * q.k;
    T  j3qk1 = j3 * q.k1;
    T  j3qu1 = j3 * q.u1;
    T  k3qi = k3 * q.i;
    T  k3qi1 = k3 * q.i1;
    T  k3qj = k3 * q.j;
    T  k3qj1 = k3 * q.j1;
    T  k3qk = k3 * q.k;
    T  k3qk3 = k3 * q.k3;
    T  k3qu1 = k3 * q.u1;

    T sum = rr - ii - jj - kk - u1u1 - i1i1 - j1j1 - k1k1 - u2u2 - i2i2 - j2j2 - k2k2 - u3u3 - i3i3 - j3j3 - k3k3;

    return Sedenion<T>(
        ((r == 0) ? r : r*(rr + ii + jj + kk + u1u1 + i1i1 + j1j1 + k1k1 + u2u2 + i2i2 + j2j2 + k2k2 + u3u3 + i3i3 + j3j3 + k3k3)),
        (sum + 2 * ii)*i + 2 * (
            -(-j2*q.k2 - j1qk1 + j3*q.k3 - k3*q.j3 + i3*q.u3 + jqk - i1qu1 - kqj - i2*q.u2 + u2*q.i2 + k1qj1 - u3*q.i3 + k2*q.j2 + u1qi1)*q.r
            + (j1qj1 + u1qu1 + j3qj3 + u2qu2 + k3qk3 + i2qi2 + jqj + i3qi3 + u3qu3 + i1qi1 + k1qk1 + kqk + k2qk2 + j2qj2)*q.i

            + (-j3qi1 - u3qk1 + i3qj1 + k3qu1)*q.j2
            + (-j3qu1 + u3qj1 + i3qk1 - k3qi1)*q.k2
            + (j2qk1 - k2qj1 - k3qj + j3qk)*q.u3
            + (-k2qk1 - j2qj1 + j3qj + k3qk)*q.i3
            + (-u3qk + k2qu1 + j2qi1 - i3qj)*q.j3
            + (-j2qu1 + k2qi1 - i3qk + u3qj)*q.k3
            ),

            (sum + 2 * jj)*j + 2 * (
                -(kqi + i1qk1 - iqk + u1qj1 - j1qu1 - k1qi1 + u2*q.j2 + i2*q.k2 - j2*q.u2 - k2*q.i2 - u3*q.j3 - i3*q.k3 + j3*q.u3 + k3*q.i3)*q.r
                + (iqi + kqk + u1qu1 + i1qi1 + j1qj1 + k1qk1 + u2qu2 + i2qi2 + j2qj2 + k2qk2 + u3qu3 + i3qi3 + j3qj3 + k3qk3)*q.j
                + (j3qi1 + u3qk1 - i3qj1 - k3qu1)*q.i2

                + (-k3qj1 - u3qi1 + j3qk1 + i3qu1)*q.k2
                + (k2qi1 + k3qi - i3qk - i2qk1)*q.u3
                + (i2qj1 - j3qi + u3qk - k2qu1)*q.i3
                + (-i2qi1 + i3qi + k3qk - k2qk1)*q.j3
                + (-u3qi - j3qk + i2qu1 + k2qj1)*q.k3
                ),

                (sum + 2 * kk)*k + 2 * (
                    -(iqj - jqi + u1qk1 - i1qj1 + j1qi1 - k1qu1 + u2*q.k2 - i2*q.j2 + j2*q.i2 - k2*q.u2 - u3*q.k3 + i3*q.j3 - j3*q.i3 + k3*q.u3)*q.r
                    + (iqi + jqj + u1qu1 + i1qi1 + j1qj1 + k1qk1 + u2qu2 + i2qi2 + j2qj2 + k2qk2 + u3qu3 + i3qi3 + j3qj3 + k3qk3)*q.k
                    + (-i3qk1 - u3qj1 + k3qi1 + j3qu1)*q.i2
                    + (u3qi1 - j3qk1 - i3qu1 + k3qj1)*q.j2

                    + (-j2qi1 - j3qi + i2qj1 + i3qj)*q.u3
                    + (i2qk1 - u3qj - k3qi + j2qu1)*q.i3
                    + (j2qk1 + u3qi - k3qj - i2qu1)*q.j3
                    + (-i2qi1 - j2qj1 + i3qi + j3qj)*q.k3
                    ),

                    (sum + 2 * u1u1)*u1 + 2 * (
                        -(-iqi1 + i2*q.i3 + u2*q.u3 - k3*q.k2 + j2*q.j3 - jqj1 - kqk1 + i1qi + j1qj + k1qk - u3*q.u2 - i3*q.i2 - j3*q.j2 + k2*q.k3)*q.r
                        + (u2qu2 + iqi + k1qk1 + kqk + k3qk3 + j2qj2 + jqj + i1qi1 + j1qj1 + u3qu3 + i3qi3 + j3qj3 + i2qi2 + k2qk2)*q.u1
                        + (k2qj1 - j2qk1 + k3qj - j3qk)*q.i2
                        + (i3qk - k3qi - k2qi1 + i2qk1)*q.j2
                        + (j3qi + j2qi1 - i2qj1 - i3qj)*q.k2

                        + (j3qk1 + k2qj - j2qk - k3qj1)*q.i3
                        + (-i3qk1 + k3qi1 + i2qk - k2qi)*q.j3
                        + (-j3qi1 + j2qi - i2qj + i3qj1)*q.k3
                        ),

                        (sum + 2 * i1i1)*i1 + 2 * (
                            -(j2*q.k3 - jqk1 + k1qj + u2*q.i3 - i2*q.u3 + iqu1 + kqj1 - u1qi - j1qk - k2*q.j3 + u3*q.i2 - i3*q.u2 + j3*q.k2 - k3*q.j2)*q.r
                            + (u2qu2 + j2qj2 + u1qu1 + j1qj1 + k1qk1 + k2qk2 + u3qu3 + i3qi3 + j3qj3 + k3qk3 + i2qi2 + iqi + jqj + kqk)*q.i1
                            + (k2qk1 - k3qk - j3qj + j2qj1)*q.i2
                            + (-i2qj1 + k2qu1 + j3qi - u3qk)*q.j2
                            + (-i2qk1 - j2qu1 + k3qi + u3qj)*q.k2
                            + (k3qj1 - k2qj + j2qk - j3qk1)*q.u3

                            + (u3qk1 - j2qi - k3qu1 + i2qj)*q.j3
                            + (j3qu1 - u3qj1 + i2qk - k2qi)*q.k3
                            ),

                            (sum + 2 * j1j1)*j1 + 2 * (
                                -(k3*q.i2 - j3*q.u2 - i3*q.k2 + u3*q.j2 - j2*q.u3 + u2*q.j3 + i1qk - u1qj - kqi1 + jqu1 + iqk1 + k2*q.i3 - i2*q.k3 - k1qi)*q.r
                                + (iqi + k2qk2 + i2qi2 + k1qk1 + kqk + jqj + u1qu1 + i1qi1 + u2qu2 + j2qj2 + u3qu3 + k3qk3 + j3qj3 + i3qi3)*q.j1
                                + (u3qk - k2qu1 - j2qi1 + i3qj)*q.i2
                                + (-k3qk + i2qi1 + k2qk1 - i3qi)*q.j2
                                + (i2qu1 - j2qk1 - u3qi + k3qj)*q.k2
                                + (-i2qk + i3qk1 + k2qi - k3qi1)*q.u3
                                + (j2qi - i2qj + k3qu1 - u3qk1)*q.i3

                                + (u3qi1 - k2qj - i3qu1 + j2qk)*q.k3
                                ),

                                (sum + 2 * k1k1)*k1 + 2 * (
                                    -(j1qi + u2*q.k3 - j2*q.i3 - k3*q.u2 - iqj1 + jqi1 + kqu1 - u1qk - i1qj + i2*q.j3 - k2*q.u3 + u3*q.k2 + i3*q.j2 - j3*q.i2)*q.r
                                    + (u3qu3 + i3qi3 + j3qj3 + k3qk3 + u2qu2 + i2qi2 + j2qj2 + k2qk2 + kqk + u1qu1 + i1qi1 + j1qj1 + jqj + iqi)*q.k1
                                    + (j2qu1 - k2qi1 + i3qk - u3qj)*q.i2
                                    + (-k2qj1 + u3qi + j3qk - i2qu1)*q.j2
                                    + (j2qj1 + i2qi1 - i3qi - j3qj)*q.k2
                                    + (-j2qi - i3qj1 + j3qi1 + i2qj)*q.u3
                                    + (u3qj1 - i2qk - j3qu1 + k2qi)*q.i3
                                    + (-u3qi1 + i3qu1 + k2qj - j2qk)*q.j3

                                    ),

                                    (sum + 2 * u2u2)*u2 + 2 * (
                                        q.r*(j*q.j2 + k*q.k2 + u1*q.u3 + i1*q.i3 + j1*q.j3 + k1*q.k3 - i2*q.i - j2*q.j - k2*q.k - u3*q.u1 - i3*q.i1 - j3*q.j1 - k3*q.k1 + i*q.i2)
                                        + (jqj + kqk + u1qu1 + i1qi1 + j1qj1 + k1qk1 + i2qi2 + j2qj2 + k2qk2 + u3qu3 + i3qi3 + j3qj3 + k3qk3 + iqi)*q.u2
                                        ),

                                        (sum + 2 * i2i2)*i2 + 2 * (
                                            -(i*q.u2 - j*q.k2 + k*q.j2 - u1*q.i3 + i1*q.u3 + j1*q.k3 - k1*q.j3 - u2*q.i - j2qk + k2qj - u3qi1 + i3qu1 + j3qk1 - k3qj1)*q.r
                                            + (iqi + j1qj1 + k1qk1 + u2qu2 + j2qj2 + k2qk2 + u3qu3 + i3qi3 + j3qj3 + k3qk3 + jqj + kqk + u1qu1 + i1qi1)*q.i2
                                            + (-i1qj1 - k1qu1 + u1qk1 + j1qi1)*q.j2
                                            + (-u1qj1 + k1qi1 - i1qk1 + j1qu1)*q.k2
                                            + (k1qj + kqj1 - jqk1 - j1qk)*q.u3
                                            + (jqj1 + kqk1 - j1qj - k1qk)*q.i3
                                            + (i1qj + u1qk - kqu1 - jqi1)*q.j3
                                            + (-u1qj + i1qk + jqu1 - kqi1)*q.k3
                                            ),

                                            (sum + 2 * j2j2)*j2 + 2 * (
                                                -(i2qk - k2qi - u3qj1 - i3qk1 + j3qu1 + k3qi1 + i*q.k2 + j*q.u2 - k*q.i2 - u1*q.j3 - i1*q.k3 + j1*q.u3 + k1*q.i3 - u2*q.j)*q.r
                                                + (i1qj1 + k1qu1 - u1qk1 - j1qi1)*q.i2
                                                + (j1qj1 + k1qk1 + u2qu2 + i2qi2 + k2qk2 + u3qu3 + i3qi3 + j3qj3 + k3qk3 + kqk + u1qu1 + i1qi1 + iqi + jqj)*q.j2
                                                + (k1qj1 + u1qi1 - j1qk1 - i1qu1)*q.k2
                                                + (iqk1 - k1qi + i1qk - kqi1)*q.u3
                                                + (-u1qk + j1qi - iqj1 + kqu1)*q.i3
                                                + (kqk1 - k1qk + iqi1 - i1qi)*q.j3
                                                + (j1qk - iqu1 - kqj1 + u1qi)*q.k3
                                                ),

                                                (sum + 2 * k2k2)*k2 + 2 * (
                                                    -(i1*q.j3 - u1*q.k3 - i2qj - u2*q.k + k1*q.u3 - j1*q.i3 + i3qj1 - u3qk1 + j2qi + k3qu1 - j3qi1 + k*q.u2 + j*q.i2 - i*q.j2)*q.r
                                                    + (-k1qi1 - j1qu1 + i1qk1 + u1qj1)*q.i2
                                                    + (j1qk1 - k1qj1 - u1qi1 + i1qu1)*q.j2
                                                    + (iqi + u1qu1 + kqk + jqj + i1qi1 + j1qj1 + j2qj2 + i2qi2 + u2qu2 + k1qk1 + k3qk3 + j3qj3 + i3qi3 + u3qu3)*q.k2
                                                    + (j1qi - i1qj + jqi1 - iqj1)*q.u3
                                                    + (-iqk1 + u1qj + k1qi - jqu1)*q.i3
                                                    + (iqu1 + k1qj - u1qi - jqk1)*q.j3
                                                    + (-j1qj - i1qi + iqi1 + jqj1)*q.k3
                                                    ),

                                                    (sum + 2 * u3u3)*u3 + 2 * (
                                                        -(i*q.i3 + j*q.j3 + k*q.k3 + u1*q.u2 - i1*q.i2 - j1*q.j2 - j3qj - k3qk - k1*q.k2 + i2qi1 + j2qj1 + k2qk1 - i3qi - u2*q.u1)*q.r
                                                        + (jqk1 - kqj1 - k1qj + j1qk)*q.i2
                                                        + (k1qi + kqi1 - i1qk - iqk1)*q.j2
                                                        + (-jqi1 + i1qj + iqj1 - j1qi)*q.k2
                                                        + (i2qi2 + k2qk2 + i3qi3 + j3qj3 + k3qk3 + j2qj2 + j1qj1 + k1qk1 + u2qu2 + u1qu1 + i1qi1 + kqk + jqj + iqi)*q.u3
                                                        + (-j1qk1 + k1qj1 - kqj + jqk)*q.i3
                                                        + (i1qk1 - k1qi1 + kqi - iqk)*q.j3
                                                        + (j1qi1 - i1qj1 - jqi + iqj)*q.k3
                                                        ),

                                                        (sum + 2 * i3i3)*i3 + 2 * (
                                                            -(-k3qj + j1*q.k2 - k1*q.j2 - u2*q.i1 - i2qu1 + j2qk1 - k2qj1 + u3qi + j3qk - i*q.u3 + j*q.k3 - k*q.j3 + u1*q.i2 + i1*q.u2)*q.r
                                                            + (k1qk - jqj1 - kqk1 + j1qj)*q.i2
                                                            + (u1qk - j1qi + iqj1 - kqu1)*q.j2
                                                            + (jqu1 - u1qj + iqk1 - k1qi)*q.k2
                                                            + (-jqk + kqj + j1qk1 - k1qj1)*q.u3
                                                            + (u1qu1 + jqj + kqk + k1qk1 + u2qu2 + i2qi2 + j2qj2 + k2qk2 + iqi + i1qi1 + j1qj1 + u3qu3 + j3qj3 + k3qk3)*q.i3
                                                            + (-iqj + k1qu1 - u1qk1 + jqi)*q.j3
                                                            + (-iqk + kqi + u1qj1 - j1qu1)*q.k3
                                                            ),

                                                            (sum + 2 * j3j3)*j3 + 2 * (
                                                                -(u1*q.j2 + j1*q.u2 - u2*q.j1 - j2qu1 + u3qj - i3qk + k3qi + k2qi1 - i2qk1 + k1*q.i2 - i1*q.k2 + k*q.i3 - i*q.k3 - j*q.u3)*q.r
                                                                + (-i1qj - u1qk + kqu1 + jqi1)*q.i2
                                                                + (-iqi1 - kqk1 + k1qk + i1qi)*q.j2
                                                                + (-k1qj + jqk1 - iqu1 + u1qi)*q.k2
                                                                + (-i1qk1 + iqk - kqi + k1qi1)*q.u3
                                                                + (u1qk1 - k1qu1 - jqi + iqj)*q.i3
                                                                + (u1qu1 + j1qj1 + k1qk1 + u2qu2 + i2qi2 + j2qj2 + k2qk2 + u3qu3 + i3qi3 + k3qk3 + jqj + kqk + iqi + i1qi1)*q.j3
                                                                + (-u1qi1 + i1qu1 - jqk + kqj)*q.k3
                                                                ),

                                                                (sum + 2 * k3k3)*k3 + 2 * (
                                                                    -(-k2qu1 + i2qj1 + i3qj + i*q.j3 - u2*q.k1 - j2qi1 + u3qk + k1*q.u2 + i1*q.j2 - k*q.u3 - j*q.i3 + u1*q.k2 - j1*q.i2 - j3qi)*q.r
                                                                    + (u1qj - i1qk - jqu1 + kqi1)*q.i2
                                                                    + (-j1qk + iqu1 - u1qi + kqj1)*q.j2
                                                                    + (-jqj1 + j1qj + i1qi - iqi1)*q.k2
                                                                    + (jqi - iqj - j1qi1 + i1qj1)*q.u3
                                                                    + (j1qu1 + iqk - u1qj1 - kqi)*q.i3
                                                                    + (jqk - kqj - i1qu1 + u1qi1)*q.j3
                                                                    + (u2qu2 + i2qi2 + j2qj2 + k2qk2 + u3qu3 + i3qi3 + j3qj3 + kqk + iqi + j1qj1 + k1qk1 + i1qi1 + u1qu1 + jqj)*q.k3
                                                                    )
        );
}

/******************************************
 * Boolean-valued Functions
 ******************************************/

template <typename T> bool Sedenion<T>::is_imaginary() const
{
    return (r == 0.0);
}

template <typename T> bool Sedenion<T>::is_inf() const
{
    return
        (r == Support<T>::POS_INF) ||
        (r == Support<T>::NEG_INF) ||
        (i == Support<T>::POS_INF) ||
        (i == Support<T>::NEG_INF) ||
        (j == Support<T>::POS_INF) ||
        (j == Support<T>::NEG_INF) ||
        (k == Support<T>::POS_INF) ||
        (k == Support<T>::NEG_INF) ||
        (u1 == Support<T>::POS_INF) ||
        (u1 == Support<T>::NEG_INF) ||
        (i1 == Support<T>::POS_INF) ||
        (i1 == Support<T>::NEG_INF) ||
        (j1 == Support<T>::POS_INF) ||
        (j1 == Support<T>::NEG_INF) ||
        (k1 == Support<T>::POS_INF) ||
        (k1 == Support<T>::NEG_INF) ||
        (u2 == Support<T>::POS_INF) ||
        (u2 == Support<T>::NEG_INF) ||
        (i2 == Support<T>::POS_INF) ||
        (i2 == Support<T>::NEG_INF) ||
        (j2 == Support<T>::POS_INF) ||
        (j2 == Support<T>::NEG_INF) ||
        (k2 == Support<T>::POS_INF) ||
        (k2 == Support<T>::NEG_INF) ||
        (u3 == Support<T>::POS_INF) ||
        (u3 == Support<T>::NEG_INF) ||
        (i3 == Support<T>::POS_INF) ||
        (i3 == Support<T>::NEG_INF) ||
        (j3 == Support<T>::POS_INF) ||
        (j3 == Support<T>::NEG_INF) ||
        (k3 == Support<T>::POS_INF) ||
        (k3 == Support<T>::NEG_INF);
}

template <typename T> bool Sedenion<T>::is_nan() const
{
    return (r != r) || (i != i) || (j != j) || (k != k) || (u1 != u1) || (i1 != i1) || (j1 != j1) || (k1 != k1) || (u2 != u2) || (i2 != i2) || (j2 != j2) || (k2 != k2) || (u3 != u3) || (i3 != i3) || (j3 != j3) || (k3 != k3);
}

template <typename T> bool Sedenion<T>::is_neg_inf() const
{
    return (r == Support<T>::NEG_INF) && (i == 0.0) && (j == 0.0) && (k == 0.0) && (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0) && (u2 == 0.0) && (i2 == 0.0) && (j2 == 0.0) && (k2 == 0.0) && (u3 == 0.0) && (i3 == 0.0) && (j3 == 0.0) && (k3 == 0.0);
}

template <typename T> bool Sedenion<T>::is_pos_inf() const
{
    return (r == Support<T>::POS_INF) && (i == 0.0) && (j == 0.0) && (k == 0.0) && (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0) && (u2 == 0.0) && (i2 == 0.0) && (j2 == 0.0) && (k2 == 0.0) && (u3 == 0.0) && (i3 == 0.0) && (j3 == 0.0) && (k3 == 0.0);
}

template <typename T> bool Sedenion<T>::is_real() const
{
    return (i == 0.0) && (j == 0.0) && (k == 0.0) && (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0) && (u2 == 0.0) && (i2 == 0.0) && (j2 == 0.0) && (k2 == 0.0) && (u3 == 0.0) && (i3 == 0.0) && (j3 == 0.0) && (k3 == 0.0);
}

template <typename T> bool Sedenion<T>::is_real_inf() const
{
    return (r == Support<T>::POS_INF || r == Support<T>::NEG_INF) && (i == 0.0) && (j == 0.0) && (k == 0.0) && (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0) && (u2 == 0.0) && (i2 == 0.0) && (j2 == 0.0) && (k2 == 0.0) && (u3 == 0.0) && (i3 == 0.0) && (j3 == 0.0) && (k3 == 0.0);
}

template <typename T> bool Sedenion<T>::is_zero() const
{
    return (r == 0.0) && (i == 0.0) && (j == 0.0) && (k == 0.0) && (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0) && (u2 == 0.0) && (i2 == 0.0) && (j2 == 0.0) && (k2 == 0.0) && (u3 == 0.0) && (i3 == 0.0) && (j3 == 0.0) && (k3 == 0.0);
}

/******************************************
 * Multiplier Function
 ******************************************/

template <typename T> inline Sedenion<T> Sedenion<T>::multiplier(T r, T mltplr, const Sedenion<T> & q)
{
    if (Support<T>::is_nan(mltplr) || Support<T>::is_inf(mltplr))
    {
        if (q.i == 0 && q.j == 0 && q.k == 0)
        {
            return Sedenion<T>(r, mltplr*q.i, mltplr*q.j, mltplr*q.k, mltplr*q.u1, mltplr*q.i1, mltplr*q.j1, mltplr*q.k1, mltplr*q.u2, mltplr*q.i2, mltplr*q.j2, mltplr*q.k2, mltplr*q.u3, mltplr*q.i3, mltplr*q.j3, mltplr*q.k3);
        }
        else
        {
            return Sedenion<T>(
                r,
                (q.i == 0) ? Support<T>::sign(mltplr)*q.i : mltplr*q.i,
                (q.j == 0) ? Support<T>::sign(mltplr)*q.j : mltplr*q.j,
                (q.k == 0) ? Support<T>::sign(mltplr)*q.k : mltplr*q.k,
                (q.u1 == 0) ? Support<T>::sign(mltplr)*q.u1 : mltplr*q.u1,
                (q.i1 == 0) ? Support<T>::sign(mltplr)*q.i1 : mltplr*q.i1,
                (q.j1 == 0) ? Support<T>::sign(mltplr)*q.j1 : mltplr*q.j1,
                (q.k1 == 0) ? Support<T>::sign(mltplr)*q.k1 : mltplr*q.k1,
                (q.u2 == 0) ? Support<T>::sign(mltplr)*q.u2 : mltplr*q.u2,
                (q.i2 == 0) ? Support<T>::sign(mltplr)*q.i2 : mltplr*q.i2,
                (q.j2 == 0) ? Support<T>::sign(mltplr)*q.j2 : mltplr*q.j2,
                (q.k2 == 0) ? Support<T>::sign(mltplr)*q.k2 : mltplr*q.k2,
                (q.u3 == 0) ? Support<T>::sign(mltplr)*q.u3 : mltplr*q.u3,
                (q.i3 == 0) ? Support<T>::sign(mltplr)*q.i3 : mltplr*q.i3,
                (q.j3 == 0) ? Support<T>::sign(mltplr)*q.j3 : mltplr*q.j3,
                (q.k3 == 0) ? Support<T>::sign(mltplr)*q.k3 : mltplr*q.k3
                );
        }
    }
    else
    {
        return Sedenion<T>(
            r, mltplr*q.i, mltplr*q.j, mltplr*q.k,
            mltplr*q.u1, mltplr*q.i1, mltplr*q.j1, mltplr*q.k1,
            mltplr*q.u2, mltplr*q.i2, mltplr*q.j2, mltplr*q.k2,
            mltplr*q.u3, mltplr*q.i3, mltplr*q.j3, mltplr*q.k3
            );
    }
}

template <typename T> inline Sedenion<T> Sedenion<T>::make_inf(T r, T i)
{
    return Sedenion<T>(r, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i);
}

template <typename T> inline Sedenion<T> Sedenion<T>::make_i(T r, T i)
{
    return Sedenion<T>(r, i, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

/******************************************
 * Exponential and Logarithmic Functions
 ******************************************/

template <typename T> Sedenion<T> Sedenion<T>::exp() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).exp();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

template <typename T> Sedenion<T> Sedenion<T>::log() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).log();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

template <typename T> Sedenion<T> Sedenion<T>::log10() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).log10();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

template <typename T> Sedenion<T> Sedenion<T>::pow(const Sedenion<T> & q) const
{
    return (log() * q).exp();
}

template <typename T> Sedenion<T> Sedenion<T>::pow(T x) const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).pow(x);

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

template <typename T> Sedenion<T> Sedenion<T>::inverse() const
{
    if (is_zero())
    {
        return Sedenion(
            Support<T>::sign(r)*Support<T>::POS_INF,
            -Support<T>::sign(i)*Support<T>::POS_INF,
            -Support<T>::sign(j)*Support<T>::POS_INF,
            -Support<T>::sign(k)*Support<T>::POS_INF,
            -Support<T>::sign(u1)*Support<T>::POS_INF,
            -Support<T>::sign(i1)*Support<T>::POS_INF,
            -Support<T>::sign(j1)*Support<T>::POS_INF,
            -Support<T>::sign(k1)*Support<T>::POS_INF,
            -Support<T>::sign(u2)*Support<T>::POS_INF,
            -Support<T>::sign(i2)*Support<T>::POS_INF,
            -Support<T>::sign(j2)*Support<T>::POS_INF,
            -Support<T>::sign(k2)*Support<T>::POS_INF,
            -Support<T>::sign(u3)*Support<T>::POS_INF,
            -Support<T>::sign(i3)*Support<T>::POS_INF,
            -Support<T>::sign(j3)*Support<T>::POS_INF,
            -Support<T>::sign(k3)*Support<T>::POS_INF
        );
    }
    else if (is_inf())
    {
        return Sedenion(
            Support<T>::sign(r)*0.0,
            -Support<T>::sign(i)*0.0,
            -Support<T>::sign(j)*0.0,
            -Support<T>::sign(k)*0.0,
            -Support<T>::sign(u1)*0.0,
            -Support<T>::sign(i1)*0.0,
            -Support<T>::sign(j1)*0.0,
            -Support<T>::sign(k1)*0.0,
            -Support<T>::sign(u2)*0.0,
            -Support<T>::sign(i2)*0.0,
            -Support<T>::sign(j2)*0.0,
            -Support<T>::sign(k2)*0.0,
            -Support<T>::sign(u3)*0.0,
            -Support<T>::sign(i3)*0.0,
            -Support<T>::sign(j3)*0.0,
            -Support<T>::sign(k3)*0.0
        );
    }
    else if (is_nan())
    {
        return Sedenion(
            Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN,
            Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN,
            Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN,
            Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN
        );
    }
    else
    {
        T denom = norm();

        return Sedenion(
            r / denom, -i / denom, -j / denom, -k / denom,
            -u1 / denom, -i1 / denom, -j1 / denom, -k1 / denom,
            -u2 / denom, -i2 / denom, -j2 / denom, -k2 / denom,
            -u3 / denom, -i3 / denom, -j3 / denom, -k3 / denom
        );
    }
}

/********************************************************************
 * Trigonometric and Hyperbolic Functions
 *
 * For each function f:S -> S, we define:
 *
 *             ~                       Im(q)     ~
 *   f(q) = Re(f(Re(q) + i|Im(q)|)) + ------- Im(f(Re(q) + i|Im(q)|))
 *                                    |Im(q)|
 *       ~
 * where f:C -> C is the complex equivalent of the
 * function f.
 ********************************************************************/

 /**********************************************************
  * Sine Function
  **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::sin() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).sin();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Complementary Sine Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::cos() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).cos();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Tangent Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::tan() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).tan();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Secant Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::sec() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).sec();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Complementary Secant Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::csc() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).csc();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Complementary Tangent Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::cot() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).cot();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Hyperbolic Sine Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::sinh() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).sinh();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Hyperbolic Complementary Sine Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::cosh() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).cosh();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Hyperbolic Tangent Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::tanh() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).tanh();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Hyperbolic Secant Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::sech() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).sech();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Hyperbolic Complementary Secant Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::csch() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).csch();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Hyperbolic Complementary Tangent Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::coth() const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).coth();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

// Real Branch Cut:    (-oo, -1) U (1, oo)

/**********************************************************
 * Inverse Sine Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::asin(const Sedenion<T> & q) const
{
    T absIm = abs_imag();

    // Branch Cuts:   (-oo, -1) U (1, oo)

    if (absIm == 0)
    {
        if (r > 1)
        {
            // Branch cut (1, oo)

            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(Support<T>::PI2, std::log(r + std::sqrt(r*r - 1)));
            }
            else
            {
                return multiplier(Support<T>::PI2, std::log(r + std::sqrt(r*r - 1)) / absq, q);
            }
        }
        else if (r < -1)
        {
            // Branch cut (-oo, -1)

            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(-Support<T>::PI2, std::log(-r + std::sqrt(r*r - 1)));
            }
            else
            {
                return multiplier(-Support<T>::PI2, std::log(-r + std::sqrt(r*r - 1)) / absq, q);
            }
        }
    }

    Complex<T> z = Complex<T>(r, absIm).asin();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

// Real Branch Cut:    (-oo, -1) U (1, oo)

/**********************************************************
 * Inverse Complementary Sine Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::acos(const Sedenion<T> & q) const
{
    T absIm = abs_imag();

    // Branch Cuts:   (-oo, -1) U (1, oo)

    if (absIm == 0)
    {
        if (r > 1)
        {
            // Branch cut (1, oo)

            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(0.0, -std::log(r + std::sqrt(r*r - 1)));
            }
            else
            {
                return multiplier(0.0, -std::log(r + std::sqrt(r*r - 1)) / absq, q);
            }
        }
        else if (r < -1)
        {
            // Branch cut (-oo, -1)

            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(Support<T>::PI, -std::log(-r + std::sqrt(r*r - 1)));
            }
            else
            {
                return multiplier(Support<T>::PI, -std::log(-r + std::sqrt(r*r - 1)) / absq, q);
            }
        }
    }

    Complex<T> z = Complex<T>(r, absIm).acos();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

// Complex Branch Cut:    (-ooi, -i] U [i, ooi)

/**********************************************************
 * Inverse Tangent Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::atan() const
{
    T absIm = abs_imag();

    if (r == 0)
    {
        if (absIm == 1)
        {
            return multiplier(Support<T>::NaN, Support<T>::POS_INF, *this);
        }
        else if (absIm > 1)
        {
            // Branch cut [ui, Inf)
            //  - ui is a unit purely-imaginary quaternion

            T p = absIm + 1;
            T m = absIm - 1;

            T mltplr = 0.25*std::log((p*p) / (m*m)) / absIm;

            return multiplier(Support<T>::sign(r)*Support<T>::PI2, mltplr, *this);
        }
    }

    Complex<T> z = Complex<T>(r, absIm).atan();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Inverse Secant Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::asec(const Sedenion<T> & q) const
{
    T absIm = abs_imag();

    // Branch Cut: (-1, 1)

    if (absIm == 0)
    {
        if (r == 0.0)
        {
            return multiplier(Support<T>::POS_INF, Support<T>::POS_INF, q);
        }
        else if (r > 0.0 && r < 1.0)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(0.0, std::log(1.0 / r + std::sqrt(1.0 / (r*r) - 1.0)));
            }
            else
            {
                return multiplier(0.0, std::log(1.0 / r + std::sqrt(1.0 / (r*r) - 1.0)) / absq, q);
            }
        }
        else if (r > -1.0 && r < 0.0)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(Support<T>::PI, std::log(-1.0 / r + std::sqrt(1.0 / (r*r) - 1.0)));
            }
            else
            {
                return multiplier(Support<T>::PI, std::log(-1.0 / r + std::sqrt(1.0 / (r*r) - 1.0)) / absq, q);
            }
        }
    }

    Complex<T> z = Complex<T>(r, absIm).asec();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

// Real Branch Cut:    (-1, 1)
/**********************************************************
 * Inverse Complementary Secant Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::acsc(const Sedenion<T> & q) const
{
    T absIm = abs_imag();

    if (absIm == 0.0)
    {
        if (r == 0.0)
        {
            return multiplier(Support<T>::POS_INF, Support<T>::POS_INF, q);
        }
        else if (r > 0.0 && r < 1.0)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(Support<T>::PI2, -std::log(1.0 / r + std::sqrt(1.0 / (r*r) - 1.0)));
            }
            else
            {
                return multiplier(Support<T>::PI2, -std::log(1.0 / r + std::sqrt(1.0 / (r*r) - 1.0)) / absq, q);
            }
        }
        else if (r > -1.0 && r < 0.0)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(-Support<T>::PI2, -std::log(-1.0 / r + std::sqrt(1.0 / (r*r) - 1.0)));
            }
            else
            {
                return multiplier(-Support<T>::PI2, -std::log(-1.0 / r + std::sqrt(1.0 / (r*r) - 1.0)) / absq, q);
            }
        }
    }

    Complex<T> z = Complex<T>(r, absIm).acsc();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Inverse Complementary Tangent Function
 * Complex Branch Cut:    (-i, i)
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::acot() const
{
    T absIm = abs_imag();

    if (r == 0)
    {
        if (absIm == 0)
        {
            return multiplier(Support<T>::PI2, -1, *this);
        }
        else if (absIm < 1)
        {
            // Branch cut [ui, Inf)
            //  - ui is a unit purely-imaginary quaternion

            T p = absIm + 1;
            T m = absIm - 1;

            T mltplr = -0.25*std::log((p*p) / (m*m)) / absIm;

            return multiplier(Support<T>::PI2, mltplr, *this);
        }
    }

    Complex<T> z = Complex<T>(r, absIm).acot();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

// Complex Branch Cut:    (-ooi, -i) U (i, ooi)

/**********************************************************
 * Inverse Hyperbolic Sine Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::asinh() const
{
    T absIm = abs_imag();

    if (r == 0)
    {
        if (absIm > 1)
        {
            return multiplier(Support<T>::sign(r)*std::log(absIm + std::sqrt(absIm*absIm - 1)), Support<T>::PI2 / absIm, *this);
        }
    }

    Complex<T> z = Complex<T>(r, absIm).asinh();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Inverse Hyperbolic Complementary Sine Function
 * Real Branch Cut:    (-oo, 1)
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::acosh(const Sedenion<T> & q) const
{
    T absIm = abs_imag();

    if (absIm == 0)
    {
        if (r < -1)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(std::log(-r + std::sqrt(r*r - 1)), Support<T>::PI*Support<T>::sign(i));
            }
            else
            {
                return multiplier(std::log(-r + std::sqrt(r*r - 1)), Support<T>::PI / absq, q);
            }
        }
        else if (r == -1)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(0.0, Support<T>::PI*Support<T>::sign(i));
            }
            else
            {
                return multiplier(0.0, Support<T>::PI / absq, q);
            }
        }
        else if (r < 0)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(0.0, std::acos(r)*Support<T>::sign(i));
            }
            else
            {
                return multiplier(0.0, std::acos(r) / absq, q);
            }
        }
        else if (r == 0)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(0.0, Support<T>::PI2*Support<T>::sign(i));
            }
            else
            {
                return multiplier(0.0, Support<T>::PI2 / absq, q);
            }
        }
        else if (r < 1)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(0.0, std::acos(r)*Support<T>::sign(i));
            }
            else
            {
                return multiplier(0.0, std::acos(r) / absq, q);
            }
        }
    }

    Complex<T> z = Complex<T>(r, absIm).acosh();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Inverse Hyperbolic Tangent Function
 * Real Branch Cut:    (-oo, -1] U [1, oo)
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::atanh(const Sedenion<T> & q) const
{
    T absIm = abs_imag();

    if (absIm == 0)
    {
        if (r == -1)
        {
            return make_inf(Support<T>::NEG_INF, Support<T>::NaN);
        }
        else if (r == 1)
        {
            return make_inf(Support<T>::POS_INF, Support<T>::NaN);
        }
        else if (r < -1 || r > 1)
        {
            T p = r + 1;
            T m = r - 1;

            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(0.25*std::log((p*p) / (m*m)), -Support<T>::sign(r)*Support<T>::PI2);
            }
            else
            {
                return multiplier(0.25*std::log((p*p) / (m*m)), -Support<T>::sign(r)*Support<T>::PI2 / absq, q);
            }
        }
    }

    Complex<T> z = Complex<T>(r, absIm).atanh();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Inverse Hyperbolic Secant Function
 * Real Branch Cut:    (-oo, 0] U (1, oo)
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::asech(const Sedenion<T> & q) const
{
    T absIm = abs_imag();

    if (absIm == 0)
    {
        if (r < -1 || r > 1)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(0.0, -std::acos(1 / r)*Support<T>::sign(i));
            }
            else
            {
                return multiplier(0.0, -std::acos(1 / r) / absq, q);
            }
        }
        else if (r == -1)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(0.0, Support<T>::PI*Support<T>::sign(i));
            }
            else
            {
                return multiplier(0.0, Support<T>::PI / absq, q);
            }
        }
        else if (r < 0)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(std::log(-1 / r + std::sqrt(1 / (r*r) - 1)), -Support<T>::PI);
            }
            else
            {
                return multiplier(std::log(-1 / r + std::sqrt(1 / (r*r) - 1)), -Support<T>::PI / absq, q);
            }
        }
        else if (r == 0)
        {
            return make_inf(Support<T>::POS_INF, Support<T>::NaN);
        }
    }

    Complex<T> z = Complex<T>(r, absIm).asech();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Inverse Hyperbolic Complementary Secant Function
 * Complex Branch Cut:    (-i, i)
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::acsch() const
{
    T absIm = abs_imag();

    if (r == 0)
    {
        if (absIm == 0)
        {
            return make_inf(Support<T>::NEG_INF, Support<T>::NaN);
        }
        else if (absIm < 1)
        {
            return multiplier(Support<T>::sign(r)*std::log(1 / absIm + std::sqrt(1 / (absIm*absIm) - 1)), -Support<T>::PI2 / absIm, *this);
        }
    }

    Complex<T> z = Complex<T>(r, absIm).acsch();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Inverse Hyperbolic Complementary Tangent Function
 * Real Branch Cut:    [-1, 1]
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::acoth(const Sedenion<T> & q) const
{
    T absIm = abs_imag();

    if (absIm == 0)
    {
        if (r == -1)
        {
            return make_inf(Support<T>::NEG_INF, Support<T>::NaN);
        }
        else if (r == 1)
        {
            return make_inf(Support<T>::POS_INF, Support<T>::NaN);
        }
        else if (r == 0)
        {
            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(0.0, -Support<T>::PI2*Support<T>::sign(i));
            }
            else
            {
                return multiplier(0.0, -Support<T>::PI2 / absq, q);
            }
        }
        else if (r > -1 && r < 0)
        {
            T p = r + 1;
            T m = r - 1;

            T absq = q.abs_imag();

            if (q == I || absq == 0)
            {
                return make_i(0.25*std::log((p*p) / (m*m)), -Support<T>::sign(r)*Support<T>::PI2);
            }
            else
            {
                return multiplier(0.25*std::log((p*p) / (m*m)), -Support<T>::sign(r)*Support<T>::PI2 / absq, q);
            }
        }
    }

    Complex<T> z = Complex<T>(r, absIm).acoth();

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/**********************************************************
 * Bessel J Function
 **********************************************************/

template <typename T> Sedenion<T> Sedenion<T>::bessel_J(int n) const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).bessel_J(n);

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/******************************************
 * Integer Functions
 ******************************************/

template <typename T> Sedenion<T> Sedenion<T>::ceil() const
{
    return Sedenion<T>(
        std::ceil(r), std::ceil(i), std::ceil(j), std::ceil(k),
        std::ceil(u1), std::ceil(i1), std::ceil(j1), std::ceil(k1),
        std::ceil(u2), std::ceil(i2), std::ceil(j2), std::ceil(k2),
        std::ceil(u3), std::ceil(i3), std::ceil(j3), std::ceil(k3)
        );
}

template <typename T> Sedenion<T> Sedenion<T>::floor() const
{
    return Sedenion<T>(
        std::floor(r), std::floor(i), std::floor(j), std::floor(k),
        std::floor(u1), std::floor(i1), std::floor(j1), std::floor(k1),
        std::floor(u2), std::floor(i2), std::floor(j2), std::floor(k2),
        std::floor(u3), std::floor(i3), std::floor(j3), std::floor(k3)
        );
}

/******************************************
 * Horner's Rule
 *
 *   The polynomial is defined by giving the highest
 *   coefficient first:
 *
 *            n - 1         n - 2
 *      v[0]*q      + v[1]*q      + ... + v[n-2]*q + v[n-1]
 *
 *   This is the same as with Matlab.  Because sedenions are
 *   not commutative, this only makes sense if the coefficients
 *   and offsets are real.
 *
 *        Re(q) + i |Imag(q)|
 *
 ******************************************/

template <typename T> Sedenion<T> Sedenion<T>::horner(T * v, unsigned int n) const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).horner(v, n);

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

template <typename T> Sedenion<T> Sedenion<T>::horner(T * v, T * c, unsigned int n) const
{
    T absIm = abs_imag();
    Complex<T> z = Complex<T>(r, absIm).horner(v, c, n);

    T mltplr;

    if (absIm == 0.0)
    {
        mltplr = z.imag_i();
    }
    else
    {
        mltplr = z.imag_i() / absIm;
    }

    return multiplier(z.real(), mltplr, *this);
}

/******************************************
 * Random Factories
 ******************************************/

template <typename T> Sedenion<T> Sedenion<T>::random()
{
    return Sedenion<T>(
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX
        );
}

template <typename T> Sedenion<T> Sedenion<T>::random_imag()
{
    return Sedenion<T>(
        0.0,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX
        );
}

template <typename T> Sedenion<T> Sedenion<T>::random_real()
{
    return Sedenion<T>((static_cast<T>(rand())) / RAND_MAX, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

/******************************************
 * Binary Arithmetic Operators
 ******************************************/

template <typename T> Sedenion<T> Sedenion<T>::operator + (const Sedenion<T> & z) const
{
    return Sedenion<T>(
        r + z.r, i + z.i, j + z.j, k + z.k,
        u1 + z.u1, i1 + z.i1, j1 + z.j1, k1 + z.k1,
        u2 + z.u2, i2 + z.i2, j2 + z.j2, k2 + z.k2,
        u3 + z.u3, i3 + z.i3, j3 + z.j3, k3 + z.k3
        );
}

template <typename T> Sedenion<T> Sedenion<T>::operator + (T x) const
{
    return Sedenion<T>(r + x, i, j, k, u1, i1, j1, k1, u2, i2, j2, k2, u3, i3, j3, k3);
}

template <typename T> Sedenion<T> operator + (T x, const Sedenion<T> & z)
{
    return Sedenion<T>(
        x + z.real(), z.imag_i(), z.imag_j(), z.imag_k(),
        z.imag_u1(), z.imag_i1(), z.imag_j1(), z.imag_k1(),
        z.imag_u2(), z.imag_i2(), z.imag_j2(), z.imag_k2(),
        z.imag_u3(), z.imag_i3(), z.imag_j3(), z.imag_k3()
        );
}

template <typename T> Sedenion<T> operator + (long x, const Sedenion<T> & q)
{
    return operator + (static_cast<T>(x), q);
}

template <typename T> Sedenion<T> Sedenion<T>::operator - (const Sedenion<T> & z) const
{
    return Sedenion<T>(
        r - z.r, i - z.i, j - z.j, k - z.k,
        u1 - z.u1, i1 - z.i1, j1 - z.j1, k1 - z.k1,
        u2 - z.u2, i2 - z.i2, j2 - z.j2, k2 - z.k2,
        u3 - z.u3, i3 - z.i3, j3 - z.j3, k3 - z.k3
        );
}

template <typename T> Sedenion<T> Sedenion<T>::operator - (T x) const
{
    return Sedenion<T>(r - x, i, j, k, u1, i1, j1, k1, u2, i2, j2, k2, u3, i3, j3, k3);
}

template <typename T> Sedenion<T> operator - (T x, const Sedenion<T> & z)
{
    return Sedenion<T>(
        x - z.real(), -z.imag_i(), -z.imag_j(), -z.imag_k(),
        -z.imag_u1(), -z.imag_i1(), -z.imag_j1(), -z.imag_k1(),
        -z.imag_u2(), -z.imag_i2(), -z.imag_j2(), -z.imag_k2(),
        -z.imag_u3(), -z.imag_i3(), -z.imag_j3(), -z.imag_k3()
        );
}

template <typename T> Sedenion<T> operator - (long x, const Sedenion<T> & q)
{
    return operator - (static_cast<T>(x), q);
}

template <typename T> Sedenion<T> Sedenion<T>::operator * (const Sedenion<T> & q) const
{
    return Sedenion<T>(
        r*q.r - i*q.i - j*q.j - k*q.k - u1*q.u1 - i1*q.i1 - j1*q.j1 - k1*q.k1 - u2*q.u2 - i2*q.i2 - j2*q.j2 - k2*q.k2 - u3*q.u3 - i3*q.i3 - j3*q.j3 - k3*q.k3,
        r*q.i + i*q.r + j*q.k - k*q.j + u1*q.i1 - i1*q.u1 - j1*q.k1 + k1*q.j1 + u2*q.i2 - i2*q.u2 - j2*q.k2 + k2*q.j2 - u3*q.i3 + i3*q.u3 + j3*q.k3 - k3*q.j3,
        r*q.j - i*q.k + j*q.r + k*q.i + u1*q.j1 + i1*q.k1 - j1*q.u1 - k1*q.i1 + u2*q.j2 + i2*q.k2 - j2*q.u2 - k2*q.i2 - u3*q.j3 - i3*q.k3 + j3*q.u3 + k3*q.i3,
        r*q.k + i*q.j - j*q.i + k*q.r + u1*q.k1 - i1*q.j1 + j1*q.i1 - k1*q.u1 + u2*q.k2 - i2*q.j2 + j2*q.i2 - k2*q.u2 - u3*q.k3 + i3*q.j3 - j3*q.i3 + k3*q.u3,
        r*q.u1 - i*q.i1 - j*q.j1 - k*q.k1 + u1*q.r + i1*q.i + j1*q.j + k1*q.k + u2*q.u3 + i2*q.i3 + j2*q.j3 + k2*q.k3 - u3*q.u2 - i3*q.i2 - j3*q.j2 - k3*q.k2,
        r*q.i1 + i*q.u1 - j*q.k1 + k*q.j1 - u1*q.i + i1*q.r - j1*q.k + k1*q.j + u2*q.i3 - i2*q.u3 + j2*q.k3 - k2*q.j3 + u3*q.i2 - i3*q.u2 + j3*q.k2 - k3*q.j2,
        r*q.j1 + i*q.k1 + j*q.u1 - k*q.i1 - u1*q.j + i1*q.k + j1*q.r - k1*q.i + u2*q.j3 - i2*q.k3 - j2*q.u3 + k2*q.i3 + u3*q.j2 - i3*q.k2 - j3*q.u2 + k3*q.i2,
        r*q.k1 - i*q.j1 + j*q.i1 + k*q.u1 - u1*q.k - i1*q.j + j1*q.i + k1*q.r + u2*q.k3 + i2*q.j3 - j2*q.i3 - k2*q.u3 + u3*q.k2 + i3*q.j2 - j3*q.i2 - k3*q.u2,
        r*q.u2 - i*q.i2 - j*q.j2 - k*q.k2 - u1*q.u3 - i1*q.i3 - j1*q.j3 - k1*q.k3 + u2*q.r + i2*q.i + j2*q.j + k2*q.k + u3*q.u1 + i3*q.i1 + j3*q.j1 + k3*q.k1,
        r*q.i2 + i*q.u2 - j*q.k2 + k*q.j2 - u1*q.i3 + i1*q.u3 + j1*q.k3 - k1*q.j3 - u2*q.i + i2*q.r - j2*q.k + k2*q.j - u3*q.i1 + i3*q.u1 + j3*q.k1 - k3*q.j1,
        r*q.j2 + i*q.k2 + j*q.u2 - k*q.i2 - u1*q.j3 - i1*q.k3 + j1*q.u3 + k1*q.i3 - u2*q.j + i2*q.k + j2*q.r - k2*q.i - u3*q.j1 - i3*q.k1 + j3*q.u1 + k3*q.i1,
        r*q.k2 - i*q.j2 + j*q.i2 + k*q.u2 - u1*q.k3 + i1*q.j3 - j1*q.i3 + k1*q.u3 - u2*q.k - i2*q.j + j2*q.i + k2*q.r - u3*q.k1 + i3*q.j1 - j3*q.i1 + k3*q.u1,
        r*q.u3 + i*q.i3 + j*q.j3 + k*q.k3 + u1*q.u2 - i1*q.i2 - j1*q.j2 - k1*q.k2 - u2*q.u1 + i2*q.i1 + j2*q.j1 + k2*q.k1 + u3*q.r - i3*q.i - j3*q.j - k3*q.k,
        r*q.i3 - i*q.u3 + j*q.k3 - k*q.j3 + u1*q.i2 + i1*q.u2 + j1*q.k2 - k1*q.j2 - u2*q.i1 - i2*q.u1 + j2*q.k1 - k2*q.j1 + u3*q.i + i3*q.r + j3*q.k - k3*q.j,
        r*q.j3 - i*q.k3 - j*q.u3 + k*q.i3 + u1*q.j2 - i1*q.k2 + j1*q.u2 + k1*q.i2 - u2*q.j1 - i2*q.k1 - j2*q.u1 + k2*q.i1 + u3*q.j - i3*q.k + j3*q.r + k3*q.i,
        r*q.k3 + i*q.j3 - j*q.i3 - k*q.u3 + u1*q.k2 + i1*q.j2 - j1*q.i2 + k1*q.u2 - u2*q.k1 + i2*q.j1 - j2*q.i1 - k2*q.u1 + u3*q.k + i3*q.j - j3*q.i + k3*q.r
        );
}

template <typename T> Sedenion<T> Sedenion<T>::operator * (T x) const
{
    if (Support<T>::is_inf(x) && norm() > 0)
    {
        return Sedenion<T>(
            ((r == 0) ? Support<T>::sign(x) : x)*r,
            ((i == 0) ? Support<T>::sign(x) : x)*i,
            ((j == 0) ? Support<T>::sign(x) : x)*j,
            ((k == 0) ? Support<T>::sign(x) : x)*k,
            ((u1 == 0) ? Support<T>::sign(x) : x)*u1,
            ((i1 == 0) ? Support<T>::sign(x) : x)*i1,
            ((j1 == 0) ? Support<T>::sign(x) : x)*j1,
            ((k1 == 0) ? Support<T>::sign(x) : x)*k1,
            ((u2 == 0) ? Support<T>::sign(x) : x)*u2,
            ((i2 == 0) ? Support<T>::sign(x) : x)*i2,
            ((j2 == 0) ? Support<T>::sign(x) : x)*j2,
            ((k2 == 0) ? Support<T>::sign(x) : x)*k2,
            ((u3 == 0) ? Support<T>::sign(x) : x)*u3,
            ((i3 == 0) ? Support<T>::sign(x) : x)*i3,
            ((j3 == 0) ? Support<T>::sign(x) : x)*j3,
            ((k3 == 0) ? Support<T>::sign(x) : x)*k3
            );
    }
    else
    {
        return Sedenion<T>(x*r, x*i, x*j, x*k, x*u1, x*i1, x*j1, x*k1, x*u2, x*i2, x*j2, x*k2, x*u3, x*i3, x*j3, x*k3);
    }
}

template <typename T> Sedenion<T> operator * (T x, const Sedenion<T> & q)
{
    return q.operator * (x);
}

template <typename T> Sedenion<T> operator * (long x, const Sedenion<T> & q)
{
    return q.operator * (static_cast<T>(x));
}

template <typename T> Sedenion<T> Sedenion<T>::operator / (const Sedenion<T> & q) const
{
    T denom = q.norm();

    return Sedenion<T>(
        (r*q.r + i*q.i + j*q.j + k*q.k + u1*q.u1 + i1*q.i1 + j1*q.j1 + k1*q.k1 + u2*q.u2 + i2*q.i2 + j2*q.j2 + k2*q.k2 + u3*q.u3 + i3*q.i3 + j3*q.j3 + k3*q.k3) / denom,
        (-r*q.i + i*q.r - j*q.k + k*q.j - u1*q.i1 + i1*q.u1 + j1*q.k1 - k1*q.j1 - u2*q.i2 + i2*q.u2 + j2*q.k2 - k2*q.j2 + u3*q.i3 - i3*q.u3 - j3*q.k3 + k3*q.j3) / denom,
        (-r*q.j + i*q.k + j*q.r - k*q.i - u1*q.j1 - i1*q.k1 + j1*q.u1 + k1*q.i1 - u2*q.j2 - i2*q.k2 + j2*q.u2 + k2*q.i2 + u3*q.j3 + i3*q.k3 - j3*q.u3 - k3*q.i3) / denom,
        (-r*q.k - i*q.j + j*q.i + k*q.r - u1*q.k1 + i1*q.j1 - j1*q.i1 + k1*q.u1 - u2*q.k2 + i2*q.j2 - j2*q.i2 + k2*q.u2 + u3*q.k3 - i3*q.j3 + j3*q.i3 - k3*q.u3) / denom,
        (-r*q.u1 + i*q.i1 + j*q.j1 + k*q.k1 + u1*q.r - i1*q.i - j1*q.j - k1*q.k - u2*q.u3 - i2*q.i3 - j2*q.j3 - k2*q.k3 + u3*q.u2 + i3*q.i2 + j3*q.j2 + k3*q.k2) / denom,
        (-r*q.i1 - i*q.u1 + j*q.k1 - k*q.j1 + u1*q.i + i1*q.r + j1*q.k - k1*q.j - u2*q.i3 + i2*q.u3 - j2*q.k3 + k2*q.j3 - u3*q.i2 + i3*q.u2 - j3*q.k2 + k3*q.j2) / denom,
        (-r*q.j1 - i*q.k1 - j*q.u1 + k*q.i1 + u1*q.j - i1*q.k + j1*q.r + k1*q.i - u2*q.j3 + i2*q.k3 + j2*q.u3 - k2*q.i3 - u3*q.j2 + i3*q.k2 + j3*q.u2 - k3*q.i2) / denom,
        (-r*q.k1 + i*q.j1 - j*q.i1 - k*q.u1 + u1*q.k + i1*q.j - j1*q.i + k1*q.r - u2*q.k3 - i2*q.j3 + j2*q.i3 + k2*q.u3 - u3*q.k2 - i3*q.j2 + j3*q.i2 + k3*q.u2) / denom,
        (-r*q.u2 + i*q.i2 + j*q.j2 + k*q.k2 + u1*q.u3 + i1*q.i3 + j1*q.j3 + k1*q.k3 + u2*q.r - i2*q.i - j2*q.j - k2*q.k - u3*q.u1 - i3*q.i1 - j3*q.j1 - k3*q.k1) / denom,
        (-r*q.i2 - i*q.u2 + j*q.k2 - k*q.j2 + u1*q.i3 - i1*q.u3 - j1*q.k3 + k1*q.j3 + u2*q.i + i2*q.r + j2*q.k - k2*q.j + u3*q.i1 - i3*q.u1 - j3*q.k1 + k3*q.j1) / denom,
        (-r*q.j2 - i*q.k2 - j*q.u2 + k*q.i2 + u1*q.j3 + i1*q.k3 - j1*q.u3 - k1*q.i3 + u2*q.j - i2*q.k + j2*q.r + k2*q.i + u3*q.j1 + i3*q.k1 - j3*q.u1 - k3*q.i1) / denom,
        (-r*q.k2 + i*q.j2 - j*q.i2 - k*q.u2 + u1*q.k3 - i1*q.j3 + j1*q.i3 - k1*q.u3 + u2*q.k + i2*q.j - j2*q.i + k2*q.r + u3*q.k1 - i3*q.j1 + j3*q.i1 - k3*q.u1) / denom,
        (-r*q.u3 - i*q.i3 - j*q.j3 - k*q.k3 - u1*q.u2 + i1*q.i2 + j1*q.j2 + k1*q.k2 + u2*q.u1 - i2*q.i1 - j2*q.j1 - k2*q.k1 + u3*q.r + i3*q.i + j3*q.j + k3*q.k) / denom,
        (-r*q.i3 + i*q.u3 - j*q.k3 + k*q.j3 - u1*q.i2 - i1*q.u2 - j1*q.k2 + k1*q.j2 + u2*q.i1 + i2*q.u1 - j2*q.k1 + k2*q.j1 - u3*q.i + i3*q.r - j3*q.k + k3*q.j) / denom,
        (-r*q.j3 + i*q.k3 + j*q.u3 - k*q.i3 - u1*q.j2 + i1*q.k2 - j1*q.u2 - k1*q.i2 + u2*q.j1 + i2*q.k1 + j2*q.u1 - k2*q.i1 - u3*q.j + i3*q.k + j3*q.r - k3*q.i) / denom,
        (-r*q.k3 - i*q.j3 + j*q.i3 + k*q.u3 - u1*q.k2 - i1*q.j2 + j1*q.i2 - k1*q.u2 + u2*q.k1 - i2*q.j1 + j2*q.i1 + k2*q.u1 - u3*q.k - i3*q.j + j3*q.i + k3*q.r) / denom
        );
}

template <typename T> Sedenion<T> Sedenion<T>::operator / (T x) const
{
    if (x == 0.0 && !is_zero())
    {
        return Sedenion<T>(
            r / ((r == 0) ? Support<T>::sign(x) : x),
            i / ((i == 0) ? Support<T>::sign(x) : x),
            j / ((j == 0) ? Support<T>::sign(x) : x),
            k / ((k == 0) ? Support<T>::sign(x) : x),
            u1 / ((u1 == 0) ? Support<T>::sign(x) : x),
            i1 / ((i1 == 0) ? Support<T>::sign(x) : x),
            j1 / ((j1 == 0) ? Support<T>::sign(x) : x),
            k1 / ((k1 == 0) ? Support<T>::sign(x) : x),
            u2 / ((u2 == 0) ? Support<T>::sign(x) : x),
            i2 / ((i2 == 0) ? Support<T>::sign(x) : x),
            j2 / ((j2 == 0) ? Support<T>::sign(x) : x),
            k2 / ((k2 == 0) ? Support<T>::sign(x) : x),
            u3 / ((u3 == 0) ? Support<T>::sign(x) : x),
            i3 / ((i3 == 0) ? Support<T>::sign(x) : x),
            j3 / ((j3 == 0) ? Support<T>::sign(x) : x),
            k3 / ((k3 == 0) ? Support<T>::sign(x) : x)
            );
    }
    else
    {
        return Sedenion<T>(r / x, i / x, j / x, k / x, u1 / x, i1 / x, j1 / x, k1 / x, u2 / x, i2 / x, j2 / x, k2 / x, u3 / x, i3 / x, j3 / x, k3 / x);
    }
}

template <typename T> Sedenion<T> operator / (T x, const Sedenion<T> & q)
{
    T mltplr = x / q.norm();

    return Sedenion<T>(
        mltplr*q.real(),
        -mltplr*q.imag_i(), -mltplr*q.imag_j(), -mltplr*q.imag_k(),
        -mltplr*q.imag_u1(), -mltplr*q.imag_i1(), -mltplr*q.imag_j1(), -mltplr*q.imag_k1(),
        -mltplr*q.imag_u2(), -mltplr*q.imag_i2(), -mltplr*q.imag_j2(), -mltplr*q.imag_k2(),
        -mltplr*q.imag_u3(), -mltplr*q.imag_i3(), -mltplr*q.imag_j3(), -mltplr*q.imag_k3()
        );
}

template <typename T> Sedenion<T> operator / (long x, const Sedenion<T> & q)
{
    return operator / (static_cast<T>(x), q);
}

/******************************************
 * Unary Arithmetic Operators
 ******************************************/

template <typename T> Sedenion<T> Sedenion<T>::operator - () const
{
    return Sedenion<T>(-r, -i, -j, -k, -u1, -i1, -j1, -k1, -u2, -i2, -j2, -k2, -u3, -i3, -j3, -k3);
}

/******************************************
 * Binary Boolean Operators
 ******************************************/

template <typename T> bool Sedenion<T>::operator == (const Sedenion<T> & q) const
{
    return
        (r == q.r) && (i == q.i) && (j == q.j) && (k == q.k) &&
        (u1 == q.u1) && (i1 == q.i1) && (j1 == q.j1) && (k1 == q.k1) &&
        (u2 == q.u2) && (i2 == q.i2) && (j2 == q.j2) && (k2 == q.k2) &&
        (u3 == q.u3) && (i3 == q.i3) && (j3 == q.j3) && (k3 == q.k3);
}

template <typename T> bool Sedenion<T>::operator == (T x) const
{
    return
        (r == x) && (i == 0.0) && (j == 0.0) && (k == 0.0) &&
        (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0) &&
        (u2 == 0.0) && (i2 == 0.0) && (j2 == 0.0) && (k2 == 0.0) &&
        (u3 == 0.0) && (i3 == 0.0) && (j3 == 0.0) && (k3 == 0.0);
}

template <typename T> bool operator == (T x, const Sedenion<T> & q)
{
    return q.operator == (x);
}

template <typename T> bool operator == (long x, const Sedenion<T> & q)
{
    return q.operator == (static_cast<T>(x));
}

template <typename T> bool Sedenion<T>::operator != (const Sedenion<T> & q) const
{
    return
        (r != q.r) || (i != q.i) || (j != q.j) || (k != q.k) ||
        (u1 != q.u1) && (i1 != q.i1) && (j1 != q.j1) && (k1 != q.k1) ||
        (u2 != q.u2) && (i2 != q.i2) && (j2 != q.j2) && (k2 != q.k2) ||
        (u3 != q.u3) && (i3 != q.i3) && (j3 != q.j3) && (k3 != q.k3);
}

template <typename T> bool Sedenion<T>::operator != (T x) const
{
    return
        (r != x) || (i != 0.0) || (j != 0.0) || (k != 0.0) ||
        (u1 != 0.0) && (i1 != 0.0) && (j1 != 0.0) && (k1 != 0.0) ||
        (u2 != 0.0) && (i2 != 0.0) && (j2 != 0.0) && (k2 != 0.0) ||
        (u3 != 0.0) && (i3 != 0.0) && (j3 != 0.0) && (k3 != 0.0);
}

template <typename T> bool operator != (T x, const Sedenion<T> & q)
{
    return q.operator != (x);
}

template <typename T> bool operator != (long x, const Sedenion<T> & q)
{
    return q.operator != (static_cast<T>(x));
}

/******************************************
 * IO Stream Operators
 ******************************************/

template <typename T> std::ostream & operator << (std::ostream & out, const Sedenion<T> & z)
{
    Support<T>::print_real(z.real(), out);
    Support<T>::print_imaginary(z.imag_i(), 'i', out);
    Support<T>::print_imaginary(z.imag_j(), 'j', out);
    Support<T>::print_imaginary(z.imag_k(), 'k', out);
    Support<T>::print_imaginary(z.imag_u1(), "u1", out);
    Support<T>::print_imaginary(z.imag_i1(), "i1", out);
    Support<T>::print_imaginary(z.imag_j1(), "j1", out);
    Support<T>::print_imaginary(z.imag_k1(), "k1", out);
    Support<T>::print_imaginary(z.imag_u2(), "u2", out);
    Support<T>::print_imaginary(z.imag_i2(), "i2", out);
    Support<T>::print_imaginary(z.imag_j2(), "j2", out);
    Support<T>::print_imaginary(z.imag_k2(), "k2", out);
    Support<T>::print_imaginary(z.imag_u3(), "u3", out);
    Support<T>::print_imaginary(z.imag_i3(), "i3", out);
    Support<T>::print_imaginary(z.imag_j3(), "j3", out);
    Support<T>::print_imaginary(z.imag_k3(), "k3", out);

    return out;
}

/******************************************
 * ************************************** *
 * *                                    * *
 * *      Procedural Functions          * *
 * *                                    * *
 * ************************************** *
 ******************************************/

 /******************************************
  * Real-valued Functions
  ******************************************/

template <typename T> T real(const Sedenion<T> & q)
{
    return q.real();
}

template <typename T> T imag_i(const Sedenion<T> & q)
{
    return q.imag_i();
}

template <typename T> T imag_j(const Sedenion<T> & q)
{
    return q.imag_j();
}

template <typename T> T imag_k(const Sedenion<T> & q)
{
    return q.imag_k();
}

template <typename T> T imag_u1(const Sedenion<T> & q)
{
    return q.imag_u1();
}

template <typename T> T imag_i1(const Sedenion<T> & q)
{
    return q.imag_i1();
}

template <typename T> T imag_j1(const Sedenion<T> & q)
{
    return q.imag_j1();
}

template <typename T> T imag_k1(const Sedenion<T> & q)
{
    return q.imag_k1();
}

template <typename T> T imag_u2(const Sedenion<T> & q)
{
    return q.imag_u2();
}

template <typename T> T imag_i2(const Sedenion<T> & q)
{
    return q.imag_i2();
}

template <typename T> T imag_j2(const Sedenion<T> & q)
{
    return q.imag_j2();
}

template <typename T> T imag_k2(const Sedenion<T> & q)
{
    return q.imag_k2();
}

template <typename T> T imag_u3(const Sedenion<T> & q)
{
    return q.imag_u3();
}

template <typename T> T imag_i3(const Sedenion<T> & q)
{
    return q.imag_i3();
}

template <typename T> T imag_j3(const Sedenion<T> & q)
{
    return q.imag_j3();
}

template <typename T> T imag_k3(const Sedenion<T> & q)
{
    return q.imag_k3();
}

template <typename T> T csgn(const Sedenion<T> & q)
{
    return q.csgn();
}

template <typename T> T abs(const Sedenion<T> & q)
{
    return q.abs();
}

template <typename T> T norm(const Sedenion<T> & q)
{
    return q.norm();
}

template <typename T> T abs_imag(const Sedenion<T> & z)
{
    return z.abs_imag();
}

template <typename T> T norm_imag(const Sedenion<T> & z)
{
    return z.norm_imag();
}

/******************************************
 * Sedenion-valued Functions
 ******************************************/

template <typename T> Sedenion<T> imag(const Sedenion<T> & q)
{
    return q.imag();
}

template <typename T> Sedenion<T> conj(const Sedenion<T> & q)
{
    return q.conj();
}

template <typename T> Sedenion<T> signum(const Sedenion<T> & q)
{
    return q.signum();
}

template <typename T> Sedenion<T> sqr(const Sedenion<T> & q)
{
    return q.sqr();
}

template <typename T> Sedenion<T> sqrt(const Sedenion<T> & q)
{
    return q.sqrt();
}

template <typename T> Sedenion<T> rotate(const Sedenion<T> & q, const Sedenion<T> & p)
{
    return q.rotate(p);
}

/******************************************
 * Exponential and Logarithmic Functions
 ******************************************/

template <typename T> Sedenion<T> exp(const Sedenion<T> & q)
{
    return q.exp();
}

template <typename T> Sedenion<T> log(const Sedenion<T> & q)
{
    return q.log();
}

template <typename T> Sedenion<T> log10(const Sedenion<T> & q)
{
    return q.log10();
}

template <typename T> Sedenion<T> pow(const Sedenion<T> & q, const Sedenion<T> & w)
{
    return q.pow(w);
}

template <typename T> Sedenion<T> pow(const Sedenion<T> & q, T x)
{
    return q.pow(x);
}

template <typename T> Sedenion<T> inverse(const Sedenion<T> & q)
{
    return q.inverse();
}

/******************************************
 * Trigonometric and Hyperbolic Functions
 ******************************************/

template <typename T> Sedenion<T> sin(const Sedenion<T> & q)
{
    return q.sin();
}

template <typename T> Sedenion<T> cos(const Sedenion<T> & q)
{
    return q.cos();
}

template <typename T> Sedenion<T> tan(const Sedenion<T> & q)
{
    return q.tan();
}

template <typename T> Sedenion<T> sec(const Sedenion<T> & q)
{
    return q.sec();
}

template <typename T> Sedenion<T> csc(const Sedenion<T> & q)
{
    return q.csc();
}

template <typename T> Sedenion<T> cot(const Sedenion<T> & q)
{
    return q.cot();
}

template <typename T> Sedenion<T> sinh(const Sedenion<T> & q)
{
    return q.sinh();
}

template <typename T> Sedenion<T> cosh(const Sedenion<T> & q)
{
    return q.cosh();
}

template <typename T> Sedenion<T> tanh(const Sedenion<T> & q)
{
    return q.tanh();
}

template <typename T> Sedenion<T> sech(const Sedenion<T> & q)
{
    return q.sech();
}

template <typename T> Sedenion<T> csch(const Sedenion<T> & q)
{
    return q.csch();
}

template <typename T> Sedenion<T> coth(const Sedenion<T> & q)
{
    return q.coth();
}

template <typename T> Sedenion<T> asin(const Sedenion<T> & q, const Sedenion<T> & p)
{
    return q.asin(p);
}

template <typename T> Sedenion<T> acos(const Sedenion<T> & q, const Sedenion<T> & p)
{
    return q.acos(p);
}

template <typename T> Sedenion<T> atan(const Sedenion<T> & q)
{
    return q.atan();
}

template <typename T> Sedenion<T> asec(const Sedenion<T> & q, const Sedenion<T> & p)
{
    return q.asec(p);
}

template <typename T> Sedenion<T> acsc(const Sedenion<T> & q, const Sedenion<T> & p)
{
    return p.acsc(p);
}

template <typename T> Sedenion<T> acot(const Sedenion<T> & q)
{
    return q.acot();
}

template <typename T> Sedenion<T> asinh(const Sedenion<T> & q)
{
    return q.asinh();
}

template <typename T> Sedenion<T> acosh(const Sedenion<T> & q, const Sedenion<T> & p)
{
    return q.acosh(p);
}

template <typename T> Sedenion<T> atanh(const Sedenion<T> & q, const Sedenion<T> & p)
{
    return q.atanh(p);
}

template <typename T> Sedenion<T> asech(const Sedenion<T> & q, const Sedenion<T> & p)
{
    return q.asech(p);
}

template <typename T> Sedenion<T> acsch(const Sedenion<T> & q)
{
    return q.acsch();
}

template <typename T> Sedenion<T> acoth(const Sedenion<T> & q, const Sedenion<T> & p)
{
    return q.acoth(p);
}

template <typename T> Sedenion<T> bessel_J(int n, const Sedenion<T> & z)
{
    return z.bessel_J(n);
}

/******************************************
 * Integer Functions
 ******************************************/

template <typename T> Sedenion<T> floor(const Sedenion<T> & q)
{
    return q.floor();
}

template <typename T> Sedenion<T> ceil(const Sedenion<T> & q)
{
    return q.ceil();
}

/******************************************
 * Horner's Rule
 ******************************************/

template <typename T> Sedenion<T> horner(const Sedenion<T> & q, T * v, unsigned int n)
{
    return q.horner(v, n);
}

template <typename T> Sedenion<T> horner(const Sedenion<T> & q, T * v, T * c, unsigned int n)
{
    return q.horner(v, c, n);
}

/**************************************************
 * ********************************************** *
 * *                                            * *
 * *    Double-precision Floating-point         * *
 * *    Instance of Template                    * *
 * *                                            * *
 * ********************************************** *
 **************************************************/

template class Sedenion<double>;

template std::ostream & operator << (std::ostream &, const Sedenion<double> &);

template Sedenion<double> operator + (double, const Sedenion<double> &);
template Sedenion<double> operator + (long, const Sedenion<double> &);
template Sedenion<double> operator - (double, const Sedenion<double> &);
template Sedenion<double> operator - (long, const Sedenion<double> &);
template Sedenion<double> operator * (double, const Sedenion<double> &);
template Sedenion<double> operator * (long, const Sedenion<double> &);
template Sedenion<double> operator / (double, const Sedenion<double> &);
template Sedenion<double> operator / (long, const Sedenion<double> &);

template bool operator == (double, const Sedenion<double> &);
template bool operator == (long, const Sedenion<double> &);
template bool operator != (double, const Sedenion<double> &);
template bool operator != (long, const Sedenion<double> &);

template <> const Sedenion<double> Sedenion<double>::ZERO = Sedenion<double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<double>  Sedenion<double>::ONE = Sedenion<double>(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<double>    Sedenion<double>::J = Sedenion<double>(0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<double>    Sedenion<double>::K = Sedenion<double>(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<double>   Sedenion<double>::U1 = Sedenion<double>(0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<double>   Sedenion<double>::I1 = Sedenion<double>(0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<double>   Sedenion<double>::J1 = Sedenion<double>(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<double>   Sedenion<double>::K1 = Sedenion<double>(0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<double>   Sedenion<double>::U2 = Sedenion<double>(0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<double>   Sedenion<double>::I2 = Sedenion<double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<double>   Sedenion<double>::J2 = Sedenion<double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
template <> const Sedenion<double>   Sedenion<double>::K2 = Sedenion<double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0);
template <> const Sedenion<double>   Sedenion<double>::U3 = Sedenion<double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0);
template <> const Sedenion<double>   Sedenion<double>::I3 = Sedenion<double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0);
template <> const Sedenion<double>   Sedenion<double>::J3 = Sedenion<double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0);
template <> const Sedenion<double>   Sedenion<double>::K3 = Sedenion<double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);

template <> const Sedenion<double> Sedenion<double>::UNITS[16] = {
    Sedenion<double>::ONE,
    Sedenion<double>::I,
    Sedenion<double>::J,
    Sedenion<double>::K,
    Sedenion<double>::U1,
    Sedenion<double>::I1,
    Sedenion<double>::J1,
    Sedenion<double>::K1,
    Sedenion<double>::U2,
    Sedenion<double>::I2,
    Sedenion<double>::J2,
    Sedenion<double>::K2,
    Sedenion<double>::U3,
    Sedenion<double>::I3,
    Sedenion<double>::J3,
    Sedenion<double>::K3
};

template double real(const Sedenion<double> &);
template double imag_i(const Sedenion<double> &);
template double imag_j(const Sedenion<double> &);
template double imag_k(const Sedenion<double> &);
template double imag_u1(const Sedenion<double> &);
template double imag_i1(const Sedenion<double> &);
template double imag_j1(const Sedenion<double> &);
template double imag_k1(const Sedenion<double> &);
template double imag_u2(const Sedenion<double> &);
template double imag_i2(const Sedenion<double> &);
template double imag_j2(const Sedenion<double> &);
template double imag_k2(const Sedenion<double> &);
template double imag_u3(const Sedenion<double> &);
template double imag_i3(const Sedenion<double> &);
template double imag_j3(const Sedenion<double> &);
template double imag_k3(const Sedenion<double> &);
template double csgn(const Sedenion<double> &);
template double abs(const Sedenion<double> &);
template double norm(const Sedenion<double> &);
template double abs_imag(const Sedenion<double> &);
template double norm_imag(const Sedenion<double> &);
template Sedenion<double> imag(const Sedenion<double> &);
template Sedenion<double> conj(const Sedenion<double> &);
template Sedenion<double> signum(const Sedenion<double> &);
template Sedenion<double> sqr(const Sedenion<double> &);
template Sedenion<double> sqrt(const Sedenion<double> &);
template Sedenion<double> rotate(const Sedenion<double> &, const Sedenion<double> &);
template Sedenion<double> exp(const Sedenion<double> &);
template Sedenion<double> log(const Sedenion<double> &);
template Sedenion<double> log10(const Sedenion<double> &);
template Sedenion<double> pow(const Sedenion<double> &, const Sedenion<double> &);
template Sedenion<double> pow(const Sedenion<double> &, double);
template Sedenion<double> inverse(const Sedenion<double> &);
template Sedenion<double> sin(const Sedenion<double> &);
template Sedenion<double> cos(const Sedenion<double> &);
template Sedenion<double> tan(const Sedenion<double> &);
template Sedenion<double> sec(const Sedenion<double> &);
template Sedenion<double> csc(const Sedenion<double> &);
template Sedenion<double> cot(const Sedenion<double> &);
template Sedenion<double> sinh(const Sedenion<double> &);
template Sedenion<double> cosh(const Sedenion<double> &);
template Sedenion<double> tanh(const Sedenion<double> &);
template Sedenion<double> sech(const Sedenion<double> &);
template Sedenion<double> csch(const Sedenion<double> &);
template Sedenion<double> coth(const Sedenion<double> &);
template Sedenion<double> asin(const Sedenion<double> &, const Sedenion<double> &);
template Sedenion<double> acos(const Sedenion<double> &, const Sedenion<double> &);
template Sedenion<double> atan(const Sedenion<double> &);
template Sedenion<double> asec(const Sedenion<double> &, const Sedenion<double> &);
template Sedenion<double> acsc(const Sedenion<double> &, const Sedenion<double> &);
template Sedenion<double> acot(const Sedenion<double> &);
template Sedenion<double> asinh(const Sedenion<double> &);
template Sedenion<double> acosh(const Sedenion<double> &, const Sedenion<double> &);
template Sedenion<double> atanh(const Sedenion<double> &, const Sedenion<double> &);
template Sedenion<double> asech(const Sedenion<double> &, const Sedenion<double> &);
template Sedenion<double> acsch(const Sedenion<double> &);
template Sedenion<double> acoth(const Sedenion<double> &, const Sedenion<double> &);
template Sedenion<double> bessel_J(int, const Sedenion<double> &);
template Sedenion<double> floor(const Sedenion<double> &);
template Sedenion<double> ceil(const Sedenion<double> &);
template Sedenion<double> horner(const Sedenion<double> &, double *, unsigned int);
template Sedenion<double> horner(const Sedenion<double> &, double *, double *, unsigned int);

/**************************************************
 * ********************************************** *
 * *                                            * *
 * *    Floating-point Instance of Template     * *
 * *                                            * *
 * ********************************************** *
 **************************************************/

template class Sedenion<float>;
template std::ostream & operator << (std::ostream &, const Sedenion<float> &);

template Sedenion<float> operator + (float, const Sedenion<float> &);
template Sedenion<float> operator + (long, const Sedenion<float> &);
template Sedenion<float> operator - (float, const Sedenion<float> &);
template Sedenion<float> operator - (long, const Sedenion<float> &);
template Sedenion<float> operator * (float, const Sedenion<float> &);
template Sedenion<float> operator * (long, const Sedenion<float> &);
template Sedenion<float> operator / (float, const Sedenion<float> &);
template Sedenion<float> operator / (long, const Sedenion<float> &);

template bool operator == (float, const Sedenion<float> &);
template bool operator == (long, const Sedenion<float> &);
template bool operator != (float, const Sedenion<float> &);
template bool operator != (long, const Sedenion<float> &);

template <> const Sedenion<float> Sedenion<float>::ZERO = Sedenion<float>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<float>  Sedenion<float>::ONE = Sedenion<float>(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<float>    Sedenion<float>::J = Sedenion<float>(0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<float>    Sedenion<float>::K = Sedenion<float>(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<float>   Sedenion<float>::U1 = Sedenion<float>(0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<float>   Sedenion<float>::I1 = Sedenion<float>(0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<float>   Sedenion<float>::J1 = Sedenion<float>(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<float>   Sedenion<float>::K1 = Sedenion<float>(0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<float>   Sedenion<float>::U2 = Sedenion<float>(0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<float>   Sedenion<float>::I2 = Sedenion<float>(0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<float>   Sedenion<float>::J2 = Sedenion<float>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
template <> const Sedenion<float>   Sedenion<float>::K2 = Sedenion<float>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0);
template <> const Sedenion<float>   Sedenion<float>::U3 = Sedenion<float>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0);
template <> const Sedenion<float>   Sedenion<float>::I3 = Sedenion<float>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0);
template <> const Sedenion<float>   Sedenion<float>::J3 = Sedenion<float>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0);
template <> const Sedenion<float>   Sedenion<float>::K3 = Sedenion<float>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);

template <> const Sedenion<float> Sedenion<float>::UNITS[16] = {
    Sedenion<float>::ONE,
    Sedenion<float>::I,
    Sedenion<float>::J,
    Sedenion<float>::K,
    Sedenion<float>::U1,
    Sedenion<float>::I1,
    Sedenion<float>::J1,
    Sedenion<float>::K1,
    Sedenion<float>::U2,
    Sedenion<float>::I2,
    Sedenion<float>::J2,
    Sedenion<float>::K2,
    Sedenion<float>::U3,
    Sedenion<float>::I3,
    Sedenion<float>::J3,
    Sedenion<float>::K3
};

template float real(const Sedenion<float> &);
template float imag_i(const Sedenion<float> &);
template float imag_j(const Sedenion<float> &);
template float imag_k(const Sedenion<float> &);
template float imag_u1(const Sedenion<float> &);
template float imag_i1(const Sedenion<float> &);
template float imag_j1(const Sedenion<float> &);
template float imag_k1(const Sedenion<float> &);
template float imag_u2(const Sedenion<float> &);
template float imag_i2(const Sedenion<float> &);
template float imag_j2(const Sedenion<float> &);
template float imag_k2(const Sedenion<float> &);
template float imag_u3(const Sedenion<float> &);
template float imag_i3(const Sedenion<float> &);
template float imag_j3(const Sedenion<float> &);
template float imag_k3(const Sedenion<float> &);
template float csgn(const Sedenion<float> &);
template float abs(const Sedenion<float> &);
template float norm(const Sedenion<float> &);
template float abs_imag(const Sedenion<float> &);
template float norm_imag(const Sedenion<float> &);
template Sedenion<float> imag(const Sedenion<float> &);
template Sedenion<float> conj(const Sedenion<float> &);
template Sedenion<float> signum(const Sedenion<float> &);
template Sedenion<float> sqr(const Sedenion<float> &);
template Sedenion<float> sqrt(const Sedenion<float> &);
template Sedenion<float> rotate(const Sedenion<float> &, const Sedenion<float> &);
template Sedenion<float> exp(const Sedenion<float> &);
template Sedenion<float> log(const Sedenion<float> &);
template Sedenion<float> log10(const Sedenion<float> &);
template Sedenion<float> pow(const Sedenion<float> &, const Sedenion<float> &);
template Sedenion<float> pow(const Sedenion<float> &, float);
template Sedenion<float> inverse(const Sedenion<float> &);
template Sedenion<float> sin(const Sedenion<float> &);
template Sedenion<float> cos(const Sedenion<float> &);
template Sedenion<float> tan(const Sedenion<float> &);
template Sedenion<float> sec(const Sedenion<float> &);
template Sedenion<float> csc(const Sedenion<float> &);
template Sedenion<float> cot(const Sedenion<float> &);
template Sedenion<float> sinh(const Sedenion<float> &);
template Sedenion<float> cosh(const Sedenion<float> &);
template Sedenion<float> tanh(const Sedenion<float> &);
template Sedenion<float> sech(const Sedenion<float> &);
template Sedenion<float> csch(const Sedenion<float> &);
template Sedenion<float> coth(const Sedenion<float> &);
template Sedenion<float> asin(const Sedenion<float> &, const Sedenion<float> &);
template Sedenion<float> acos(const Sedenion<float> &, const Sedenion<float> &);
template Sedenion<float> atan(const Sedenion<float> &);
template Sedenion<float> asec(const Sedenion<float> &, const Sedenion<float> &);
template Sedenion<float> acsc(const Sedenion<float> &, const Sedenion<float> &);
template Sedenion<float> acot(const Sedenion<float> &);
template Sedenion<float> asinh(const Sedenion<float> &);
template Sedenion<float> acosh(const Sedenion<float> &, const Sedenion<float> &);
template Sedenion<float> atanh(const Sedenion<float> &, const Sedenion<float> &);
template Sedenion<float> asech(const Sedenion<float> &, const Sedenion<float> &);
template Sedenion<float> acsch(const Sedenion<float> &);
template Sedenion<float> acoth(const Sedenion<float> &, const Sedenion<float> &);
template Sedenion<float> bessel_J(int, const Sedenion<float> &);
template Sedenion<float> floor(const Sedenion<float> &);
template Sedenion<float> ceil(const Sedenion<float> &);
template Sedenion<float> horner(const Sedenion<float> &, float *, unsigned int);
template Sedenion<float> horner(const Sedenion<float> &, float *, float *, unsigned int);

/************************************************************************
 * ******************************************************************** *
 * *                                                                  * *
 * *    Long Double-precision Floating-point Instance of Template     * *
 * *                                                                  * *
 * ******************************************************************** *
 ************************************************************************/

template class Sedenion<long double>;
template std::ostream & operator << (std::ostream &, const Sedenion<long double> &);

template Sedenion<long double> operator + (long double, const Sedenion<long double> &);
template Sedenion<long double> operator + (long, const Sedenion<long double> &);
template Sedenion<long double> operator - (long double, const Sedenion<long double> &);
template Sedenion<long double> operator - (long, const Sedenion<long double> &);
template Sedenion<long double> operator * (long double, const Sedenion<long double> &);
template Sedenion<long double> operator * (long, const Sedenion<long double> &);
template Sedenion<long double> operator / (long double, const Sedenion<long double> &);
template Sedenion<long double> operator / (long, const Sedenion<long double> &);

template bool operator == (long double, const Sedenion<long double> &);
template bool operator == (long, const Sedenion<long double> &);
template bool operator != (long double, const Sedenion<long double> &);
template bool operator != (long, const Sedenion<long double> &);

template <> const Sedenion<long double> Sedenion<long double>::ZERO = Sedenion<long double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>  Sedenion<long double>::ONE = Sedenion<long double>(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>    Sedenion<long double>::J = Sedenion<long double>(0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>    Sedenion<long double>::K = Sedenion<long double>(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>   Sedenion<long double>::U1 = Sedenion<long double>(0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>   Sedenion<long double>::I1 = Sedenion<long double>(0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>   Sedenion<long double>::J1 = Sedenion<long double>(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>   Sedenion<long double>::K1 = Sedenion<long double>(0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>   Sedenion<long double>::U2 = Sedenion<long double>(0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>   Sedenion<long double>::I2 = Sedenion<long double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>   Sedenion<long double>::J2 = Sedenion<long double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0);
template <> const Sedenion<long double>   Sedenion<long double>::K2 = Sedenion<long double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0);
template <> const Sedenion<long double>   Sedenion<long double>::U3 = Sedenion<long double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0);
template <> const Sedenion<long double>   Sedenion<long double>::I3 = Sedenion<long double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0);
template <> const Sedenion<long double>   Sedenion<long double>::J3 = Sedenion<long double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0);
template <> const Sedenion<long double>   Sedenion<long double>::K3 = Sedenion<long double>(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);

template <> const Sedenion<long double> Sedenion<long double>::UNITS[16] = {
    Sedenion<long double>::ONE,
    Sedenion<long double>::I,
    Sedenion<long double>::J,
    Sedenion<long double>::K,
    Sedenion<long double>::U1,
    Sedenion<long double>::I1,
    Sedenion<long double>::J1,
    Sedenion<long double>::K1,
    Sedenion<long double>::U2,
    Sedenion<long double>::I2,
    Sedenion<long double>::J2,
    Sedenion<long double>::K2,
    Sedenion<long double>::U3,
    Sedenion<long double>::I3,
    Sedenion<long double>::J3,
    Sedenion<long double>::K3
};

template long double real(const Sedenion<long double> &);
template long double imag_i(const Sedenion<long double> &);
template long double imag_j(const Sedenion<long double> &);
template long double imag_k(const Sedenion<long double> &);
template long double imag_u1(const Sedenion<long double> &);
template long double imag_i1(const Sedenion<long double> &);
template long double imag_j1(const Sedenion<long double> &);
template long double imag_k1(const Sedenion<long double> &);
template long double imag_u2(const Sedenion<long double> &);
template long double imag_i2(const Sedenion<long double> &);
template long double imag_j2(const Sedenion<long double> &);
template long double imag_k2(const Sedenion<long double> &);
template long double imag_u3(const Sedenion<long double> &);
template long double imag_i3(const Sedenion<long double> &);
template long double imag_j3(const Sedenion<long double> &);
template long double imag_k3(const Sedenion<long double> &);
template long double csgn(const Sedenion<long double> &);
template long double abs(const Sedenion<long double> &);
template long double norm(const Sedenion<long double> &);
template long double abs_imag(const Sedenion<long double> &);
template long double norm_imag(const Sedenion<long double> &);
template Sedenion<long double> imag(const Sedenion<long double> &);
template Sedenion<long double> conj(const Sedenion<long double> &);
template Sedenion<long double> signum(const Sedenion<long double> &);
template Sedenion<long double> sqr(const Sedenion<long double> &);
template Sedenion<long double> sqrt(const Sedenion<long double> &);
template Sedenion<long double> rotate(const Sedenion<long double> &, const Sedenion<long double> &);
template Sedenion<long double> exp(const Sedenion<long double> &);
template Sedenion<long double> log(const Sedenion<long double> &);
template Sedenion<long double> log10(const Sedenion<long double> &);
template Sedenion<long double> pow(const Sedenion<long double> &, const Sedenion<long double> &);
template Sedenion<long double> pow(const Sedenion<long double> &, long double);
template Sedenion<long double> inverse(const Sedenion<long double> &);
template Sedenion<long double> sin(const Sedenion<long double> &);
template Sedenion<long double> cos(const Sedenion<long double> &);
template Sedenion<long double> tan(const Sedenion<long double> &);
template Sedenion<long double> sec(const Sedenion<long double> &);
template Sedenion<long double> csc(const Sedenion<long double> &);
template Sedenion<long double> cot(const Sedenion<long double> &);
template Sedenion<long double> sinh(const Sedenion<long double> &);
template Sedenion<long double> cosh(const Sedenion<long double> &);
template Sedenion<long double> tanh(const Sedenion<long double> &);
template Sedenion<long double> sech(const Sedenion<long double> &);
template Sedenion<long double> csch(const Sedenion<long double> &);
template Sedenion<long double> coth(const Sedenion<long double> &);
template Sedenion<long double> asin(const Sedenion<long double> &, const Sedenion<long double> &);
template Sedenion<long double> acos(const Sedenion<long double> &, const Sedenion<long double> &);
template Sedenion<long double> atan(const Sedenion<long double> &);
template Sedenion<long double> asec(const Sedenion<long double> &, const Sedenion<long double> &);
template Sedenion<long double> acsc(const Sedenion<long double> &, const Sedenion<long double> &);
template Sedenion<long double> acot(const Sedenion<long double> &);
template Sedenion<long double> asinh(const Sedenion<long double> &);
template Sedenion<long double> acosh(const Sedenion<long double> &, const Sedenion<long double> &);
template Sedenion<long double> atanh(const Sedenion<long double> &, const Sedenion<long double> &);
template Sedenion<long double> asech(const Sedenion<long double> &, const Sedenion<long double> &);
template Sedenion<long double> acsch(const Sedenion<long double> &);
template Sedenion<long double> acoth(const Sedenion<long double> &, const Sedenion<long double> &);
template Sedenion<long double> bessel_J(int, const Sedenion<long double> &);
template Sedenion<long double> floor(const Sedenion<long double> &);
template Sedenion<long double> ceil(const Sedenion<long double> &);
template Sedenion<long double> horner(const Sedenion<long double> &, long double *, unsigned int);
template Sedenion<long double> horner(const Sedenion<long double> &, long double *, long double *, unsigned int);
