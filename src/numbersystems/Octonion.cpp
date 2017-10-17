/******************************************
 * C++ Octonions
 * Version: 1.0.9
 * Author:  Douglas Wilhelm Harder
 * Date:    2008/03/03
 *
 * Copyright (c) 2006-2008 by Douglas Wilhelm Harder.
 * All rights reserved.
 ******************************************/

#include "Complex.h"
#include "Octonion.h"
#include "Support.h"
#include <iostream>
#include <cmath>
#include <string>

// template <> const Octonion<double>    Octonion<double>::I = Octonion<double>(0, 1, 0, 0, 0, 0, 0, 0);
// template <> const Octonion<float>    Octonion<float>::I = Octonion<float>(0, 1, 0, 0, 0, 0, 0, 0);
// template <> const Octonion<long double>    Octonion<long double>::I = Octonion<long double>(0, 1, 0, 0, 0, 0, 0, 0);

 /******************************************
  * Constructors
  ******************************************/

template <typename T> Octonion<T>::Octonion(T real, T imagi, T imagj, T imagk, T imagu, T imagie, T imagje, T imagke) :
    r(real), i(imagi), j(imagj), k(imagk),
    u1(imagu), i1(imagie), j1(imagje), k1(imagke)
{
}

template <typename T> Octonion<T>::Octonion(T real) :
    r(real), i(0.0), j(0.0), k(0.0),
    u1(0.0), i1(0.0), j1(0.0), k1(0.0)
{
}

/******************************************
 * Assignment Operator
 ******************************************/

template <typename T> const Octonion<T> & Octonion<T>::operator = (const T & real)
{
    r = real;
    i = 0.0;
    j = 0.0;
    k = 0.0;
    u1 = 0.0;
    i1 = 0.0;
    j1 = 0.0;
    k1 = 0.0;

    return *this;
}

/******************************************
 * Mutating Arithmetic Operators
 ******************************************/

template <typename T> Octonion<T> & Octonion<T>::operator += (const Octonion<T> & q)
{
    r += q.r;
    i += q.i;
    j += q.j;
    k += q.k;
    u1 += q.u1;
    i1 += q.i1;
    j1 += q.j1;
    k1 += q.k1;

    return *this;
}

template <typename T> Octonion<T> & Octonion<T>::operator -= (const Octonion<T> & q)
{
    r -= q.r;
    i -= q.i;
    j -= q.j;
    k -= q.k;
    u1 -= q.u1;
    i1 -= q.i1;
    j1 -= q.j1;
    k1 -= q.k1;

    return *this;
}

template <typename T> Octonion<T> & Octonion<T>::operator *= (const Octonion<T> & q)
{
    T RE = r, I = i, J = j, K = k, E = u1, IE = i1, JE = j1;

    r = RE*q.r - I*q.i - J*q.j - K*q.k - E*q.u1 - IE*q.i1 - JE*q.j1 - k1*q.k1;
    i = RE*q.i + I*q.r + J*q.k - K*q.j + E*q.i1 - IE*q.u1 - JE*q.k1 + k1*q.j1;
    j = RE*q.j - I*q.k + J*q.r + K*q.i + E*q.j1 + IE*q.k1 - JE*q.u1 - k1*q.i1;
    k = RE*q.k + I*q.j - J*q.i + K*q.r + E*q.k1 - IE*q.j1 + JE*q.i1 - k1*q.u1;
    u1 = RE*q.u1 - I*q.i1 - J*q.j1 - K*q.k1 + E*q.r + IE*q.i + JE*q.j + k1*q.k;
    i1 = RE*q.i1 + I*q.u1 - J*q.k1 + K*q.j1 - E*q.i + IE*q.r - JE*q.k + k1*q.j;
    j1 = RE*q.j1 + I*q.k1 + J*q.u1 - K*q.i1 - E*q.j + IE*q.k + JE*q.r - k1*q.i;
    k1 = RE*q.k1 - I*q.j1 + J*q.i1 + K*q.u1 - E*q.k - IE*q.j + JE*q.i + k1*q.r;

    return *this;
}

template <typename T> Octonion<T> & Octonion<T>::operator /= (const Octonion<T> & q)
{
    T denom = q.norm();
    T RE = r, I = i, J = j, K = k, E = u1, IE = i1, JE = j1;

    r = (RE*q.r + I*q.i + J*q.j + K*q.k + E*q.u1 + IE*q.i1 + JE*q.j1 + k1*q.k1) / denom;
    i = (-RE*q.i + I*q.r - J*q.k + K*q.j - E*q.i1 + IE*q.u1 + JE*q.k1 - k1*q.j1) / denom;
    j = (-RE*q.j + I*q.k + J*q.r - K*q.i - E*q.j1 - IE*q.k1 + JE*q.u1 + k1*q.i1) / denom;
    k = (-RE*q.k - I*q.j + J*q.i + K*q.r - E*q.k1 + IE*q.j1 - JE*q.i1 + k1*q.u1) / denom;
    u1 = (-RE*q.u1 + I*q.i1 + J*q.j1 + K*q.k1 + E*q.r - IE*q.i - JE*q.j - k1*q.k) / denom;
    i1 = (-RE*q.i1 - I*q.u1 + J*q.k1 - K*q.j1 + E*q.i + IE*q.r + JE*q.k - k1*q.j) / denom;
    j1 = (-RE*q.j1 - I*q.k1 - J*q.u1 + K*q.i1 + E*q.j - IE*q.k + JE*q.r + k1*q.i) / denom;
    k1 = (-RE*q.k1 + I*q.j1 - J*q.i1 - K*q.u1 + E*q.k + IE*q.j - JE*q.i + k1*q.r) / denom;

    return *this;
}

template <typename T> Octonion<T> & Octonion<T>::operator += (T x)
{
    r += x;

    return *this;
}

template <typename T> Octonion<T> & Octonion<T>::operator -= (T x)
{
    r -= x;

    return *this;
}

template <typename T> Octonion<T> & Octonion<T>::operator *= (T x)
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
    }

    return *this;
}

template <typename T> Octonion<T> & Octonion<T>::operator /= (T x)
{
    if (x == 0.0 && norm() > 0)
    {
        r /= (r == 0) ? Support<T>::sign(x) : x;
        i /= (i == 0) ? Support<T>::sign(x) : x;
        j /= (j == 0) ? Support<T>::sign(x) : x;
        k /= (k == 0) ? Support<T>::sign(x) : x;
        u1 /= (u1 == 0) ? Support<T>::sign(x) : x;
        i1 /= (i1 == 0) ? Support<T>::sign(x) : x;
        j1 /= (j1 == 0) ? Support<T>::sign(x) : x;
        k1 /= (k1 == 0) ? Support<T>::sign(x) : x;
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
    }

    return *this;
}

template <typename T> Octonion<T> & Octonion<T>::operator ++()
{
    ++r;

    return *this;
}

template <typename T> Octonion<T> Octonion<T>::operator ++(int)
{
    Octonion copy = *this;
    ++r;

    return copy;
}

template <typename T> Octonion<T> & Octonion<T>::operator --()
{
    --r;

    return *this;
}

template <typename T> Octonion<T> Octonion<T>::operator --(int)
{
    Octonion copy = *this;
    --r;

    return copy;
}

/******************************************
 * Real-valued Functions
 ******************************************/

template <typename T> T Octonion<T>::real() const
{
    return r;
}

template <typename T> T Octonion<T>::operator [](int n) const
{
    return reinterpret_cast<const T *>(this)[n];
}

template <typename T> T& Octonion<T>::operator [](int n)
{
    return reinterpret_cast<T *>(this)[n];
}

template <typename T> T Octonion<T>::imag_i() const
{
    return i;
}

template <typename T> T Octonion<T>::imag_j() const
{
    return j;
}

template <typename T> T Octonion<T>::imag_k() const
{
    return k;
}

template <typename T> T Octonion<T>::imag_u1() const
{
    return u1;
}

template <typename T> T Octonion<T>::imag_i1() const
{
    return i1;
}

template <typename T> T Octonion<T>::imag_j1() const
{
    return j1;
}

template <typename T> T Octonion<T>::imag_k1() const
{
    return k1;
}

template <typename T> T Octonion<T>::csgn() const
{
    return is_zero() ? 0.0 : Support<T>::sign(r);
}

template <typename T> T Octonion<T>::abs() const
{
    return is_inf() ? Support<T>::POS_INF : std::sqrt(r*r + i*i + j*j + k*k + u1*u1 + i1*i1 + j1*j1 + k1*k1);
}

template <typename T> T Octonion<T>::norm() const
{
    return is_inf() ? Support<T>::POS_INF : r*r + i*i + j*j + k*k + u1*u1 + i1*i1 + j1*j1 + k1*k1;
}

template <typename T> T Octonion<T>::abs_imag() const
{
    return is_inf() ? Support<T>::POS_INF : std::sqrt(i*i + j*j + k*k + u1*u1 + i1*i1 + j1*j1 + k1*k1);
}

template <typename T> T Octonion<T>::norm_imag() const
{
    return is_inf() ? Support<T>::POS_INF : i*i + j*j + k*k + u1*u1 + i1*i1 + j1*j1 + k1*k1;
}

template <typename T> T Octonion<T>::arg() const
{
    return std::atan2(abs_imag(), r);
}

/******************************************
 * Octonion<T>-valued Functions
 ******************************************/

template <typename T> Octonion<T> Octonion<T>::imag() const
{
    return Octonion<T>(0.0, i, j, k, u1, i1, j1, k1);
}

template <typename T> Octonion<T> Octonion<T>::conj() const
{
    return Octonion<T>(r, -i, -j, -k, -u1, -i1, -j1, -k1);
}

template <typename T> Octonion<T> Octonion<T>::operator * () const
{
    return Octonion<T>(r, -i, -j, -k, -u1, -i1, -j1, -k1);
}

template <typename T> Octonion<T> Octonion<T>::signum() const
{
    T absq = abs();

    if (absq == 0.0 || Support<T>::is_nan(absq) || Support<T>::is_inf(absq))
    {
        return *this;
    }
    else
    {
        return Octonion<T>(r / absq, i / absq, j / absq, k / absq, u1 / absq, i1 / absq, j1 / absq, k1 / absq);
    }
}

template <typename T> Octonion<T> Octonion<T>::sqr() const
{
    return Octonion<T>(
        r*r - i*i - j*j - k*k - u1*u1 - i1*i1 - j1*j1 - k1*k1,
        2 * r*i, 2 * r*j, 2 * r*k, 2 * r*u1, 2 * r*i1, 2 * r*j1, 2 * r*k1
        );
}

template <typename T> Octonion<T> Octonion<T>::sqrt() const
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

template <typename T> Octonion<T> Octonion<T>::rotate(const Octonion<T> & q) const
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

    T iqi = i*q.i;
    T jqj = j*q.j;
    T kqk = k*q.k;
    T u1qu1 = u1*q.u1;
    T i1qi1 = i1*q.i1;
    T j1qj1 = j1*q.j1;
    T k1qk1 = k1*q.k1;

    T sum = rr - ii - jj - kk - u1u1 - i1i1 - j1j1 - k1k1;

    return Octonion<T>(
        ((r == 0) ? r : r*(rr + ii + jj + kk + u1u1 + i1i1 + j1j1 + k1k1)),
        (sum + 2 * ii)*i + 2 * (
            q.r*(-j*q.k + k*q.j - u1*q.i1 + i1*q.u1 + j1*q.k1 - k1*q.j1) +
            q.i*(jqj + kqk + u1qu1 + i1qi1 + j1qj1 + k1qk1)
            ),
            (sum + 2 * jj)*j + 2 * (
                q.r*(i*q.k - k*q.i - u1*q.j1 - i1*q.k1 + j1*q.u1 + k1*q.i1) +
                q.j*(i1qi1 + j1qj1 + k1qk1 + u1qu1 + kqk + iqi)
                ),
                (sum + 2 * kk)*k + 2 * (
                    q.r*(-i*q.j + j*q.i - u1*q.k1 + i1*q.j1 - j1*q.i1 + k1*q.u1) +
                    q.k*(k1qk1 + jqj + u1qu1 + i1qi1 + iqi + j1qj1)
                    ),
                    (sum + 2 * u1u1)*u1 + 2 * (
                        q.r*(i*q.i1 + j*q.j1 + k*q.k1 - i1*q.i - j1*q.j - k1*q.k) +
                        q.u1*(iqi + jqj + kqk + j1qj1 + k1qk1 + i1qi1)
                        ),
                        (sum + 2 * i1i1)*i1 + 2 * (
                            q.r*(-i*q.u1 + j*q.k1 - k*q.j1 + u1*q.i + j1*q.k - k1*q.j) +
                            q.i1*(iqi + jqj + u1qu1 + j1qj1 + k1qk1 + kqk)
                            ),
                            (sum + 2 * j1j1)*j1 + 2 * (
                                q.r*(-i*q.k1 - j*q.u1 - i1*q.k + k1*q.i + k*q.i1 + u1*q.j) +
                                q.j1*(u1qu1 + i1qi1 + k1qk1 + iqi + jqj + kqk)
                                ),
                                (sum + 2 * k1k1)*k1 + 2 * (
                                    q.r*(i*q.j1 - j*q.i1 - k*q.u1 + u1*q.k + i1*q.j - j1*q.i) +
                                    q.k1*(j1qj1 + jqj + kqk + u1qu1 + iqi + i1qi1)
                                    )
        );
}

/******************************************
 * Boolean-valued Functions
 ******************************************/

template <typename T> bool Octonion<T>::is_imaginary() const
{
    return (r == 0.0);
}

template <typename T> bool Octonion<T>::is_inf() const
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
        (k1 == Support<T>::NEG_INF);
}

template <typename T> bool Octonion<T>::is_nan() const
{
    return (r != r) || (i != i) || (j != j) || (k != k) || (u1 != u1) || (i1 != i1) || (j1 != j1) || (k1 != k1);
}

template <typename T> bool Octonion<T>::is_neg_inf() const
{
    return (r == Support<T>::NEG_INF) && (i == 0.0) && (j == 0.0) && (k == 0.0) && (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0);
}

template <typename T> bool Octonion<T>::is_pos_inf() const
{
    return (r == Support<T>::POS_INF) && (i == 0.0) && (j == 0.0) && (k == 0.0) && (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0);
}

template <typename T> bool Octonion<T>::is_real() const
{
    return (i == 0.0) && (j == 0.0) && (k == 0.0) && (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0);
}

template <typename T> bool Octonion<T>::is_real_inf() const
{
    return (r == Support<T>::POS_INF || r == Support<T>::NEG_INF) && (i == 0.0) && (j == 0.0) && (k == 0.0) && (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0);
}

template <typename T> bool Octonion<T>::is_zero() const
{
    return (r == 0.0) && (i == 0.0) && (j == 0.0) && (k == 0.0) && (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0);
}

/******************************************
 * Multiplier Function
 ******************************************/

template <typename T> inline Octonion<T> Octonion<T>::multiplier(T r, T mltplr, const Octonion<T> & q)
{
    if (Support<T>::is_nan(mltplr) || Support<T>::is_inf(mltplr))
    {
        if (q.i == 0 && q.j == 0 && q.k == 0)
        {
            return Octonion<T>(r, mltplr*q.i, mltplr*q.j, mltplr*q.k, mltplr*q.u1, mltplr*q.i1, mltplr*q.j1, mltplr*q.k1);
        }
        else
        {
            return Octonion<T>(
                r,
                (q.i == 0) ? Support<T>::sign(mltplr)*q.i : mltplr*q.i,
                (q.j == 0) ? Support<T>::sign(mltplr)*q.j : mltplr*q.j,
                (q.k == 0) ? Support<T>::sign(mltplr)*q.k : mltplr*q.k,
                (q.u1 == 0) ? Support<T>::sign(mltplr)*q.u1 : mltplr*q.u1,
                (q.i1 == 0) ? Support<T>::sign(mltplr)*q.i1 : mltplr*q.i1,
                (q.j1 == 0) ? Support<T>::sign(mltplr)*q.j1 : mltplr*q.j1,
                (q.k1 == 0) ? Support<T>::sign(mltplr)*q.k1 : mltplr*q.k1
                );
        }
    }
    else
    {
        return Octonion<T>(r, mltplr*q.i, mltplr*q.j, mltplr*q.k, mltplr*q.u1, mltplr*q.i1, mltplr*q.j1, mltplr*q.k1);
    }
}

template <typename T> inline Octonion<T> Octonion<T>::make_inf(T r, T i)
{
    return Octonion<T>(r, i, i, i, i, i, i, i);
}

template <typename T> inline Octonion<T> Octonion<T>::make_i(T r, T i)
{
    return Octonion<T>(r, i, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

/******************************************
 * Exponential and Logarithmic Functions
 ******************************************/

template <typename T> Octonion<T> Octonion<T>::exp() const
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

template <typename T> Octonion<T> Octonion<T>::log() const
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

template <typename T> Octonion<T> Octonion<T>::log10() const
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

template <typename T> Octonion<T> Octonion<T>::pow(const Octonion<T> & q) const
{
    return (log() * q).exp();
}

template <typename T> Octonion<T> Octonion<T>::pow(T x) const
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

template <typename T> Octonion<T> Octonion<T>::inverse() const
{
    if (is_zero())
    {
        return Octonion(
            Support<T>::sign(r)*Support<T>::POS_INF,
            -Support<T>::sign(i)*Support<T>::POS_INF,
            -Support<T>::sign(j)*Support<T>::POS_INF,
            -Support<T>::sign(k)*Support<T>::POS_INF,
            -Support<T>::sign(u1)*Support<T>::POS_INF,
            -Support<T>::sign(i1)*Support<T>::POS_INF,
            -Support<T>::sign(j1)*Support<T>::POS_INF,
            -Support<T>::sign(k1)*Support<T>::POS_INF
        );
    }
    else if (is_inf())
    {
        return Octonion(
            Support<T>::sign(r)*0.0,
            -Support<T>::sign(i)*0.0,
            -Support<T>::sign(j)*0.0,
            -Support<T>::sign(k)*0.0,
            -Support<T>::sign(u1)*0.0,
            -Support<T>::sign(i1)*0.0,
            -Support<T>::sign(j1)*0.0,
            -Support<T>::sign(k1)*0.0
        );
    }
    else if (is_nan())
    {
        return Octonion(Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN);
    }
    else
    {
        T denom = norm();

        return Octonion(r / denom, -i / denom, -j / denom, -k / denom, -u1 / denom, -i1 / denom, -j1 / denom, -k1 / denom);
    }
}

/********************************************************************
 * Trigonometric and Hyperbolic Functions
 *
 * For each function f:O -> O, we define:
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

template <typename T> Octonion<T> Octonion<T>::sin() const
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

template <typename T> Octonion<T> Octonion<T>::cos() const
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

template <typename T> Octonion<T> Octonion<T>::tan() const
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

template <typename T> Octonion<T> Octonion<T>::sec() const
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

template <typename T> Octonion<T> Octonion<T>::csc() const
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

template <typename T> Octonion<T> Octonion<T>::cot() const
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

template <typename T> Octonion<T> Octonion<T>::sinh() const
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

template <typename T> Octonion<T> Octonion<T>::cosh() const
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

template <typename T> Octonion<T> Octonion<T>::tanh() const
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

template <typename T> Octonion<T> Octonion<T>::sech() const
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

template <typename T> Octonion<T> Octonion<T>::csch() const
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

template <typename T> Octonion<T> Octonion<T>::coth() const
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

template <typename T> Octonion<T> Octonion<T>::asin(const Octonion<T> & q) const
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

template <typename T> Octonion<T> Octonion<T>::acos(const Octonion<T> & q) const
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

template <typename T> Octonion<T> Octonion<T>::atan() const
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

template <typename T> Octonion<T> Octonion<T>::asec(const Octonion<T> & q) const
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

template <typename T> Octonion<T> Octonion<T>::acsc(const Octonion<T> & q) const
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

template <typename T> Octonion<T> Octonion<T>::acot() const
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

template <typename T> Octonion<T> Octonion<T>::asinh() const
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

template <typename T> Octonion<T> Octonion<T>::acosh(const Octonion<T> & q) const
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

template <typename T> Octonion<T> Octonion<T>::atanh(const Octonion<T> & q) const
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

template <typename T> Octonion<T> Octonion<T>::asech(const Octonion<T> & q) const
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

template <typename T> Octonion<T> Octonion<T>::acsch() const
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

template <typename T> Octonion<T> Octonion<T>::acoth(const Octonion<T> & q) const
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

template <typename T> Octonion<T> Octonion<T>::bessel_J(int n) const
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

template <typename T> Octonion<T> Octonion<T>::ceil() const
{
    return Octonion<T>(
        std::ceil(r), std::ceil(i), std::ceil(j), std::ceil(k),
        std::ceil(u1), std::ceil(i1), std::ceil(j1), std::ceil(k1)
        );
}

template <typename T> Octonion<T> Octonion<T>::floor() const
{
    return Octonion<T>(
        std::floor(r), std::floor(i), std::floor(j), std::floor(k),
        std::floor(u1), std::floor(i1), std::floor(j1), std::floor(k1)
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
 *   This is the same as with Matlab.  Because octonions are
 *   not commutative, this only makes sense if the coefficients
 *   and offsets are real.
 *
 *        Re(q) + i |Imag(q)|
 *
 ******************************************/

template <typename T> Octonion<T> Octonion<T>::horner(T * v, unsigned int n) const
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

template <typename T> Octonion<T> Octonion<T>::horner(T * v, T * c, unsigned int n) const
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

template <typename T> Octonion<T> Octonion<T>::random()
{
    return Octonion<T>(
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

template <typename T> Octonion<T> Octonion<T>::random_imag()
{
    return Octonion<T>(
        0.0,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX,
        (static_cast<T>(rand())) / RAND_MAX
        );
}

template <typename T> Octonion<T> Octonion<T>::random_real()
{
    return Octonion<T>((static_cast<T>(rand())) / RAND_MAX, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

/******************************************
 * Binary Arithmetic Operators
 ******************************************/

template <typename T> Octonion<T> Octonion<T>::operator + (const Octonion<T> & z) const
{
    return Octonion<T>(
        r + z.r, i + z.i, j + z.j, k + z.k,
        u1 + z.u1, i1 + z.i1, j1 + z.j1, k1 + z.k1
        );
}

template <typename T> Octonion<T> Octonion<T>::operator + (T x) const
{
    return Octonion<T>(r + x, i, j, k, u1, i1, j1, k1);
}

template <typename T> Octonion<T> operator + (T x, const Octonion<T> & z)
{
    return Octonion<T>(
        x + z.real(), z.imag_i(), z.imag_j(), z.imag_k(),
        z.imag_u1(), z.imag_i1(), z.imag_j1(), z.imag_k1()
        );
}

template <typename T> Octonion<T> operator + (long x, const Octonion<T> & z)
{
    return Octonion<T>(
        static_cast<T>(x) + z.real(), z.imag_i(), z.imag_j(), z.imag_k(),
        z.imag_u1(), z.imag_i1(), z.imag_j1(), z.imag_k1()
        );
}

template <typename T> Octonion<T> Octonion<T>::operator - (const Octonion<T> & z) const
{
    return Octonion<T>(
        r - z.r, i - z.i, j - z.j, k - z.k,
        u1 - z.u1, i1 - z.i1, j1 - z.j1, k1 - z.k1
        );
}

template <typename T> Octonion<T> Octonion<T>::operator - (T x) const
{
    return Octonion<T>(r - x, i, j, k, u1, i1, j1, k1);
}

template <typename T> Octonion<T> operator - (T x, const Octonion<T> & z)
{
    return Octonion<T>(
        static_cast<T>(x) - z.real(), -z.imag_i(), -z.imag_j(), -z.imag_k(),
        -z.imag_u1(), -z.imag_i1(), -z.imag_j1(), -z.imag_k1()
        );
}

template <typename T> Octonion<T> operator - (long x, const Octonion<T> & z)
{
    return Octonion<T>(
        x - z.real(), -z.imag_i(), -z.imag_j(), -z.imag_k(),
        -z.imag_u1(), -z.imag_i1(), -z.imag_j1(), -z.imag_k1()
        );
}

template <typename T> Octonion<T> Octonion<T>::operator * (const Octonion<T> & q) const
{
    return Octonion<T>(
        r*q.r - i*q.i - j*q.j - k*q.k - u1*q.u1 - i1*q.i1 - j1*q.j1 - k1*q.k1,
        r*q.i + i*q.r + j*q.k - k*q.j + u1*q.i1 - i1*q.u1 - j1*q.k1 + k1*q.j1,
        r*q.j - i*q.k + j*q.r + k*q.i + u1*q.j1 + i1*q.k1 - j1*q.u1 - k1*q.i1,
        r*q.k + i*q.j - j*q.i + k*q.r + u1*q.k1 - i1*q.j1 + j1*q.i1 - k1*q.u1,
        r*q.u1 - i*q.i1 - j*q.j1 - k*q.k1 + u1*q.r + i1*q.i + j1*q.j + k1*q.k,
        r*q.i1 + i*q.u1 - j*q.k1 + k*q.j1 - u1*q.i + i1*q.r - j1*q.k + k1*q.j,
        r*q.j1 + i*q.k1 + j*q.u1 - k*q.i1 - u1*q.j + i1*q.k + j1*q.r - k1*q.i,
        r*q.k1 - i*q.j1 + j*q.i1 + k*q.u1 - u1*q.k - i1*q.j + j1*q.i + k1*q.r
        );
}

template <typename T> Octonion<T> Octonion<T>::operator * (T x) const
{
    if (Support<T>::is_inf(x) && norm() > 0)
    {
        return Octonion<T>(
            ((r == 0) ? Support<T>::sign(x) : x)*r,
            ((i == 0) ? Support<T>::sign(x) : x)*i,
            ((j == 0) ? Support<T>::sign(x) : x)*j,
            ((k == 0) ? Support<T>::sign(x) : x)*k,
            ((u1 == 0) ? Support<T>::sign(x) : x)*u1,
            ((i1 == 0) ? Support<T>::sign(x) : x)*i1,
            ((j1 == 0) ? Support<T>::sign(x) : x)*j1,
            ((k1 == 0) ? Support<T>::sign(x) : x)*k1
            );
    }
    else
    {
        return Octonion<T>(x*r, x*i, x*j, x*k, x*u1, x*i1, x*j1, x*k1);
    }
}

template <typename T> Octonion<T> operator * (T x, const Octonion<T> & q)
{
    return q.operator * (x);
}

template <typename T> Octonion<T> operator * (long x, const Octonion<T> & q)
{
    return q.operator * (static_cast<T>(x));
}

template <typename T> Octonion<T> Octonion<T>::operator / (const Octonion<T> & q) const
{
    T denom = q.norm();

    return Octonion<T>(
        (r*q.r + i*q.i + j*q.j + k*q.k + u1*q.u1 + i1*q.i1 + j1*q.j1 + k1*q.k1) / denom,
        (-r*q.i + i*q.r - j*q.k + k*q.j - u1*q.i1 + i1*q.u1 + j1*q.k1 - k1*q.j1) / denom,
        (-r*q.j + i*q.k + j*q.r - k*q.i - u1*q.j1 - i1*q.k1 + j1*q.u1 + k1*q.i1) / denom,
        (-r*q.k - i*q.j + j*q.i + k*q.r - u1*q.k1 + i1*q.j1 - j1*q.i1 + k1*q.u1) / denom,
        (-r*q.u1 + i*q.i1 + j*q.j1 + k*q.k1 + u1*q.r - i1*q.i - j1*q.j - k1*q.k) / denom,
        (-r*q.i1 - i*q.u1 + j*q.k1 - k*q.j1 + u1*q.i + i1*q.r + j1*q.k - k1*q.j) / denom,
        (-r*q.j1 - i*q.k1 - j*q.u1 + k*q.i1 + u1*q.j - i1*q.k + j1*q.r + k1*q.i) / denom,
        (-r*q.k1 + i*q.j1 - j*q.i1 - k*q.u1 + u1*q.k + i1*q.j - j1*q.i + k1*q.r) / denom
        );
}

template <typename T> Octonion<T> Octonion<T>::operator / (T x) const
{
    if (x == 0.0 && norm() > 0)
    {
        return Octonion<T>(
            r / ((r == 0) ? Support<T>::sign(x) : x),
            i / ((i == 0) ? Support<T>::sign(x) : x),
            j / ((j == 0) ? Support<T>::sign(x) : x),
            k / ((k == 0) ? Support<T>::sign(x) : x),
            u1 / ((u1 == 0) ? Support<T>::sign(x) : x),
            i1 / ((i1 == 0) ? Support<T>::sign(x) : x),
            j1 / ((j1 == 0) ? Support<T>::sign(x) : x),
            k1 / ((k1 == 0) ? Support<T>::sign(x) : x)
            );
    }
    else
    {
        return Octonion<T>(r / x, i / x, j / x, k / x, u1 / x, i1 / x, j1 / x, k1 / x);
    }
}

template <typename T> Octonion<T> operator / (T x, const Octonion<T> & q)
{
    T mltplr = x / q.norm();
    return Octonion<T>(mltplr*q.real(), -mltplr*q.imag_i(), -mltplr*q.imag_j(), -mltplr*q.imag_k(), -mltplr*q.imag_u1(), -mltplr*q.imag_i1(), -mltplr*q.imag_j1(), -mltplr*q.imag_k1());
}

template <typename T> Octonion<T> operator / (long x, const Octonion<T> & q)
{
    T mltplr = static_cast<T>(x) / q.norm();
    return Octonion<T>(mltplr*q.real(), -mltplr*q.imag_i(), -mltplr*q.imag_j(), -mltplr*q.imag_k(), -mltplr*q.imag_u1(), -mltplr*q.imag_i1(), -mltplr*q.imag_j1(), -mltplr*q.imag_k1());
}

/******************************************
 * Unary Arithmetic Operators
 ******************************************/

template <typename T> Octonion<T> Octonion<T>::operator - () const
{
    return Octonion<T>(-r, -i, -j, -k, -u1, -i1, -j1, -k1);
}

/******************************************
 * Binary Boolean Operators
 ******************************************/

template <typename T> bool Octonion<T>::operator == (const Octonion<T> & q) const
{
    return
        (r == q.r) && (i == q.i) && (j == q.j) && (k == q.k) &&
        (u1 == q.u1) && (i1 == q.i1) && (j1 == q.j1) && (k1 == q.k1);
}

template <typename T> bool Octonion<T>::operator == (T x) const
{
    return
        (r == x) && (i == 0.0) && (j == 0.0) && (k == 0.0) &&
        (u1 == 0.0) && (i1 == 0.0) && (j1 == 0.0) && (k1 == 0.0);
}

template <typename T> bool operator == (T x, const Octonion<T> & q)
{
    return q.operator == (x);
}

template <typename T> bool operator == (long x, const Octonion<T> & q)
{
    return q.operator == (static_cast<T>(x));
}

template <typename T> bool Octonion<T>::operator != (const Octonion<T> & q) const
{
    return
        (r != q.r) || (i != q.i) || (j != q.j) || (k != q.k) ||
        (u1 != q.u1) && (i1 != q.i1) && (j1 != q.j1) && (k1 != q.k1);
}

template <typename T> bool Octonion<T>::operator != (T x) const
{
    return
        (r != x) || (i != 0.0) || (j != 0.0) || (k != 0.0) ||
        (u1 != 0.0) && (i1 != 0.0) && (j1 != 0.0) && (k1 != 0.0);
}

template <typename T> bool operator != (T x, const Octonion<T> & q)
{
    return q.operator != (x);
}

template <typename T> bool operator != (long x, const Octonion<T> & q)
{
    return q.operator != (static_cast<T>(x));
}

/******************************************
 * IO Stream Operators
 ******************************************/

template <typename T> std::ostream & operator << (std::ostream & out, const Octonion<T> & z)
{
    Support<T>::print_real(z.real(), out);
    Support<T>::print_imaginary(z.imag_i(), 'i', out);
    Support<T>::print_imaginary(z.imag_j(), 'j', out);
    Support<T>::print_imaginary(z.imag_k(), 'k', out);
    Support<T>::print_imaginary(z.imag_u1(), "u1", out);
    Support<T>::print_imaginary(z.imag_i1(), "i1", out);
    Support<T>::print_imaginary(z.imag_j1(), "j1", out);
    Support<T>::print_imaginary(z.imag_k1(), "k1", out);

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

template <typename T> T real(const Octonion<T> & q)
{
    return q.real();
}

template <typename T> T imag_i(const Octonion<T> & q)
{
    return q.imag_i();
}

template <typename T> T imag_j(const Octonion<T> & q)
{
    return q.imag_j();
}

template <typename T> T imag_k(const Octonion<T> & q)
{
    return q.imag_k();
}

template <typename T> T imag_u1(const Octonion<T> & q)
{
    return q.imag_u1();
}

template <typename T> T imag_i1(const Octonion<T> & q)
{
    return q.imag_i1();
}

template <typename T> T imag_j1(const Octonion<T> & q)
{
    return q.imag_j1();
}

template <typename T> T imag_k1(const Octonion<T> & q)
{
    return q.imag_k1();
}

template <typename T> T csgn(const Octonion<T> & q)
{
    return q.csgn();
}

template <typename T> T abs(const Octonion<T> & q)
{
    return q.abs();
}

template <typename T> T norm(const Octonion<T> & q)
{
    return q.norm();
}

template <typename T> T abs_imag(const Octonion<T> & z)
{
    return z.abs_imag();
}

template <typename T> T norm_imag(const Octonion<T> & z)
{
    return z.norm_imag();
}

/******************************************
 * Octonion-valued Functions
 ******************************************/

template <typename T> Octonion<T> imag(const Octonion<T> & q)
{
    return q.imag();
}

template <typename T> Octonion<T> conj(const Octonion<T> & q)
{
    return q.conj();
}

template <typename T> Octonion<T> signum(const Octonion<T> & q)
{
    return q.signum();
}

template <typename T> Octonion<T> sqr(const Octonion<T> & q)
{
    return q.sqr();
}

template <typename T> Octonion<T> sqrt(const Octonion<T> & q)
{
    return q.sqrt();
}

template <typename T> Octonion<T> rotate(const Octonion<T> & q, const Octonion<T> & p)
{
    return q.rotate(p);
}

/******************************************
 * Exponential and Logarithmic Functions
 ******************************************/

template <typename T> Octonion<T> exp(const Octonion<T> & q)
{
    return q.exp();
}

template <typename T> Octonion<T> log(const Octonion<T> & q)
{
    return q.log();
}

template <typename T> Octonion<T> log10(const Octonion<T> & q)
{
    return q.log10();
}

template <typename T> Octonion<T> pow(const Octonion<T> & q, const Octonion<T> & w)
{
    return q.pow(w);
}

template <typename T> Octonion<T> pow(const Octonion<T> & q, T x)
{
    return q.pow(x);
}

template <typename T> Octonion<T> inverse(const Octonion<T> & q)
{
    return q.inverse();
}

/******************************************
 * Trigonometric and Hyperbolic Functions
 ******************************************/

template <typename T> Octonion<T> sin(const Octonion<T> & q)
{
    return q.sin();
}

template <typename T> Octonion<T> cos(const Octonion<T> & q)
{
    return q.cos();
}

template <typename T> Octonion<T> tan(const Octonion<T> & q)
{
    return q.tan();
}

template <typename T> Octonion<T> sec(const Octonion<T> & q)
{
    return q.sec();
}

template <typename T> Octonion<T> csc(const Octonion<T> & q)
{
    return q.csc();
}

template <typename T> Octonion<T> cot(const Octonion<T> & q)
{
    return q.cot();
}

template <typename T> Octonion<T> sinh(const Octonion<T> & q)
{
    return q.sinh();
}

template <typename T> Octonion<T> cosh(const Octonion<T> & q)
{
    return q.cosh();
}

template <typename T> Octonion<T> tanh(const Octonion<T> & q)
{
    return q.tanh();
}

template <typename T> Octonion<T> sech(const Octonion<T> & q)
{
    return q.sech();
}

template <typename T> Octonion<T> csch(const Octonion<T> & q)
{
    return q.csch();
}

template <typename T> Octonion<T> coth(const Octonion<T> & q)
{
    return q.coth();
}

template <typename T> Octonion<T> asin(const Octonion<T> & q, const Octonion<T> & p)
{
    return q.asin(p);
}

template <typename T> Octonion<T> acos(const Octonion<T> & q, const Octonion<T> & p)
{
    return q.acos(p);
}

template <typename T> Octonion<T> atan(const Octonion<T> & q)
{
    return q.atan();
}

template <typename T> Octonion<T> asec(const Octonion<T> & q, const Octonion<T> & p)
{
    return q.asec(p);
}

template <typename T> Octonion<T> acsc(const Octonion<T> & q, const Octonion<T> & p)
{
    return p.acsc(p);
}

template <typename T> Octonion<T> acot(const Octonion<T> & q)
{
    return q.acot();
}

template <typename T> Octonion<T> asinh(const Octonion<T> & q)
{
    return q.asinh();
}

template <typename T> Octonion<T> acosh(const Octonion<T> & q, const Octonion<T> & p)
{
    return q.acosh(p);
}

template <typename T> Octonion<T> atanh(const Octonion<T> & q, const Octonion<T> & p)
{
    return q.atanh(p);
}

template <typename T> Octonion<T> asech(const Octonion<T> & q, const Octonion<T> & p)
{
    return q.asech(p);
}

template <typename T> Octonion<T> acsch(const Octonion<T> & q)
{
    return q.acsch();
}

template <typename T> Octonion<T> acoth(const Octonion<T> & q, const Octonion<T> & p)
{
    return q.acoth(p);
}

template <typename T> Octonion<T> bessel_J(int n, const Octonion<T> & z)
{
    return z.bessel_J(n);
}

/******************************************
 * Integer Functions
 ******************************************/

template <typename T> Octonion<T> floor(const Octonion<T> & q)
{
    return q.floor();
}

template <typename T> Octonion<T> ceil(const Octonion<T> & q)
{
    return q.ceil();
}

/******************************************
 * Horner's Rule
 ******************************************/

template <typename T> Octonion<T> horner(const Octonion<T> & q, T * v, unsigned int n)
{
    return q.horner(v, n);
}

template <typename T> Octonion<T> horner(const Octonion<T> & q, T * v, T * c, unsigned int n)
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

template class Octonion<double>;
template std::ostream & operator << (std::ostream &, const Octonion<double> &);

template Octonion<double> operator + (double, const Octonion<double> &);
template Octonion<double> operator + (long, const Octonion<double> &);
template Octonion<double> operator - (double, const Octonion<double> &);
template Octonion<double> operator - (long, const Octonion<double> &);
template Octonion<double> operator * (double, const Octonion<double> &);
template Octonion<double> operator * (long, const Octonion<double> &);
template Octonion<double> operator / (double, const Octonion<double> &);
template Octonion<double> operator / (long, const Octonion<double> &);

template bool operator == (double, const Octonion<double> &);
template bool operator == (long, const Octonion<double> &);
template bool operator != (double, const Octonion<double> &);
template bool operator != (long, const Octonion<double> &);

template <> const Octonion<double> Octonion<double>::ZERO = Octonion<double>(0, 0, 0, 0, 0, 0, 0, 0);
template <> const Octonion<double>  Octonion<double>::ONE = Octonion<double>(1, 0, 0, 0, 0, 0, 0, 0);
template <> const Octonion<double>    Octonion<double>::J = Octonion<double>(0, 0, 1, 0, 0, 0, 0, 0);
template <> const Octonion<double>    Octonion<double>::K = Octonion<double>(0, 0, 0, 1, 0, 0, 0, 0);
template <> const Octonion<double>   Octonion<double>::U1 = Octonion<double>(0, 0, 0, 0, 1, 0, 0, 0);
template <> const Octonion<double>   Octonion<double>::I1 = Octonion<double>(0, 0, 0, 0, 0, 1, 0, 0);
template <> const Octonion<double>   Octonion<double>::J1 = Octonion<double>(0, 0, 0, 0, 0, 0, 1, 0);
template <> const Octonion<double>   Octonion<double>::K1 = Octonion<double>(0, 0, 0, 0, 0, 0, 0, 1);

template <> const Octonion<double> Octonion<double>::UNITS[8] = {
    Octonion<double>::ONE,
    Octonion<double>::I,
    Octonion<double>::J,
    Octonion<double>::K,
    Octonion<double>::U1,
    Octonion<double>::I1,
    Octonion<double>::J1,
    Octonion<double>::K1
};

template double real(const Octonion<double> &);
template double imag_i(const Octonion<double> &);
template double imag_j(const Octonion<double> &);
template double imag_k(const Octonion<double> &);
template double imag_u1(const Octonion<double> &);
template double imag_i1(const Octonion<double> &);
template double imag_j1(const Octonion<double> &);
template double imag_k1(const Octonion<double> &);
template double csgn(const Octonion<double> &);
template double abs(const Octonion<double> &);
template double norm(const Octonion<double> &);
template double abs_imag(const Octonion<double> &);
template double norm_imag(const Octonion<double> &);
template Octonion<double> imag(const Octonion<double> &);
template Octonion<double> conj(const Octonion<double> &);
template Octonion<double> signum(const Octonion<double> &);
template Octonion<double> sqr(const Octonion<double> &);
template Octonion<double> sqrt(const Octonion<double> &);
template Octonion<double> rotate(const Octonion<double> &, const Octonion<double> &);
template Octonion<double> exp(const Octonion<double> &);
template Octonion<double> log(const Octonion<double> &);
template Octonion<double> log10(const Octonion<double> &);
template Octonion<double> pow(const Octonion<double> &, const Octonion<double> &);
template Octonion<double> pow(const Octonion<double> &, double);
template Octonion<double> inverse(const Octonion<double> &);
template Octonion<double> sin(const Octonion<double> &);
template Octonion<double> cos(const Octonion<double> &);
template Octonion<double> tan(const Octonion<double> &);
template Octonion<double> sec(const Octonion<double> &);
template Octonion<double> csc(const Octonion<double> &);
template Octonion<double> cot(const Octonion<double> &);
template Octonion<double> sinh(const Octonion<double> &);
template Octonion<double> cosh(const Octonion<double> &);
template Octonion<double> tanh(const Octonion<double> &);
template Octonion<double> sech(const Octonion<double> &);
template Octonion<double> csch(const Octonion<double> &);
template Octonion<double> coth(const Octonion<double> &);
template Octonion<double> asin(const Octonion<double> &, const Octonion<double> &);
template Octonion<double> acos(const Octonion<double> &, const Octonion<double> &);
template Octonion<double> atan(const Octonion<double> &);
template Octonion<double> asec(const Octonion<double> &, const Octonion<double> &);
template Octonion<double> acsc(const Octonion<double> &, const Octonion<double> &);
template Octonion<double> acot(const Octonion<double> &);
template Octonion<double> asinh(const Octonion<double> &);
template Octonion<double> acosh(const Octonion<double> &, const Octonion<double> &);
template Octonion<double> atanh(const Octonion<double> &, const Octonion<double> &);
template Octonion<double> asech(const Octonion<double> &, const Octonion<double> &);
template Octonion<double> acsch(const Octonion<double> &);
template Octonion<double> acoth(const Octonion<double> &, const Octonion<double> &);
template Octonion<double> bessel_J(int, const Octonion<double> &);
template Octonion<double> floor(const Octonion<double> &);
template Octonion<double> ceil(const Octonion<double> &);
template Octonion<double> horner(const Octonion<double> &, double *, unsigned int);
template Octonion<double> horner(const Octonion<double> &, double *, double *, unsigned int);

/**************************************************
 * ********************************************** *
 * *                                            * *
 * *    Floating-point Instance of Template     * *
 * *                                            * *
 * ********************************************** *
 **************************************************/

template class Octonion<float>;
template std::ostream & operator << (std::ostream &, const Octonion<float> &);

template Octonion<float> operator + (float, const Octonion<float> &);
template Octonion<float> operator + (long, const Octonion<float> &);
template Octonion<float> operator - (float, const Octonion<float> &);
template Octonion<float> operator - (long, const Octonion<float> &);
template Octonion<float> operator * (float, const Octonion<float> &);
template Octonion<float> operator * (long, const Octonion<float> &);
template Octonion<float> operator / (float, const Octonion<float> &);
template Octonion<float> operator / (long, const Octonion<float> &);

template bool operator == (float, const Octonion<float> &);
template bool operator == (long, const Octonion<float> &);
template bool operator != (float, const Octonion<float> &);
template bool operator != (long, const Octonion<float> &);

template <> const Octonion<float> Octonion<float>::ZERO = Octonion<float>(0, 0, 0, 0, 0, 0, 0, 0);
template <> const Octonion<float>  Octonion<float>::ONE = Octonion<float>(1, 0, 0, 0, 0, 0, 0, 0);
template <> const Octonion<float>    Octonion<float>::J = Octonion<float>(0, 0, 1, 0, 0, 0, 0, 0);
template <> const Octonion<float>    Octonion<float>::K = Octonion<float>(0, 0, 0, 1, 0, 0, 0, 0);
template <> const Octonion<float>   Octonion<float>::U1 = Octonion<float>(0, 0, 0, 0, 1, 0, 0, 0);
template <> const Octonion<float>   Octonion<float>::I1 = Octonion<float>(0, 0, 0, 0, 0, 1, 0, 0);
template <> const Octonion<float>   Octonion<float>::J1 = Octonion<float>(0, 0, 0, 0, 0, 0, 1, 0);
template <> const Octonion<float>   Octonion<float>::K1 = Octonion<float>(0, 0, 0, 0, 0, 0, 0, 1);

template <> const Octonion<float> Octonion<float>::UNITS[8] = {
    Octonion<float>::ONE,
    Octonion<float>::I,
    Octonion<float>::J,
    Octonion<float>::K,
    Octonion<float>::U1,
    Octonion<float>::I1,
    Octonion<float>::J1,
    Octonion<float>::K1
};

template float real(const Octonion<float> &);
template float imag_i(const Octonion<float> &);
template float imag_j(const Octonion<float> &);
template float imag_k(const Octonion<float> &);
template float imag_u1(const Octonion<float> &);
template float imag_i1(const Octonion<float> &);
template float imag_j1(const Octonion<float> &);
template float imag_k1(const Octonion<float> &);
template float csgn(const Octonion<float> &);
template float abs(const Octonion<float> &);
template float norm(const Octonion<float> &);
template float abs_imag(const Octonion<float> &);
template float norm_imag(const Octonion<float> &);
template Octonion<float> imag(const Octonion<float> &);
template Octonion<float> conj(const Octonion<float> &);
template Octonion<float> signum(const Octonion<float> &);
template Octonion<float> sqr(const Octonion<float> &);
template Octonion<float> sqrt(const Octonion<float> &);
template Octonion<float> rotate(const Octonion<float> &, const Octonion<float> &);
template Octonion<float> exp(const Octonion<float> &);
template Octonion<float> log(const Octonion<float> &);
template Octonion<float> log10(const Octonion<float> &);
template Octonion<float> pow(const Octonion<float> &, const Octonion<float> &);
template Octonion<float> pow(const Octonion<float> &, float);
template Octonion<float> inverse(const Octonion<float> &);
template Octonion<float> sin(const Octonion<float> &);
template Octonion<float> cos(const Octonion<float> &);
template Octonion<float> tan(const Octonion<float> &);
template Octonion<float> sec(const Octonion<float> &);
template Octonion<float> csc(const Octonion<float> &);
template Octonion<float> cot(const Octonion<float> &);
template Octonion<float> sinh(const Octonion<float> &);
template Octonion<float> cosh(const Octonion<float> &);
template Octonion<float> tanh(const Octonion<float> &);
template Octonion<float> sech(const Octonion<float> &);
template Octonion<float> csch(const Octonion<float> &);
template Octonion<float> coth(const Octonion<float> &);
template Octonion<float> asin(const Octonion<float> &, const Octonion<float> &);
template Octonion<float> acos(const Octonion<float> &, const Octonion<float> &);
template Octonion<float> atan(const Octonion<float> &);
template Octonion<float> asec(const Octonion<float> &, const Octonion<float> &);
template Octonion<float> acsc(const Octonion<float> &, const Octonion<float> &);
template Octonion<float> acot(const Octonion<float> &);
template Octonion<float> asinh(const Octonion<float> &);
template Octonion<float> acosh(const Octonion<float> &, const Octonion<float> &);
template Octonion<float> atanh(const Octonion<float> &, const Octonion<float> &);
template Octonion<float> asech(const Octonion<float> &, const Octonion<float> &);
template Octonion<float> acsch(const Octonion<float> &);
template Octonion<float> acoth(const Octonion<float> &, const Octonion<float> &);
template Octonion<float> bessel_J(int, const Octonion<float> &);
template Octonion<float> floor(const Octonion<float> &);
template Octonion<float> ceil(const Octonion<float> &);
template Octonion<float> horner(const Octonion<float> &, float *, unsigned int);
template Octonion<float> horner(const Octonion<float> &, float *, float *, unsigned int);

/************************************************************************
 * ******************************************************************** *
 * *                                                                  * *
 * *    Long Double-precision Floating-point Instance of Template     * *
 * *                                                                  * *
 * ******************************************************************** *
 ************************************************************************/

template class Octonion<long double>;
template std::ostream & operator << (std::ostream &, const Octonion<long double> &);

template Octonion<long double> operator + (long double, const Octonion<long double> &);
template Octonion<long double> operator + (long, const Octonion<long double> &);
template Octonion<long double> operator - (long double, const Octonion<long double> &);
template Octonion<long double> operator - (long, const Octonion<long double> &);
template Octonion<long double> operator * (long double, const Octonion<long double> &);
template Octonion<long double> operator * (long, const Octonion<long double> &);
template Octonion<long double> operator / (long double, const Octonion<long double> &);
template Octonion<long double> operator / (long, const Octonion<long double> &);

template bool operator == (long double, const Octonion<long double> &);
template bool operator == (long, const Octonion<long double> &);
template bool operator != (long double, const Octonion<long double> &);
template bool operator != (long, const Octonion<long double> &);

template <> const Octonion<long double> Octonion<long double>::ZERO = Octonion<long double>(0, 0, 0, 0, 0, 0, 0, 0);
template <> const Octonion<long double>  Octonion<long double>::ONE = Octonion<long double>(1, 0, 0, 0, 0, 0, 0, 0);
template <> const Octonion<long double>    Octonion<long double>::J = Octonion<long double>(0, 0, 1, 0, 0, 0, 0, 0);
template <> const Octonion<long double>    Octonion<long double>::K = Octonion<long double>(0, 0, 0, 1, 0, 0, 0, 0);
template <> const Octonion<long double>   Octonion<long double>::U1 = Octonion<long double>(0, 0, 0, 0, 1, 0, 0, 0);
template <> const Octonion<long double>   Octonion<long double>::I1 = Octonion<long double>(0, 0, 0, 0, 0, 1, 0, 0);
template <> const Octonion<long double>   Octonion<long double>::J1 = Octonion<long double>(0, 0, 0, 0, 0, 0, 1, 0);
template <> const Octonion<long double>   Octonion<long double>::K1 = Octonion<long double>(0, 0, 0, 0, 0, 0, 0, 1);

template <> const Octonion<long double> Octonion<long double>::UNITS[8] = {
    Octonion<long double>::ONE,
    Octonion<long double>::I,
    Octonion<long double>::J,
    Octonion<long double>::K,
    Octonion<long double>::U1,
    Octonion<long double>::I1,
    Octonion<long double>::J1,
    Octonion<long double>::K1
};

template long double real(const Octonion<long double> &);
template long double imag_i(const Octonion<long double> &);
template long double imag_j(const Octonion<long double> &);
template long double imag_k(const Octonion<long double> &);
template long double imag_u1(const Octonion<long double> &);
template long double imag_i1(const Octonion<long double> &);
template long double imag_j1(const Octonion<long double> &);
template long double imag_k1(const Octonion<long double> &);
template long double csgn(const Octonion<long double> &);
template long double abs(const Octonion<long double> &);
template long double norm(const Octonion<long double> &);
template long double abs_imag(const Octonion<long double> &);
template long double norm_imag(const Octonion<long double> &);
template Octonion<long double> imag(const Octonion<long double> &);
template Octonion<long double> conj(const Octonion<long double> &);
template Octonion<long double> signum(const Octonion<long double> &);
template Octonion<long double> sqr(const Octonion<long double> &);
template Octonion<long double> sqrt(const Octonion<long double> &);
template Octonion<long double> rotate(const Octonion<long double> &, const Octonion<long double> &);
template Octonion<long double> exp(const Octonion<long double> &);
template Octonion<long double> log(const Octonion<long double> &);
template Octonion<long double> log10(const Octonion<long double> &);
template Octonion<long double> pow(const Octonion<long double> &, const Octonion<long double> &);
template Octonion<long double> pow(const Octonion<long double> &, long double);
template Octonion<long double> inverse(const Octonion<long double> &);
template Octonion<long double> sin(const Octonion<long double> &);
template Octonion<long double> cos(const Octonion<long double> &);
template Octonion<long double> tan(const Octonion<long double> &);
template Octonion<long double> sec(const Octonion<long double> &);
template Octonion<long double> csc(const Octonion<long double> &);
template Octonion<long double> cot(const Octonion<long double> &);
template Octonion<long double> sinh(const Octonion<long double> &);
template Octonion<long double> cosh(const Octonion<long double> &);
template Octonion<long double> tanh(const Octonion<long double> &);
template Octonion<long double> sech(const Octonion<long double> &);
template Octonion<long double> csch(const Octonion<long double> &);
template Octonion<long double> coth(const Octonion<long double> &);
template Octonion<long double> asin(const Octonion<long double> &, const Octonion<long double> &);
template Octonion<long double> acos(const Octonion<long double> &, const Octonion<long double> &);
template Octonion<long double> atan(const Octonion<long double> &);
template Octonion<long double> asec(const Octonion<long double> &, const Octonion<long double> &);
template Octonion<long double> acsc(const Octonion<long double> &, const Octonion<long double> &);
template Octonion<long double> acot(const Octonion<long double> &);
template Octonion<long double> asinh(const Octonion<long double> &);
template Octonion<long double> acosh(const Octonion<long double> &, const Octonion<long double> &);
template Octonion<long double> atanh(const Octonion<long double> &, const Octonion<long double> &);
template Octonion<long double> asech(const Octonion<long double> &, const Octonion<long double> &);
template Octonion<long double> acsch(const Octonion<long double> &);
template Octonion<long double> acoth(const Octonion<long double> &, const Octonion<long double> &);
template Octonion<long double> bessel_J(int, const Octonion<long double> &);
template Octonion<long double> floor(const Octonion<long double> &);
template Octonion<long double> ceil(const Octonion<long double> &);
template Octonion<long double> horner(const Octonion<long double> &, long double *, unsigned int);
template Octonion<long double> horner(const Octonion<long double> &, long double *, long double *, unsigned int);
