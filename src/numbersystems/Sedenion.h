/******************************************
 * C++ Sedenions
 * Version: 1.0.9
 * Author:  Douglas Wilhelm Harder
 * Date:    2008/03/03
 *
 * Copyright (c) 2006-2008 by Douglas Wilhelm Harder.
 * All rights reserved.
 ******************************************/

#ifndef CA_UWATERLOO_ALUMNI_DWHARDER_SEDENION
#define CA_UWATERLOO_ALUMNI_DWHARDER_SEDENION

#include "internal/NumberSystemsDLL.h"

#include <iostream>
#include <cmath>
#include <string>

template <typename T> class Sedenion;

template <typename T> std::ostream & operator << (std::ostream &, const Sedenion<T> &);

template <typename T> Sedenion<T> operator + (T, const Sedenion<T> &);
template <typename T> Sedenion<T> operator + (long, const Sedenion<T> &);
template <typename T> Sedenion<T> operator - (T, const Sedenion<T> &);
template <typename T> Sedenion<T> operator - (long, const Sedenion<T> &);
template <typename T> Sedenion<T> operator * (T, const Sedenion<T> &);
template <typename T> Sedenion<T> operator * (long, const Sedenion<T> &);
template <typename T> Sedenion<T> operator / (T, const Sedenion<T> &);
template <typename T> Sedenion<T> operator / (long, const Sedenion<T> &);

template <typename T> bool operator == (T, const Sedenion<T> &);
template <typename T> bool operator == (long, const Sedenion<T> &);
template <typename T> bool operator != (T, const Sedenion<T> &);
template <typename T> bool operator != (long, const Sedenion<T> &);

template <typename T> T real(const Sedenion<T> &);
template <typename T> T imag_i(const Sedenion<T> &);
template <typename T> T imag_j(const Sedenion<T> &);
template <typename T> T imag_k(const Sedenion<T> &);
template <typename T> T imag_u1(const Sedenion<T> &);
template <typename T> T imag_i1(const Sedenion<T> &);
template <typename T> T imag_j1(const Sedenion<T> &);
template <typename T> T imag_k1(const Sedenion<T> &);
template <typename T> T imag_u2(const Sedenion<T> &);
template <typename T> T imag_i2(const Sedenion<T> &);
template <typename T> T imag_j2(const Sedenion<T> &);
template <typename T> T imag_k2(const Sedenion<T> &);
template <typename T> T imag_u3(const Sedenion<T> &);
template <typename T> T imag_i3(const Sedenion<T> &);
template <typename T> T imag_j3(const Sedenion<T> &);
template <typename T> T imag_k3(const Sedenion<T> &);
template <typename T> T csgn(const Sedenion<T> &);
template <typename T> T abs(const Sedenion<T> &);
template <typename T> T norm(const Sedenion<T> &);
template <typename T> T abs_imag(const Sedenion<T> &);
template <typename T> T norm_imag(const Sedenion<T> &);
template <typename T> T arg(const Sedenion<T> &);
template <typename T> Sedenion<T> imag(const Sedenion<T> &);
template <typename T> Sedenion<T> conj(const Sedenion<T> &);
template <typename T> Sedenion<T> signum(const Sedenion<T> &);
template <typename T> Sedenion<T> sqr(const Sedenion<T> &);
template <typename T> Sedenion<T> sqrt(const Sedenion<T> &);
template <typename T> Sedenion<T> rotate(const Sedenion<T> &, const Sedenion<T> &);
template <typename T> Sedenion<T> exp(const Sedenion<T> &);
template <typename T> Sedenion<T> log(const Sedenion<T> &);
template <typename T> Sedenion<T> log10(const Sedenion<T> &);
template <typename T> Sedenion<T> pow(const Sedenion<T> &, const Sedenion<T> &);
template <typename T> Sedenion<T> pow(const Sedenion<T> &, T);
template <typename T> Sedenion<T> inverse(const Sedenion<T> &);
template <typename T> Sedenion<T> sin(const Sedenion<T> &);
template <typename T> Sedenion<T> cos(const Sedenion<T> &);
template <typename T> Sedenion<T> tan(const Sedenion<T> &);
template <typename T> Sedenion<T> sec(const Sedenion<T> &);
template <typename T> Sedenion<T> csc(const Sedenion<T> &);
template <typename T> Sedenion<T> cot(const Sedenion<T> &);
template <typename T> Sedenion<T> sinh(const Sedenion<T> &);
template <typename T> Sedenion<T> cosh(const Sedenion<T> &);
template <typename T> Sedenion<T> tanh(const Sedenion<T> &);
template <typename T> Sedenion<T> sech(const Sedenion<T> &);
template <typename T> Sedenion<T> csch(const Sedenion<T> &);
template <typename T> Sedenion<T> coth(const Sedenion<T> &);
template <typename T> Sedenion<T> asin(const Sedenion<T> &, const Sedenion<T> & = Sedenion<T>::I);
template <typename T> Sedenion<T> acos(const Sedenion<T> &, const Sedenion<T> & = Sedenion<T>::I);
template <typename T> Sedenion<T> atan(const Sedenion<T> &);
template <typename T> Sedenion<T> asec(const Sedenion<T> &, const Sedenion<T> & = Sedenion<T>::I);
template <typename T> Sedenion<T> acsc(const Sedenion<T> &, const Sedenion<T> & = Sedenion<T>::I);
template <typename T> Sedenion<T> acot(const Sedenion<T> &);
template <typename T> Sedenion<T> asinh(const Sedenion<T> &);
template <typename T> Sedenion<T> acosh(const Sedenion<T> &, const Sedenion<T> & = Sedenion<T>::I);
template <typename T> Sedenion<T> atanh(const Sedenion<T> &, const Sedenion<T> & = Sedenion<T>::I);
template <typename T> Sedenion<T> asech(const Sedenion<T> &, const Sedenion<T> & = Sedenion<T>::I);
template <typename T> Sedenion<T> acsch(const Sedenion<T> &);
template <typename T> Sedenion<T> acoth(const Sedenion<T> &, const Sedenion<T> & = Sedenion<T>::I);
template <typename T> Sedenion<T> bessel_J(int, const Sedenion<T> &);
template <typename T> Sedenion<T> floor(const Sedenion<T> &);
template <typename T> Sedenion<T> ceil(const Sedenion<T> &);
template <typename T> Sedenion<T> horner(const Sedenion<T> &, T *, unsigned int);
template <typename T> Sedenion<T> horner(const Sedenion<T> &, T *, T*, unsigned int);

template <typename T = double>
class NumberSystems_API Sedenion
{
private:
    T r, i, j, k, u1, i1, j1, k1, u2, i2, j2, k2, u3, i3, j3, k3;

    inline static Sedenion multiplier(T, T, const Sedenion &);
    inline static Sedenion make_inf(T, T);
    inline static Sedenion make_i(T, T);

public:
    const static Sedenion ZERO;
    const static Sedenion ONE;
    const static Sedenion I;
    const static Sedenion J;
    const static Sedenion K;
    const static Sedenion U1;
    const static Sedenion I1;
    const static Sedenion J1;
    const static Sedenion K1;
    const static Sedenion U2;
    const static Sedenion I2;
    const static Sedenion J2;
    const static Sedenion K2;
    const static Sedenion U3;
    const static Sedenion I3;
    const static Sedenion J3;
    const static Sedenion K3;

    const static Sedenion UNITS[16];

    /******************************************
     * Constructor and Copy Constructor
     ******************************************/

    Sedenion(T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T);
    Sedenion(T = 0.0);

    /******************************************
     * Assignment Operator
     ******************************************/

    const Sedenion & operator = (const T &);

    /******************************************
     * Mutating Arithmetic Operators
     ******************************************/

    Sedenion & operator += (const Sedenion &);
    Sedenion & operator -= (const Sedenion &);
    Sedenion & operator *= (const Sedenion &);
    Sedenion & operator /= (const Sedenion &);

    Sedenion & operator += (T);
    Sedenion & operator -= (T);
    Sedenion & operator *= (T);
    Sedenion & operator /= (T);

    Sedenion & operator ++();
    Sedenion operator ++(int);
    Sedenion & operator --();
    Sedenion operator --(int);

    /******************************************
     * Real-valued Functions
     ******************************************/

    T real() const;
    T operator [](int) const;
    T& operator [](int);
    T imag_i() const;
    T imag_j() const;
    T imag_k() const;
    T imag_u1() const;
    T imag_i1() const;
    T imag_j1() const;
    T imag_k1() const;
    T imag_u2() const;
    T imag_i2() const;
    T imag_j2() const;
    T imag_k2() const;
    T imag_u3() const;
    T imag_i3() const;
    T imag_j3() const;
    T imag_k3() const;
    T csgn() const;
    T abs() const;
    T norm() const;
    T abs_imag() const;
    T norm_imag() const;
    T arg() const;

    /******************************************
     * Sedenion-valued Functions
     ******************************************/

    Sedenion imag() const;
    Sedenion conj() const;
    Sedenion operator * () const;
    Sedenion signum() const;
    Sedenion sqr() const;
    Sedenion sqrt() const;
    Sedenion rotate(const Sedenion &) const;

    /******************************************
     * Boolean-valued Functions
     ******************************************/

    bool is_imaginary() const;
    bool is_inf() const;
    bool is_nan() const;
    bool is_neg_inf() const;
    bool is_pos_inf() const;
    bool is_real() const;
    bool is_real_inf() const;
    bool is_zero() const;

    /******************************************
     * Exponential and Logarithmic Functions
     ******************************************/

    Sedenion exp() const;
    Sedenion log() const;
    Sedenion log10() const;
    Sedenion pow(const Sedenion &) const;
    Sedenion pow(T) const;
    Sedenion inverse() const;

    /******************************************
     * Trigonometric and Hyperbolic Functions
     ******************************************/

    Sedenion   sin() const;   Sedenion   cos() const;   Sedenion   tan() const;
    Sedenion   sec() const;   Sedenion   csc() const;   Sedenion   cot() const;
    Sedenion  sinh() const;   Sedenion  cosh() const;   Sedenion  tanh() const;
    Sedenion  sech() const;   Sedenion  csch() const;   Sedenion  coth() const;
    Sedenion  asin(const Sedenion & = I) const;
    Sedenion  acos(const Sedenion & = I) const;       Sedenion  atan() const;
    Sedenion  asec(const Sedenion & = I) const;
    Sedenion  acsc(const Sedenion & = I) const;       Sedenion  acot() const;

    Sedenion asinh() const;
    Sedenion acosh(const Sedenion & = I) const;
    Sedenion atanh(const Sedenion & = I) const;

    Sedenion asech(const Sedenion & = I) const;
    Sedenion acsch() const;
    Sedenion acoth(const Sedenion & = I) const;

    Sedenion bessel_J(int) const;

    /******************************************
     * Integer Functions
     ******************************************/

    Sedenion ceil() const;
    Sedenion floor() const;

    /******************************************
     * Horner's Rule
     ******************************************/

    Sedenion horner(T *, unsigned int) const;
    Sedenion horner(T *, T *, unsigned int) const;

    /******************************************
     * Random Factories
     ******************************************/

    static Sedenion random();
    static Sedenion random_imag();
    static Sedenion random_real();

    /******************************************
     * Binary Arithmetic Operators
     ******************************************/

    Sedenion operator + (const Sedenion &) const;
    Sedenion operator + (T) const;

    Sedenion operator - (const Sedenion &) const;
    Sedenion operator - (T) const;

    Sedenion operator * (const Sedenion &) const;
    Sedenion operator * (T) const;

    Sedenion operator / (const Sedenion &) const;
    Sedenion operator / (T) const;

    /******************************************
     * Unary Arithmetic Operators
     ******************************************/

    Sedenion operator - () const;

    /******************************************
     * Binary Boolean Operators
     ******************************************/

    bool operator == (const Sedenion &) const;
    bool operator == (T) const;

    bool operator != (const Sedenion &) const;
    bool operator != (T) const;
};

#endif
