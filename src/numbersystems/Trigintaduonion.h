/******************************************
 * C++ Trigintaduonions
 * Version: 1.0.9
 * Author:  Douglas Wilhelm Harder
 * Date:    2008/03/03
 *
 * Copyright (c) 2006-2008 by Douglas Wilhelm Harder.
 * All rights reserved.
 ******************************************/

#ifndef CA_UWATERLOO_ALUMNI_DWHARDER_TRIGINTADUONION
#define CA_UWATERLOO_ALUMNI_DWHARDER_TRIGINTADUONION

#include "internal/NumberSystemsDLL.h"

#include <iostream>
#include <cmath>
#include <string>

template <typename T> class Trigintaduonion;

template <typename T> std::ostream & operator << (std::ostream &, const Trigintaduonion<T> &);

template <typename T> Trigintaduonion<T> operator + (T, const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> operator + (long, const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> operator - (T, const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> operator - (long, const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> operator * (T, const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> operator * (long, const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> operator / (T, const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> operator / (long, const Trigintaduonion<T> &);

template <typename T> bool operator == (T, const Trigintaduonion<T> &);
template <typename T> bool operator == (long, const Trigintaduonion<T> &);
template <typename T> bool operator != (T, const Trigintaduonion<T> &);
template <typename T> bool operator != (long, const Trigintaduonion<T> &);

template <typename T> T real(const Trigintaduonion<T> &);
template <typename T> T imag_i(const Trigintaduonion<T> &);
template <typename T> T imag_j(const Trigintaduonion<T> &);
template <typename T> T imag_k(const Trigintaduonion<T> &);
template <typename T> T imag_u1(const Trigintaduonion<T> &);
template <typename T> T imag_i1(const Trigintaduonion<T> &);
template <typename T> T imag_j1(const Trigintaduonion<T> &);
template <typename T> T imag_k1(const Trigintaduonion<T> &);
template <typename T> T imag_u2(const Trigintaduonion<T> &);
template <typename T> T imag_i2(const Trigintaduonion<T> &);
template <typename T> T imag_j2(const Trigintaduonion<T> &);
template <typename T> T imag_k2(const Trigintaduonion<T> &);
template <typename T> T imag_u3(const Trigintaduonion<T> &);
template <typename T> T imag_i3(const Trigintaduonion<T> &);
template <typename T> T imag_j3(const Trigintaduonion<T> &);
template <typename T> T imag_k3(const Trigintaduonion<T> &);
template <typename T> T csgn(const Trigintaduonion<T> &);
template <typename T> T abs(const Trigintaduonion<T> &);
template <typename T> T norm(const Trigintaduonion<T> &);
template <typename T> T abs_imag(const Trigintaduonion<T> &);
template <typename T> T norm_imag(const Trigintaduonion<T> &);
template <typename T> T arg(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> imag(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> conj(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> signum(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> sqr(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> sqrt(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> rotate(const Trigintaduonion<T> &, const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> exp(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> log(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> log10(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> pow(const Trigintaduonion<T> &, const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> pow(const Trigintaduonion<T> &, T);
template <typename T> Trigintaduonion<T> inverse(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> sin(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> cos(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> tan(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> sec(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> csc(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> cot(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> sinh(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> cosh(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> tanh(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> sech(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> csch(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> coth(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> asin(const Trigintaduonion<T> &, const Trigintaduonion<T> & = Trigintaduonion<T>::I);
template <typename T> Trigintaduonion<T> acos(const Trigintaduonion<T> &, const Trigintaduonion<T> & = Trigintaduonion<T>::I);
template <typename T> Trigintaduonion<T> atan(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> asec(const Trigintaduonion<T> &, const Trigintaduonion<T> & = Trigintaduonion<T>::I);
template <typename T> Trigintaduonion<T> acsc(const Trigintaduonion<T> &, const Trigintaduonion<T> & = Trigintaduonion<T>::I);
template <typename T> Trigintaduonion<T> acot(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> asinh(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> acosh(const Trigintaduonion<T> &, const Trigintaduonion<T> & = Trigintaduonion<T>::I);
template <typename T> Trigintaduonion<T> atanh(const Trigintaduonion<T> &, const Trigintaduonion<T> & = Trigintaduonion<T>::I);
template <typename T> Trigintaduonion<T> asech(const Trigintaduonion<T> &, const Trigintaduonion<T> & = Trigintaduonion<T>::I);
template <typename T> Trigintaduonion<T> acsch(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> acoth(const Trigintaduonion<T> &, const Trigintaduonion<T> & = Trigintaduonion<T>::I);
template <typename T> Trigintaduonion<T> bessel_J(int, const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> floor(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> ceil(const Trigintaduonion<T> &);
template <typename T> Trigintaduonion<T> horner(const Trigintaduonion<T> &, T *, unsigned int);
template <typename T> Trigintaduonion<T> horner(const Trigintaduonion<T> &, T *, T *, unsigned int);

template <typename T = double>
class NumberSystems_API Trigintaduonion
{
private:
    T r, i, j, k, u1, i1, j1, k1, u2, i2, j2, k2, u3, i3, j3, k3, u4, i4, j4, k4, u5, i5, j5, k5, u6, i6, j6, k6, u7, i7, j7, k7;

    inline static Trigintaduonion multiplier(T, T, const Trigintaduonion &);
    inline static Trigintaduonion make_inf(T, T);
    inline static Trigintaduonion make_i(T, T);

public:
    const static Trigintaduonion ZERO;
    const static Trigintaduonion ONE;
    const static Trigintaduonion I;
    const static Trigintaduonion J;
    const static Trigintaduonion K;
    const static Trigintaduonion U1;
    const static Trigintaduonion I1;
    const static Trigintaduonion J1;
    const static Trigintaduonion K1;
    const static Trigintaduonion U2;
    const static Trigintaduonion I2;
    const static Trigintaduonion J2;
    const static Trigintaduonion K2;
    const static Trigintaduonion U3;
    const static Trigintaduonion I3;
    const static Trigintaduonion J3;
    const static Trigintaduonion K3;
    const static Trigintaduonion U4;
    const static Trigintaduonion I4;
    const static Trigintaduonion J4;
    const static Trigintaduonion K4;
    const static Trigintaduonion U5;
    const static Trigintaduonion I5;
    const static Trigintaduonion J5;
    const static Trigintaduonion K5;
    const static Trigintaduonion U6;
    const static Trigintaduonion I6;
    const static Trigintaduonion J6;
    const static Trigintaduonion K6;
    const static Trigintaduonion U7;
    const static Trigintaduonion I7;
    const static Trigintaduonion J7;
    const static Trigintaduonion K7;

    const static Trigintaduonion UNITS[32];

    /******************************************
     * Constructor and Copy Constructor
     ******************************************/

    Trigintaduonion(T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T, T);
    Trigintaduonion(T = 0.0);

    /******************************************
     * Assignment Operator
     ******************************************/

    const Trigintaduonion & operator = (T);

    /******************************************
     * Mutating Arithmetic Operators
     ******************************************/

    Trigintaduonion & operator += (const Trigintaduonion &);
    Trigintaduonion & operator -= (const Trigintaduonion &);
    Trigintaduonion & operator *= (const Trigintaduonion &);
    Trigintaduonion & operator /= (const Trigintaduonion &);

    Trigintaduonion & operator += (T);
    Trigintaduonion & operator -= (T);
    Trigintaduonion & operator *= (T);
    Trigintaduonion & operator /= (T);

    Trigintaduonion & operator ++();
    Trigintaduonion operator ++(int);
    Trigintaduonion & operator --();
    Trigintaduonion operator --(int);

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
    T imag_u4() const;
    T imag_i4() const;
    T imag_j4() const;
    T imag_k4() const;
    T imag_u5() const;
    T imag_i5() const;
    T imag_j5() const;
    T imag_k5() const;
    T imag_u6() const;
    T imag_i6() const;
    T imag_j6() const;
    T imag_k6() const;
    T imag_u7() const;
    T imag_i7() const;
    T imag_j7() const;
    T imag_k7() const;
    T csgn() const;
    T abs() const;
    T norm() const;
    T abs_imag() const;
    T norm_imag() const;
    T arg() const;

    /******************************************
     * Trigintaduonion-valued Functions
     ******************************************/

    Trigintaduonion imag() const;
    Trigintaduonion conj() const;
    Trigintaduonion operator * () const;
    Trigintaduonion signum() const;
    Trigintaduonion sqr() const;
    Trigintaduonion sqrt() const;
    Trigintaduonion rotate(const Trigintaduonion &) const;

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

    Trigintaduonion exp() const;
    Trigintaduonion log() const;
    Trigintaduonion log10() const;
    Trigintaduonion pow(const Trigintaduonion &) const;
    Trigintaduonion pow(T) const;
    Trigintaduonion inverse() const;

    /******************************************
     * Trigonometric and Hyperbolic Functions
     ******************************************/

    Trigintaduonion   sin() const;   Trigintaduonion   cos() const;   Trigintaduonion   tan() const;
    Trigintaduonion   sec() const;   Trigintaduonion   csc() const;   Trigintaduonion   cot() const;
    Trigintaduonion  sinh() const;   Trigintaduonion  cosh() const;   Trigintaduonion  tanh() const;
    Trigintaduonion  sech() const;   Trigintaduonion  csch() const;   Trigintaduonion  coth() const;
    Trigintaduonion  asin(const Trigintaduonion & = I) const;
    Trigintaduonion  acos(const Trigintaduonion & = I) const;       Trigintaduonion  atan() const;
    Trigintaduonion  asec(const Trigintaduonion & = I) const;
    Trigintaduonion  acsc(const Trigintaduonion & = I) const;       Trigintaduonion  acot() const;

    Trigintaduonion asinh() const;
    Trigintaduonion acosh(const Trigintaduonion & = I) const;
    Trigintaduonion atanh(const Trigintaduonion & = I) const;

    Trigintaduonion asech(const Trigintaduonion & = I) const;
    Trigintaduonion acsch() const;
    Trigintaduonion acoth(const Trigintaduonion & = I) const;

    Trigintaduonion bessel_J(int) const;

    /******************************************
     * Integer Functions
     ******************************************/

    Trigintaduonion ceil() const;
    Trigintaduonion floor() const;

    /******************************************
     * Horner's Rule
     ******************************************/

    Trigintaduonion horner(T *, unsigned int) const;
    Trigintaduonion horner(T *, T *, unsigned int) const;

    /******************************************
     * Random Factories
     ******************************************/

    static Trigintaduonion random();
    static Trigintaduonion random_imag();
    static Trigintaduonion random_real();

    /******************************************
     * Binary Arithmetic Operators
     ******************************************/

    Trigintaduonion operator + (const Trigintaduonion &) const;
    Trigintaduonion operator + (T) const;

    Trigintaduonion operator - (const Trigintaduonion &) const;
    Trigintaduonion operator - (T) const;

    Trigintaduonion operator * (const Trigintaduonion &) const;
    Trigintaduonion operator * (T) const;

    Trigintaduonion operator / (const Trigintaduonion &) const;
    Trigintaduonion operator / (T) const;

    /******************************************
     * Unary Arithmetic Operators
     ******************************************/

    Trigintaduonion operator - () const;

    /******************************************
     * Binary Boolean Operators
     ******************************************/

    bool operator == (const Trigintaduonion &) const;
    bool operator == (T) const;

    bool operator != (const Trigintaduonion &) const;
    bool operator != (T) const;
};

#endif
