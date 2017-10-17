/******************************************
 * C++ Octonions
 * Version: 1.0.9
 * Author:  Douglas Wilhelm Harder
 * Date:    2008/03/03
 *
 * Copyright (c) 2006-2008 by Douglas Wilhelm Harder.
 * All rights reserved.
 ******************************************/

#ifndef CA_UWATERLOO_ALUMNI_DWHARDER_OCTONION
#define CA_UWATERLOO_ALUMNI_DWHARDER_OCTONION

#include "internal/NumberSystemsDLL.h"

#include <iostream>
#include <cmath>
#include <string>

template <typename T> class Octonion;

template <typename T> std::ostream & operator << (std::ostream &, const Octonion<T> &);

template <typename T> Octonion<T> operator + (T, const Octonion<T> &);
template <typename T> Octonion<T> operator + (long, const Octonion<T> &);
template <typename T> Octonion<T> operator - (T, const Octonion<T> &);
template <typename T> Octonion<T> operator - (long, const Octonion<T> &);
template <typename T> Octonion<T> operator * (T, const Octonion<T> &);
template <typename T> Octonion<T> operator * (long, const Octonion<T> &);
template <typename T> Octonion<T> operator / (T, const Octonion<T> &);
template <typename T> Octonion<T> operator / (long, const Octonion<T> &);

template <typename T> bool operator == (T, const Octonion<T> &);
template <typename T> bool operator == (long, const Octonion<T> &);
template <typename T> bool operator != (T, const Octonion<T> &);
template <typename T> bool operator != (long, const Octonion<T> &);

template <typename T> T real(const Octonion<T> &);
template <typename T> T imag_i(const Octonion<T> &);
template <typename T> T imag_j(const Octonion<T> &);
template <typename T> T imag_k(const Octonion<T> &);
template <typename T> T imag_u1(const Octonion<T> &);
template <typename T> T imag_i1(const Octonion<T> &);
template <typename T> T imag_j1(const Octonion<T> &);
template <typename T> T imag_k1(const Octonion<T> &);
template <typename T> T csgn(const Octonion<T> &);
template <typename T> T abs(const Octonion<T> &);
template <typename T> T norm(const Octonion<T> &);
template <typename T> T abs_imag(const Octonion<T> &);
template <typename T> T norm_imag(const Octonion<T> &);
template <typename T> T arg(const Octonion<T> &);
template <typename T> Octonion<T> imag(const Octonion<T> &);
template <typename T> Octonion<T> conj(const Octonion<T> &);
template <typename T> Octonion<T> signum(const Octonion<T> &);
template <typename T> Octonion<T> sqr(const Octonion<T> &);
template <typename T> Octonion<T> sqrt(const Octonion<T> &);
template <typename T> Octonion<T> rotate(const Octonion<T> &, const Octonion<T> &);
template <typename T> Octonion<T> exp(const Octonion<T> &);
template <typename T> Octonion<T> log(const Octonion<T> &);
template <typename T> Octonion<T> log10(const Octonion<T> &);
template <typename T> Octonion<T> pow(const Octonion<T> &, const Octonion<T> &);
template <typename T> Octonion<T> pow(const Octonion<T> &, T);
template <typename T> Octonion<T> inverse(const Octonion<T> &);
template <typename T> Octonion<T> sin(const Octonion<T> &);
template <typename T> Octonion<T> cos(const Octonion<T> &);
template <typename T> Octonion<T> tan(const Octonion<T> &);
template <typename T> Octonion<T> sec(const Octonion<T> &);
template <typename T> Octonion<T> csc(const Octonion<T> &);
template <typename T> Octonion<T> cot(const Octonion<T> &);
template <typename T> Octonion<T> sinh(const Octonion<T> &);
template <typename T> Octonion<T> cosh(const Octonion<T> &);
template <typename T> Octonion<T> tanh(const Octonion<T> &);
template <typename T> Octonion<T> sech(const Octonion<T> &);
template <typename T> Octonion<T> csch(const Octonion<T> &);
template <typename T> Octonion<T> coth(const Octonion<T> &);
template <typename T> Octonion<T> asin(const Octonion<T> &, const Octonion<T> & = Octonion<T>::I);
template <typename T> Octonion<T> acos(const Octonion<T> &, const Octonion<T> & = Octonion<T>::I);
template <typename T> Octonion<T> atan(const Octonion<T> &);
template <typename T> Octonion<T> asec(const Octonion<T> &, const Octonion<T> & = Octonion<T>::I);
template <typename T> Octonion<T> acsc(const Octonion<T> &, const Octonion<T> & = Octonion<T>::I);
template <typename T> Octonion<T> acot(const Octonion<T> &);
template <typename T> Octonion<T> asinh(const Octonion<T> &);
template <typename T> Octonion<T> acosh(const Octonion<T> &, const Octonion<T> & = Octonion<T>::I);
template <typename T> Octonion<T> atanh(const Octonion<T> &, const Octonion<T> & = Octonion<T>::I);
template <typename T> Octonion<T> asech(const Octonion<T> &, const Octonion<T> & = Octonion<T>::I);
template <typename T> Octonion<T> acsch(const Octonion<T> &);
template <typename T> Octonion<T> acoth(const Octonion<T> &, const Octonion<T> & = Octonion<T>::I);
template <typename T> Octonion<T> bessel_J(int, const Octonion<T> &);
template <typename T> Octonion<T> floor(const Octonion<T> &);
template <typename T> Octonion<T> ceil(const Octonion<T> &);
template <typename T> Octonion<T> horner(const Octonion<T> &, T *, unsigned int);
template <typename T> Octonion<T> horner(const Octonion<T> &, T *, T *, unsigned int);

template <typename T = double>
class NumberSystems_API Octonion
{
private:
    T r, i, j, k, u1, i1, j1, k1;

    inline static Octonion multiplier(T, T, const Octonion &);
    inline static Octonion make_inf(T, T);
    inline static Octonion make_i(T, T);

public:
    const static Octonion ZERO;
    const static Octonion ONE;
    const static Octonion I;
    const static Octonion J;
    const static Octonion K;
    const static Octonion U1;
    const static Octonion I1;
    const static Octonion J1;
    const static Octonion K1;

    const static Octonion UNITS[8];

    /******************************************
     * Constructor and Copy Constructor
     ******************************************/

    Octonion(T, T, T, T, T, T, T, T);
    Octonion(T = 0.0);

    /******************************************
     * Assignment Operator
     ******************************************/

    const Octonion & operator = (const T &);

    /******************************************
     * Mutating Arithmetic Operators
     ******************************************/

    Octonion & operator += (const Octonion &);
    Octonion & operator -= (const Octonion &);
    Octonion & operator *= (const Octonion &);
    Octonion & operator /= (const Octonion &);

    Octonion & operator += (T);
    Octonion & operator -= (T);
    Octonion & operator *= (T);
    Octonion & operator /= (T);

    Octonion & operator ++();
    Octonion operator ++(int);
    Octonion & operator --();
    Octonion operator --(int);

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
    T csgn() const;
    T abs() const;
    T norm() const;
    T abs_imag() const;
    T norm_imag() const;
    T arg() const;

    /******************************************
     * Octonion-valued Functions
     ******************************************/

    Octonion imag() const;
    Octonion conj() const;
    Octonion operator * () const;
    Octonion signum() const;
    Octonion sqr() const;
    Octonion sqrt() const;
    Octonion rotate(const Octonion &) const;

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

    Octonion exp() const;
    Octonion log() const;
    Octonion log10() const;
    Octonion pow(const Octonion &) const;
    Octonion pow(T) const;
    Octonion inverse() const;

    /******************************************
     * Trigonometric and Hyperbolic Functions
     ******************************************/

    Octonion   sin() const;   Octonion   cos() const;   Octonion   tan() const;
    Octonion   sec() const;   Octonion   csc() const;   Octonion   cot() const;
    Octonion  sinh() const;   Octonion  cosh() const;   Octonion  tanh() const;
    Octonion  sech() const;   Octonion  csch() const;   Octonion  coth() const;
    Octonion  asin(const Octonion & = I) const;
    Octonion  acos(const Octonion & = I) const;       Octonion  atan() const;
    Octonion  asec(const Octonion & = I) const;
    Octonion  acsc(const Octonion & = I) const;       Octonion  acot() const;

    Octonion asinh() const;
    Octonion acosh(const Octonion & = I) const;
    Octonion atanh(const Octonion & = I) const;

    Octonion asech(const Octonion & = I) const;
    Octonion acsch() const;
    Octonion acoth(const Octonion & = I) const;

    Octonion bessel_J(int) const;

    /******************************************
     * Integer Functions
     ******************************************/

    Octonion ceil() const;
    Octonion floor() const;

    /******************************************
     * Horner's Rule
     ******************************************/

    Octonion horner(T *, T *, unsigned int) const;
    Octonion horner(T *, unsigned int) const;

    /******************************************
     * Random Factories
     ******************************************/

    static Octonion random();
    static Octonion random_imag();
    static Octonion random_real();

    /******************************************
     * Binary Arithmetic Operators
     ******************************************/

    Octonion operator + (const Octonion &) const;
    Octonion operator + (T) const;

    Octonion operator - (const Octonion &) const;
    Octonion operator - (T) const;

    Octonion operator * (const Octonion &) const;
    Octonion operator * (T) const;

    Octonion operator / (const Octonion &) const;
    Octonion operator / (T) const;

    /******************************************
     * Unary Arithmetic Operators
     ******************************************/

    Octonion operator - () const;

    /******************************************
     * Binary Boolean Operators
     ******************************************/

    bool operator == (const Octonion &) const;
    bool operator == (T) const;

    bool operator != (const Octonion &) const;
    bool operator != (T) const;
};

#endif
