#ifndef COMMON_UTIL_H
#define COMMON_UTIL_H

#include "internal/UtilDLL.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <chrono>
#include <cmath>

/* Macros ********************************************************************/
/**
* Redirects cout to a file, at the end FILE_REDIRECTION_END() should be called
*/
#define FILE_REDIRECTION_BEGIN(File)\
std::cout << "Cout is redirected to: " << File << std::endl; \
std::ofstream out(File); \
std::streambuf *coutbuf = std::cout.rdbuf(); \
std::cout.rdbuf(out.rdbuf());

#define FILE_REDIRECTION_END() \
std::cout.rdbuf(coutbuf);

/* Time **********************************************************************/

/**
* Starts measuring time
*/
Util_API std::chrono::high_resolution_clock::time_point startClock(
    std::string msg = "", bool useMessage = false);

/**
* Stops measuring time
*/
Util_API double stopClock(std::chrono::high_resolution_clock::time_point startTime,
    std::string msg = "", bool useMessage = false);

/**
* Linear mapping of x $y = a*x + b$
*/
Util_API double linearMapping(double x, double a, double b);

/**
* Transform x in [0, 1] to y in [a, b]
*/
Util_API double linearTransform(double x, double a, double b);

/**
* y = a / (1 + exp(-b(x-x0)))
*/
Util_API double sigm(double x, double x0, double a, double b);

/* Math **********************************************************************/

template <typename T = double>
std::vector<T> linspace(T a, T b, size_t N) {
    T h = (b - a) / static_cast<T>(N - 1);
    std::vector<T> xs(N);
    typename std::vector<T>::iterator x;
    T val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
        *x = val;
    }

    return xs;
}

/* Misc **********************************************************************/

Util_API bool systemCall(std::string command);

template<typename T>
T rms(const T* r, int size) {
    T sum = 0;
    for (int i = 0; i < size; i++) {
        sum += pow(r[i], 2);
    }
    return sqrt(sum / size);
}

template<typename T>
std::vector<T> decompose(std::string str, char token) {
    std::vector<T> data;
    std::stringstream ss(str);

    T d;
    while (ss >> d) {
        data.push_back(d);
        if (ss.peek() == token) {
            ss.ignore();
        }
    }
    return data;
}

/**
* To string
*/
template<typename T>
std::string changeToString(const T& value,
    int precision = std::numeric_limits<int>::infinity()) {
    std::ostringstream oss;
    if (precision != std::numeric_limits<int>::infinity()) {
        oss << std::setprecision(precision);
    }
    oss << value;
    return oss.str();
}

/**
* Safe delete of a pointer
*/
template<class T>
void safeDelete(T*& pVal) {
    delete pVal;
    pVal = NULL;
}

/**
* Safe delete of an array of pointer
*/
template<class T>
void safeDeleteaArray(T*& pVal) {
    delete[] pVal;
    pVal = NULL;
}

#endif
