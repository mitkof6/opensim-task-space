#include "CommonUtil.h"

#include <ctime>

using namespace std;

/* Time ***********************************************************************************/

std::chrono::high_resolution_clock::time_point startClock(std::string msg,
    bool useMessage) {
    if (useMessage) {
        std::cout << std::endl;
        std::cout << "================================================================\n";
        std::cout << msg << std::endl;
        std::cout << "\n================================================================\n";
        std::cout.flush();
    }

    return chrono::high_resolution_clock::now();
}

double stopClock(std::chrono::high_resolution_clock::time_point t1,
    std::string msg, bool useMessage) {
    chrono::high_resolution_clock::time_point t2 =
        chrono::high_resolution_clock::now();
    chrono::duration<double> duration =
        chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    if (useMessage) {
        std::cout << "================================================================\n";
        std::cout << msg << std::endl;
        std::cout << "================================================================\n";
        std::cout << "Elapsed time = " << duration.count() << " seconds.\n";
        std::cout << "================================================================\n";
        std::cout << "================================================================\n";
        std::cout.flush();
    }

    return duration.count();
}

/* Math ***********************************************************************************/

double linearMapping(double x, double a, double b) {
    return a * x + b;
}

double linearTransform(double x, double a, double b) {
    return a + (b - a) * x;
}

double sigm(double x, double x0, double a, double b) {
    return a / (1 + exp(-b * (x - x0)));
}

/* Misk ***********************************************************************************/

bool systemCall(std::string command) {
    int status = system(command.c_str());
    if (status != 0) {
        cout << "Command: " << command << " failed" << endl;
        return true;
    }
    else {
        return false;
    }
}
