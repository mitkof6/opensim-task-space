#include <iostream>

#include "INIReader.h"
#include "Settings.h"
#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

int main()
{
    try
    {
        INIReader ini = INIReader(INI_FILE);
    }
    catch (const std::exception& ex)
    {
        std::cout << "Exception: " << ex.what() << std::endl;
#if PAUSE
        system("pause");
#endif
        return 1;
    }
    catch (...)
    {
        std::cout << "Unrecognized exception " << std::endl;
#if PAUSE
        system("pause");
#endif
        return 1;
    }

#if PAUSE
    system("pause");
#endif

    return 0;
}