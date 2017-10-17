#include <iostream>

#include "INIReader.h"
#include "Settings.h"
#include "OpenSimUtil.h"
#include "CommonUtil.h"

#include "TaskSpaceComputedMuscleControl.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

void test(INIReader& ini) {
    // settings
    double t0 = ini.GetReal("TASK_SPACE_MC", "START_TIME", 0);
    double tf = ini.GetReal("TASK_SPACE_MC", "END_TIME", 0);
    string modelPath = BASE_DIR + ini.Get("TASK_SPACE_MC", "MODEL_PATH", "");
    string resultDir = BASE_DIR + ini.Get("TASK_SPACE_MC", "RESULTS_DIR", "");
    string trcFile = BASE_DIR + ini.Get("TASK_SPACE_MC", "TRC", "");
    string externalLoadsXML = BASE_DIR + ini.Get("TASK_SPACE_MC", "EXT_LOADS_XML", "");
    string externalLoadsData = BASE_DIR + ini.Get("TASK_SPACE_MC", "EXT_LOADS_DATA", "");
    string externalLoadsMot = BASE_DIR + ini.Get("TASK_SPACE_MC", "EXT_LOADS_Mot", "");
    bool filter = ini.GetBoolean("TASK_SPACE_MC", "FILTER", false);
    bool verbose = ini.GetBoolean("TASK_SPACE_MC", "VERBOSE", true);

    // perform TOMC
    TaskSpaceComputedMuscleControl tscmc(ConstraintModel::Type::AGHILI,
                                         modelPath, trcFile, t0, tf,
                                         externalLoadsXML, externalLoadsData,
                                         externalLoadsMot, "", filter, verbose);

    auto start = startClock("Task Space Computed Muscle Control run...", true);
    tscmc.run();
    stopClock(start, "Task Space Computed Muscle Control finished", true);

    tscmc.printResults("TSCMC", resultDir);
}

/**
 * Main function
 */
int main() {
    try {
        INIReader ini = INIReader(INI_FILE);
        test(ini);
    }
    catch (const std::exception& ex) {
        std::cout << "Exception: " << ex.what() << std::endl;
#if PAUSE
        system("pause");
#endif
        return 1;
    }
    catch (...) {
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
