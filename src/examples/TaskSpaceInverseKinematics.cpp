#include <iostream>

#include "INIReader.h"
#include "Settings.h"
#include "OpenSimUtil.h"
#include "CommonUtil.h"

#include "TaskSpaceInverseKinematics.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

void test(INIReader& ini) {
    // settings
    double t0 = ini.GetReal("TASK_SPACE_INVERSE_KINEMATICS", "START_TIME", 0);
    double tf = ini.GetReal("TASK_SPACE_INVERSE_KINEMATICS", "END_TIME", 0);
    string modelPath = BASE_DIR + ini.Get("TASK_SPACE_INVERSE_KINEMATICS", "MODEL_PATH", "");
    string resultDir = BASE_DIR + ini.Get("TASK_SPACE_INVERSE_KINEMATICS", "RESULTS_DIR", "");
    string trcFile = BASE_DIR + ini.Get("TASK_SPACE_INVERSE_KINEMATICS", "TRC", "");
    bool filter = ini.GetBoolean("TASK_SPACE_INVERSE_KINEMATICS", "FILTER", false);
    bool verbose = ini.GetBoolean("TASK_SPACE_INVERSE_KINEMATICS", "VERBOSE", true);
    auto constraintModel = static_cast<ConstraintModel::Type>(
        ini.GetInteger("TASK_SPACE_INVERSE_KINEMATICS", "CONSTRAINT_MODEL", 1));

    //FILE_REDIRECTION_BEGIN(resultDir + "out.txt");

    // perform IK
    TaskSpaceInverseKinematics tsik(constraintModel,
                                    modelPath, trcFile, t0, tf, "",
                                    filter, verbose);

    auto start = startClock("Task Space Inverse Kinematics run...", true);
    tsik.run();
    stopClock(start, "Task Space Inverse Kinematics finished", true);

    tsik.printResults("TSDIK", resultDir);

    //FILE_REDIRECTION_END()
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
