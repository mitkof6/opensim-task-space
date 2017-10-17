#include <iostream>

#include "INIReader.h"
#include "Settings.h"
#include "OpenSimUtil.h"

#include "CommonUtil.h"
#include "MarkerPositionAnalysis.h"
#include "MarkerErrorAnalysis.h"
#include "TaskSpaceInverseKinematics.h"
#include <OpenSim/Analyses/Kinematics.h>
#include <simbody/internal/AssemblyCondition_Markers.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

ofstream optIK;
ofstream tSDIK;

//#define DEBUG

void performTSDIK(int index, std::string modelPath, std::string trcFile,
                 double startTime, double endTime, std::string resultDir,
                 std::string markersPath = "", bool verbose = false) {
    string path = modelPath + changeToString(index) + ".osim";

    TaskSpaceInverseKinematics ik(ConstraintModel::Type::AGHILI,
                                  path, trcFile, startTime, endTime,
                                  markersPath, false, verbose);

    auto start = startClock("Task Oriented Inverse Kinematics run...", true);
    ik.run();
    tSDIK << stopClock(start, "Task Oriented Inverse Kinematics finished", true)
     << endl;

    ik.printResults("TSDIK" + changeToString(index), resultDir);
}

void performOptimizationIK(int index, std::string modelPath, std::string trcFile,
                           double startTime, double endTime, std::string resultDir,
                           std::string markersPath = "", bool verbose = false) {
    string path = modelPath + changeToString(index) + ".osim";

    MarkerData markerData(trcFile);

    Model model(path);

    Kinematics* kinematics = new Kinematics(&model);
    kinematics->setInDegrees(true);
    kinematics->setRecordAccelerations(true);
    model.addAnalysis(kinematics);

    MarkerErrorAnalysis* markerError = new MarkerErrorAnalysis(&model, markerData);
    model.addAnalysis(markerError);

    MarkerPositionAnalysis* markerAnalysis = new MarkerPositionAnalysis(&model);
    model.addAnalysis(markerAnalysis);

    State s = model.initSystem();

    // start ik
    auto start = startClock("Task Oriented Inverse Kinematics run...", true);

    SimTK::Assembler ik(model.updMultibodySystem());
    //ik.setUseRMSErrorNorm(true);

    MarkerSet& ms = model.updMarkerSet();

    SimTK::Markers* markers = new SimTK::Markers();
    map<string, Markers::MarkerIx> markerToIndex;
    Array_<Markers::MarkerIx> markerIx;
    for (int i = 0; i < ms.getSize(); i++) {
        Markers::MarkerIx index = markers->addMarker(
            model.updBodySet().get(ms[i].getFrameName()).getMobilizedBodyIndex(),
            ms[i].get_location());
        markerToIndex.insert(pair<string, Markers::MarkerIx>(ms[i].getName(), index));
        markerIx.push_back(index);
    }
    markers->defineObservationOrder(markerIx);
    ik.adoptAssemblyGoal(markers);

    int firstFrame, lastFrame;
    markerData.findFrameRange(s.getTime(), s.getTime() + 0.1, firstFrame, lastFrame);
    OpenSim::MarkerFrame frame = markerData.getFrame(firstFrame);
    for (auto marker : markerToIndex) {
        int index = markerData.getMarkerIndex(marker.first);
        if (index != -1) // if marker exist in marker data
        {
            Vec3 pos = frame.getMarker(index);
            markers->moveOneObservation(
                markers->getObservationIxForMarker(marker.second), pos);
        }
    }

    ik.initialize(s);
    ik.assemble(s);

    kinematics->begin(s);
    markerAnalysis->begin(s);
    markerError->begin(s);

    for (int i = 0; i < markerData.getNumFrames(); ++i) {
        //model.realizeReport(s);

        // move observations
        const OpenSim::MarkerFrame& frame = markerData.getFrame(i);
        for (auto marker : markerToIndex) {
            int index = markerData.getMarkerIndex(marker.first);
            if (index != -1) // if marker exist in marker data
            {
                Vec3 pos = frame.getMarker(index);
                //cout << pos << endl;
                markers->moveOneObservation(
                    markers->getObservationIxForMarker(marker.second), pos);
            }
        }

        // track
        s.updTime() = frame.getFrameTime();
        ik.track(s.getTime());

        // report
#ifdef DEBUG
        std::cout << "Frame: " << i << " (t=" << s.getTime() << ")\n";
        std::cout << "Error: " << ik.calcCurrentErrorNorm() << "\n";
        std::flush(std::cout);
#endif

        // store
        //model.realizeReport(ik.getInternalState());
        kinematics->step(ik.getInternalState(), i);
        markerAnalysis->step(ik.getInternalState(), i);
        markerError->step(ik.getInternalState(), i);
    }

    optIK << stopClock(start, "Task Oriented Inverse Kinematics finished", true)
          << endl;

    kinematics->end(s);
    markerAnalysis->end(s);
    markerError->end(s);

    // print results
    kinematics->printResults("IK" + changeToString(index), resultDir);
    markerAnalysis->printResults("IK" + changeToString(index), resultDir);
    markerError->printResults("IK" + changeToString(index), resultDir);
}

void test(INIReader& ini) {
    // settings
    double t0 = ini.GetReal("IK_BATCH", "START_TIME", 0);
    double tf = ini.GetReal("IK_BATCH", "END_TIME", 0);
    string modelPath = BASE_DIR + ini.Get("IK_BATCH", "MODEL_PATH", "");
    string resultDir = BASE_DIR + ini.Get("IK_BATCH", "RESULTS_DIR", "");
    string trcFile = BASE_DIR + ini.Get("IK_BATCH", "TRC", "");

    optIK = ofstream(resultDir + "/optIK.txt", ofstream::out);
    tSDIK = ofstream(resultDir + "/tSDIK.txt", ofstream::out);

    //FILE_REDIRECTION_BEGIN(resultDir + "/log.txt");

    for (int i = 1; i < 6; i++) {
        performOptimizationIK(i, modelPath, trcFile, t0, tf, resultDir, "", false);
        performTSDIK(i, modelPath, trcFile, t0, tf, resultDir, "", false);
    }

    //FILE_REDIRECTION_END();

    optIK.close();
    tSDIK.close();
}

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
