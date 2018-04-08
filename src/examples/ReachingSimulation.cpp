#include <iostream>

#include "INIReader.h"
#include "Settings.h"
#include "OpenSimUtil.h"

#include "StationPointAnalysis.h"
#include "TaskBasedForce.h"
#include "TaskPrimitive.h"
#include "TaskTrackingController.h"
#include "TaskDynamics.h"
#include "TaskBasedForce.h"
#include "MuscleBasedController.h"
#include "MuscleOptimization.h"
#include "MuscleOptimizationTarget.h"

#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/ForceReporter.h>
#include <OpenSim/Analyses/JointReaction.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>
#include <OpenSim/Analyses/InverseDynamics.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#define USE_POSITION_TASK 0
OpenSim::Manager* manager;

Vec3 trajectoryGenerator(double t);

void testTaskSpace(INIReader& ini) {
    // ini
    std::string indexBodyName = ini.Get("REACHING", "BODY_NAME", "");
    Vec3 indexOffset = Vec3(
        ini.GetReal("REACHING", "INDEX_OFFSET_X", 0),
        ini.GetReal("REACHING", "INDEX_OFFSET_Y", 0),
        ini.GetReal("REACHING", "INDEX_OFFSET_Z", 0));
    bool useMuscles = ini.GetBoolean("REACHING", "USE_MUSCLES", false);

    double dt = ini.GetReal("REACHING", "INTEGRATION_STEP", 0);
    double t_start = ini.GetReal("REACHING", "START_TIME", 0);
    double t_end = ini.GetReal("REACHING", "END_TIME", 1);
    double accur = ini.GetReal("REACHING", "ACCURACY", 1e-10);
    std::string prefix = ini.Get("REACHING", "PREFIX", "");
    auto constraintModel = static_cast<ConstraintModel::Type>(
        ini.GetInteger("REACHING", "CONSTRAINT_MODEL", 1));

    // model build
    Model model(BASE_DIR + ini.Get("REACHING", "MODEL_PATH", ""));

    Kinematics* kinematics = new Kinematics(&model);
    model.addAnalysis(kinematics);

    StationPointAnalysis* endEff = new StationPointAnalysis
    (&model, indexBodyName, indexOffset);
    model.addAnalysis(endEff);

    ForceReporter* forces = new ForceReporter(&model);
    model.addAnalysis(forces);

    TaskDynamics* taskDynamics = new TaskDynamicsPrioritization(constraintModel);
    model.addComponent(taskDynamics);

#if USE_POSITION_TASK == 1
    PositionTask* hand = new
        PositionTask(indexBodyName, indexOffset,
                     new PDController(500, 20));
    taskDynamics->addTask(hand, "hand", "");
#else
    SpatialTask* hand = new
        SpatialTask(indexBodyName, indexOffset,
                    new PDController(500, 30));
    taskDynamics->addTask(hand, "hand", "");
#endif

    MuscleOptimization muscleOptimization = MuscleOptimization(
        &model,
        new ApproximationOptimizationTarget(true), true);
    MuscleBasedController* controller;
    TaskBasedForce* forceController;
    if (useMuscles) {
        controller =
            new MuscleBasedController(model, taskDynamics, &muscleOptimization);
        model.addController(controller);
    } else {
        forceController = new TaskBasedForce(taskDynamics);
        model.addForce(forceController);
    }

    // configure the visualizer.
    model.setUseVisualizer(true);

    // simulation
    State& s = model.initSystem();
    setUseMuscles(s, model, useMuscles);
    if (useMuscles) {
        muscleOptimization.initializeFromState(s);
    }

    //model.updMatterSubsystem().setShowDefaultGeometry(true);
    //auto visualizer = &model.updVisualizer().updSimbodyVisualizer();
    //visualizer->setBackgroundColor(White);

    // pre-simulation
    model.getMultibodySystem().realize(s, SimTK::Stage::Position);
    Vector hand0 = hand->x(s);

    //setup integrator
    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    //integrator.setAccuracy(accur);

    //manager
    manager = new Manager(model, integrator);
    manager->setInitialTime(t_start);

    for (unsigned int i = 1; i*dt <= t_end; ++i) {
        model.realizeVelocity(s);

        double t0 = s.getTime();
        double tf = i * dt;
        std::cout << "Integrate from: " << t0 << " to: " << tf << std::endl;

#if USE_POSITION_TASK == 1
        Vector hp = hand0(0, 3) + Vector(trajectoryGenerator(i*dt));
        hand->setGoal(s, hp);
#else
        Vector x = hand->x(s);
        Vector hp = hand0(3, 3) + Vector(trajectoryGenerator(i*dt));
        Vector h(6, 0.0);
        h[0] = x[0];
        h[1] = x[1];
        h[2] = x[2];
        h[3] = hp[0];
        h[4] = hp[1];
        h[5] = hp[2];
        hand->setGoal(s, h);
#endif

        manager->setFinalTime(tf);
        manager->integrate(s);
        manager->setInitialTime(tf);
        //// new version v4.0.0
        //manager->integrate(s, tf);
    }

    //store results
    kinematics->printResults(prefix,
                             BASE_DIR + ini.Get("REACHING", "RESULTS_DIR", ""));
    endEff->printResults(prefix,
                         BASE_DIR + ini.Get("REACHING", "RESULTS_DIR", ""));
    forces->printResults(prefix,
                         BASE_DIR + ini.Get("REACHING", "RESULTS_DIR", ""));
    if (useMuscles) {
        controller->printResults(prefix,
                                 BASE_DIR + ini.Get("REACHING", "RESULTS_DIR", ""));
    } else {
        forceController->printResults(prefix,
                                      BASE_DIR + ini.Get("REACHING", "RESULTS_DIR", ""));
    }
    manager->getStateStorage().print(BASE_DIR + ini.Get("REACHING", "RESULTS_DIR", "")
                                     + prefix + "_State.sto");
}

Vec3 trajectoryGenerator(double t) {
    double theta = Pi * t;
    double r = 0.07 * sin(2 * theta);
    double y = r * sin(theta);
    double z = r * cos(theta);
    return Vec3(0, y, z);
}

void performPerformID(INIReader& ini) {
    string prefix = ini.Get("REACHING", "PREFIX", "");
    double t_start = ini.GetReal("REACHING", "START_TIME", 0);
    double t_end = ini.GetReal("REACHING", "END_TIME", 1);

    InverseDynamicsTool tool;
    tool.setName(prefix);
    tool.setModelFileName(BASE_DIR + ini.Get("REACHING", "MODEL_PATH", ""));
    tool.setCoordinatesFileName(BASE_DIR + ini.Get("REACHING", "RESULTS_DIR", "")
                                + prefix + "_Kinematics_q.sto");
    tool.setLowpassCutoffFrequency(5);
    tool.setStartTime(t_start);
    tool.setEndTime(t_end);

    tool.setOutputGenForceFileName(prefix + "_ID.sto");
    tool.setResultsDir(BASE_DIR + ini.Get("REACHING", "RESULTS_DIR", ""));
    tool.run();
}

void performReactionAnalysis(INIReader& ini) {
    string prefix = ini.Get("REACHING", "PREFIX", "");
    Model temp(BASE_DIR + ini.Get("REACHING", "MODEL_PATH", ""));
    double t_start = ini.GetReal("REACHING", "START_TIME", 0);
    double t_end = ini.GetReal("REACHING", "END_TIME", 1);

    JointReaction* reaction = new JointReaction(&temp);
    reaction->setName("ReactionAnalysis");
    temp.addAnalysis(reaction);

    AnalyzeTool tool;
    tool.setModel(temp);
    tool.setName(prefix);
    tool.setStatesStorage(manager->getStateStorage());
    tool.setStartTime(t_start);
    tool.setFinalTime(t_end);
    tool.run();

    tool.printResults(prefix + "2",
                      BASE_DIR + ini.Get("REACHING", "RESULTS_DIR", ""));
}

int main() {
    try {
        INIReader ini = INIReader(INI_FILE);
        testTaskSpace(ini);
        performPerformID(ini);
        performReactionAnalysis(ini);
    } catch (const std::exception& ex) {
        std::cout << "Exception: " << ex.what() << std::endl;
#if PAUSE
        system("pause");
#endif
        return 1;
    } catch (...) {
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