#include <iostream>

#include "INIReader.h"
#include "Settings.h"
#include "OpenSimUtil.h"
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/BallJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PointOnLineConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/PointConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Analyses/JointReaction.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/ForceReporter.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>
#include <OpenSim/Analyses/InverseDynamics.h>
#include "TaskDynamics.h"
#include "StationPointAnalysis.h"
#include "ConstraintAnalysis.h"
#include "TaskPrimitive.h"
#include "TaskBasedForce.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

string prefix;
bool isConstrained;
ConstraintModel::Type constraintModel;

class KinematicChainModel {
public:
    KinematicChainModel(INIReader& ini) : ini(ini) {
        constructModel();
    }
    ~KinematicChainModel() {};

    void visualizeModel() {
        visualizer->drawFrameNow(state);
        cout << "Press key to terminate visualizer" << endl;
        system("pause");
    }

    void simulateModel() {
        RungeKuttaMersonIntegrator integrator(model.getSystem());
        integrator.setConstraintTolerance(1e-9);
        manager = new Manager(model, integrator);
        manager->setInitialTime(0);

        // pre-simulation
        model.getMultibodySystem().realize(state, SimTK::Stage::Position);
        Vector task0 = task->x(state);

        double dt = 0.01;
        double t_end = ini.GetReal("CLOSED_KINEMATIC_CHAIN", "END_TIME", 1);
        for (unsigned int i = 1; i*dt <= t_end; ++i) {
            model.realizeVelocity(state);

            double t0 = state.getTime();
            double tf = i * dt;
            std::cout << "Integrate from: " << t0 << " to: " << tf << std::endl;

            Vector hp = task0(0, 3) + Vector(trajectoryGenerator(i*dt));
            task->setGoal(state, hp);

            manager->setFinalTime(tf);
            manager->integrate(state);
            manager->setInitialTime(tf);
        }
    }

    void printResults() {
        kinematics->printResults(prefix,
            BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", ""));
        reaction->printResults(prefix,
            BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", ""));
        forces->printResults(prefix,
            BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", ""));
        forceController->printResults(prefix,
            BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", ""));
        bodyKinematics->printResults(prefix,
            BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", ""));
        station->printResults(prefix,
            BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", ""));
        constraints->printResults(prefix,
            BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", ""));
        manager->getStateStorage().print(
            BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", "")
            + prefix + "_State.sto");
    }

    Vec3 trajectoryGenerator(double t) {
        //return Vec3(0, 1.0 * sin(2 * Pi * t), 0);
        return Vec3(0, 0, 1.54 * sin(2 * Pi * t));
    }

    void performPerformID() {
        InverseDynamicsTool tool;
        tool.setName(prefix);
        tool.setModelFileName(
        BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "MODEL_PATH", ""));
        tool.setCoordinatesFileName(
        BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", "")
            + prefix + "_Kinematics_q.sto");
        tool.setLowpassCutoffFrequency(5);
        tool.setStartTime(manager->getStateStorage().getFirstTime());
        tool.setEndTime(manager->getStateStorage().getLastTime());

        tool.setOutputGenForceFileName(prefix + "_ID.sto");
        tool.setResultsDir(
        BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", ""));
        tool.run();
    }

    void performReactionAnalysis() {
        Model temp(
        BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "MODEL_PATH", ""));

        JointReaction* reaction = new JointReaction(&model);
        reaction->setName("ReactionAnalysis");
        temp.addAnalysis(reaction);

        ConstraintAnalysis* constraints = new ConstraintAnalysis(&model);
        model.addAnalysis(constraints);

        AnalyzeTool tool;
        tool.setModel(temp);
        tool.setName(prefix);
        tool.setStatesStorage(manager->getStateStorage());
        tool.setStartTime(manager->getStateStorage().getFirstTime());
        tool.setFinalTime(manager->getStateStorage().getLastTime());
        tool.run();

        tool.printResults(prefix + "2",
            BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", ""));
        constraints->printResults(prefix + "2",
            BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "RESULTS_DIR", ""));
    }

private:

    void constructImplicitJointModel() {
        auto& ground = model.getGround();
        double body3_length = 1;

        // body1
        double body1_m = 1, body1_length = 1, body1_radius = 0.03;
        Vec3 body1_com = Vec3(0);
        Inertia body1_I = body1_m * Inertia::cylinderAlongY(
            body1_radius, body1_length);
        auto body1_body = new OpenSim::Body("body1", body1_m, body1_com, body1_I);
        auto body1_geom = new OpenSim::Cylinder(body1_radius, body1_length / 2);
        body1_geom->setName("body1_cylinder");
        body1_body->attachGeometry(*body1_geom);
        Vec3 body1_distal(0, -body1_length / 2, 0);
        Vec3 body1_proximal(0, body1_length / 2, 0);
        auto ground_body1 = new OpenSim::PinJoint("ground_body1", ground,
            Vec3(0), Vec3(0), *body1_body, body1_distal, Vec3(0));
        ground_body1->upd_CoordinateSet()[0]
            .setDefaultValue(SimTK::convertDegreesToRadians(q1));
        model.addBody(body1_body);
        model.addJoint(ground_body1);

        // body2
        double body2_m = 1, body2_length = 1, body2_radius = 0.03;
        Vec3 body2_com = Vec3(0);
        Inertia body2_I = body2_m * Inertia::cylinderAlongY(
            body2_radius, body2_length);
        auto body2_body = new OpenSim::Body("body2", body2_m, body2_com, body2_I);
        auto body2_geom = new OpenSim::Cylinder(body2_radius, body2_length / 2);
        body2_geom->setName("body2_cylinder");
        body2_body->attachGeometry(*body2_geom);
        Vec3 body2_distal(0, -body2_length / 2, 0);
        Vec3 body2_proximal(0, body2_length / 2, 0);
        auto ground_body2 = new OpenSim::PinJoint("body1_body2", ground,
            Vec3(body3_length, 0, 0), Vec3(0), *body2_body, body2_distal, Vec3(0));
        ground_body2->upd_CoordinateSet()[0]
            .setDefaultValue(SimTK::convertDegreesToRadians(q1));
        model.addBody(body2_body);
        model.addJoint(ground_body2);

        // body3
        double body3_m = 1, body3_radius = 0.03;
        Vec3 body3_com = Vec3(0);
        Inertia body3_I = body3_m * Inertia::cylinderAlongY(
            body3_radius, body3_length);
        auto body3_body = new OpenSim::Body("body3", body3_m, body3_com, body3_I);
        auto body3_geom = new OpenSim::Cylinder(body3_radius, body3_length / 2);
        body3_geom->setName("body3_cylinder");
        body3_body->attachGeometry(*body3_geom);
        Vec3 body3_distal(0, -body3_length / 2, 0);
        Vec3 body3_proximal(0, body3_length / 2, 0);
        auto body1_body3 = new OpenSim::PinJoint("body1_body3", *body1_body,
            body1_proximal, Vec3(0),
            *body3_body, body3_distal, Vec3(0));
        auto body2_body3 = new OpenSim::PinJoint("body2_body3", *body2_body,
            body2_proximal, Vec3(0),
            *body3_body, body3_proximal, Vec3(0));
        /*body1_body3->upd_CoordinateSet()[0]
            .setDefaultValue(SimTK::convertDegreesToRadians(-90 - q1));*/
        model.addBody(body3_body);
        model.addJoint(body1_body3);
        model.addJoint(body2_body3);
    }

    void constructConstraintModel() {
        auto& ground = model.getGround();
        double body3_length = 1;

        // body1
        double body1_m = 1, body1_length = 1, body1_radius = 0.03;
        Vec3 body1_com = Vec3(0);
        Inertia body1_I = body1_m * Inertia::cylinderAlongY(
            body1_radius, body1_length);
        auto body1_body = new OpenSim::Body("body1", body1_m, body1_com, body1_I);
        auto body1_geom = new OpenSim::Cylinder(body1_radius, body1_length / 2);
        body1_geom->setName("body1_cylinder");
        body1_body->attachGeometry(*body1_geom);
        Vec3 body1_distal(0, -body1_length / 2, 0);
        Vec3 body1_proximal(0, body1_length / 2, 0);
        auto ground_body1 = new OpenSim::PinJoint("ground_body1", ground,
            Vec3(0), Vec3(0), *body1_body, body1_distal, Vec3(0));
        ground_body1->upd_CoordinateSet()[0]
            .setDefaultValue(SimTK::convertDegreesToRadians(q1));
        model.addBody(body1_body);
        model.addJoint(ground_body1);

        // body2
        double body2_m = 1, body2_length = 1, body2_radius = 0.03;
        Vec3 body2_com = Vec3(0);
        Inertia body2_I = body2_m * Inertia::cylinderAlongY(
            body2_radius, body2_length);
        auto body2_body = new OpenSim::Body("body2", body2_m, body2_com, body2_I);
        auto body2_geom = new OpenSim::Cylinder(body2_radius, body2_length / 2);
        body2_geom->setName("body2_cylinder");
        body2_body->attachGeometry(*body2_geom);
        Vec3 body2_distal(0, -body2_length / 2, 0);
        Vec3 body2_proximal(0, body2_length / 2, 0);
        auto ground_body2 = new OpenSim::PinJoint("body1_body2", ground,
            Vec3(body3_length, 0, 0), Vec3(0), *body2_body, body2_distal, Vec3(0));
        ground_body2->upd_CoordinateSet()[0]
            .setDefaultValue(SimTK::convertDegreesToRadians(q1));
        model.addBody(body2_body);
        model.addJoint(ground_body2);

        // body3
        double body3_m = 1, body3_radius = 0.03;
        Vec3 body3_com = Vec3(0);
        Inertia body3_I = body3_m * Inertia::cylinderAlongY(
            body3_radius, body3_length);
        auto body3_body = new OpenSim::Body("body3", body3_m, body3_com, body3_I);
        auto body3_geom = new OpenSim::Cylinder(body3_radius, body3_length / 2);
        body3_geom->setName("body3_cylinder");
        body3_body->attachGeometry(*body3_geom);
        Vec3 body3_distal(0, -body3_length / 2, 0);
        Vec3 body3_proximal(0, body3_length / 2, 0);
        auto ground_body3 = new OpenSim::FreeJoint("body1_body3", ground,
            Vec3(0), Vec3(0), *body3_body, body3_distal, Vec3(0));
        model.addBody(body3_body);
        model.addJoint(ground_body3);

        // connect two free bodies

        auto pointConstraint1 = new PointConstraint(*body1_body, body1_proximal,
            *body3_body, body3_distal);
        model.addConstraint(pointConstraint1);

        auto pointConstraint2 = new PointConstraint(*body2_body, body2_proximal,
            *body3_body, body3_proximal);
        model.addConstraint(pointConstraint2);
    }
    void constructModel() {
        if (isConstrained) {
            constructConstraintModel();
        }
        else {
            constructImplicitJointModel();
        }
        model.print(
        BASE_DIR + ini.Get("CLOSED_KINEMATIC_CHAIN", "MODEL_PATH", ""));

        // analysis
        reaction = new JointReaction(&model);
        reaction->setName("ReactionAnalysis");
        model.addAnalysis(reaction);

        kinematics = new Kinematics(&model);
        kinematics->setInDegrees(false);
        model.addAnalysis(kinematics);

        bodyKinematics = new BodyKinematics(&model);
        bodyKinematics->setInDegrees(false);
        model.addAnalysis(bodyKinematics);

        station = new StationPointAnalysis(&model, taskBody, Vec3(0));
        model.addAnalysis(station);

        forces = new ForceReporter(&model);
        model.addAnalysis(forces);

        constraints = new ConstraintAnalysis(&model);
        model.addAnalysis(constraints);

        // task controller
        TaskDynamics* taskDynamics = new TaskDynamicsPrioritization(constraintModel);
        model.addComponent(taskDynamics);

        task = new OrientationTask(taskBody, Vec3(0),
            new PDController(500, 30));
        taskDynamics->addTask(task, "task", "");

        forceController = new TaskBasedForce(taskDynamics);
        model.addForce(forceController);

        // build model
        model.setUseVisualizer(true);
        model.buildSystem();
        state = model.initializeState();

        if (!isConstrained) {
            model.updCoordinateSet()[0].setValue(state,
                         SimTK::convertDegreesToRadians(q1));
            model.updCoordinateSet()[1].setValue(state,
                         SimTK::convertDegreesToRadians(q1));
        }

        cout << "Constraints: " << state.getNMultipliers() << endl;

        // configure the visualizer
        model.updMatterSubsystem().setShowDefaultGeometry(true);
        visualizer = &model.updVisualizer().updSimbodyVisualizer();
        //visualizer->setBackgroundColor(White);
    }
private:
    INIReader& ini;
    Model model;
    Manager* manager;
    State state;
    Visualizer* visualizer;
    Kinematics* kinematics;
    JointReaction* reaction;
    ForceReporter* forces;
    BodyKinematics* bodyKinematics;
    StationPointAnalysis* station;
    ConstraintAnalysis* constraints;
    TaskPrimitive* task;
    TaskBasedForce* forceController;
    string taskBody = "body1";

    const double q1 = -0.0;
};

void batch(INIReader& ini) {
    KinematicChainModel ch(ini);
    //ch.visualizeModel();
    ch.simulateModel();
    ch.printResults();
    ch.performReactionAnalysis();
    ch.performPerformID();
}

int main() {
    try {
        INIReader ini = INIReader(INI_FILE);

        prefix = "CONS_AG";
        constraintModel = ConstraintModel::Type::AGHILI;
        isConstrained = true;
        batch(ini);

        prefix = "CONS_DS";
        constraintModel = ConstraintModel::Type::DESAPIO;
        isConstrained = true;
        batch(ini);
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
