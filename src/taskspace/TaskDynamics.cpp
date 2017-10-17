#include "TaskDynamics.h"

#include <algorithm>

#include <OpenSim/Simulation/Model/Model.h>

#include "AbstractPrimitive.h"
#include "DynamicCompensator.h"
#include "TaskPriorityGraph.h"
#include "ConstraintModel.h"
#include "OpenSimUtil.h"
#include "CommonUtil.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

/* TaskDynamics ***************************************************************/

// selection matrix is under development
#define USE_SELECTION 0

const std::string TaskDynamics::CACHE_B = "cache-B";

TaskDynamics::TaskDynamics(ConstraintModel::Type type) {
    compensator = new DynamicCompensator();
    taskGraph = new TaskPriorityGraph();

    if (type == ConstraintModel::Type::UNCONSTRAINED) {
        constraintModel = new UnconstraintModel();
    }
    else if (type == ConstraintModel::Type::AGHILI) {
        constraintModel = new AghiliConstraintModel();
    }
    else if (type == ConstraintModel::Type::DESAPIO) {
        constraintModel = new DeSapioConstraintModel();
    }
    else if (type == ConstraintModel::Type::DESAPIO_AGHILI) {
        constraintModel = new DeSapioAghiliConstraintModel();
    }
    else {
        throw OpenSim::Exception(string("Constrained model of type: ")
                                 + changeToString((int)type) +
                                 " is not supported", __FILE__, __LINE__);
    }
}

TaskDynamics::~TaskDynamics() {
}

void TaskDynamics::addTask(AbstractPrimitive* taskPrimitive, string name,
                           string parent) {
    if (taskPrimitive == NULL) {
        throw OpenSim::Exception("Provided task is null", __FILE__, __LINE__);
    }

    taskGraph->addTask(taskPrimitive, name, parent);
}

const DynamicCompensator& TaskDynamics::getDynamicCompensator() const {
    return *compensator;
}

DynamicCompensator& TaskDynamics::updDynamicCompensator() {
    return *compensator;
}

void TaskDynamics::printResults(std::string prefix, std::string dir) {
    analytics.print(dir + "/" + prefix + "_TaskAnalytics.sto");
}

void TaskDynamics::printPriorityGraph() const {
    taskGraph->print();
}

Matrix TaskDynamics::B(const SimTK::State& s) const {
    PRE_PROCESS_CACHE(s, CACHE_B, value, Matrix);

    value.resize(s.getNU(), s.getNU());
    value = 0;
    const auto& cs = _model->getCoordinateSet();
    if (cs.getSize() != s.getNU()) {
        throw OpenSim::Exception("Coordinate set and NU does not agree",
                                 __FILE__, __LINE__);
    }
    for (int i = 0; i < cs.getSize(); i++) {
        if (cs[i].isConstrained(s)) {
            value[i][i] = 0;
        }
        else {
            //if (cs.getSize() != 2 && i != 2 && i != 8)
	    //continue; // this was for a demo
            value[i][i] = 1;
            // TODO in case of pelvis and similar joints
        }
    }

    //cout << value << endl;

    POST_PROCESS_CACHE(s, CACHE_B, value, Matrix);
}

void TaskDynamics::appendAnalytics(const SimTK::State& s,
                                   const SimTK::Vector& taskTorques,
                                   const SimTK::Vector& constraintTorques,
                                   const SimTK::Vector& nullspaceTorques,
                                   const SimTK::Vector& lambda) {
    int n = _model->getNumCoordinates();
    int nb = _model->getNumBodies();
    int m = s.getNMultipliers();

    SimTK_ASSERT(n == taskTorques.size(), "Num of coordinates is not the same");
    SimTK_ASSERT(m == lambda.size(), "Num of Lagrange multipliers is not the same");

    Vector data(3 * n + m + 6 * nb + 5, 0.0);

    data(0, n) = taskTorques;
    data(n, n) = constraintTorques;
    data(2 * n, n) = nullspaceTorques;
    data(3 * n, m) = lambda;

    // TODO should improve the conversation of constraint forces in general
    //Vector_<SpatialVec> consBodyFrc;
    //Vector consMobFrc;
    //_model->getMatterSubsystem().calcConstraintForcesFromMultipliers(
    //    s, -lambda, consBodyFrc, consMobFrc);

    //consBodyFrc[0] = -consBodyFrc[0]; // Ground is "welded" at origin
    //for (int i = 1; i < consBodyFrc.size(); i++) {
    //    auto body = _model->getBodySet()[i - 1].getMobilizedBody();
    //    auto p_BM = body.getOutboardFrame(s).p();
    //    const Rotation& R_GB = body.getBodyTransform(s).R();
    //    consBodyFrc[i] = shiftForceFromTo(consBodyFrc[i],
    //        Vec3(0), body.getBodyTransform(s).p() + R_GB*p_BM);
    //    consBodyFrc[i][0] = Vec3(0);
    //}
    //consBodyFrc[1] = consBodyFrc[0];
    //Vector temp(6 * consBodyFrc.size(), 0.0);
    //for (int i = 1; i < consBodyFrc.size(); i++) {
    //    temp(6 * (i - 1), 3) = Vector(consBodyFrc[i][0]);
    //    temp(6 * (i - 1) + 3, 3) = Vector(consBodyFrc[i][1]);
    //}
    //data(3 * n + m, 6 * nb) = temp(0, 6 * consBodyFrc.size());

    data[data.size() - 5] = taskTorques.norm();
    data[data.size() - 4] = constraintTorques.norm();
    data[data.size() - 3] = nullspaceTorques.norm();
    data[data.size() - 2] =
        data[data.size() - 3] +
        data[data.size() - 4] +
        data[data.size() - 5];

    // kinetic energy TODO
    /*Vector qDot = s.getQDot();
      MATRIX_SIZE(q, qDot);
      MATRIX_SIZE(mc, constraintModel->Mc(s));
      data[data.size() - 1] = ~qDot * (constraintModel->Mc(s) * qDot) / 2.0;*/
    data[data.size() - 1] = 0;

    analytics.append(s.getTime(), data.size(), &data[0], true);
}

void TaskDynamics::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    Matrix mat;
    addCacheVariable(CACHE_B, mat, Stage::Topology);
}

void TaskDynamics::extendConnectToModel(Model& model) {
    ModelComponent::extendConnectToModel(model);

    model.addModelComponent(compensator);
    model.addModelComponent(taskGraph);
    model.addModelComponent(constraintModel);
}

void TaskDynamics::initializeFromState(const SimTK::State& s) const {
    //Super::extendInitStateFromProperties(s);

    // analytics
    Array<string> labels;
    labels.append("time");

    const auto& cs = _model->getCoordinateSet();
    for (int i = 0; i < cs.getSize(); i++) {
        labels.append("task_" + cs[i].getName());
    }

    for (int i = 0; i < cs.getSize(); i++) {
        labels.append("constraint_forces_" + cs[i].getName());
    }

    for (int i = 0; i < cs.getSize(); i++) {
        labels.append("nullspace_" + cs[i].getName());
    }

    for (int i = 0; i < s.getNMultipliers(); i++) {
        labels.append("lambda_" + changeToString(i));
    }

    for (int i = 0; i < _model->getNumBodies(); i++) {
        for (int j = 0; j < 6; j++) {
            labels.append("reaction_forces" + changeToString(i) + "_" +
                          changeToString(j));
        }
    }

    labels.append("magnitude_t");
    labels.append("magnitude_c");
    labels.append("magnitude_n");
    labels.append("magnitude_a");
    labels.append("kinetic_energy");

    analytics.reset(0);
    analytics.setColumnLabels(labels);
}

/* TaskDynamicsPrioritization ************************************************/

TaskDynamicsPrioritization::TaskDynamicsPrioritization(ConstraintModel::Type type)
    : TaskDynamics(type) {
}

TaskDynamicsPrioritization::~TaskDynamicsPrioritization() {
}

Vector TaskDynamicsPrioritization::calcTaskTorques(const State& s) {
    if (!initialized) {
        initialized = true;
        initializeFromState(s);
    }
    auto graph = taskGraph->updGraph();

    // constraint related
    Vector f = compensator->f(s);
    Vector bc = constraintModel->bc(s);
    Matrix NcT = constraintModel->NcT(s);
    Matrix McInv = constraintModel->McInv(s);
    Vector fPara = NcT * f;

    // local variables
    Vector tauTotal(s.getNU(), 0.0); // total task torques
    Matrix NtT(s.getNU(), s.getNU()); // total task null space
    Matrix NpT(s.getNU(), s.getNU()); // prioritized null space of higher priority task
    Vector tauP(s.getNU(), 0.0); // induced acceleration by higher priority tasks
    tauP = 0;
    NtT = 1;
    NpT = NcT;

    // depth first
    for (tree<TaskPriorityGraph::TaskData*>::pre_order_iterator it = graph.begin();
         it != graph.end(); it++) {
        auto& taskData = *it.node->data;
        auto& t = taskData.task;

        // get prioritized null space and induced torques
        if (it.node->parent != 0) {// has parent, get parent's
            NpT = it.node->parent->data->NpT;
            tauP = it.node->parent->data->tauP;
        }

        // update current task dependencies
        t->setMInv(s, constraintModel->McInv(s));
        t->setPT(s, NpT);

        // calculate task torque
        Vector tauE = fPara + bc - tauP;
        Vector tau = t->tau(s, t->getGoal(), tauE);
        tauTotal += tau;

        // update
        taskData.NpT = NpT * t->NPT(s);
        taskData.tauP = tauP + tau;
        NtT = NtT * t->NPT(s);
    }

    Vector lambda = constraintModel->lambda(s, tauTotal, f);
    Vector JcTLambda = constraintModel->JcTLambda(s, tauTotal, f);
    Vector tauResidual = NcT * NtT * (f + bc);

#if USE_SELECTION == 0
    appendAnalytics(s, tauTotal, JcTLambda, tauResidual, lambda);
    return tauTotal + tauResidual;
#else
    Matrix Q = (1 - B(s)) * NtT, QInv;
    pseudoInverse(Q, true, QInv);
    Matrix P = (1 - NtT * QInv);

    appendAnalytics(s, P * tauTotal, JcTLambda, P * tauResidual, lambda);
    return P * (tauTotal + tauResidual);
#endif
}
