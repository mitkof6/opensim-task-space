#include "MuscleOptimizationTarget.h"

#include <OpenSim/Simulation/Manager/Manager.h>

#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

/* AbstractMuscleOptimization ************************************************/

AbstractMuscleOptimization::AbstractMuscleOptimization(bool verbose)
    : OptimizerSystem(), verbose(verbose) {
    numericalGradientFunction = true;
    numericalJacobianFunction = true;

    exponent = 2;
    gamma = 1;
    maxResidualForce = 1000;
}

AbstractMuscleOptimization::~AbstractMuscleOptimization() {
}

bool AbstractMuscleOptimization::useNumericalGradientFunction() const {
    return numericalGradientFunction;
}

bool AbstractMuscleOptimization::useNumericalJacobianFunction() const {
    return numericalJacobianFunction;
}

void AbstractMuscleOptimization::setModel(const Model& m) {
    model = new Model(m.getInputFileName());
    model->updForceSet() = m.getForceSet();
    model->initSystem();
}

void AbstractMuscleOptimization::setActivationExponent(double e) {
    exponent = e;
}

void AbstractMuscleOptimization::setRresidualsPenaly(double g) {
    gamma = g;
}

void AbstractMuscleOptimization::setMaxResidualForce(double r) {
    maxResidualForce = r;
}

Vector AbstractMuscleOptimization::mapToActivations(const Vector& x) const {
    return x;
}

void AbstractMuscleOptimization::initializeFromState(State& s) {
    calcActuatorIndicies(s, *model, activeActuators, disableActuators);
    calcCoordinateIndicies(s, *model, activeCoordinates, constrainedCoordinates);

    numResiduals = model->getActuators().getSize() - activeActuators.size()
        - disableActuators.size();
    numActuators = activeActuators.size();

    if (numResiduals == 0) {
        throw OpenSim::Exception("The model should append residual actuators",
            __FILE__, __LINE__);
    }

    // parameter bounds
    int nm = numActuators;
    int nc = activeCoordinates.size() + constrainedCoordinates.size();
    int nr = numResiduals;
    Vector lowerBounds(nm + nr);
    Vector upperBounds(nm + nr);
    for (int i = 0; i < nm + nr; i++) {
        if (i < nm) {
            lowerBounds[i] = 0;
            upperBounds[i] = 1;
        }
        else {
            /*lowerBounds[i] = -SimTK::Infinity;
            upperBounds[i] = SimTK::Infinity;*/
            lowerBounds[i] = -maxResidualForce;
            upperBounds[i] = maxResidualForce;
        }
    }

    // optimization target
    setNumParameters(nm + nr);
    setNumEqualityConstraints(nc);
    setParameterLimits(lowerBounds, upperBounds);

    if (verbose) {
        cout << "Num parameter: " << getNumParameters() << endl;
        cout << "Num constrains: " << getNumConstraints() << endl;

        cout << "Lower bounds: " << lowerBounds << endl;
        cout << "Upper bounds: " << upperBounds << endl;
    }
}

int OpenSim::AbstractMuscleOptimization::objectiveFunc(const Vector& x,
    bool newPar, Real& f) const {
    f = 0.0;
    for (int i = 0; i < numActuators; i++) {
        f += pow(fabs(x[i]), exponent);
    }

    for (int i = numActuators; i < getNumParameters(); i++) {
        f += gamma * pow(fabs(x[i]) / maxResidualForce, 2);
    }

    return 0;
}

int AbstractMuscleOptimization::gradientFunc(const Vector& x, bool newPar,
    Vector& gradient) const {
    for (int i = 0; i < numActuators; i++) {
        if (x[i] > 0) {
            gradient[i] = exponent * pow(fabs(x[i]), exponent - 1);
        }
        else {
            gradient[i] = -exponent * pow(fabs(x[i]), exponent - 1);
        }
    }

    for (int i = numActuators; i < getNumParameters(); i++) {
        if (x[i] > 0) {
            gradient[i] = gamma * 2 * fabs(x[i]) / maxResidualForce;
        }
        else {
            gradient[i] = -gamma * 2 * fabs(x[i]) / maxResidualForce;
        }
    }

    return 0;
}

int AbstractMuscleOptimization::constraintJacobian(const Vector& x, bool newPar,
    Matrix& jac) const {
    //jac = momentArmMatrix;
    throw OpenSim::Exception("Constraint Jacobian not implemented, use numerical\n",
        __FILE__, __LINE__);

    return 0;
}

void AbstractMuscleOptimization::updateUpperLimit(const Vector& x, bool toOverride) {
    Vector lower(getNumParameters(), 0.0);
    Vector upper(getNumParameters(), 1.0);

    for (int i = 0; i < numActuators; i++) {
        if (!toOverride) {
            upper[i] -= x[i];
        }
        else {
            upper[i] = x[i];
        }
    }
    for (int i = numActuators; i < getNumParameters(); i++) {
        lower[i] = -SimTK::Infinity;
        upper[i] = SimTK::Infinity;
    }

    setParameterLimits(lower, upper);
}

void AbstractMuscleOptimization::evaluateScore(const State& s, const Vector& x) {
    double actObjective, resObjective;
    s.getY();
    Vector temp(getNumParameters(), 0.0);
    temp(0, numActuators) = x(0, numActuators);
    objectiveFunc(temp, true, actObjective);

    temp = Vector(getNumParameters(), 0.0);
    temp(numActuators, numResiduals) = x(numActuators, numResiduals);
    objectiveFunc(temp, true, resObjective);

    SimTK::Vector constraints(getNumConstraints());
    constraintFunc(x, true, constraints);

    //cout.setf(ios::fixed, ios::floatfield);
    //cout.precision(3);
    cout << "Time = " << s.getTime()
        << " activations = " << actObjective
        << " residuals = " << resObjective
        << " constraint violation = " << sqrt(~constraints * constraints) << endl;
}

/* PredictiveOptimizationTarget **********************************************/

PredictiveOptimizationTarget::PredictiveOptimizationTarget(bool verbose)
    : AbstractMuscleOptimization(verbose) {
    numericalGradientFunction = false;
    throw OpenSim::Exception("PredictiveOptimizationTarget not implemented correctly yet",
        __FILE__, __LINE__);
}

void PredictiveOptimizationTarget::prepareToOptimize(const SimTK::State& s,
    const SimTK::Vector& desiredTau) {
    initialState = model->updWorkingState();
    model->initStateWithoutRecreatingSystem(initialState);
    initialState.setTime(s.getTime());
    initialState.setQ(s.getQ());
    initialState.setU(s.getU());
    initialState.setZ(s.getZ());

    Vector x(getNumParameters(), 0.0);
    x(numActuators, numResiduals) = desiredTau;
    
    auto tempState = State(initialState);
    computeAcceleration(tempState, x, desiredAcc, true);
}

int PredictiveOptimizationTarget::constraintFunc(const Vector& x,
    const bool newPar, Vector& constraints) const {
    Vector acc;

    auto tempState = State(initialState);
    computeAcceleration(tempState, x, acc, false);

    constraints = desiredAcc - acc;

    return 0;
}

void PredictiveOptimizationTarget::computeAcceleration(State& s, const Vector& x,
    Vector& udot, bool overrideActuation) const {
    auto& as = model->getActuators();
    Vector controls(model->getNumControls(), 0.0);
    vector<double> muscleExc;
    assert(as.getSize() == x.size());
    for (int i = 0; i < as.getSize(); i++) {
        ScalarActuator* act = dynamic_cast<ScalarActuator*>(&as.get(i));
        if (overrideActuation) {
            act->setOverrideActuation(s, x[i]);
            act->overrideActuation(s, true);
        }
        else {
            act->setControls(Vector(1, x[i]), controls);
            const Muscle* mus = dynamic_cast<const Muscle*>(&as.get(i));
            if (mus) {
                muscleExc.push_back(x[i]);
            }
        }
    }
    //cout << controls << endl;
    // update controls
    //model->setDefaultControls(controls);
    model->realizeVelocity(s);
    model->setControls(s, controls);
    //cout << model->getControls(s) << endl;

    // in case of muscles predict and assign an activation level
    if (1 && !overrideActuation) {
        // predict muscle activation
        const auto& ms = model->getMuscles();
        model->realizeDynamics(s);
        vector<double> muscleAct;
        for (int i = 0; i < ms.getSize(); i++) {
            double adot = ms[i].getStateVariableDerivativeValue(s, "activation");
            double a = ms[i].getStateVariableValue(s, "activation");

            double apred = a + 0.1 * adot;
            muscleAct.push_back(apred);

            if (i == 0) {
                /*cout << ms[i].getName() << "Excitation: " << ms[i].getExcitation(s) << endl;
                cout << ms[i].getName() << "Prev: " << a << " Next: " << apred << endl;*/
            }
        }

        // override muscle activation
        for (int i = 0; i < ms.getSize(); i++) {
            //ms[i].setActivation(s, muscleAct[i]);
            ms[i].setStateVariableValue(s, "activation", muscleAct[i]);
        }
    }
    //cout << s.getSystemStage() << endl;
    // calculate induced acceleration
    model->realizeAcceleration(s);
    udot = s.getUDot();
}

/* ApproximationOptimizationTarget *******************************************/

ApproximationOptimizationTarget::ApproximationOptimizationTarget(bool verbose)
    : AbstractMuscleOptimization(verbose) {
    numericalGradientFunction = false;
}

void ApproximationOptimizationTarget::prepareToOptimize(const SimTK::State& s,
    const SimTK::Vector& desiredTau) {
    tau = desiredTau;
    calcMomentArm(s, *model, activeCoordinates, activeActuators, R);
    Fmax = calcActuatorForce(s, *model, activeActuators,
        Vector(activeActuators.size(), 1.0));
}

int ApproximationOptimizationTarget::constraintFunc(const Vector& x,
    const bool newPar, Vector& constraints) const {
    // act =  x(actuators)
    Vector a = x(0, numActuators);

    //MATRIX_SIZE(r, R);
    //MATRIX_SIZE(F, Fmax);
    //MATRIX_SIZE(aa, a);
    // tau = R * Fmax * a
    Vector temp = R * Fmax.elementwiseMultiply(a);

    // get actuator torques
    Vector actuatorTorques(tau.size(), 0.0);
    int j = 0;
    for (int i = 0; i < activeCoordinates.size(); i++) {
        actuatorTorques[i] = temp[j];
        j++;
    }

    // get residual actuator
    Vector residualTorques(tau.size(), 0.0);
    if (numResiduals != 0) {
        residualTorques = x(numActuators, numResiduals);
    }

    // R * Fmax * a + tau_res - tau_des = 0
    constraints = actuatorTorques + residualTorques - tau;

    return 0;
}

/* SynergyOptimizationTarget *************************************************/

SynergyOptimizationTarget::SynergyOptimizationTarget(bool verbose)
    : ApproximationOptimizationTarget(verbose) {
    numericalGradientFunction = false;
}

Vector SynergyOptimizationTarget::mapToActivations(const Vector& x) const {
    Vector a(activeActuators.size() + numResiduals, 0.0);
    Vector act = S * x(0, numActuators);

    for (int i = 0; i < act.size(); i++) {
        a[i] = act[i];
    }
    int j = getNumParameters() - numResiduals;
    for (int i = a.size() - numResiduals; i < a.size(); i++) {
        a[i] = x[j];
        j++;
    }

    return a;
}

void SynergyOptimizationTarget::setWeightMatrix(const Matrix& w) {
    if (w.ncol() != Fmax.size()) {
        throw OpenSim::Exception("Synergy matrix cols != muscles",
            __FILE__, __LINE__);
    }

    S = w;
    setNumParameters(S.ncol() + numResiduals);
    numActuators = S.ncol();

    if (verbose) {
        MATRIX_SIZE(Synergy, S);
    }
}

const Matrix& OpenSim::SynergyOptimizationTarget::getWeightMatrix() const {
    return S;
}

int SynergyOptimizationTarget::constraintFunc(const Vector& x,
    const bool newPar, Vector& constraints) const {
    // act = S * x
    Vector a = S * x(0, numActuators);

    // tau = R * Fmax * a
    Vector temp = R * Fmax.elementwiseMultiply(a);

    // get actuator torques
    Vector actuatorTorques(tau.size(), 0.0);
    int j = 0;
    for (int i = 0; i < activeCoordinates.size(); i++) {
        actuatorTorques[i] = temp[j];
        j++;
    }

    // get residual actuator
    Vector residualTorques(tau.size(), 0.0);
    if (numResiduals != 0) {
        residualTorques = x(numActuators, numResiduals);
    }

    // R * Fmax * a + tau_res - tau_des = 0
    constraints = actuatorTorques + residualTorques - tau;

    return 0;
}

void SynergyOptimizationTarget::initializeFromState(SimTK::State& s) {
    AbstractMuscleOptimization::initializeFromState(s);

    S = Matrix(activeActuators.size(), activeActuators.size());
    S = 1;
}
