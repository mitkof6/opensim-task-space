#include "MuscleOptimization.h"

#include "MuscleOptimizationTarget.h"
#include "OpenSimUtil.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

MuscleOptimization::MuscleOptimization(Model* model,
                                       AbstractMuscleOptimization* target,
                                       bool verbose)
    : model(model), target(target), verbose(verbose)
{
    addReserveActuators(*model, reserveActuatorNames, reserveActuatorIndicies);
}

void MuscleOptimization::tryToOptimize(const State& s, const Vector& tau,
                                       Vector& activations) const
{
    try
    {
        target->prepareToOptimize(s, tau);
        optimizer->optimize(activations);
    }
    catch (const SimTK::Exception::Base& ex)
    {
        if (1 || verbose)
        {
            cout << "OPTIMIZATION FAILED..." << endl;
            cout << ex.getMessage() << endl;
        }
    }

    if (verbose)
    {
        target->evaluateScore(s, activations);
    }

    activations = target->mapToActivations(activations);
}

AbstractMuscleOptimization& MuscleOptimization::updOptimizationTarget()
{
    return *target;
}

void MuscleOptimization::initializeFromState(SimTK::State& s)
{
    target->setModel(*model);
    target->initializeFromState(s);

    // optimization
    optimizer = new Optimizer(*target, OptimizerAlgorithm::InteriorPoint);
    optimizer->setConvergenceTolerance(1E-4);
    optimizer->setConstraintTolerance(1E-4);
    optimizer->setMaxIterations(1000);
    optimizer->setLimitedMemoryHistory(1000);
    optimizer->setDiagnosticsLevel(0);
    //optimizer->setAdvancedBoolOption("warm_start", true);
    //optimizer->setAdvancedRealOption("obj_scaling_factor", 1);
    //optimizer->setAdvancedRealOption("nlp_scaling_max_gradient", 1);
    optimizer->useNumericalGradient(target->useNumericalGradientFunction());
    optimizer->useNumericalJacobian(target->useNumericalJacobianFunction());

    if (verbose)
    {
        cout << "Use numerical gradient: "
         << target->useNumericalGradientFunction() << endl;
        cout << "Use numerical Jacobian: "
         << target->useNumericalJacobianFunction() << endl;
    }
}
