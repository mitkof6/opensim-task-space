#ifndef MUSCLE_OPTIMIZATION_H
#define MUSCLE_OPTIMIZATION_H

#include "internal/MuscleOptimizationDLL.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim
{
    class AbstractMuscleOptimization;

    /**
     * A muscle optimization wrapper, that can use a custom optimization
     * target. For a given set of torques, computes the muscle activations.
     *
     * @author Dimiatar Stanev
     */
    class MuscleOptimization_API MuscleOptimization
    {
    public:

        MuscleOptimization(OpenSim::Model* model,
                           AbstractMuscleOptimization* target,
                           bool verbose = false);

        /**
         * Perform optimization for the desired torques
         *
         * @param s the state of the system
         * @param desiredTorques the desired torques under optimization
         * @param activations [out] the muscle activations (are also used as initial condition)
         */
        void tryToOptimize(const SimTK::State& s,
               const SimTK::Vector& desiredTorques,
                           SimTK::Vector& activations) const;

        AbstractMuscleOptimization& updOptimizationTarget();

        /**
         * Should be called after the model have been finalized (build) and
         * before the simulation is carried.
         */
        void initializeFromState(SimTK::State& s);

    private:

        // private variables

        bool verbose;
        SimTK::ReferencePtr<Model> model;
        std::vector<int> reserveActuatorIndicies;
        std::vector<std::string> reserveActuatorNames;
        SimTK::ReferencePtr<AbstractMuscleOptimization> target;
        SimTK::ReferencePtr<SimTK::Optimizer> optimizer;
    };
};

#endif
