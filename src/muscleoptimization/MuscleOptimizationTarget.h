#ifndef MUSCLE_OPTIMIZATION_TARGET_H
#define MUSCLE_OPTIMIZATION_TARGET_H

#include "internal/MuscleOptimizationDLL.h"

#include <simmath/Optimizer.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {
    class SynergyOptimizationTarget;
    class PredictiveOptimizationTarget;
    class ApproximationOptimizationTarget;

    /**
     * An abstract class for creating optimization targets that solve the muscle
     * redundancy problem. We assume that there are na actuators and nr residual
     * generalized forces.
     *
     * \f$ min \sum^{i=1}_{na} a_i^p + \gamma \sum^{i=na+1}_{nr} a_i^2 \f$
     * subject to achieve tau (depends on the underlying target)
     *
     * @author Dimitar Stanev
     */
    class MuscleOptimization_API AbstractMuscleOptimization
        : public SimTK::OptimizerSystem {
    public:

        AbstractMuscleOptimization(bool verbose);

        virtual ~AbstractMuscleOptimization();

        /* should override ****************************************************/

        /**
         * Used to cast this abstract object as ApproximationOptimizationTarget.
         */
        virtual ApproximationOptimizationTarget* getAsApproximationOptimizationTarget() {
            throw OpenSim::Exception(
        "Can't be cast as ApproximationOptimizationTarget \n",
                                     __FILE__, __LINE__);
            return NULL;
        }

        /**
         * Used to cast this abstract object as SynergyOptimizationTarget.
         */
        virtual SynergyOptimizationTarget* getAsSynergyOptimizationTarget() {
            throw OpenSim::Exception(
        "Can't be cast as SynergyOptimizationTarget \n",
                                     __FILE__, __LINE__);
            return NULL;
        }

        /**
         * Used to cast this abstract object as PredictiveOptimizationTarget.
         */
        virtual PredictiveOptimizationTarget* getAsPredictiveOptimizationTarget() {
            throw OpenSim::Exception(
        "Can't be cast as PredictiveOptimizationTarget \n",
                                     __FILE__, __LINE__);
            return NULL;
        }

        /**
         * Should be called before optimize, in order to update the internal
         * variables at each time step.
         */
        virtual void prepareToOptimize(const SimTK::State& s,
                                       const SimTK::Vector& tau) = 0;

        /**
         * The solution of the optimization target may not correspond to the
         * muscle activations. (default return x).
         *
         * For example if act = W * x, where x is the optimal solution of the
         * optimization.
         */
        virtual SimTK::Vector mapToActivations(const SimTK::Vector& x) const;

        virtual void initializeFromState(SimTK::State& s);

        // Required methods from OptimizaerSystem

        virtual int objectiveFunc(const SimTK::Vector& x, bool newPar,
                                  SimTK::Real& f) const;
        virtual int gradientFunc(const SimTK::Vector& x, bool newPar,
                                 SimTK::Vector& gradient) const;
        virtual int constraintFunc(const SimTK::Vector& x, bool newPar,
                                   SimTK::Vector& constraints) const = 0;
        virtual int constraintJacobian(const SimTK::Vector& x, bool newPar,
                                       SimTK::Matrix& jac) const;

        /**********************************************************************/

        /**
         * In case of multiple tasks this function should be called, to
         * constraint the upper bound of the lower priority task in order to
         * constrain the residual activation space.
         */
        void updateUpperLimit(const SimTK::Vector& x, bool toOverride = false);

        /**
         * Prints the performance and constraint violation.
         */
        void evaluateScore(const SimTK::State& s, const SimTK::Vector& x);

        bool useNumericalGradientFunction() const;

        bool useNumericalJacobianFunction() const;

        void setModel(const Model& model);

        void setActivationExponent(double exponent);

        void setRresidualsPenaly(double gamma);

        void setMaxResidualForce(double rMax);

    protected:

        bool verbose;

        /**
         * Activation exponent.
         */
        double exponent;

        /**
         * Residual actuator penalty.
         */
        double gamma;

        /**
         * This variable is used to normalize the residual exponents.
         */
        double maxResidualForce;

        int numActuators, numResiduals;
        SimTK::ReferencePtr<Model> model;
        bool numericalGradientFunction, numericalJacobianFunction;
        Array<int> activeActuators, disableActuators,
            activeCoordinates, constrainedCoordinates;
    };

    /* PredictiveOptimizationTarget *******************************************/

    /**
     * Solves the muscle optimization problem and defines the constraint
     * function as the difference between the induced acceleration defined by
     * the target generalized forces vs the induced acceleration of the
     * solution. This type of optimization approximates the muscle force better,
     * because we compare the actual result of the excitation signals. On the
     * other hand it is considered slower that computing the product R * F(x) =
     * tau.
     *
     * @author Dimiatar Stanev
     */
    class MuscleOptimization_API PredictiveOptimizationTarget
        : public AbstractMuscleOptimization {
    public:

        PredictiveOptimizationTarget(bool verbose);

        PredictiveOptimizationTarget* getAsPredictiveOptimizationTarget()
        override {
            return this;
        }

        void prepareToOptimize(const SimTK::State& s, const SimTK::Vector& tau)
        override;

        // Required methods from AbstractMuscleOptimization

        int constraintFunc(const SimTK::Vector& x, bool newPar,
                           SimTK::Vector& constraints) const override;

    private:

        void computeAcceleration(SimTK::State& s, const SimTK::Vector& x,
                                 SimTK::Vector& udot,
                 bool overrideActuation) const;

    private:

        SimTK::State initialState;
        SimTK::Vector desiredAcc;
    };

    /* ApproximationOptimizationTarget ****************************************/

    /**
     * Solves the muscle optimization by providing an estimate of the maximum
     * muscle force thus the constraints are given by solving the equality R *
     * Fmax * a = tau. This target is fast, and works well for linear muscles,
     * while it can still approximate the non-linear muscle models.
     *
     * @author Dimiatar Stanev
     */
    class MuscleOptimization_API ApproximationOptimizationTarget
        : public AbstractMuscleOptimization {
    public:

        ApproximationOptimizationTarget(bool verbose);

        ApproximationOptimizationTarget* getAsApproximationOptimizationTarget()
        override {
            return this;
        }

        void prepareToOptimize(const SimTK::State& s, const SimTK::Vector& tau)
        override;

        // Required methods from OptimizaerSystem

        int constraintFunc(const SimTK::Vector& x, bool newPar,
                           SimTK::Vector& constraints) const override;

    protected:

        SimTK::Matrix R;
        SimTK::Vector tau, Fmax;
    };

    /* SynergyOptimizationTarget **********************************************/

    /**
     * This class provides an interface specification for static optimization
     * objective function. This implements a synergy optimization [1].
     *
     * [1] Kathrine Steele et al., Consequences of biomechanically constrained
     * tasks in the design and interpretation of synergy analyses, Journal of
     * neurophysiology, 2015
     *
     * @author Dimiatar Stanev
     */
    class MuscleOptimization_API SynergyOptimizationTarget
        : public ApproximationOptimizationTarget {
    public:

        SynergyOptimizationTarget(bool verbose);

        SynergyOptimizationTarget* getAsSynergyOptimizationTarget() override {
            return this;
        }

        SimTK::Vector mapToActivations(const SimTK::Vector& parameters)
        const override;

        void initializeFromState(SimTK::State& s) override;

        // Required methods from OptimizaerSystem

        int constraintFunc(const SimTK::Vector& x, bool newPar,
                           SimTK::Vector& constraints) const override;

        /**
         * This method is used to set the synergy matrix. The dimension of
         * optimization parameters change by the number of columns that the
         * weighting matrix posses.
         */
        void setWeightMatrix(const SimTK::Matrix& weight);

        const SimTK::Matrix& getWeightMatrix() const;

    private:

        SimTK::Matrix S;
    };
}

#endif
