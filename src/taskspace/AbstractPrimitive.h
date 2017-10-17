#ifndef ABSTRACT_PRIMITIVE_H
#define ABSTRACT_PRIMITIVE_H

#include "internal/TaskSpaceDLL.h"

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
    /* AbstractPrimitive ******************************************************/

    // forward declaration
    class TaskPrimitive;

    /**
     * An abstract primitive.
     *
     * @author Dimitar Stanev
     */
    class TaskSpace_API AbstractPrimitive : public ModelComponent {
        OpenSim_DECLARE_ABSTRACT_OBJECT(AbstractPrimitive, ModelComponent);
    public:

        enum PrimitiveType {
                            TASK = 0,
                            UNKNOWN
        };

        AbstractPrimitive();

        virtual ~AbstractPrimitive();

        virtual TaskPrimitive* getAsTaskPrimitive() {
            throw OpenSim::Exception("Can't get as MuscleBasedArmController",
                                     __FILE__, __LINE__);
            return NULL;
        }

        /**
         * Get primitive type
         */
        PrimitiveType getPrimitiveType() const;
        std::string getFromPrimitiveType(PrimitiveType type) const;
        std::string getPrimitiveTypeAsString() const;

        /**
         * The setGoal can be override, in order to implement a control law,
         * by the subclasses of the TaskPrimitive class.
         */
        virtual void setGoal(const SimTK::State& s, const SimTK::Vector& goal);

        /**
         * Goal getter
         */
        SimTK::Vector getGoal() const;

        /**
         * Overrides the goal without the need of state.
         */
        void setOverrideGoal(const SimTK::Vector& goal);

        /**
         * The prioritized null space transpose matrix
         */
        void setPT(const SimTK::State& s, const SimTK::Matrix& PT);

        /**
         * Sets the inverse mass matrix
         */
        void setMInv(const SimTK::State& s, const SimTK::Matrix& MInv);

        /**
         * Task Jacobian
         *
         * \f$ J \in R^{t x n} \f$
         */
        virtual SimTK::Matrix J(const SimTK::State& s) const = 0;

        /**
         * Task priority Jacobian
         *
         * \f$ J N_{p*} \in R^{t x n} \f$
         */
        SimTK::Matrix JP(const SimTK::State& s) const;

        /**
         * Task Jacobian transpose
         *
         * \f$ J^T \in R^{n x t} \f$
         */
        SimTK::Matrix JT(const SimTK::State& s) const;

        /**
         * Task priority Jacobian transpose
         *
         * \f$ N^T_{p*} J^T \in R^{n x t} \f$
         */
        SimTK::Matrix JPT(const SimTK::State& s) const;

        /**
         * Task effective inertia matrix
         *
         * \f$ L = (J * M^{-1} * J^{T})^{-1} = L^{-1} \in R^{t x t} \f$
         */
        SimTK::Matrix L(const SimTK::State& s) const;

        /**
         * Task prioritized effective inertia matrix
         *
         * \f$ L_p = (J * M^{-1} * J_p^{T})^{-1} = L_p^{-1} \in R^{t x t} \f$
         */
        SimTK::Matrix LP(const SimTK::State& s) const;

        /**
         * Task effective inertia inverse matrix (dim x dim)
         *
         * \f$ L^{-1} = (J * M^{-1} * J^{T}) \in R^{t x t} \f$
         */
        SimTK::Matrix LInv(const SimTK::State& s) const;

        /**
         * Task prioritized effective inertia inverse matrix
         *
         * \f$ L_p^{-1} = (J * M^{-1} * J_p^{T})  \in R^{t x t} \f$
         */
        SimTK::Matrix LPInv(const SimTK::State& s) const;

        /**
         * Dynamically consistent generalized inverse task Jacobian
         *
         * \f$ \bar{J}^T = ~JBarT  \in R^{n x t} \f$
         */
        SimTK::Matrix JBar(const SimTK::State& s) const;

        /**
         * Dynamically consistent prioritized generalized inverse task Jacobian
         *
         * \f$ \bar{J}_p = ~JPBarT \in R^{n x t} \f$
         */
        SimTK::Matrix JPBar(const SimTK::State& s) const;

        /**
         * Dynamically consistent generalized inverse transpose task Jacobian
         *
         * \f$ \bar{J}^T = L_p J^{T} M^{-1} \in R^{t x n} \f$
         */
        SimTK::Matrix JBarT(const SimTK::State& s) const;

        /**
         * Dynamically consistent prioritized generalized inverse transpose task
         * Jacobian
         * (dim x nu)
         *
         * \f$ \bar{J}_p^T =  L_p * J^{T} * M^{-1} * N^T_{p*}\in R^{t x n} \f$
         */
        SimTK::Matrix JPBarT(const SimTK::State& s) const;

        /**
         * Calculates the bias term, which depends on the task type
         *
         * $\f b \in R^{n} \f$
         */
        virtual SimTK::Vector b(const SimTK::State& s) const = 0;

        /**
         * Task null space matrix
         *
         * \f$ N = ~NT \f$
         */
        SimTK::Matrix N(const SimTK::State& s) const;

        /**
         * Task prioritized null space matrix
         *
         * \f$ N = ~NPT \f$
         */
        SimTK::Matrix NP(const SimTK::State& s) const;

        /**
         * Task null space transpose matrix
         *
         * \f$ N = I - J^T_p \bar{J}^{T} \in R^{n x n} \f$
         */
        SimTK::Matrix NT(const SimTK::State& s) const;

        /**
         * Task prioritized null space transpose matrix
         *
         * \f$ N = I - J^T \bar{J}_p^T \in R^{n x n} \f$
         */
        SimTK::Matrix NPT(const SimTK::State& s) const;

        /**
         * This method computes the induced torques by the task, for a given
         * goal \f$ \ddot{x}\f$ and any acting forces \f$ tau_p \f$ on the model
         * and any torque induced by higher priority tasks.
         */
        virtual SimTK::Vector tau(const SimTK::State& s, const SimTK::Vector& goal,
                                  const SimTK::Vector& tauP) const = 0;

    private:
        /**
         * Task prioritized null space matrix, corresponding to the parents
         * level subspace.
         */
        SimTK::Matrix PT(const SimTK::State& s) const;

        /**
         * System inverse mass matrix.
         */
        SimTK::Matrix MInv(const SimTK::State& s) const;

    protected:

        void extendAddToSystem(SimTK::MultibodySystem& system) const override;

        PrimitiveType primitiveType;
        SimTK::Vector xddot;

        static const std::string CACHE_MInv;
        static const std::string CACHE_J;
        static const std::string CACHE_JP;
        static const std::string CACHE_JT;
        static const std::string CACHE_JPT;
        static const std::string CACHE_L;
        static const std::string CACHE_LP;
        static const std::string CACHE_LInv;
        static const std::string CACHE_LPInv;
        static const std::string CACHE_JBar;
        static const std::string CACHE_JPBar;
        static const std::string CACHE_JBarT;
        static const std::string CACHE_JPBarT;
        static const std::string CACHE_B;
        static const std::string CACHE_N;
        static const std::string CACHE_NP;
        static const std::string CACHE_NT;
        static const std::string CACHE_NPT;
        static const std::string CACHE_PT;
    };
}
#endif
