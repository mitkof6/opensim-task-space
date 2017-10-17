#ifndef DYNAMIC_COMPENSATOR_H
#define DYNAMIC_COMPENSATOR_H

#include "internal/TaskSpaceDLL.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {
    /**
     * This is a utility class, that is used in the derivation of the task
     * torques and contains the terms of the equation of motion that are common
     * to all tasks, such as gravitational, Coriolis forces and other forces
     * acting in the system.
     *
     * @author Dimitar Stanev
     */
    class TaskSpace_API DynamicCompensator : public ModelComponent {
        OpenSim_DECLARE_CONCRETE_OBJECT(DynamicCompensator, ModelComponent);
    public:

        DynamicCompensator();

        ~DynamicCompensator();

        /**
         * Calculates the contribution of gravity translated to generalized
         * forces.  Must be subtracted, because in this notation gravity is on
         * the same side as the applied forces.
         */
        SimTK::Vector g(const SimTK::State& s) const;

        /**
         * Calculates the contribution of the inertial forces translated to
         * generalized forces. Must be subtracted, because in this notation the
         * inertial forces are on the same side as the applied forces.
         */
        SimTK::Vector c(const SimTK::State& s) const;

        /**
         * Calculates the total force and torques acting on the system, in the
         * joint space.  These forces are projected into task space.
         *
         * \f$ f = \tau_{generalized} + J^T_s f_{spatial} \f$
         * \f$ f = c - g \f$
         *
         */
        SimTK::Vector f(const SimTK::State& s) const;

        /**
         * Calculates a dumping term \f$ f = d * u\f$.
         */
        SimTK::Vector calcJointDumping(const SimTK::State& s, double dumping) const;

        /**
         * Adds a force component that will be excluded during internal
         * computations.
         *
         * This is used in case of a generalized Force that acts as a
         * controller.
         */
        void addExcludedForces(OpenSim::Force* forces);

    protected:

        // ModelComponent methods

        void extendAddToSystem(SimTK::MultibodySystem& system) const override;

        void extendInitStateFromProperties(SimTK::State& s) const override;

    private:

        // private members

        static const std::string CACHE_C; // Coriolis centrifugal
        static const std::string CACHE_G; // gravity
        static const std::string CACHE_F; // total generalized forces applied
                                          // except for constraints

        mutable Model workingModel;
        std::vector<OpenSim::Force*> excludedForces;
    };
}

#endif
