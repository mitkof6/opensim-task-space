#ifndef CONSTRAINT_MODEL
#define CONSTRAINT_MODEL

#include "internal/TaskSpaceDLL.h"

#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
    /* ConstraintModel ********************************************************/

    /**
     * There are different types of constraint inverse dynamics projections. For
     * example DeSaptio et al. solves for the Lagrange multipliers and then
     * derives the dynamically consistent projection operator. Alternatively,
     * Aghili has shown a more generic constraint projection operator that is a
     * pure kinematic quantity. This abstraction enables us to treat different
     * projection operators consistently according to our convention. 
     */
    class TaskSpace_API ConstraintModel : public ModelComponent {
        OpenSim_DECLARE_ABSTRACT_OBJECT(ConstraintModel, ModelComponent);
    public:

        enum Type : short {
                           UNCONSTRAINED = 0,
                           AGHILI,
                           DESAPIO,
                           DESAPIO_AGHILI
        };

        ConstraintModel();

        virtual ~ConstraintModel();

        /**
         * Constraint inertia matrix
         */
        virtual SimTK::Matrix Mc(const SimTK::State& s) const = 0;

        /**
         * Constraint inertia matrix inverse
         */
        SimTK::Matrix McInv(const SimTK::State& s) const;

        /**
         * Induced constraint acceleration
         */
        virtual SimTK::Vector bc(const SimTK::State& s) const = 0;

        /**
         * Constraint null space
         */
        virtual SimTK::Matrix NcT(const SimTK::State& s) const;

        /**
         * Calculates constraint forces
         */
        virtual SimTK::Vector JcTLambda(const SimTK::State& s,
                                        const SimTK::Vector& tau,
					const SimTK::Vector& f) const = 0;

        /**
         * Calculates constraint multipliers
         */
        virtual SimTK::Vector lambda(const SimTK::State& s,
                                     const SimTK::Vector& tau,
				     const SimTK::Vector& f) const = 0;

        /**
         * Constraint Jacobian
         */
        SimTK::Matrix Jc(const SimTK::State& s) const;

        /**
         * Constraint Transpose
         */
        SimTK::Matrix JcT(const SimTK::State& s) const;

    protected:

        /**
         * Inertia mass matrix
         */
        SimTK::Matrix M(const SimTK::State& s) const;

        /**
         * Inverse inertia mass matrix
         */
        SimTK::Matrix MInv(const SimTK::State& s) const;

        /**
         * Constraint bias
         */
        SimTK::Vector b(const SimTK::State& s) const;

        /**
         * Constraint Jacobian inverse
         */
        virtual SimTK::Matrix JcBar(const SimTK::State& s) const;

        /**
         * Constraint Jacobian inverse transposed
         */
        virtual SimTK::Matrix JcBarT(const SimTK::State& s) const = 0;

    protected:

        void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    protected:

        static const std::string CACHE_M;
        static const std::string CACHE_MInv;
        static const std::string CACHE_Mc;
        static const std::string CACHE_McInv;
        static const std::string CACHE_Jc;
        static const std::string CACHE_JcT;
        static const std::string CACHE_JcBar;
        static const std::string CACHE_JcBarT;
        static const std::string CACHE_NcT;
        static const std::string CACHE_b;
        static const std::string CACHE_bc;
        static const std::string CACHE_JcTLambda;
        static const std::string CACHE_lambda;
    };

    /* DeSapioConstraintModel *************************************************/

    class TaskSpace_API DeSapioConstraintModel : public ConstraintModel {
        OpenSim_DECLARE_CONCRETE_OBJECT(DeSapioConstraintModel, ConstraintModel);
    public:

        DeSapioConstraintModel();

        ~DeSapioConstraintModel();

        SimTK::Matrix Mc(const SimTK::State& s) const override;

        SimTK::Vector bc(const SimTK::State& s) const override;

        SimTK::Vector JcTLambda(const SimTK::State& s,
                                const SimTK::Vector& tau,
				const SimTK::Vector& f) const override;

        SimTK::Vector lambda(const SimTK::State& s,
                             const SimTK::Vector& tau,
			     const SimTK::Vector& f) const override;

    protected:

        SimTK::Matrix Lc(const SimTK::State& s) const;

        SimTK::Matrix JcBarT(const SimTK::State& s) const override;

        void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    protected:

        static const std::string CACHE_Lc;
    };

    /* AghiliConstraintModel **************************************************/

    class TaskSpace_API AghiliConstraintModel : public ConstraintModel {
        OpenSim_DECLARE_CONCRETE_OBJECT(AghiliConstraintModel, ConstraintModel);
    public:

        AghiliConstraintModel();

        ~AghiliConstraintModel();

        SimTK::Matrix Mc(const SimTK::State& s) const override;

        SimTK::Vector bc(const SimTK::State& s) const override;

        SimTK::Vector JcTLambda(const SimTK::State& s,
                                const SimTK::Vector& tau,
				const SimTK::Vector& f) const override;

        SimTK::Vector lambda(const SimTK::State& s,
                             const SimTK::Vector& tau,
			     const SimTK::Vector& f) const override;

    protected:

        SimTK::Matrix JcBar(const SimTK::State& s) const override;

        SimTK::Matrix JcBarT(const SimTK::State& s) const override;
    };

    /* UnconstraintModel ******************************************************/

    class TaskSpace_API UnconstraintModel : public ConstraintModel {
        OpenSim_DECLARE_CONCRETE_OBJECT(UnconstraintModel, ConstraintModel);
    public:

        UnconstraintModel();

        ~UnconstraintModel();

        SimTK::Matrix Mc(const SimTK::State& s) const override;

        SimTK::Vector bc(const SimTK::State& s) const override;

        SimTK::Matrix NcT(const SimTK::State& s) const override;

        SimTK::Vector JcTLambda(const SimTK::State& s,
                                const SimTK::Vector& tau,
				const SimTK::Vector& f) const override;

        SimTK::Vector lambda(const SimTK::State& s,
                             const SimTK::Vector& tau,
			     const SimTK::Vector& f) const override;

    protected:

        SimTK::Matrix JcBar(const SimTK::State& s) const override;

        SimTK::Matrix JcBarT(const SimTK::State& s) const override;
    };

    /* DeSapioAghiliConstraintModel *******************************************/

    /**
     * This has not been fully validated !PLEASE DO NOT USE YET!.
     */
    class TaskSpace_API DeSapioAghiliConstraintModel : public ConstraintModel {
        OpenSim_DECLARE_CONCRETE_OBJECT(DeSapioAghiliConstraintModel, ConstraintModel);
    public:

        DeSapioAghiliConstraintModel();

        ~DeSapioAghiliConstraintModel();

        SimTK::Matrix Mc(const SimTK::State& s) const override;

        SimTK::Vector bc(const SimTK::State& s) const override;

        SimTK::Vector JcTLambda(const SimTK::State& s,
                                const SimTK::Vector& tau,
				const SimTK::Vector& f) const override;

        SimTK::Vector lambda(const SimTK::State& s,
                             const SimTK::Vector& tau,
			     const SimTK::Vector& f) const override;

    protected:

        SimTK::Matrix Lc(const SimTK::State& s) const;

        SimTK::Matrix JcBar(const SimTK::State& s) const override;

        SimTK::Matrix JcBarT(const SimTK::State& s) const override;

        void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    protected:

        static const std::string CACHE_Lc;
    };
}
#endif
