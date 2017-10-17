#ifndef CONSTANT_ANGLE_CONSTRAINT_H
#define CONSTANT_ANGLE_CONSTRAINT_H

#include <OpenSim/OpenSim.h>
#include "internal/OpenSimUtilDLL.h"

namespace OpenSim
{
    /**
     * This constraint consists of a single constraint equation that enforces
     * that a unit vector v1 fixed to one body (the "base body") must maintain a
     * fixed angle theta with respect to a unit vector v2 fixed on the other
     * body (the "follower body"). This can be done with a single constraint
     * equation as long as theta is sufficiently far away from 0 and +/-Pi (180
     * degrees), with the numerically best performance at theta=Pi/2 (90
     * degrees).
     */
    class OpenSimUtil_API ConstantAngleConstraint : public OpenSim::Constraint {
        OpenSim_DECLARE_CONCRETE_OBJECT(ConstantAngleConstraint, Constraint);
    public:
        OpenSim_DECLARE_PROPERTY(baseBody, PhysicalFrame,
            "The base body.");
        OpenSim_DECLARE_PROPERTY(followerBody, PhysicalFrame,
            "The follower body.");
        OpenSim_DECLARE_PROPERTY(axisInBaseBody, SimTK::Vec3,
            "The axis fixed in the base body local frame.");
        OpenSim_DECLARE_PROPERTY(axisInFollowerBody, SimTK::Vec3,
            "The axis fixed in the follower body local frame.");
        OpenSim_DECLARE_PROPERTY(angle, double,
            "The angle between the two axis.");

        /**
         * Constructor.
         *
         * @param baseBody               the base body frame
         * @param axisInBaseBody         the axis fixed in the base body local frame
         * @param followerBody           the follower body frame
         * @param axisInFollowerBody:    the axis fixed in the follower body local frame
         * @param angle:                 the angle between the two axis
         */
        ConstantAngleConstraint(
            const PhysicalFrame& baseBody, const SimTK::Vec3& axisInBaseBody,
            const PhysicalFrame& followerBody, const SimTK::Vec3& axisInFollowerBody,
            double angle) : Constraint() {
            constructProperties();
            set_baseBody(baseBody);
            set_followerBody(followerBody);
            set_axisInBaseBody(axisInBaseBody);
            set_axisInFollowerBody(axisInFollowerBody);
            set_angle(angle);
        }

        virtual ~ConstantAngleConstraint() {};

        void extendAddToSystem(SimTK::MultibodySystem& system) const {
            Super::extendAddToSystem(system);

            SimTK::MobilizedBody baseMobB = get_baseBody().getMobilizedBody();
            SimTK::MobilizedBody followerMobB = get_followerBody().getMobilizedBody();

            SimTK::Constraint::ConstantAngle constAngle(
                baseMobB, SimTK::UnitVec3(get_axisInBaseBody()),
                followerMobB, SimTK::UnitVec3(get_axisInFollowerBody()),
                get_angle());

            assignConstraintIndex(constAngle.getConstraintIndex());
        }

    protected:
        void constructProperties() override {
            constructProperty_baseBody(OpenSim::Body());
            constructProperty_followerBody(OpenSim::Body());
            constructProperty_axisInBaseBody(SimTK::UnitVec3());
            constructProperty_axisInFollowerBody(SimTK::UnitVec3());
            constructProperty_angle(SimTK::Pi / 2);
        }
    };
}

#endif
