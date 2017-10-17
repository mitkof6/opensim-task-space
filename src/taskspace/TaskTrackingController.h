#ifndef TASK_TRACKING_CONTROLLER_H
#define TASK_TRACKING_CONTROLLER_H

#include "internal/TaskSpaceDLL.h"

#include <SimTKcommon.h>

namespace OpenSim
{
    /* TaskTrackingController *************************************************/

    /**
     * An interface for creating task control laws. By default the desired goal
     * is the same as the assigned goal (no control law).
     *
     * @author Dimitar Stanev
     */
    class TaskSpace_API TaskTrackingController
    {
    public:

        TaskTrackingController();

        virtual ~TaskTrackingController();

        /**
         * This method should be derived by the subclasses.
         */
        virtual SimTK::Vector calcControl(const SimTK::Vector& desired,
            const SimTK::Vector& x,
            const SimTK::Vector& v) const;
    };

    /* PDController ***********************************************************/

    /**
     * For a desired position, compute desired acceleration given:
     *
     * \f$ a_d = -kp * (x - x_d) - kv * v \f$.
     *
     * @author Dimitar Stanev
     */
    class TaskSpace_API PDController : public TaskTrackingController
    {
    public:

        PDController(double kp, double kv);

        ~PDController();

        SimTK::Vector calcControl(const SimTK::Vector& desired,
            const SimTK::Vector& x,
            const SimTK::Vector& v) const override;

    private:
        double kp, kv;
    };

    /* VelocitySaturationController *******************************************/

    /**
    * For a desired position, compute the desired acceleration with an upper
    * bound in the permissive velocity:
    *
    * \f$ a_d = -kv * (v - v_v * v_d) - kv * v \f$
    *
    * \f$ v_v = min(1, \frac{v_{max}}{\|v_d\|}) \f$
    *
    * \f$ v_d = -\frac{k_p}{k_v} (x - x_d) \f$
    *
    * If the \f$ \|v_d\| \le v_max \f$ this control law will be equivalent to a
    * PD controller, else if the desired velocity reaches values beyond the
    * maximum the above control law will yield a constant velocity in the
    * direction of the goal.
    *
    * Khatib O., Real-time obstacle avoidance for manipulators and mobile
    * robots.  International Journal of Robotics Research 5(1), 90-8, (1986).
    *
    * @author Dimitar Stanev
    */

    class TaskSpace_API VelocitySaturationController : public TaskTrackingController
    {
    public:

        VelocitySaturationController(double kp, double kv, double uMax);

        ~VelocitySaturationController();

        SimTK::Vector calcControl(const SimTK::Vector& desired,
            const SimTK::Vector& x,
            const SimTK::Vector& v) const override;

    private:
        double kp, kv, uMax;
    };

    /* AccelerationSaturationController ***************************************/

    /**
     * For a desired position, compute the desired acceleration with an upper
     * bound in the permissive acceleration:
     *
     * \f$ a_{ref} = v_a * a_d \f$
     *
     * \f$ v_v = min(1, \frac{v_{max}}{\|v_d\|}) \f$
     *
     * \f$ v_a = min(1, \frac{a_{max}}{\|a_d\|}) \f$
     *
     * \f$ v_d = -\frac{k_p}{k_v} (x - x_d) \f$
     *
     * \f$ a_d = -kv * (v - v_v * v_d) - kv * v \f$
     *
     * If the \f$ \|a_d\| \le a_max \f$ this control law will be equivalent to a
     * PD controller, else if the desired acceleration reaches values beyond the
     * maximum the above control law will yield a constant acceleration in the
     * direction of the goal. Consequently, there will be a restriction in the
     * velocity.
     *
     * Khatib O., Real-time obstacle avoidance for manipulators and mobile
     * robots.  International Journal of Robotics Research 5(1), 90-8, (1986).
     *
     * @author Dimitar Stanev
     */
    class TaskSpace_API AccelerationSaturationController : public TaskTrackingController
    {
    public:

        AccelerationSaturationController(double kp, double kv, double uMax, double aMax);

        ~AccelerationSaturationController();

        SimTK::Vector calcControl(const SimTK::Vector& desired,
            const SimTK::Vector& x,
            const SimTK::Vector& v) const override;

    private:
        double kp, kv, uMax, aMax;
    };
}

#endif
