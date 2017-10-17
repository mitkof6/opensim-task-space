#include "TaskTrackingController.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

/* TaskTrackingController *****************************************************/

TaskTrackingController::TaskTrackingController()
{
}

TaskTrackingController::~TaskTrackingController()
{
}

Vector TaskTrackingController::calcControl(const Vector& desired,
					   const Vector& x,
					   const Vector& v) const
{
    return desired;
}

/* PDController ***************************************************************/

PDController::PDController(double kp, double kv)
    : TaskTrackingController(), kp(kp), kv(kv)
{
}

PDController::~PDController()
{
}

Vector PDController::calcControl(const Vector& d, const Vector& x,
				 const Vector& v) const
{
    return -kp * (x - d) - kv * v;
}

/* VelocitySaturationController ***********************************************/

VelocitySaturationController::VelocitySaturationController(double kp,
							   double kv,
							   double uMax)
    : TaskTrackingController(), kp(kp), kv(kv), uMax(uMax)
{
}

VelocitySaturationController::~VelocitySaturationController()
{
}

Vector VelocitySaturationController::calcControl(const Vector& d,
						 const Vector& x,
						 const Vector& v) const
{
    Vector vd = -kp / kv * (x - d);

    Vector comp(2, 0.0);
    comp[0] = 1;
    comp[1] = uMax / vd.norm();
    double vv = min(comp);

    return -kv * (v - vv * vd);
}

/* AccelerationSaturationController *******************************************/

AccelerationSaturationController::AccelerationSaturationController(double p,
								   double v,
								   double um,
								   double am)
    : TaskTrackingController(), kp(p), kv(v), uMax(um), aMax(am)
{
}

AccelerationSaturationController::~AccelerationSaturationController()
{
}

Vector AccelerationSaturationController::calcControl(const Vector& d,
						     const Vector& x,
						     const Vector& v) const
{
    Vector vd = -kp / kv * (x - d);

    Vector comp(2, 0.0);
    comp[0] = 1;
    comp[1] = uMax / vd.norm();
    double vv = min(comp);

    Vector ad = -kv * (v - vv * vd);

    comp[1] = aMax / ad.norm();
    double va = min(comp);

    return va * ad;
}
